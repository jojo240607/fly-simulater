//! 阶段 8：Fly Simulator Web 后端。
//!
//! 架构：**仿真在原生 Rust 进程**（复用 `fly-sim-core` + `phy-demo` 软件光栅化），
//! 经 WebSocket 把渲染帧（像素）推给浏览器 `<canvas>`，并接收前端控制指令。
//!
//! - 零额外 crate 依赖：HTTP 静态服务 + WebSocket(RFC6455) 均手写（见 `ws.rs`）。
//! - 渲染复用 `fly_sim_core::render::render_frame` —— 与原生窗口同一渲染源。
//! - 仿真以"增量步进"驱动（`SimLoop::step_frame`），每显示帧推进若干物理步，
//!   支持场景中途热切换（场景/控制律/故障电机/风/相机）。
//!
//! 启动：`cargo run -p fly-sim-server` → 打开 http://127.0.0.1:8080/

// 整个 Web 后端依赖真实物理引擎 + 软件光栅化原语（phy feature）。
// 非 phy 构建下整模块禁用，仅保留一个提示性 main。
#![cfg(feature = "phy")]

mod ws;

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use fly_sim_core::controller::{hover_setpoint, ControllerKind};
use fly_sim_core::physics::{ContactModel, DynamicObstacle, Obstacle, PhySdkWorld};
use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig};
use fly_sim_core::sim::SimLoop;
use fly_sim_core::wind::{WindConfig, WindField};
use fly_sim_core::render::{RenderInput, RenderObstacle, RenderRay, RenderWind};
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;

const PORT: u16 = 8080;
const FRAME_W: u32 = 720;
const FRAME_H: u32 = 540;
const STEPS_PER_FRAME: u64 = 8; // 每显示帧推进的物理步（dt=4ms → ~32ms/帧 ≈ 31fps 仿真时钟）
const DT: f64 = 0.004;
const DEGRADE_STABILIZE_STEPS: u64 = 1000; // 退化场景先稳态 4s

/// 前端→后端 的控制状态（由 WS 文本消息更新，仿真线程读取）。
#[derive(Clone)]
struct ControlState {
    scenario: String,  // "hover" | "wind" | "degraded" | "avoidance"
    controller: String, // "pid" | "lqr" | "indi"
    wind: f64,          // 北向基础风速 m/s
    fail_motor: Option<u8>,
    degrade: Option<(usize, f32)>,
    cam_yaw: f32,
    cam_pitch: f32,
    cam_distance: f32,
    dirty: bool, // 配置变更 → 需要重建仿真
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            scenario: "hover".to_string(),
            controller: "pid".to_string(),
            wind: 0.0,
            fail_motor: None,
            degrade: None,
            cam_yaw: 0.6,
            cam_pitch: 0.45,
            cam_distance: 28.0,
            dirty: true,
        }
    }
}

/// 仿真驱动内部状态（每 WS 连接一个）。
struct SimDriver {
    sim: Option<SimLoop<PhySdkWorld>>,
    cfg: VehicleConfig,
    sensor_cfg: SensorConfig,
    sp: Setpoint,
    phase_steps: u64,      // 当前场景已步进步数
    degrade_injected: bool,
    trail: Vec<[f64; 3]>,  // 渲染系轨迹
}

impl SimDriver {
    fn new(cfg: VehicleConfig, sensor_cfg: SensorConfig) -> Self {
        Self {
            sim: None,
            cfg,
            sensor_cfg,
            sp: hover_setpoint(0.0, 0.0, -5.0),
            phase_steps: 0,
            degrade_injected: false,
            trail: Vec::new(),
        }
    }

    fn controller_kind(s: &str) -> ControllerKind {
        match s {
            "lqr" => ControllerKind::Lqr,
            "indi" => ControllerKind::Indi,
            "tecs" => ControllerKind::Tecs,
            _ => ControllerKind::Pid,
        }
    }

    fn rebuild(&mut self, c: &ControlState) {
        let wind = if c.wind > 0.0 {
            Some(WindField::new(WindConfig {
                base: [c.wind, 0.0, 0.0],
                ..Default::default()
            }))
        } else {
            None
        };
        let kind = Self::controller_kind(&c.controller);
        // 场景障碍：avoidance 场景放"静态矮墙 + 动态逼近球"（渲染可视化 + 真实碰撞/避障）。
        let mut obstacles: Vec<Obstacle> = Vec::new();
        if c.scenario == "avoidance" {
            // 北侧一道矮墙（盒）与西北角一个球，丰富场景；机体悬停原点 (0,5,0)。
            obstacles.push(Obstacle::Box { min: [14.0, 0.0, -10.0], max: [16.0, 3.0, 10.0] });
            obstacles.push(Obstacle::Sphere { center: [-4.0, 5.0, -13.0], radius: 2.5 });
        }
        let mut sim = SimLoop::new(
            PhySdkWorld::create_empty(),
            &self.cfg,
            DT,
            wind,
            self.sensor_cfg.clone(),
            kind,
            Some(ContactModel::default()),
            obstacles,
        );
        if c.scenario == "avoidance" {
            // 动态障碍：南侧球匀速向北逼近（与 avoidance 测试同构）。
            sim.plant_set_dynamic_obstacles(vec![DynamicObstacle {
                base: Obstacle::Sphere { center: [-26.0, 5.0, 0.0], radius: 3.0 },
                velocity: [2.0, 0.0, 0.0],
            }]);
            // 扇式多射线测距（±60°×5 条，量程 12m）+ 避障闭环（危险距离 11m，横向闪避 2m/s）。
            sim.configure_avoidance(
                RangeFinderModel::new(
                    12.0,
                    0.5,
                    0.02,
                    0.0,
                    0.0,
                    std::f64::consts::PI / 3.0,
                    5,
                    0xABCD,
                ),
                AvoidanceConfig::new(11.0, 0.0, 2.0),
            );
        }
        // 故障注入
        if let Some(m) = c.fail_motor {
            let mut mask = [false; 4];
            if (m as usize) < 4 {
                mask[m as usize] = true;
            }
            sim.set_motor_failure(mask);
        } else if let Some((m, e)) = c.degrade {
            let mut eff = [1.0f32; 4];
            if m < 4 {
                eff[m] = e;
            }
            sim.set_motor_eff(eff);
        }
        self.sim = Some(sim);
        self.phase_steps = 0;
        self.degrade_injected = false;
        self.trail.clear();
    }

    /// 推进一帧，返回 (渲染输入, 遥测 JSON 字符串, 场景是否结束)。
    fn advance(&mut self, c: &ControlState) -> Option<(RenderInput, String)> {
        let sim = self.sim.as_mut()?;
        let mut last = None;
        let mut diverged = false;
        for _ in 0..STEPS_PER_FRAME {
            let (st, cmd) = sim.step_frame(&self.sp);
            self.phase_steps += 1;

            // 退化场景：先稳态再注入
            if c.scenario == "degraded" {
                if !self.degrade_injected && self.phase_steps >= DEGRADE_STABILIZE_STEPS {
                    if let Some((m, e)) = c.degrade {
                        let mut eff = [1.0f32; 4];
                        if m < 4 {
                            eff[m] = e;
                        }
                        sim.set_motor_eff(eff);
                    } else if let Some(m) = c.fail_motor {
                    let mut eff = [1.0f32; 4];
                    if (m as usize) < 4 {
                        eff[m as usize] = 0.0;
                    }
                    sim.set_motor_eff(eff);
                }
                    self.degrade_injected = true;
                }
                // 发散检测
                let w = st.omega;
                let rate = (w[0].0 * w[0].0 + w[1].0 * w[1].0 + w[2].0 * w[2].0).sqrt();
                if rate > 1.0 || !flyctrl_core::invariants::state_finite(&st) {
                    diverged = true;
                }
            }
            last = Some((st, cmd));
        }
        let (st, cmd) = last?;

        // 构造渲染输入：渲染世界 = 引擎世界（同为 Y-up，上=+Y）。直接用引擎位姿。
        // 引擎悬停时机体 +Z（推力轴）指向 +Y，即旋翼盘水平 → 渲染必然水平。
        // 不做任何 NED 或 z 镜像变换（镜像会把水平机体翻成侧躺）。
        let (pos, q_up) = sim.debug_up();
        let quat = q_up;
        // 速度：`st.vel` 是 NED (n,e,d)，转引擎世界系 (x=n, y=-d, z=-e) 与渲染一致。
        let vel = [
            st.vel[0].0 as f64,
            -st.vel[2].0 as f64,
            -st.vel[1].0 as f64,
        ];
        let mut motors = [0.0f64; 4];
        for i in 0..4 {
            motors[i] = cmd.motor[i] as f64;
        }
        let mut eff = [1.0f32; 4];
        if let Some((m, e)) = c.degrade {
            if m < 4 {
                eff[m] = e;
            }
        } else if let Some(m) = c.fail_motor {
            if (m as usize) < 4 {
                eff[m as usize] = 0.0;
            }
        }

        self.trail.push(pos);
        if self.trail.len() > 120 {
            self.trail.remove(0);
        }

        // ---- P3-D3：障碍 / 射线 / 风场可视化数据（引擎世界系 = 渲染世界系）----
        // 1) 障碍：当前生效（静态 + 动态展平），转轻量渲染表示。
        let obstacles: Vec<RenderObstacle> = sim
            .current_obstacles()
            .iter()
            .filter_map(|o| match o {
                Obstacle::Sphere { center, radius } => {
                    Some(RenderObstacle::Sphere { center: *center, radius: *radius })
                }
                Obstacle::Box { min, max } => {
                    Some(RenderObstacle::Box { min: *min, max: *max })
                }
                Obstacle::ConvexHull { .. } => None, // 已展平，不应出现
            })
            .collect();

        // 2) 射线：最近一次扇式测距帧。读数方向是 NED，转引擎世界系与渲染一致。
        let mut rays: Vec<RenderRay> = Vec::new();
        if let Some(frame) = sim.ranger_frame() {
            for r in frame.rays {
                let d = r.dir_ned;
                // NED (n,e,d) → 引擎 (x=n, y=-d, z=-e)
                let dir = [d[0], -d[2], -d[1]];
                rays.push(RenderRay { dir, distance: r.distance, valid: r.valid });
            }
        }

        // 3) 风场：机体周围水平面 5×5 网格只读采样箭头（有风场景才画）。
        let mut wind: Vec<RenderWind> = Vec::new();
        if c.wind > 0.0 {
            for ix in -2i32..=2 {
                for iz in -2i32..=2 {
                    let p = [pos[0] + ix as f64 * 4.0, pos[1], pos[2] + iz as f64 * 4.0];
                    let vec = sim.wind_at(p);
                    if vec[0] != 0.0 || vec[1] != 0.0 || vec[2] != 0.0 {
                        wind.push(RenderWind { pos: p, vec });
                    }
                }
            }
        }

        let inp = RenderInput {
            pos,
            quat,
            motors,
            vel,
            eff,
            trail: self.trail.clone(),
            arm: self.cfg.arm_length as f64,
            visual_scale: 2.5,
            cam_yaw: c.cam_yaw,
            cam_pitch: c.cam_pitch,
            cam_distance: c.cam_distance,
            blink: self.phase_steps as f64 * 0.05,
            obstacles,
            rays,
            wind,
        };

        let tele = telemetry_json(&st, &cmd, c, self.phase_steps, diverged);
        if diverged {
            // 退化发散后重置，保持连续动画
            self.rebuild(c);
        }
        Some((inp, tele))
    }
}

fn telemetry_json(
    st: &flyctrl_core::vehicle::VehicleState,
    cmd: &flyctrl_core::vehicle::ActuatorCmd,
    c: &ControlState,
    steps: u64,
    diverged: bool,
) -> String {
    let alt = -st.pos[2].0; // NED d 向下，高度 = -d
    let horiz = (st.pos[0].0 * st.pos[0].0 + st.pos[1].0 * st.pos[1].0).sqrt();
    let speed = (st.vel[0].0 * st.vel[0].0 + st.vel[1].0 * st.vel[1].0 + st.vel[2].0 * st.vel[2].0)
        .sqrt();
    let q = st.att;
    // 由四元数(NED)导出欧拉角（roll/pitch/yaw）
    let roll = (2.0 * (q.w * q.x + q.y * q.z))
        .atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    let pitch = (2.0 * (q.w * q.y - q.z * q.x)).asin().clamp(-1.57, 1.57);
    let yaw = (2.0 * (q.w * q.z + q.x * q.y))
        .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    format!(
        "{{\"alt\":{:.3},\"horiz\":{:.3},\"speed\":{:.3},\"roll\":{:.3},\"pitch\":{:.3},\"yaw\":{:.3},\"m\":[{},{},{},{}],\"steps\":{},\"scenario\":\"{}\",\"diverged\":{}}}",
        alt, horiz, speed, roll, pitch, yaw,
        cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
        steps, c.scenario, diverged
    )
}

fn handle_ws(stream: TcpStream, ctrl: Arc<Mutex<ControlState>>) {
    let cfg = VehicleConfig::default_quad();
    let sensor_cfg = SensorConfig::default();
    let mut driver = SimDriver::new(cfg, sensor_cfg);

    // 读线程：接收前端控制消息
    let reader_stream = stream.try_clone().unwrap();
    let reader_ctrl = ctrl.clone();
    let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();
    thread::spawn(move || {
        let mut s = reader_stream;
        loop {
            match ws::read_frame(&mut s) {
                Ok((op, payload)) if op == 0x1 || op == 0x2 => {
                    if let Ok(txt) = String::from_utf8(payload) {
                        apply_control(&reader_ctrl, &txt);
                    }
                }
                Ok((op, _)) if op == 0x8 => {
                    let _ = stop_tx.send(());
                    break;
                }
                Ok((op, _)) if op == 0x9 => {
                    let _ = ws::send_frame(&mut s, 0xA, &[]);
                }
                _ => {
                    let _ = stop_tx.send(());
                    break;
                }
            }
        }
    });

    let mut stream = stream;
    let frame_interval = Duration::from_millis(33);
    let mut last = Instant::now();
    loop {
        if stop_rx.try_recv().is_ok() {
            break;
        }
        // 每帧：取控制、按需重建、步进、渲染、发送
        let c = {
            let mut g = ctrl.lock().unwrap();
            if g.dirty {
                driver.rebuild(&g);
                g.dirty = false;
            }
            g.clone()
        };
        if let Some((inp, tele)) = driver.advance(&c) {
            let pixels = fly_sim_core::render::render_frame(FRAME_W, FRAME_H, &inp);
            // 二进制帧：w(4) + h(4) + RGBA 字节（u32 小端即 B,G,R,A）
            let mut bin = Vec::with_capacity(8 + pixels.len() * 4);
            bin.extend_from_slice(&FRAME_W.to_le_bytes());
            bin.extend_from_slice(&FRAME_H.to_le_bytes());
            let bytes: &[u8] = bytemuck_pixels(&pixels);
            bin.extend_from_slice(bytes);
            let _ = ws::send_frame(&mut stream, 0x2, &bin);
            let _ = ws::send_frame(&mut stream, 0x1, tele.as_bytes());
        }
        // 节流到 ~30fps 显示节奏
        let elapsed = last.elapsed();
        if elapsed < frame_interval {
            thread::sleep(frame_interval - elapsed);
        }
        last = Instant::now();
    }
}

/// 把 `Vec<u32>` 当作 `&[u8]`（小端 RGBA）零拷贝视图。
fn bytemuck_pixels(pixels: &[u32]) -> &[u8] {
    let len = pixels.len() * 4;
    unsafe { std::slice::from_raw_parts(pixels.as_ptr() as *const u8, len) }
}

fn apply_control(ctrl: &Arc<Mutex<ControlState>>, txt: &str) {
    // 极简 JSON 解析（只认我们发的扁平字段），避免引入 serde。
    let mut g = ctrl.lock().unwrap();
    let mut changed = false;
    if let Some(v) = json_str(txt, "scenario") {
        if v != g.scenario {
            g.scenario = v.to_string();
            changed = true;
        }
    }
    if let Some(v) = json_str(txt, "controller") {
        if v != g.controller {
            g.controller = v.to_string();
            changed = true;
        }
    }
    if let Some(v) = json_num(txt, "wind") {
        if (v - g.wind).abs() > 1e-6 {
            g.wind = v;
            changed = true;
        }
    }
    if let Some(v) = json_num(txt, "cam_yaw") {
        g.cam_yaw = v as f32;
    }
    if let Some(v) = json_num(txt, "cam_pitch") {
        g.cam_pitch = (v as f32).clamp(-1.5, 1.5);
    }
    if let Some(v) = json_num(txt, "cam_distance") {
        g.cam_distance = (v as f32).clamp(2.0, 200.0);
    }
    // fail_motor: 整数或 null
    if let Some(v) = json_int(txt, "fail_motor") {
        if g.fail_motor != Some(v as u8) {
            g.fail_motor = Some(v as u8);
            g.degrade = None;
            changed = true;
        }
    } else if txt.contains("\"fail_motor\":null") {
        if g.fail_motor.is_some() {
            g.fail_motor = None;
            changed = true;
        }
    }
    // degrade: [m, e] 或 null
    if let Some((m, e)) = json_degrade(txt) {
        if g.degrade != Some((m, e)) {
            g.degrade = Some((m, e));
            g.fail_motor = None;
            changed = true;
        }
    } else if txt.contains("\"degrade\":null") {
        if g.degrade.is_some() {
            g.degrade = None;
            changed = true;
        }
    }
    if changed {
        g.dirty = true;
    }
}

/// 取 JSON 字符串字段（格式 "key":"value"）。
fn json_str<'a>(s: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{}\":", key);
    let idx = s.find(&pat)? + pat.len();
    let rest = &s[idx..];
    let start = rest.find('"')? + 1;
    let end = rest[start..].find('"')? + start;
    Some(&rest[start..end])
}

/// 取 JSON 数值字段（格式 "key":123.4）。
fn json_num(s: &str, key: &str) -> Option<f64> {
    let pat = format!("\"{}\":", key);
    let idx = s.find(&pat)? + pat.len();
    let rest = &s[idx..];
    let end = rest.find(|c: char| c == ',' || c == '}' || c == ' ' || c == '\n')?;
    rest[..end].trim().parse::<f64>().ok()
}

/// 取 JSON 整数字段。
fn json_int(s: &str, key: &str) -> Option<i64> {
    let pat = format!("\"{}\":", key);
    let idx = s.find(&pat)? + pat.len();
    let rest = &s[idx..];
    let end = rest.find(|c: char| c == ',' || c == '}' || c == ' ' || c == '\n')?;
    rest[..end].trim().parse::<i64>().ok()
}

/// 取 JSON 退化字段 "degrade":[m,e]。
fn json_degrade(s: &str) -> Option<(usize, f32)> {
    let pat = "\"degrade\":[";
    let idx = s.find(pat)? + pat.len();
    let rest = &s[idx..];
    let end = rest.find(']')?;
    let nums: Vec<f64> = rest[..end]
        .split(',')
        .filter_map(|x| x.trim().parse::<f64>().ok())
        .collect();
    if nums.len() == 2 {
        Some((nums[0] as usize, nums[1] as f32))
    } else {
        None
    }
}

fn serve_static(stream: &mut TcpStream, path: &str) -> std::io::Result<()> {
    // 防目录穿越
    let clean = path.trim_start_matches('/');
    let clean = if clean.is_empty() { "index.html" } else { clean };
    let clean = clean.split("?").next().unwrap_or("index.html");
    if clean.contains("..") {
        return write_404(stream);
    }
    // 定位 web/ 目录：优先「当前工作目录」（项目根，cargo 运行时的位置），
    // 回退到「可执行文件所在目录的上两级」（target/debug -> 项目根），
    // 再回退到「可执行文件所在目录」。无论如何都能找到 web/。
    let cwd = std::env::current_dir().unwrap_or_default();
    let exe_parent = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.to_path_buf()));
    let candidates: Vec<std::path::PathBuf> = vec![
        cwd.join("web"),
        exe_parent
            .as_ref()
            .and_then(|d| d.parent())
            .map(|gp| gp.join("web"))
            .unwrap_or_default(),
        exe_parent.clone().unwrap_or_default().join("web"),
    ];
    let base = candidates
        .into_iter()
        .find(|p| p.exists() && p.is_dir())
        .unwrap_or_else(|| cwd.join("web"));
    let file = base.join(clean);
    if !file.exists() || !file.is_file() {
        return write_404(stream);
    }
    let data = std::fs::read(&file)?;
    let mime = match file.extension().and_then(|e| e.to_str()) {
        Some("html") => "text/html; charset=utf-8",
        Some("js") => "application/javascript; charset=utf-8",
        Some("css") => "text/css; charset=utf-8",
        Some("json") => "application/json; charset=utf-8",
        _ => "application/octet-stream",
    };
    let resp = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: {}\r\nContent-Length: {}\r\n\r\n",
        mime,
        data.len()
    );
    stream.write_all(resp.as_bytes())?;
    stream.write_all(&data)?;
    Ok(())
}

fn write_404(stream: &mut TcpStream) -> std::io::Result<()> {
    let body = b"404 Not Found";
    stream.write_all(b"HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nContent-Length: 13\r\n\r\n")?;
    stream.write_all(body)?;
    Ok(())
}

fn handle_http(stream: &mut TcpStream) {
    let mut buf = [0u8; 4096];
    let n = match stream.read(&mut buf) {
        Ok(n) if n > 0 => n,
        _ => return,
    };
    let req = String::from_utf8_lossy(&buf[..n]);
    let path = req
        .lines()
        .next()
        .and_then(|l| l.split_whitespace().nth(1))
        .unwrap_or("/");
    if path.starts_with("/ws") {
        // 升级为 WebSocket：handshake 解析已读取的请求字节并回 101。
        if ws::handshake(stream, &buf[..n]).is_err() {
            return;
        }
        let ctrl = Arc::new(Mutex::new(ControlState::default()));
        handle_ws(stream.try_clone().expect("clone 失败"), ctrl);
    } else {
        let _ = serve_static(stream, path);
    }
}

fn main() {
    let listener = TcpListener::bind(("127.0.0.1", PORT)).expect("无法绑定端口");
    println!("[server] Fly Simulator Web 后端已启动: http://127.0.0.1:{}/", PORT);
    println!("[server] 控制: 场景(hover/wind/degraded/avoidance) 控制律(pid/lqr/indi) 故障(fail_motor/degrade) 风(wind) 相机(cam_*)");

    // 预热：探测静态目录是否存在
    let base = std::env::current_dir().unwrap_or_default().join("web");
    if !base.exists() {
        println!("[server][警告] web/ 目录不存在，静态文件将无法服务");
    }

    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                // 每条连接独立线程（调试用，localhost 单客户端足够）。
                thread::spawn(move || handle_http(&mut { s }));
            }
            Err(_) => continue,
        }
    }
}

#[cfg(not(feature = "phy"))]
fn main() {
    eprintln!("[server] 此 Web 后端依赖真实物理引擎渲染（phy feature）。");
    eprintln!("[server] 请以 `cargo run -p fly-sim-server --features phy` 构建运行。");
    std::process::exit(2);
}
