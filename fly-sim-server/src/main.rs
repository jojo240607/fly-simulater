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

mod hil_link;
mod ws;

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use fly_sim_core::controller::{hover_setpoint, ControllerKind};
use fly_sim_core::physics::{ContactModel, DynamicObstacle, Obstacle, PhySdkWorld};
use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig, SensorModel};
use fly_sim_core::sim::SimLoop;
use fly_sim_core::wind::{WindConfig, WindField};
use fly_sim_core::render::{RenderInput, RenderObstacle, RenderRay, RenderWind};
use flyctrl_core::comm::mavlink;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample};

use crate::hil_link::{dbg_report_f64, HilLink, LinkState};

const PORT: u16 = 8080;
const FRAME_W: u32 = 720;
const FRAME_H: u32 = 540;
const STEPS_PER_FRAME: u64 = 8; // 每显示帧推进的物理步（dt=4ms → ~32ms/帧 ≈ 31fps 仿真时钟）
/// HIL 导航注入节流：HIL_GPS/SET_POSITION 每 N 物理步发一次（8 步 = 32ms ≈ 31Hz）。
/// EKF 位置观测（GPS/气压）需求远低于 IMU 的 4ms 节拍（模拟器 non-HIL GPS 仅 20Hz），
/// 而每条消息经 send_chunked 分块睡眠 ~2ms×块数。若 3 条消息全量每步发送，每步注入
/// ≥8ms，远超飞控 4ms 控制周期 → 飞控多数周期无新 IMU 回退 SimImu（水平重力+零角速度）
/// → EKF 位置预测爆炸、位置环指令大倾角 → 姿态发散（实测 1s 仿真耗时 ~9s、t=2s 后
/// R/P≈±100°）。节流后每步仅 1 条 HIL_SENSOR（2 块 1 次睡眠 ≈ 2-3ms），节奏跟上飞控。
const HIL_NAV_EVERY: u64 = 8;
const DT: f64 = 0.004;
const DEGRADE_STABILIZE_STEPS: u64 = 1000; // 退化场景先稳态 4s
/// HIL 起飞台释放阈值（4 电机归一化指令总和）：ARM 后 MCU 需时间完成模式切换/EKF
/// 初始化才输出推力，期间电机≈0。若立即推进物理，机体从 5m 高自由落体 ~10m 撞地，
/// 接触模型产生 ~800 m/s² 加速度尖峰注入 HIL_SENSOR → EKF 状态 NaN（实测）。
/// 小于该阈值视为"未产生推力"，保持机体静止于初始悬停点；达到后释放物理。
const HIL_HOLD_THRUST: f32 = 0.05;

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
    sensor_noise: bool, // realistic 传感器噪声（SIL 用 SensorConfig::realistic，HIL 注入前叠加）
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
            sensor_noise: false,
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
    // HIL：USB 链路（场景 "hil" 时驱动），连接错误回退提示。
    hil_probe: hil_link::CdcProbe, // 后台 USB-CDC 探测，主循环读取缓存，永不阻塞
    hil: Option<HilLink>,
    hil_err: Option<String>,
    hil_state: LinkState,
    hil_retry_at: Option<Instant>, // 连接失败后的重试节流
    hil_armed: bool,   // 已向 MCU 发送 ARM
    hil_time_us: u64,  // HIL 仿真时间戳（单调递增，注入用）
    hil_next_log_us: u64, // 下一次周期状态日志时间戳（us）
    hil_step: u64,    // HIL 物理步计数（导航注入节流用）
    hil_hold: bool,   // HIL 起飞台保持：MCU 产生推力前静止于初始悬停点（防自由落体撞地尖峰）
    hil_noise: Option<SensorModel>, // HIL realistic 噪声：注入 HIL_SENSOR 前对真值叠加消费级噪声
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
            hil_probe: hil_link::CdcProbe::start(),
            hil: None,
            hil_err: None,
            hil_state: LinkState::Connecting,
            hil_retry_at: None,
            hil_armed: false,
            hil_time_us: 0,
            hil_next_log_us: 1_000_000,
            hil_step: 0,
            hil_hold: true,
            hil_noise: None,
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
        // 传感器噪声开关：SIL 用 SensorConfig 注入（SimLoop 内部），HIL 用 SensorModel
        // 在注入 HIL_SENSOR 前对真值叠加（与 SIL realistic 同一套噪声模型/参数/seed）。
        self.sensor_cfg = if c.sensor_noise {
            SensorConfig::realistic()
        } else {
            SensorConfig::default()
        };
        self.hil_noise = if c.sensor_noise {
            Some(SensorModel::new(SensorConfig::realistic(), DT))
        } else {
            None
        };
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

        let inp = self.render_input(&st, &cmd, c);

        let tele = telemetry_json(&st, &cmd, c, self.phase_steps, diverged);
        if diverged {
            // 退化发散后重置，保持连续动画
            self.rebuild(c);
        }
        Some((inp, tele))
    }

    /// 构造渲染输入：渲染世界 = 引擎世界（同为 Y-up，上=+Y）。直接用引擎位姿。
    /// 引擎悬停时机体 +Z（推力轴）指向 +Y，即旋翼盘水平 → 渲染必然水平。
    /// 不做任何 NED 或 z 镜像变换（镜像会把水平机体翻成侧躺）。
    fn render_input(
        &mut self,
        st: &flyctrl_core::vehicle::VehicleState,
        cmd: &ActuatorCmd,
        c: &ControlState,
    ) -> RenderInput {
        let sim = self.sim.as_mut().unwrap();
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

        RenderInput {
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
        }
    }

    /// 尝试建立 HIL 链路（USB-CDC 探测 + 打开 + 等心跳）。
    fn try_connect_hil(&mut self) -> Result<(), String> {
        // 探测 USB-CDC 端口（后台线程缓存，主循环不阻塞；复位后枚举需数秒，失败则下帧重试）。
        let name = self
            .hil_probe
            .port_name()
            .ok_or("未找到 HIL USB-CDC(0483:5740)：复位后等待数秒枚举，或用 `taskkill /F /T` 释放占用")?;
        match HilLink::open(&name, Duration::from_secs(5)) {
            Ok(link) => {
                self.hil = Some(link);
                self.hil_state = LinkState::WaitingHeartbeat;
                self.hil_armed = false;
                self.hil_err = None;
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// HIL 模式单帧推进（场景 "hil"）：经 USB 与真实飞控 MCU 闭环。
    ///
    /// 闭环数据流（渲染/遥测分家）：
    ///   - 上行：把物理引擎 IMU/位置/速度/偏航真值注入 MCU（HIL_SENSOR + SET_POSITION）；
    ///   - 下行：MCU 回传 HIL_ACTUATOR_CONTROLS 四电机指令 → 驱动物理（`step_hil`），
    ///     同时解析 ATTITUDE / LOCAL_POSITION_NED 作为 MCU 估计供遥测显示；
    ///   - 渲染用物理真值，遥测优先用 MCU 估计、未到回退真值。
    fn advance_hil(&mut self, c: &ControlState) -> Option<(Option<RenderInput>, String)> {
        // 临时诊断：确认 advance_hil 是否被持续调用。
        {
            use std::sync::atomic::{AtomicU64, Ordering};
            static CALL: AtomicU64 = AtomicU64::new(0);
            let k = CALL.fetch_add(1, Ordering::Relaxed);
            if k < 3 || k % 120 == 0 {
                eprintln!("[hil][dbg] advance_hil #{k} scenario={}", c.scenario);
            }
        }
        // 连接/重连：USB-CDC 探测失败或打开失败都转入错误态，周期性重试。
        if self.hil.is_none() {
            if let Some(t) = self.hil_retry_at {
                if t > Instant::now() {
                    // 重试节流中：仅上报遥测状态，不推进。
                    return Some((
                        None,
                        hil_telemetry_json(None, None, None, self.hil_state, self.hil_err.as_deref()),
                    ));
                }
            }
            match self.try_connect_hil() {
                Ok(()) => eprintln!("[hil] USB-CDC 已打开，等待飞控心跳…"),
                Err(e) => {
                    eprintln!("[hil] 连接失败: {e}");
                    self.hil_err = Some(e);
                    self.hil_state = LinkState::Error("连接失败");
                    self.hil_retry_at = Some(Instant::now() + Duration::from_secs(2));
                    return Some((
                        None,
                        hil_telemetry_json(None, None, None, self.hil_state, self.hil_err.as_deref()),
                    ));
                }
            }
        }

        let link = self.hil.as_mut()?;
        link.poll(); // 排空下行：执行器指令 / MCU 估计 / 心跳

        // 链路离线（>3s 无下行）：丢弃重建，回到探测态。
        if link.idle() > Duration::from_secs(3) {
            self.hil = None;
            self.hil_state = LinkState::Connecting;
            self.hil_retry_at = Some(Instant::now() + Duration::from_secs(1));
            return Some((
                None,
                hil_telemetry_json(None, None, None, self.hil_state, self.hil_err.as_deref()),
            ));
        }

        // 等待 HIL 心跳（首次进入），超时判链路问题。
        if !link.is_hil_ready() {
            self.hil_state = LinkState::WaitingHeartbeat;
            // 临时诊断：每 ~0.5s 打印链路空闲时长与收包缓冲（确认下行是否在流）。
            use std::sync::atomic::{AtomicU64, Ordering};
            static DBG: AtomicU64 = AtomicU64::new(0);
            let n = DBG.fetch_add(1, Ordering::Relaxed);
            if n % 15 == 0 {
                eprintln!(
                    "[hil][dbg] waiting idle={:.1}s rx_buf={}",
                    link.idle().as_secs_f64(),
                    link.rx_buf_len(),
                );
            }
            return Some((
                None,
                hil_telemetry_json(None, None, None, self.hil_state, self.hil_err.as_deref()),
            ));
        }
        self.hil_state = LinkState::Running;

        // 首次进入闭环：向 MCU 发送 ARM。失败忽略（重试节流内自然重发）。
        if !self.hil_armed {
            eprintln!("[hil] 链路建立（HIL 心跳确认），发送 ARM，闭环开始");
            let _ = link.send_arm(true);
            self.hil_armed = true;
        }

        // 下行执行器指令 → 推进物理（HIL 控制律在 MCU，PC 只施加指令 + 采样 IMU 真值）。
        // 关键同步：飞控控制周期=4ms，必须【每物理步】读回最新执行器指令并注入一帧 IMU，
        // 否则飞控 8 个控制周期只有 1 个能拿到新 IMU 真值，其余回退 SimImu（水平重力+零角速度）
        // 会把 EKF 姿态持续拉向水平，导致姿态发散（实测 t=2s R/P≈±100°）。
        let mut st = None;
        let mut cmd = ActuatorCmd { motor: [0.0; 4] };
        // 帧级计时（定位 5.1x 慢放隐藏开销）：poll / 物理步 / 注入 各自耗时累加。
        let dg_frame0 = Instant::now();
        let mut dg_phy_us: u64 = 0;
        let mut dg_inj_us: u64 = 0;
        let mut dg_nav: u64 = 0;
        let mut dg_nav_us: u64 = 0;
        for _ in 0..STEPS_PER_FRAME {
            let t0 = Instant::now();
            // 每步读回最近一帧执行器指令（减少 8 步共用同一指令的滞后）。
            cmd = ActuatorCmd { motor: link.actuator() };
            // 【起飞台保持】ARM 后 MCU 需时间完成上电/模式切换/EKF 初始化才输出推力，
            // 期间电机指令≈0。若立即推进物理，机体从 5m 高自由落体 ~10m 撞地，接触模型
            // 产生 ~800 m/s² 加速度尖峰注入 HIL_SENSOR → EKF 状态 NaN（实测）。故在 MCU
            // 产生有效推力前，保持机体静止于初始悬停点（只注入悬停 IMU），达到阈值后释放。
            // （thrust 为 NaN 也保持：MCU 异常输出时宁可停于起飞台，也不自由落体坠地。）
            let thrust = cmd.motor[0] + cmd.motor[1] + cmd.motor[2] + cmd.motor[3];
            let held = self.hil_hold && (thrust.is_nan() || thrust < HIL_HOLD_THRUST);
            if held {
                // 保持：不推进物理，用初始世界状态（NED 0,0,-5 静止水平）作注入真值。
                if let Some(sim) = self.sim.as_ref() {
                    st = Some(sim.snapshot().0);
                }
            } else {
                self.hil_hold = false;
                if let Some(sim) = self.sim.as_mut() {
                    st = Some(sim.step_hil(&cmd));
                }
            }
            self.phase_steps += 1;
            dg_phy_us += t0.elapsed().as_micros() as u64;
            // 上行注入：每物理步（4ms）注入一帧 IMU 真值 + 偏航 + 气压高度（-D）。
            // HIL_GPS/SET_POSITION 节流（HIL_NAV_EVERY 步一次 ≈ 31Hz，见常量注释），
            // 避免 send_chunked 分块睡眠拖慢注入节奏导致飞控缺 IMU 回退 SimImu 发散。
            let mut nav = self.hil_step % HIL_NAV_EVERY == 0;
            self.hil_step += 1;
            let t1 = Instant::now();
            if let Some(sim) = self.sim.as_ref() {
                let stp = st.as_ref().unwrap();
                // 保持期间物理未推进、sim.last_imu() 仍是初始占位（accel=0），须显式给
                // 悬停真值（FRD 静止比力 (0,0,-9.81)、角速度 0）——与 sim 悬停输出同约定，
                // 供飞控 EKF 在校验台上完成陀螺零偏收敛/初始化，不触发自由落体。
                let imu_true = if held {
                    ImuSample {
                        accel: [
                            MeterPerSecondSquared(0.0),
                            MeterPerSecondSquared(0.0),
                            MeterPerSecondSquared(-9.81),
                        ],
                        gyro: [RadianPerSecond(0.0); 3],
                    }
                } else {
                    sim.last_imu()
                };
                // 【HIL realistic 噪声】与 SIL realistic 同一套噪声模型（SensorModel +
                // SensorConfig::realistic，同参数同 seed）：IMU（bias/白噪/随机游走/BI/振动）
                // 每步叠加，GPS（延迟 0.15s + 20Hz 降频 + 位置/速度噪声），气压（白噪 + 慢漂移）。
                // 使真实 MCU 经历与 SIL realistic 一致的消费级传感器噪声 → 对比 HIL vs SIL 表现。
                // GPS 注入触发：noise 模式跟随 SensorModel 的 20Hz GPS 输出（与 SIL realistic
                // 同节奏）；否则保持 HIL_NAV_EVERY 节流（31Hz，零噪声基线，已验证稳定）。
                let pos = [stp.pos[0].0, stp.pos[1].0, stp.pos[2].0];
                let vel = [stp.vel[0].0, stp.vel[1].0, stp.vel[2].0];
                let (imu, gps_pos, gps_vel, baro_alt) = match &mut self.hil_noise {
                    Some(nm) => {
                        let (imu_n, gps_n) = nm.process(
                            DT,
                            [imu_true.accel[0].0, imu_true.accel[1].0, imu_true.accel[2].0],
                            [imu_true.gyro[0].0, imu_true.gyro[1].0, imu_true.gyro[2].0],
                            pos, vel,
                        );
                        let baro = nm.process_baro(DT, -stp.pos[2].0 as f64);
                        let (gp, gv) = gps_n
                            .map(|g| {
                                let gv = g.vel.unwrap_or([
                                    MeterPerSecond(vel[0]),
                                    MeterPerSecond(vel[1]),
                                    MeterPerSecond(vel[2]),
                                ]);
                                (
                                    [g.pos[0].0, g.pos[1].0, g.pos[2].0],
                                    [gv[0].0, gv[1].0, gv[2].0],
                                )
                            })
                            .unwrap_or((pos, vel));
                        nav = gps_n.is_some(); // 20Hz GPS 观测（与 SIL realistic 同节奏）
                        (imu_n, gp, gv, baro.altitude as f32)
                    }
                    None => (imu_true, pos, vel, -stp.pos[2].0 as f32),
                };
                let q = stp.att;
                let yaw = (2.0 * (q.w * q.z + q.x * q.y))
                    .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                let _ = link.inject(
                    self.hil_time_us,
                    &imu,
                    yaw as f32,
                    baro_alt, // 气压高度 = -D（noise 模式为带噪 + 慢漂移值）
                    gps_pos,  // GPS 位置（noise 模式为带噪 + 延迟真值）
                    gps_vel,  // GPS 速度
                    nav,
                );
            }
            let e = t1.elapsed().as_micros() as u64;
            dg_inj_us += e;
            if nav {
                dg_nav += 1;
                dg_nav_us += e;
            }
            self.hil_time_us += (DT * 1e6) as u64;
        }
        let st = st?;
        // 帧级计时日志（每 ~30 帧 ≈ 1s 一条）：区分 poll / 物理 / 注入 / 导航步 的耗时占比。
        use std::sync::atomic::{AtomicU64, Ordering};
        static FDBG: AtomicU64 = AtomicU64::new(0);
        static PWR: AtomicU64 = AtomicU64::new(0);
        static PSL: AtomicU64 = AtomicU64::new(0);
        static PCH: AtomicU64 = AtomicU64::new(0);
        if FDBG.fetch_add(1, Ordering::Relaxed) % 30 == 0 {
            let dt0 = dg_frame0.elapsed().as_micros() as u64;
            let poll_us = {
                let t0 = Instant::now();
                link.poll();
                t0.elapsed().as_micros() as u64
            };
            // send_chunked 累计写/睡耗时与块数的【本窗口增量】→ 每帧均值 = 增量/30。
            let (w_us, s_us, chunks) = link.dbg_tx();
            let dw = w_us.saturating_sub(PWR.swap(w_us, Ordering::Relaxed));
            let ds = s_us.saturating_sub(PSL.swap(s_us, Ordering::Relaxed));
            let dc = chunks.saturating_sub(PCH.swap(chunks, Ordering::Relaxed));
            eprintln!(
                "[hil][timing] frame={}ms poll={}ms phy={}ms inj={}ms (nav{}/{}ms) | tx +{}/30f write={:.1}ms/f sleep={:.1}ms/f",
                dt0 / 1000, poll_us / 1000, dg_phy_us / 1000, dg_inj_us / 1000,
                dg_nav, dg_nav_us / 1000,
                dc, dw as f64 / 30e3, ds as f64 / 30e3,
            );
        }

        // 周期状态日志（每 ~1s 一条），确认物理被真实飞控电机指令驱动、悬停收敛。
        if self.hil_time_us >= self.hil_next_log_us {
            self.hil_next_log_us += 1_000_000;
            let q = st.att;
            let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
            let pitch = (2.0 * (q.w * q.y - q.z * q.x)).asin();
            let yaw = (2.0 * (q.w * q.z + q.x * q.y))
                .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            eprintln!(
                "[hil] t={:>5.1}s alt={:>6.2}m vD={:>+5.2} R/P={:>+5.1}/{:>+5.1}° yaw={:>+6.1}° motor={:.2}/{:.2}/{:.2}/{:.2}",
                self.hil_time_us as f64 / 1e6,
                -st.pos[2].0, st.vel[2].0,
                roll.to_degrees(), pitch.to_degrees(),
                yaw.to_degrees(),
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
            );
            // 并入 NDJSON 落盘（eprintln 经重定向易丢，NDJSON 始终可靠）：
            // 追加 MCU 估计（mcu_att/mcu_local）对照真值，定位发散源（估计误差 vs 物理失控）。
            let (ma, ml) = (link.mcu_att(), link.mcu_local());
            dbg_report_f64(
                "C",
                "main.rs:advance_hil",
                "hil hover",
                &[
                    ("t_s", self.hil_time_us as f64 / 1e6),
                    ("alt_m", -st.pos[2].0 as f64),
                    ("vD_ms", st.vel[2].0 as f64),
                    ("roll_deg", roll.to_degrees() as f64),
                    ("pitch_deg", pitch.to_degrees() as f64),
                    ("yaw_deg", yaw.to_degrees() as f64),
                    ("mroll_deg", ma.map(|a| a.roll.to_degrees() as f64).unwrap_or(f64::NAN)),
                    ("mpitch_deg", ma.map(|a| a.pitch.to_degrees() as f64).unwrap_or(f64::NAN)),
                    ("myaw_deg", ma.map(|a| a.yaw.to_degrees() as f64).unwrap_or(f64::NAN)),
                    ("malt_m", ml.map(|l| -l.z as f64).unwrap_or(f64::NAN)),
                    ("m0", cmd.motor[0] as f64),
                    ("m1", cmd.motor[1] as f64),
                    ("m2", cmd.motor[2] as f64),
                    ("m3", cmd.motor[3] as f64),
                ],
            );
        }

        // 上行注入已在每物理步循环内完成（见上），此处不再重复。

        // 渲染用真值，遥测用 MCU 估计（未到回退真值）。
        // 先取出 MCU 估计（`link` 对 self 的借用到此结束），再借用 self 渲染。
        let (mcu_att, mcu_local) = (link.mcu_att(), link.mcu_local());
        let inp = self.render_input(&st, &cmd, c);
        let tele = hil_telemetry_json(
            Some((&st, &cmd)),
            mcu_att,
            mcu_local,
            self.hil_state,
            self.hil_err.as_deref(),
        );
        Some((Some(inp), tele))
    }
}

/// HIL 遥测 JSON：连接状态 + MCU 估计（ATTITUDE/LOCAL_POSITION_NED）+ 物理真值。
///
/// 渲染用真值，遥测数字优先展示 MCU 估计（真实飞控的"所见"），真值作对照；
/// `truth` 为 `None` 时表示尚未闭环（连接中），仅输出状态。
fn hil_telemetry_json(
    truth: Option<(&flyctrl_core::vehicle::VehicleState, &ActuatorCmd)>,
    mcu_att: Option<mavlink::Attitude>,
    mcu_local: Option<mavlink::LocalPositionNed>,
    state: LinkState,
    err: Option<&str>,
) -> String {
    use std::fmt::Write as _;
    let mut s = String::from(
        "{\"scenario\":\"hil\",\"hil\":{\"state\":\"",
    );
    let state_str = match state {
        LinkState::Connecting => "connecting",
        LinkState::WaitingHeartbeat => "waiting_heartbeat",
        LinkState::Running => "running",
        LinkState::Error(_) => "error",
    };
    let _ = write!(s, "{state_str}\"");
    if let Some(msg) = err {
        let _ = write!(s, ",\"msg\":\"{}\"", msg.replace('"', "'"));
    }

    // MCU 估计（姿态 + NED 位置/速度）。
    if let (Some(a), Some(l)) = (mcu_att, mcu_local) {
        let alt = -l.z; // NED d 向下，高度 = -d
        let speed = (l.vx * l.vx + l.vy * l.vy + l.vz * l.vz).sqrt();
        let _ = write!(
            s,
            ",\"mcu\":{{\"roll\":{:.3},\"pitch\":{:.3},\"yaw\":{:.3},\"alt\":{:.3},\"speed\":{:.3}}}",
            a.roll, a.pitch, a.yaw, alt, speed
        );
    }

    // 物理真值（对照）。
    if let Some((st, cmd)) = truth {
        let alt = -st.pos[2].0;
        let horiz = (st.pos[0].0 * st.pos[0].0 + st.pos[1].0 * st.pos[1].0).sqrt();
        let speed =
            (st.vel[0].0 * st.vel[0].0 + st.vel[1].0 * st.vel[1].0 + st.vel[2].0 * st.vel[2].0)
                .sqrt();
        let q = st.att;
        let roll = (2.0 * (q.w * q.x + q.y * q.z))
            .atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        let pitch = (2.0 * (q.w * q.y - q.z * q.x)).asin().clamp(-1.57, 1.57);
        let yaw = (2.0 * (q.w * q.z + q.x * q.y))
            .atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        let _ = write!(
            s,
            ",\"truth\":{{\"alt\":{:.3},\"horiz\":{:.3},\"speed\":{:.3},\"roll\":{:.3},\"pitch\":{:.3},\"yaw\":{:.3}}}",
            alt, horiz, speed, roll, pitch, yaw
        );
        let _ = write!(
            s,
            ",\"m\":[{},{},{},{}]",
            cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3]
        );
    } else {
        let _ = write!(s, ",\"m\":[0,0,0,0]");
    }
    let _ = write!(s, ",\"steps\":0,\"diverged\":false");
    s.push('}');
    s
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
    use std::sync::atomic::{AtomicU32, Ordering as AOrder};
    static CLIENTS: AtomicU32 = AtomicU32::new(0);
    let n = CLIENTS.fetch_add(1, AOrder::Relaxed) + 1;
    eprintln!("[ws][dbg] client connected (#{n} active)");
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
                    eprintln!("[ws][dbg] reader: close frame");
                    let _ = stop_tx.send(());
                    break;
                }
                Ok((op, _)) if op == 0x9 => {
                    let _ = ws::send_frame(&mut s, 0xA, &[]);
                }
                // pong(0xA)/continuation(0x0) 等非致命帧：忽略并继续。
                Ok((op, _)) if op == 0xA || op == 0x0 => {}
                r => {
                    eprintln!("[ws][dbg] reader break: {:?}", r.as_ref().map(|(op, _)| *op));
                    let _ = stop_tx.send(());
                    break;
                }
            }
        }
    });

    let mut stream = stream;
    let frame_interval = Duration::from_millis(33);
    let mut last = Instant::now();
    let mut iter: u64 = 0;
    loop {
        iter += 1;
        if stop_rx.try_recv().is_ok() {
            eprintln!("[ws][dbg] loop break at iter {iter}");
            break;
        }
        if iter % 300 == 0 {
            eprintln!("[ws][dbg] loop alive iter {iter}");
        }
        // 每帧：取控制、按需重建、步进、渲染、发送。
        // HIL 场景走 USB 闭环（advance_hil），其余走 PC 控制律闭环（advance）。
        let c = {
            let mut g = ctrl.lock().unwrap();
            if g.dirty {
                driver.rebuild(&g);
                g.dirty = false;
            }
            g.clone()
        };
        let out = if c.scenario == "hil" {
            driver.advance_hil(&c)
        } else {
            driver.advance(&c).map(|(inp, tele)| (Some(inp), tele))
        };
        if iter % 30 == 0 {
            eprintln!(
                "[ws][dbg] iter {iter} scenario={} out={}",
                c.scenario,
                out.as_ref().map(|(i, _)| if i.is_some() { "render" } else { "status" }).unwrap_or("none")
            );
        }
        if let Some((inp, tele)) = out {
            if let Some(inp) = inp {
                let pixels = fly_sim_core::render::render_frame(FRAME_W, FRAME_H, &inp);
                // 二进制帧：w(4) + h(4) + RGBA 字节（u32 小端即 B,G,R,A）
                let mut bin = Vec::with_capacity(8 + pixels.len() * 4);
                bin.extend_from_slice(&FRAME_W.to_le_bytes());
                bin.extend_from_slice(&FRAME_H.to_le_bytes());
                let bytes: &[u8] = bytemuck_pixels(&pixels);
                bin.extend_from_slice(bytes);
                let _ = ws::send_frame(&mut stream, 0x2, &bin);
            }
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
    // sensor_noise: true/false —— realistic 传感器噪声（SIL/HIL 一致性验证用）。
    if let Some(v) = json_bool(txt, "sensor_noise") {
        if v != g.sensor_noise {
            g.sensor_noise = v;
            changed = true;
        }
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

/// 取 JSON 布尔字段（格式 "key":true/false）。
fn json_bool(s: &str, key: &str) -> Option<bool> {
    let pat = format!("\"{}\":", key);
    let idx = s.find(&pat)? + pat.len();
    let rest = &s[idx..];
    let end = rest.find(|c: char| c == ',' || c == '}' || c == ' ' || c == '\n')?;
    match rest[..end].trim() {
        "true" => Some(true),
        "false" => Some(false),
        _ => None,
    }
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
    println!("[server] 控制: 场景(hover/wind/degraded/avoidance/hil) 控制律(pid/lqr/indi) 故障(fail_motor/degrade) 风(wind) 相机(cam_*)");

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
