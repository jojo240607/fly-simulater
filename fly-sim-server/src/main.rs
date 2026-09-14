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
use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig, SensorModel};
use fly_sim_core::sim::SimLoop;
use fly_sim_core::wind::{WindConfig, WindField};
use fly_sim_core::render::{RenderInput, RenderObstacle, RenderRay, RenderWind};
use flyctrl_core::comm::mavlink;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample};

use fly_sim_hil::hil_link::{dbg_report_f64, HilLink, LinkState};

// 虚拟 MCU（Unicorn 模拟固件）——场景 "vperiph" 的 HIL 后端。
use mcu_simulater::machine::Machine;
use mcu_simulater::peripheral::vperiph::data_source::FlySimState;
use openh264::encoder::{BitRate, Encoder, EncoderConfig, FrameRate, FrameType, IntraFramePeriod, UsageType};
use openh264::formats::YUVSource;

/// 待编码的 I420 帧（YUV420 平面：Y(w*h) + U(w/2*h/2) + V(w/2*h/2)），
/// 直接包装为 openh264 的 YUVSource。
struct I420Source {
    w: usize,
    h: usize,
    data: Vec<u8>,
}

impl YUVSource for I420Source {
    fn dimensions(&self) -> (usize, usize) {
        (self.w, self.h)
    }
    fn strides(&self) -> (usize, usize, usize) {
        (self.w, self.w / 2, self.w / 2)
    }
    fn y(&self) -> &[u8] {
        &self.data[..self.w * self.h]
    }
    fn u(&self) -> &[u8] {
        &self.data[self.w * self.h..self.w * self.h * 5 / 4]
    }
    fn v(&self) -> &[u8] {
        &self.data[self.w * self.h * 5 / 4..]
    }
}

/// RGBA（小端 u32 = [B,G,R,A] 字节序）→ I420，BT.601 限制范围（16-235）。
fn rgba_to_i420(rgba: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut out = vec![0u8; w * h * 3 / 2];
    let y_off = 0usize;
    let u_off = w * h;
    let v_off = w * h * 5 / 4;
    for y in 0..h {
        for x in 0..w {
            let i = (y * w + x) * 4;
            let (r, g, b) = (rgba[i + 2] as i32, rgba[i + 1] as i32, rgba[i] as i32);
            let yy = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            out[y_off + y * w + x] = yy.clamp(0, 255) as u8;
        }
    }
    // 2×2 块平均出 U/V
    for y in (0..h).step_by(2) {
        for x in (0..w).step_by(2) {
            let mut r = 0i32;
            let mut g = 0i32;
            let mut b = 0i32;
            let mut n = 0i32;
            for dy in 0..2 {
                for dx in 0..2 {
                    let yy = y + dy;
                    let xx = x + dx;
                    if yy < h && xx < w {
                        let i = (yy * w + xx) * 4;
                        r += rgba[i + 2] as i32;
                        g += rgba[i + 1] as i32;
                        b += rgba[i] as i32;
                        n += 1;
                    }
                }
            }
            r /= n;
            g /= n;
            b /= n;
            let u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            let v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
            let uv = (y / 2) * (w / 2) + (x / 2);
            out[u_off + uv] = u.clamp(0, 255) as u8;
            out[v_off + uv] = v.clamp(0, 255) as u8;
        }
    }
    out
}

/// 新建 openh264 编码器（H.264 视频流：vperiph 640×480 @ 1200kbps）。
/// 带宽实测：640×480 动态 8 字场景 ~1.2-1.5Mbps，在公网 2.5Mbps 内且比 PNG(320×240,
/// 2.1Mbps) 分辨率更高带宽更低（帧间压缩）。
/// 新建 openh264 编码器：bitrate=目标码率、max_fps=码控按实际帧率标定、
/// intra_period=关键帧间隔（帧数）。GOP 拉疏（1.5s）可大幅降带宽（IDR 全帧
/// 开销占比高：原 15 帧@40fps=0.375s 一个 IDR，带宽主要吃在这）。
fn new_h264_encoder(bitrate_bps: u32, max_fps: u32, intra_period: u32) -> Result<Encoder, openh264::Error> {
    let cfg = EncoderConfig::new()
        .bitrate(BitRate::from_bps(bitrate_bps))
        .max_frame_rate(FrameRate::from_hz(max_fps as f32))
        .usage_type(UsageType::CameraVideoRealTime)
        // GOP：新客户端 ≤1.5s 等到关键帧出画面（视频流常规值）
        .intra_frame_period(IntraFramePeriod::from_num_frames(intra_period));
    Encoder::with_api_config(openh264::OpenH264API::from_source(), cfg)
}

// vperiph 固件/系统镜像路径（与 mcu_simulater tests/x_hover_env.rs 同一套）。
const VP_SYS: &str = "/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf";
const VP_APP: &str = "/tmp/flyctrl_clean.bin";
// vperiph 固定 GPS 原点（悬停点附近，与 x_vperiph/x_hover_env 同一约定）。
const VP_LAT0: f32 = 31.2304;
const VP_LON0: f32 = 121.4737;
const VP_ALT0: f32 = 4.0;
// TIM 基址（固件 pwm0..3 = TIM3/TIM2/TIM1/TIM4 CH1）——读固件 PWM 推力用。
const VP_TIM3: u64 = 0x4000_0400;
const VP_TIM2: u64 = 0x4000_0000;
const VP_TIM1: u64 = 0x4001_0000;
const VP_TIM4: u64 = 0x4000_0800;
const VP_OFF_CRR1: u64 = 0x34;
const VP_OFF_ARR: u64 = 0x2C;

const PORT: u16 = 8080; // 可用 FLY_SIM_PORT 环境变量覆盖（8080 常被其他服务占用时）
const FRAME_W: u32 = 720;
const FRAME_H: u32 = 540;
const STEPS_PER_FRAME: u64 = 8; // 每显示帧推进的物理步（dt=4ms → ~32ms/帧 ≈ 31fps 仿真时钟）
/// vperiph 帧步数：8 步/帧 = 32ms 仿真。Unicorn 模拟比真机慢 ~10×（每条 ~50ns vs 6ns），
/// 帧率 = 1/(步数×run耗时+固定开销)，8 步在 run(200k) 下 ≈ 7.5fps（比 16 步/帧的 3.8fps
/// 高近一倍，帧率优先——画面连贯），仿真/墙钟 ≈ 0.24（4 倍慢放）。8 字机动周期 16s 仿真
/// ≈ 66s 墙钟，加高幅度摇杆让画面可见飞行（悬停原样会"看起来静止"）。
/// 每帧物理步数。8 步/帧 = 132ms run → 7.5fps（卡）；4 步/帧 = 66ms → ~19fps
/// （更顺，画面移动幅度减半但帧率翻倍，位移速度不变）。固件控制律每 3.3 物理步
/// 一个周期（run 200k=0.3 周期），每步 run 13ms/周期安全（150k 等效 18ms 已发散）。
/// 2 步/帧 = 33ms → 上限测试（编码吞吐决定实际可达 fps）。
/// 每帧物理步数（用户选定 40fps 档）：
/// - 8 步/帧 = 7.5fps（原，卡）
/// - 4 步/帧 = 19fps（流畅，2.5Mbps）
/// - 2 步/帧 = 40fps（人眼感知极限，3.8Mbps）← 当前
/// - 1 步/帧 = 80fps（绝对上限：run(200k)≈12ms 是硬下限，4.9Mbps，CPU 最高）
/// 固件控制律每 3.3 物理步一个周期（run 200k=0.3 周期，每步 run 13ms/周期安全；
/// 150k 等效 18ms 已实测发散），各档均稳定（alt 起伏 0.64m）。
const VP_STEPS_PER_FRAME: u64 = 2;

/// 固件推进频率：每 N 物理步 run 一次。实测每 2 步 run（控制律周期 ~26ms 物理）
/// 超过稳定临界（~15ms，150k/步 等效）→ 姿态发散坠机；必须每步 run（~13ms 周期）。
/// 提速只能靠降低 Unicorn 单步成本（编译优化），此处保持 1。
const VP_RUN_EVERY: u64 = 1;
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

/// 虚拟 MCU：Unicorn 模拟固件 + fly-sim 虚拟外设直通（场景 "vperiph" 的 HIL 后端）。
///
/// 与真实 USB-CDC HIL 的区别仅在"MCU 指令来源"与"传感器注入通路"：
/// - 指令来源：读 Unicorn 内固件 PWM（TIM CCR/ARR）而非 USB 下行；
/// - 注入通路：写 `FlySimState`（attach_flysim 虚拟 I2C/UART）而非 HIL_SENSOR 消息。
/// 闭环骨架（`SimLoop::step_hil` + 起飞台保持 + 渲染/遥测）与 `advance_hil` 共用。
struct VperiphMc {
    machine: Arc<Mutex<Machine>>,
    state: Arc<Mutex<FlySimState>>,
    hold: bool,          // 起飞台保持：固件产生推力前机体静止（防自由落体撞地尖峰）
    steps: u64,          // 固件执行步数（诊断）
    hold_z: Option<f32>, // 定高基准（NED z，向下为正）：起飞稳定后锁定（延迟 2s），与固件 hold_alt 接近
    hold_z_timer: f32,   // 脱离起飞台后的稳定计时（s）
    alt_int: f32,        // 定高外环积分项（抑制静差）
    z_filt: f32,         // 外环输入高度低通滤波（去真值噪声，防油门抖动）
}

impl VperiphMc {
    /// 完整装配：boot → EKF 收敛 → ARM。Unicorn 偶发 `UC_ERR_INSN_INVALID`（M4F 模拟
    /// 瞬时不稳定，x_hover 系实测偶发）→ 内部重建重试，最多 8 次。
    fn boot() -> Result<Self, String> {
        let mut last_err = String::new();
        for attempt in 0..8 {
            match Self::boot_once() {
                Ok(vp) => {
                    eprintln!("[vperiph] 虚拟 MCU boot 完成（尝试 {}）", attempt + 1);
                    return Ok(vp);
                }
                Err(e) => {
                    eprintln!("[vperiph] boot 尝试 {} 失败：{e}", attempt + 1);
                    last_err = e;
                }
            }
        }
        Err(format!("虚拟 MCU boot 连续失败：{last_err}"))
    }

    fn boot_once() -> Result<Self, String> {
        let mut m = Machine::new_m4f().map_err(|e| format!("Machine: {e:?}"))?;
        m.map_stm32f407_layout().map_err(|e| format!("map: {e:?}"))?;
        let state = Arc::new(Mutex::new(FlySimState::default()));
        m.attach_flysim_sensors(state.clone());
        m.attach_flysim_uart_slaves(state.clone());

        // boot 前注入初始真值（静止水平悬停 + 气压 h=0 + GPS 有效，同 x_vperiph）
        {
            let mut st = state.lock().unwrap();
            st.imu_acc = [0.0, 0.0, -9.81];
            st.imu_gyr = [0.0, 0.0, 0.0];
            st.baro_pa = 101_325.0f32;
            st.gps_lat = VP_LAT0;
            st.gps_lon = VP_LON0;
            st.gps_alt = VP_ALT0 + 5.0;
            st.gps_fix = 3.0;
            st.gps_vel = [0.0, 0.0, 0.0];
            st.rc_ch = [1500.0; 16];
        }

        m.load_elf(std::path::Path::new(VP_SYS)).map_err(|e| format!("elf: {e:?}"))?;
        m.load_app_partition(std::path::Path::new(VP_APP)).map_err(|e| format!("app: {e:?}"))?;
        m.reset().map_err(|e| format!("reset: {e:?}"))?;
        for _ in 0..12 {
            m.run(1_000_000).map_err(|e| format!("boot run: {e:?}"))?;
        }
        let m = Arc::new(Mutex::new(m));

        // ARM 前 EKF 高度收敛（同 x_vperiph：等 RC 建立 + EKF 高度拉回原点）
        let mut mm = m.lock().unwrap();
        let mut z = f32::NAN;
        for i in 0..400 {
            mm.run(1_000_000).map_err(|e| format!("settle run: {e:?}"))?;
            z = f32::from_le_bytes(
                mm.cpu.mem_read(0x2000_9074 + 28, 4).map_err(|e| format!("mem: {e:?}"))?
                    .try_into().unwrap(),
            );
            if i % 100 == 0 {
                eprintln!("[vperiph] 收敛推进 i={i} ekf_z={z:.3}");
            }
            if z.abs() < 0.6 {
                break;
            }
        }
        drop(mm);
        eprintln!("[vperiph] EKF 收敛完成 ekf_z={z:.3}");

        // ARM + RC 解锁
        m.lock().unwrap().cpu.mem_write(0x2000_b669, &[1u8]).map_err(|e| format!("arm: {e:?}"))?;
        {
            let mut st = state.lock().unwrap();
            st.rc_ch[4] = 2000.0;
            st.rc_ch[3] = 1500.0;
        }
        Ok(Self { machine: m, state, hold: true, steps: 0, hold_z: None, hold_z_timer: 0.0, alt_int: 0.0, z_filt: 0.0 })
    }

    /// 读 4 路 PWM 的 CCR1/ARR → 归一化推力（m = (duty_us - 1000)/1000）。
    fn read_thrust(&self) -> [f32; 4] {
        let mut m = self.machine.lock().unwrap();
        let tims = [VP_TIM3, VP_TIM2, VP_TIM1, VP_TIM4];
        let mut out = [0f32; 4];
        for (i, &t) in tims.iter().enumerate() {
            let mut rd = |a: u64| -> u32 {
                m.cpu.mem_read(a, 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]])).unwrap_or(0)
            };
            let arr = rd(t + VP_OFF_ARR) as f32;
            let ccr = rd(t + VP_OFF_CRR1) as f32;
            let duty = if arr > 0.0 { ccr / arr } else { 0.0 };
            let us = duty * 2500.0;
            out[i] = ((us - 1000.0) / 1000.0).clamp(0.0, 1.0);
        }
        out
    }

    /// 把 fly-sim 的噪声化传感器读数注入虚拟外设（固件经虚拟 I2C/UART 收到，
    /// 与 SIL 控制律同源——`SimLoop` 的 `SensorConfig` 已叠加噪声/风作用后的真值）。
    fn inject<W: fly_sim_core::physics::RigidBodyWorld>(&self, sim: &SimLoop<W>) {
        let mut st = self.state.lock().unwrap();
        let imu = sim.last_imu();
        st.imu_acc = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        st.imu_gyr = [imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0];
        match sim.last_gps() {
            Some(g) => {
                st.gps_lat = VP_LAT0 + g.pos[0].0 / 111_320.0;
                st.gps_lon = VP_LON0 + g.pos[1].0 / (111_320.0 * VP_LAT0.to_radians().cos());
                st.gps_alt = VP_ALT0 - g.pos[2].0;
                st.gps_fix = 3.0;
                if let Some(v) = g.vel {
                    st.gps_vel = [v[0].0, v[1].0, v[2].0];
                }
            }
            None => {
                st.gps_fix = 3.0;
            }
        }
        // 气压：家庭点=起飞台 d=-5，baro h=0 对齐（与 x_vperiph 同约定）
        let baro_up = sim.last_baro_alt();
        let h = baro_up - 5.0;
        st.baro_pa = 101_325.0 * (-h / 8434.5).exp();
        st.rc_ch[4] = 2000.0;
    }

    /// 演示机动：注入 SBUS 摇杆做水平 8 字（画面可见飞行）+ 注入层定高外环。
    /// 固件为 real-sensors（非 HIL），SBUS 摇杆真实进入控制；油门语义
    /// `target_alt = hold_alt - (throttle-0.5)*2`：中位=保持基准高度。
    /// 水平机动倾斜 → 升力垂直分量减 → 高度下沉，固件高度环（Unicorn 慢，0.3
    /// 周期/步）补偿滞后 → ±1m 起伏。这里在注入层按高度误差微调油门通道补偿
    /// （与固件内环级联）：高度偏低 → 推油门 → 固件目标高度抬升。
    fn inject_maneuver(&mut self, t_sim: f64, z: f32) {
        // 8 字幅度（速率模式：摇杆 → 期望速度，半径 = 速度/角频率）：
        // 周期 16s→8s（机动墙钟减半，感知更快）；roll/pitch 0.75 → 速度
        // 0.75×1.7×~2 ≈ 2.55m/s → 半径 2.55÷(2π/8) ≈ 3.25m（对称 8 字）
        let roll_amp = 0.75f32;
        let pitch_amp = 0.75f32;
        let w = 2.0f64 * std::f64::consts::PI / 8.0; // 8 字周期 8s 仿真
        let mut st = self.state.lock().unwrap();
        st.rc_ch[0] = 1500.0 + roll_amp * (w * t_sim).sin() as f32 * 500.0;
        st.rc_ch[1] = 1500.0 + pitch_amp * (w * t_sim).cos() as f32 * 500.0;
        // 定高外环（注入层）：目标=起飞基准 hold_z，误差 → 油门通道
        if let Some(hz) = self.hold_z {
            // z 一阶低通（去真值噪声，防油门高频抖动）
            if self.z_filt == 0.0 {
                self.z_filt = z;
            }
            self.z_filt = 0.85 * self.z_filt + 0.15 * z;
            let err = self.z_filt - hz; // NED z 向下为正：err>0 = 高度偏低
            self.alt_int = (self.alt_int + err * DT as f32).clamp(-0.8, 0.8);
            // 前馈：机动倾斜越大升力损失越大 → 同步预推油（无滞后补偿下沉）
            let tilt = (st.rc_ch[0] - 1500.0).abs() / 500.0 + (st.rc_ch[1] - 1500.0).abs() / 500.0;
            let ff = tilt * 0.14;
            // kp=0.55 → 偏低 1m 推油近饱和（固件目标抬 ~1.1m）；ki 抑静差；
            // 前馈随摇杆同步推油，抵消机动倾斜的升力损失（与 8 字同相位）
            let thr = (0.5 + ff + (0.55 * err + 0.10 * self.alt_int)).clamp(0.25, 0.95);
            st.rc_ch[3] = thr * 1000.0 + 1000.0;
        }
    }

    /// 推进 Unicorn 固件一步。偶发非法指令 → Err（上层重建）。
    ///
    /// 指令数 150k ≈ 固件 0.22 个控制周期（4ms@168MHz≈67 万条）。实测：
    /// run(300k)=20.9ms / run(200k)=16.5ms / run(150k)=11.5ms。
    /// 旧注释：150k 控制律滞后发散——但彼时无定高外环；当前有注入层定高外环
    /// + 速率模式，150k（0.22 周期/步 → 控制律 ~18ms 物理/周期）实测稳定与否
    /// 以联调为准。稳定则 fps 7.5→10.5（1.4×），否则回 200k。
    fn run_step(&self) -> Result<(), ()> {
        self.machine.lock().unwrap().run(200_000).map_err(|e| {
            eprintln!("[vperiph] Unicorn run 失败：{e:?}（重建重启）");
        })
    }

    /// 固件 EKF 估计高度 est.pos[2]（NED 向下正）。
    fn ekf_z(&self) -> f32 {
        let mut m = self.machine.lock().unwrap();
        m.cpu.mem_read(0x2000_9074 + 28, 4)
            .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
            .unwrap_or(f32::NAN)
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
    hil_probe: fly_sim_hil::hil_link::CdcProbe, // 后台 USB-CDC 探测，主循环读取缓存，永不阻塞
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
    // vperiph：Unicorn 虚拟 MCU（场景 "vperiph" 时驱动，HIL 的虚拟后端）。
    vp: Option<VperiphMc>,
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
            hil_probe: fly_sim_hil::hil_link::CdcProbe::start(),
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
            vp: None,
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
        // vperiph 场景固定全场景真实化（realistic + 风场），与 x_hover_env 对齐。
        let vperiph = c.scenario == "vperiph";
        self.sensor_cfg = if vperiph || c.sensor_noise {
            SensorConfig::realistic()
        } else {
            SensorConfig::default()
        };
        self.hil_noise = if !vperiph && c.sensor_noise {
            Some(SensorModel::new(SensorConfig::realistic(), DT))
        } else {
            None
        };
        let wind = if vperiph {
            // 全场景真实化风场（同 x_hover_env）：2.5m/s 北向稳态 + 阵风 + 湍流 + 风切。
            Some(WindField::new(WindConfig {
                base: [2.5, 0.0, -1.0],
                gust_amp: [1.2, 0.0, 0.0],
                gust_freq: 0.12,
                turb_sigma: [0.3, 0.1, -0.3],
                turb_tau: 0.5,
                seed: 0x1234_5678,
                shear_exponent: 0.2,
                shear_ref_height: 10.0,
                spatial_scale: 2.0,
                ..WindConfig::default()
            }))
        } else if c.wind > 0.0 {
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
        // 场景/配置变更 → 虚拟 MCU 重建（下次 advance_vperiph 重新 boot）
        self.vp = None;
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

    /// vperiph 模式单帧推进（场景 "vperiph"）：Unicorn 虚拟 MCU 闭环。
    ///
    /// 与 `advance_hil` 同一闭环骨架（`SimLoop::step_hil` + 起飞台保持 + 渲染/遥测），
    /// 差异仅在 MCU 来源：指令读 Unicorn 固件 PWM，传感器经虚拟外设注入
    /// （fly-sim 的 `SensorConfig::realistic` 噪声化读数 → `FlySimState` → 固件）。
    fn advance_vperiph(&mut self, c: &ControlState) -> Option<(Option<RenderInput>, String)> {
        // 虚拟 MCU 装配（boot → EKF 收敛 → ARM），内部已对 Unicorn 偶发崩溃重试。
        if self.vp.is_none() {
            match VperiphMc::boot() {
                Ok(vp) => self.vp = Some(vp),
                Err(e) => {
                    eprintln!("[vperiph] boot 失败：{e}");
                    return Some((
                        None,
                        format!(
                            "{{\"alt\":0.0,\"horiz\":0.0,\"speed\":0.0,\"roll\":0.0,\"pitch\":0.0,\"yaw\":0.0,\"m\":[0,0,0,0],\"steps\":0,\"scenario\":\"vperiph\",\"diverged\":true}}"
                        ),
                    ));
                }
            }
        }
        let vp = self.vp.as_mut()?;

        let mut last: Option<flyctrl_core::vehicle::VehicleState> = None;
        let mut motors = [0f32; 4];
        // vperiph 帧步数多于 SIL（16 步/帧 = 64ms 仿真）：Unicorn 比真机慢 ~5×，
        // 加速仿真时钟让机动/风扰动在画面上可见（同时降渲染分辨率保帧率）。
        for _ in 0..VP_STEPS_PER_FRAME {
            // 每步读回固件最新 PWM 推力（同 advance_hil 的 link.actuator() 节奏）
            motors = vp.read_thrust();
            let thrust: f32 = motors.iter().sum();
            let held = vp.hold && thrust < 0.05;
            let cmd = ActuatorCmd { motor: motors };
            let st = if held {
                // 起飞台保持：固件未产生推力前机体静止于初始悬停点（防自由落体尖峰）
                if let Some(sim) = self.sim.as_ref() {
                    Some(sim.snapshot().0)
                } else {
                    None
                }
            } else {
                vp.hold = false;
                if let Some(sim) = self.sim.as_mut() {
                    let st = sim.step_hil(&cmd);
                    // 脱离起飞台瞬间锁定定高基准（实测比延迟锁定更稳：起飞后高度
                    // 随机动自然稳定，立即锁定让外环全程补偿下沉）
                    if vp.hold_z.is_none() {
                        vp.hold_z = Some(st.pos[2].0);
                    }
                    Some(st)
                } else {
                    None
                }
            };
            self.phase_steps += 1;
            // 注入噪声化读数（SimLoop SensorConfig 已叠加 realistic 噪声/风作用）
            if let Some(sim) = self.sim.as_ref() {
                vp.inject(sim);
            }
            // 演示机动：水平 8 字摇杆 + 注入层定高外环（高度保持）
            let z = st.as_ref().map(|s| s.pos[2].0).unwrap_or(0.0);
            vp.inject_maneuver(self.phase_steps as f64 * DT, z);
            // Unicorn 推进（每 VP_RUN_EVERY 物理步一次：run 是帧率唯一成本，
            // 物理步由 SimLoop 免费推进）；偶发非法指令 → 重建（下帧重新 boot）
            if self.phase_steps % VP_RUN_EVERY == 0 && vp.run_step().is_err() {
                self.vp = None;
                return Some((
                    None,
                    "{\"alt\":0.0,\"horiz\":0.0,\"speed\":0.0,\"roll\":0.0,\"pitch\":0.0,\"yaw\":0.0,\"m\":[0,0,0,0],\"steps\":0,\"scenario\":\"vperiph\",\"diverged\":true}"
                        .to_string(),
                ));
            }
            last = st;
        }
        let st = last?;
        let cmd = ActuatorCmd { motor: motors };
        let inp = self.render_input(&st, &cmd, c);
        let tele = telemetry_json(&st, &cmd, c, self.phase_steps, false);
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

fn handle_ws(stream: TcpStream, ctrl: Arc<Mutex<ControlState>>, fmt: String, q: String) {
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

    // 渲染线程：软件光栅化 + 编码 + WS 推帧，与仿真解耦（Unicorn 仿真不被渲染/网络阻塞）。
    // 有界通道(2)：渲染慢于仿真时 try_send 丢新帧——仿真保持实时，画面延迟 ≤2 帧。
    // 帧格式：w(4B LE) + h(4B LE) + fmt(1B) + [fmt=1: PNG | fmt=2: key(1B)+H.264 Annex-B]。
    // 推帧格式由客户端协商（fmt 参数）：h264（默认，vperiph 视频流）/ png（无 WebCodecs 回退）。
    let use_h264 = fmt == "h264";
    // 移动端降档：q=low（640×480+600k+20fps，用户要求移动端同分辨率）；
    // q=lowest（320×240+350k+20fps，前端解码 640 失败自动退档重试）。
    // 桌面正常（q=""）：640×480 + 800kbps + 40fps。
    let low = q == "low" || q == "lowest";
    let q320 = q == "lowest";
    let (render_tx, render_rx) =
        std::sync::mpsc::sync_channel::<(u32, u32, RenderInput, String)>(2);
    let render_stream = stream.try_clone().unwrap();
    thread::spawn(move || {
        let mut s = render_stream;
        // vperiph(320×240/640×480) 协商为 h264 时用视频流（帧间压缩，带宽比 PNG 省 ~5×）；
        // SIL(720×540) 恒用 PNG。编码器每连接一个。
        let mut h264: Option<Encoder> = None;
        let frame_skip = if low { 2 } else { 1 }; // 手机档每 2 帧推 1（40→20fps）
        let mut skip = 0u32;
        for (fw, fh, inp, tele) in render_rx {
            skip += 1;
            if skip % frame_skip != 0 { continue; }
            let pixels = fly_sim_core::render::render_frame(fw, fh, &inp);
            let bytes: &[u8] = bytemuck_pixels(&pixels);
            if use_h264 && (low || (fw == 640 && fh == 480)) {
                if h264.is_none() {
                    // 桌面：640×480 @800kbps GOP30（0.75s@40fps）；
                    // 手机 low：640×480 @600kbps GOP20；lowest：320×240 @350kbps GOP20
                    let (br, mfps, gop) = if q320 {
                        (350_000u32, 20u32, 20u32)
                    } else if low {
                        (600_000u32, 20u32, 20u32)
                    } else {
                        (800_000u32, 40u32, 30u32)
                    };
                    h264 = new_h264_encoder(br, mfps, gop).ok();
                }
                if let Some(enc) = h264.as_mut() {
                    let i420 = I420Source { w: fw as usize, h: fh as usize, data: rgba_to_i420(bytes, fw as usize, fh as usize) };
                    if let Ok(bs) = enc.encode(&i420) {
                        let key = bs.frame_type() == FrameType::IDR;
                        let mut nal = Vec::new();
                        bs.write_vec(&mut nal);
                        let mut bin = Vec::with_capacity(10 + nal.len());
                        bin.extend_from_slice(&fw.to_le_bytes());
                        bin.extend_from_slice(&fh.to_le_bytes());
                        bin.push(2u8); // fmt=2: H.264 Annex-B
                        bin.push(if key { 1u8 } else { 0u8 });
                        bin.extend_from_slice(&nal);
                        let _ = ws::send_frame(&mut s, 0x2, &bin);
                        let _ = ws::send_frame(&mut s, 0x1, tele.as_bytes());
                        continue;
                    }
                }
                // 编码失败 → 回退 PNG（fmt=1）
            }
            let mut png_buf = Vec::new();
            {
                let mut enc = png::Encoder::new(&mut png_buf, fw, fh);
                enc.set_color(png::ColorType::Rgba);
                enc.set_depth(png::BitDepth::Eight);
                enc.set_compression(png::Compression::Fast);
                if let Ok(mut writer) = enc.write_header() {
                    let _ = writer.write_image_data(bytes);
                }
            }
            let mut bin = Vec::with_capacity(9 + png_buf.len());
            bin.extend_from_slice(&fw.to_le_bytes());
            bin.extend_from_slice(&fh.to_le_bytes());
            bin.push(1u8); // fmt=1: PNG
            bin.extend_from_slice(&png_buf);
            let _ = ws::send_frame(&mut s, 0x2, &bin);
            let _ = ws::send_frame(&mut s, 0x1, tele.as_bytes());
        }
    });

    loop {
        iter += 1;
        if stop_rx.try_recv().is_ok() {
            eprintln!("[ws][dbg] loop break at iter {iter}");
            break;
        }
        if iter % 300 == 0 {
            eprintln!("[ws][dbg] loop alive iter {iter}");
        }
        // 每帧：取控制、按需重建、步进（仿真线程只做物理+固件模拟，渲染交给渲染线程）。
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
        } else if c.scenario == "vperiph" {
            driver.advance_vperiph(&c)
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
                // vperiph：H.264 视频流用 640×480（帧间压缩带宽余量大，分辨率提升 4×）；
                // PNG 推帧保持 320×240（PNG 逐帧压缩，带宽 ~2.1Mbps 已近上限）。
                // 前端按帧头 w/h 自适应显示。
                let (fw, fh) = if c.scenario == "vperiph" {
                    // 桌面/手机 low：H.264 640×480；lowest：320×240（前端解码 640 失败退档）；
                    // PNG 推帧保持 320×240
                    if use_h264 {
                        if q320 { (320u32, 240u32) } else { (640u32, 480u32) }
                    } else { (320u32, 240u32) }
                } else { (FRAME_W, FRAME_H) };
                if render_tx.try_send((fw, fh, inp, tele)).is_err() {
                    // 渲染线程忙（通道满）：丢本帧保仿真实时，画面延迟 ≤2 帧。
                }
            } else {
                // 无渲染帧的状态遥测（如 vperiph boot 中/失败）：主线程直发。
                let _ = ws::send_frame(&mut stream, 0x1, tele.as_bytes());
            }
        }
        // 节流到 ~30fps 显示节奏。vperiph 例外：Unicorn 推进已远慢于 30fps，
        // sleep 只会白白增加延迟，跳过（仿真本身已是限速器）。
        if c.scenario != "vperiph" {
            let elapsed = last.elapsed();
            if elapsed < frame_interval {
                thread::sleep(frame_interval - elapsed);
            }
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
        // 前端按能力协商推帧格式：?fmt=png（非安全上下文/老浏览器无 WebCodecs）| h264（默认）。
        // 移动端可附加 ?q=low（320×240+500kbps+20fps，省流量/解码友好）。
        let query = path.split('?').nth(1).unwrap_or("");
        let fmt = query
            .split('&')
            .find_map(|kv| kv.strip_prefix("fmt="))
            .unwrap_or("h264")
            .to_string();
        let q = query
            .split('&')
            .find_map(|kv| kv.strip_prefix("q="))
            .unwrap_or("")
            .to_string();
        let ctrl = Arc::new(Mutex::new(ControlState::default()));
        handle_ws(stream.try_clone().expect("clone 失败"), ctrl, fmt, q);
    } else {
        let _ = serve_static(stream, path);
    }
}

fn main() {
    let port = std::env::var("FLY_SIM_PORT")
        .ok()
        .and_then(|v| v.parse::<u16>().ok())
        .unwrap_or(PORT);
    let bind_ip = std::env::var("FLY_SIM_BIND").unwrap_or_else(|_| "0.0.0.0".to_string());
    let listener = TcpListener::bind((bind_ip.as_str(), port)).expect("无法绑定端口");
    println!("[server] Fly Simulator Web 后端已启动: http://{}:{}/", bind_ip, port);
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
