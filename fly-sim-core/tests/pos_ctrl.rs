//! 阶段 4：**速度/位置控制外环**测试（H 场）。
//!
//! # 隔离手段（与阶段 2/3 同一套思路）
//!
//! - **外环驱动**：调 `PidController::control`（含位置外环 → 速度中环 → 姿态内环），
//!   设定点用 `Setpoint{pos, yaw, vel, acc}`。
//! - **两段隔离**：
//!   - `use_est=false`（**4A**）：控制反馈用 `plant.state_ned()` **真值** →
//!     单独评价外环本身（不受估计误差影响）。
//!   - `use_est=true`（**4B**）：反馈换成 `HilContext::step_hil` 的 EKF 估计 →
//!     量化"估计误差注入外环"的代价。
//! - **顺序：先速度环（4.1），再位置环（4.2/4.3）** —— 外环调参必须内环先稳。
//!
//! # 设定点约定
//!
//! `PidController` 的位置外环是 `des_v = kp_xy·(sp.pos − est.pos) + sp.vel`。
//! 所以**纯速度指令**必须让 `sp.pos` 跟随真值位置（位置误差恒为 0），
//! 否则位置环会与速度指令对抗。

use fly_sim_core::metrics::{StepTrace, TrackMetrics};
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::plant::QuadrotorPlant;
use fly_sim_core::sensor::SensorConfig;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian, Second};
use flyctrl_core::vehicle::VehicleState;
use std::sync::Mutex;

static LOCK: Mutex<()> = Mutex::new(());
fn lock() -> std::sync::MutexGuard<'static, ()> {
    let g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    // ⚠️ **每个测试都从已知默认值开始**：`G_*` 是**进程级**全局静态，前一个测试的写入
    // 会泄漏给后续测试 —— 实测曾把 `G_AW_GPS` 留在 0（关），使所有 4B 用例丢掉平移补偿
    // 而擦线失败，且**结果随测试调度顺序漂移**（并行/乱序下随机挂）。
    // 锁只保证串行，不保证状态干净 ⇒ 在这里统一复位，与执行顺序无关。
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_AW_GPS),
            0.0, // = 生产默认（见 ekf.rs 该静态初值：“默认关”的安全理由）
        );
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_AW_TAU),
            0.0, // = 用内置默认 tau
        );
    }
    g
}

const START_NED: [f32; 3] = [0.0, 0.0, -5.0];

/// 显式开启平移补偿（`G_AW_GPS=1`）。
///
/// ⚠️ **生产默认是"关"**（见 `ekf.rs` 该静态的"默认关"安全理由：`a_world` 源的质量）。
/// 而 `pos_ctrl` 的 4B 判据是按"补偿开"标定的 ⇒ 这些用例必须**显式声明**自己测的是
/// **已补偿配置**，不能依赖生产默认值（否则生产一改默认，测试就无声地测错了对象）。
fn enable_translation_comp() {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_AW_GPS),
            1.0,
        );
    }
}

/// 悬停设定点（单轴位置）。
pub fn hover_sp(pos: [f32; 3]) -> Setpoint {
    Setpoint {
        pos: [Meter(pos[0]), Meter(pos[1]), Meter(pos[2])],
        yaw: Radian(0.0),
        vel: [MeterPerSecond::ZERO; 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
    }
}

pub struct OuterResult {
    /// 被控量（真值）vs 设定点的跟踪误差。
    pub track: TrackMetrics,
    /// 北向位置时间序列（用于阶跃指标）。
    pub north: StepTrace,
    /// 高度（D）时间序列。
    pub down: StepTrace,
    /// **倾角指令饱和率**（结算窗口内，`tilt` 被 `clamp` 到 `±tilt_max` 的步数占比）。
    ///
    /// H2 专项的关键观测：`tilt = clamp(acc/g, ±tilt_max)`，顶满即"倾角指令打满"
    /// → 姿态环被推到极限 → 电机饱和 → 0.3~0.7Hz 极限环。见 `DBG_TILT`。
    pub tilt_sat: f32,
}

/// 阶段 4 台架：外环控制 + 可选真值/估计反馈。
pub fn run_outer(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    sp_fn: impl Fn(f32, &VehicleState) -> Setpoint,
    use_est: bool,
) -> OuterResult {
    // ⚠️ `use_est=true` 必须是 **mode 7（位置 + 速度 + 姿态全取估计）**。
    // 曾误写为 `1`（= 仅位置取估计）→ 测试"静默换掉了被测对象"，
    // 4B 从 10.1m 的失败变成 0.498m 的通过。教训：位掩码语义要写清。
    run_outer_mode(vc, dur_s, scfg, settle_s, sp_fn, if use_est { 7 } else { 0 })
}

/// 四元数 nlerp（半球对齐 + 归一化）。`a=0` → `qt`（真值），`a=1` → `qe`（估计）。
fn quat_nlerp(qt: flyctrl_core::vehicle::Quaternion, qe: flyctrl_core::vehicle::Quaternion, a: f32) -> flyctrl_core::vehicle::Quaternion {
    use flyctrl_core::vehicle::Quaternion;
    let dot = qt.w * qe.w + qt.x * qe.x + qt.y * qe.y + qt.z * qe.z;
    let s = if dot < 0.0 { -1.0 } else { 1.0 };
    let q = Quaternion {
        w: qt.w * (1.0 - a) + qe.w * a * s,
        x: qt.x * (1.0 - a) + qe.x * a * s,
        y: qt.y * (1.0 - a) + qe.y * a * s,
        z: qt.z * (1.0 - a) + qe.z * a * s,
    };
    let n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if n < 1e-9 {
        Quaternion::IDENTITY
    } else {
        Quaternion { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n }
    }
}

/// **连续插值**版：每个通道的反馈在"真值 ↔ 估计"之间按 `alpha` 插值
/// （0 = 全真值，1 = 全估计）。用于量化**误差灵敏度**。
pub fn run_outer_blend(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    sp_fn: impl Fn(f32, &VehicleState) -> Setpoint,
    att_a: f32,
    vel_a: f32,
    pos_a: f32,
) -> OuterResult {
    run_outer_impl(vc, dur_s, scfg, settle_s, sp_fn, None, (att_a, vel_a, pos_a))
}

/// 同 [`run_outer`]，但可**逐通道**选择反馈来源 —— 用于二分定位正反馈路径。
///
/// `fb_mode` 位掩码（`0` = 全真值）：
/// - **bit0**：`pos` 取估计
/// - **bit1**：`vel` 取估计
/// - **bit2**：`att` + `omega` 取估计
///
/// 例：`5` = 位置与姿态取估计、速度真值。
pub fn run_outer_mode(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    sp_fn: impl Fn(f32, &VehicleState) -> Setpoint,
    fb_mode: u8,
) -> OuterResult {
    run_outer_impl(vc, dur_s, scfg, settle_s, sp_fn, Some(fb_mode), (0.0, 0.0, 0.0))
}

fn run_outer_impl(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    sp_fn: impl Fn(f32, &VehicleState) -> Setpoint,
    fb_mode: Option<u8>,
    blend: (f32, f32, f32),
) -> OuterResult {
    run_outer_full(vc, dur_s, scfg, settle_s, sp_fn, fb_mode, blend, false)
}

/// 最底层：多一个 `rate_mode`（= 固件的 ALT_HOLD：水平位置环旁路，只留速度环）。
fn run_outer_full(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    sp_fn: impl Fn(f32, &VehicleState) -> Setpoint,
    fb_mode: Option<u8>,
    blend: (f32, f32, f32),
    rate_mode: bool,
) -> OuterResult {
    let cfg = vc.ctrl_params();
    let dt_f = 0.004f64;
    let dt = dt_f as f32;

    let mut plant = QuadrotorPlant::new_at(
        PhySdkWorld::create_empty(),
        vc,
        dt_f,
        None,
        scfg.clone(),
        None,
        Vec::new(),
        START_NED,
    );
    let mut ctrl = PidController::from_config(&cfg);
    ctrl.set_rate_mode_xy(rate_mode);
    let mut hil = HilContext::new(
        EkfEstimator::default_quad(),
        PidController::default_quad(),
        Second(dt),
    );
    hil.est.set_mag_declination(scfg.mag_decl_deg as f32);
    hil.est.set_mag_hard_iron([
        scfg.mag_hard_iron[0] as f32,
        scfg.mag_hard_iron[1] as f32,
        scfg.mag_hard_iron[2] as f32,
    ]);
    hil.est.set_initial_position(START_NED);
    hil.baro_ref = 0.0;
    hil.baro_locked = true; // 气压 = 绝对高度观测（与 GPS NED 原点一致，见阶段3）

    let mut sim_imu = SimImu::new();
    let sp0 = hover_sp(START_NED);

    let n = (dur_s / dt) as u32;
    let t0 = (settle_s / dt) as u32;
    let mut track = TrackMetrics::default();
    let mut tilt_sat_n = 0u32;
    let mut tilt_n = 0u32;
    let mut north = StepTrace::new(dt, START_NED[0], START_NED[0], 0.05);
    let mut down = StepTrace::new(dt, START_NED[2], START_NED[2], 0.05);

    for i in 0..n {
        let t = i as f32 * dt;
        let truth = plant.state_ned();

        let (imu, gps) = plant.read_sensors();
        let (mag, baro) = plant.read_sensors_attitude();
        let r = hil.step_hil(
            Some(imu),
            gps,
            Some(baro.altitude as f32),
            None,
            None,
            Some([
                mag.field[0] as f32,
                mag.field[1] as f32,
                mag.field[2] as f32,
            ]),
            &sp0,
            false,
            false,
            true,
            &mut sim_imu,
        );

        let sp = sp_fn(t, &truth);
        let mut fb = truth;
        match fb_mode {
            Some(m) => {
                if m & 1 != 0 {
                    fb.pos = r.est.pos;
                }
                if m & 2 != 0 {
                    fb.vel = r.est.vel;
                }
                if m & 4 != 0 {
                    fb.att = r.est.att;
                    fb.omega = r.est.omega;
                }
            }
            None => {
                let (aa, va, pa) = blend;
                if pa > 0.0 {
                    for k in 0..3 {
                        fb.pos[k] = Meter(truth.pos[k].0 * (1.0 - pa) + r.est.pos[k].0 * pa);
                    }
                }
                if va > 0.0 {
                    for k in 0..3 {
                        fb.vel[k] = MeterPerSecond(truth.vel[k].0 * (1.0 - va) + r.est.vel[k].0 * va);
                    }
                }
                if aa > 0.0 {
                    fb.att = quat_nlerp(truth.att, r.est.att, aa);
                    for k in 0..3 {
                        fb.omega[k] = flyctrl_core::units::RadianPerSecond(
                            truth.omega[k].0 * (1.0 - aa) + r.est.omega[k].0 * aa,
                        );
                    }
                }
            }
        }
        let cmd = ctrl.control(Second(dt), &sp, &fb);
        if i >= t0 {
            let t = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(
                    flyctrl_core::controller::pid::DBG_TILT
                ))
            };
            let tm = cfg.tilt_max;
            if t[2].abs() >= tm * 0.999 || t[3].abs() >= tm * 0.999 {
                tilt_sat_n += 1;
            }
            tilt_n += 1;
        }
        plant.apply_actuators(&cmd);
        plant.step();

        if i >= t0 {
            track.push(
                [truth.pos[0].0, truth.pos[1].0, truth.pos[2].0],
                [truth.vel[0].0, truth.vel[1].0, truth.vel[2].0],
                [sp.pos[0].0, sp.pos[1].0, sp.pos[2].0],
                [sp.vel[0].0, sp.vel[1].0, sp.vel[2].0],
            );
        }
        north.target = sp.pos[0].0;
        down.target = sp.pos[2].0;
        north.push(truth.pos[0].0);
        down.push(truth.pos[2].0);
    }
    let tilt_sat = if tilt_n > 0 {
        tilt_sat_n as f32 / tilt_n as f32
    } else {
        f32::NAN
    };
    OuterResult {
        track,
        north,
        down,
        tilt_sat,
    }
}

// ============================================================ 4.1 速度指令（先速度环）

/// 4.1 速度指令跟踪：`sp.pos` 跟随真值（位置环不参与），`sp.vel` 给阶跃。
#[test]
fn outer_velocity_command() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let v_cmd = 1.0f32; // m/s 北向
    let sp_fn = move |t: f32, truth: &VehicleState| {
        let mut sp = hover_sp([truth.pos[0].0, truth.pos[1].0, truth.pos[2].0]);
        if t >= 1.0 {
            sp.vel[0] = MeterPerSecond(v_cmd);
        }
        sp
    };
    for (tag, use_est) in [("4A 真值反馈", false), ("4B 估计反馈", true)] {
        if use_est {
            enable_translation_comp(); // 4B 判据按"已补偿配置"标定
        }
        let r = run_outer(&vc, 10.0, SensorConfig::default(), 2.5, sp_fn, use_est);
        println!(
            "{}",
            r.track.summary(&format!("4.1 速度指令 {v_cmd} m/s [{tag}]（测 2.5~10s）"))
        );
        assert!(!r.track.diverged(), "4.1 [{tag}] 发散");
        // 判据（由实测推出）：稳态速度跟踪误差应小
        // 判据（实测 4A=0.375 m/s）：纯速度指令下位置环被旁路（sp.pos 跟随真值），
        // 稳态速度误差由**旋翼阻力**决定（`acc = kv_xy·(v_cmd − v)`，需非零误差提供倾角）。
        assert!(
            r.track.vel_rmse() < 0.5,
            "4.1 [{tag}] 速度跟踪 RMSE {:.3}m/s 应 <0.5",
            r.track.vel_rmse()
        );
    }
}

// ============================================================ 4.2 位置阶跃

/// 4.2 位置阶跃：北向 3m。
#[test]
fn outer_position_step() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |t: f32, _truth: &VehicleState| {
        let n = if t >= 1.0 { 3.0 } else { 0.0 };
        hover_sp([n, 0.0, START_NED[2]])
    };
    for (tag, use_est) in [("4A 真值反馈", false), ("4B 估计反馈", true)] {
        if use_est {
            enable_translation_comp();
        }
        let r = run_outer(&vc, 15.0, SensorConfig::default(), 1.0, sp_fn, use_est);
        println!(
            "{}",
            r.track.summary(&format!("4.2 位置阶跃 3m [{tag}]（含瞬态）"))
        );
        println!("  北向阶跃: {}", r.north.summary("north"));
        assert!(!r.track.diverged(), "4.2 [{tag}] 发散");
        // 含瞬态的 RMSE 只作记录（阶跃本身 3m）；判据看下面的阶跃指标与末值
        // 高度不应被水平机动带跑（阈值 0.5m）。
        // 注：高度目标是**恒定**的，`StepTrace` 的 amp≈0 → 其超调/调节无意义，
        // 只看末值（这是我上一版打印出 2.18e10% 超调的原因）。
        // 门槛分级：4A（真值反馈）0.5m；4B（估计反馈）**立案** —— 实测 1.944m，
        // 水平机动把高度带跑（与 M 场 x_hover_noise 同源，见 stage4 文档）。
        let dz = (r.down.y[r.down.y.len() - 1] - START_NED[2]).abs();
        let dz_lim = if use_est { 0.5 } else { 0.5 };
        assert!(
            dz < dz_lim,
            "4.2 [{tag}] 阶跃后高度漂移 {dz:.3}m 应 <{dz_lim}m（水平机动→高度耦合）"
        );
        // 阶跃指标用**北向** StepTrace（那才是真阶跃）
        let ns = r.north.settle_s();
        assert!(ns > 0.0, "4.2 [{tag}] 北向阶跃未在窗口内进入 ±5% 带");
        assert!(
            r.north.ss_err().abs() < 0.3,
            "4.2 [{tag}] 北向稳态误差 {:.3}m 应 <0.3",
            r.north.ss_err()
        );
    }
}

// ============================================================ 4.3 悬停

/// 4.3 悬停：位置应稳定保持在设定点。
#[test]
fn outer_hover_hold() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _truth: &VehicleState| hover_sp(START_NED);
    for (tag, scfg) in [
        ("4A clean", SensorConfig::default()),
        ("4B realistic", SensorConfig::realistic()),
    ] {
        enable_translation_comp(); // 4.3 用估计反馈 ⇒ 已补偿配置
        let r = run_outer(&vc, 30.0, scfg, 3.0, sp_fn, true);
        println!("{}", r.track.summary(&format!("4.3 悬停 [{tag}]")));
        assert!(!r.track.diverged(), "4.3 [{tag}] 发散");
        let lim = if tag.starts_with("4A") { 0.5 } else { 1.5 };
        assert!(
            r.track.pos_rmse() < lim,
            "4.3 [{tag}] 悬停位置误差 {:.3}m 应 <{lim} —— 4B（估计反馈+逼真传感器）\
             实测 10.1m，与 M 场 `x_hover_noise` 同源（见 docs/stage4-*-findings.md）",
            r.track.pos_rmse()
        );
    }
}

/// 4.4 **二分**：4B 悬停漂移 10m，根因在姿态估计、位置/速度估计，还是外环本身？
#[test]
fn outer_bisect_feedback_source() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _truth: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.4 反馈来源二分（悬停 20s，realistic 传感器）===");
    println!("{:>28} {:>14} {:>14}", "反馈组合", "pos_rmse(m)", "pos_max(m)");
    for (tag, mode) in [
        ("全真值（4A）", 0u8),
        ("仅姿态取估计", 4),
        ("仅位置/速度取估计", 3),
        ("全取估计（4B）", 7),
    ] {
        let r = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, mode);
        println!(
            "{:>28} {:>14.3} {:>14.3}",
            tag,
            r.track.pos_rmse(),
            r.track.pos_max()
        );
        assert!(!r.track.diverged(), "{tag} 发散");
    }
}

/// 4.4b **全 8 组合**：逐通道定位正反馈路径。
///
/// 位掩码：bit0=pos、bit1=vel、bit2=att/omega。找"哪两条通道同时取估计会爆"。
#[test]
fn outer_bisect_all_combinations() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _truth: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.4b 全组合（悬停 20s，realistic）===");
    println!("{:>6} {:>22} {:>14} {:>14}", "mode", "取估计的通道", "pos_rmse", "pos_max");
    let mut prev = 0.0f32;
    for mode in 0u8..8 {
        let mut parts = Vec::new();
        if mode & 1 != 0 {
            parts.push("pos");
        }
        if mode & 2 != 0 {
            parts.push("vel");
        }
        if mode & 4 != 0 {
            parts.push("att");
        }
        let name = if parts.is_empty() {
            "（全真值）".to_string()
        } else {
            parts.join("+")
        };
        let r = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, mode);
        println!(
            "{:>6} {:>22} {:>14.3} {:>14.3}",
            mode,
            name,
            r.track.pos_rmse(),
            r.track.pos_max()
        );
        // 时间尺度诊断：mode 6（爆点）打印北向位置轨迹（每 2s），区分漂移/振荡
        if mode == 6 {
            let dt = r.north.dt;
            print!("       mode6 北向位置(每0.5s,m): ");
            let mut k = 0;
            while ((k as f32 * 0.5 / dt) as usize) < r.north.y.len() {
                let i = (k as f32 * 0.5 / dt) as usize;
                print!("{:.2} ", r.north.y[i]);
                k += 1;
            }
            println!();
        }
        assert!(!r.track.diverged(), "mode={mode} 发散");
        prev = r.track.pos_rmse();
    }
    let _ = prev;
}

/// 4.5 **外环增益扫描**：`vel+att` 耦合的失稳是"裕度可调"还是"结构性"？
///
/// 扫 `(kp_xy, kv_xy)`，看是否存在能稳住 mode 6/7 的组合。
#[test]
fn outer_gain_scan_for_coupling() {
    let _g = lock();
    println!("\n=== 4.5 外环增益扫描（悬停 20s，realistic，mode 7=全取估计）===");
    println!("{:>8} {:>8} {:>14} {:>14}", "kp_xy", "kv_xy", "pos_rmse", "pos_max");
    for (kp, kv) in [
        (0.3f32, 0.8f32), // 出厂
        (0.3, 0.4),
        (0.3, 0.2),
        (0.15, 0.4),
        (0.5, 0.8),
        (0.5, 1.2),
    ] {
        let vc = VehicleConfig::default_quad().with_gains(3.0, 0.3, kp, kv);
        let sp_fn = |_t: f32, _truth: &VehicleState| hover_sp(START_NED);
        let r = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 7);
        println!(
            "{:>8.2} {:>8.2} {:>14.3} {:>14.3}",
            kp,
            kv,
            r.track.pos_rmse(),
            r.track.pos_max()
        );
        assert!(!r.track.diverged(), "kp={kp} kv={kv} 发散");
    }
}

/// 4.6 **联合扫描 `(att_kp, kv_xy)`**：更强的内环阻尼能否允许更高的外环增益？
///
/// 这是阶段 2（`att_kp` 3.0→2.0?）与阶段 4（`kv_xy` 0.8→?）两个决策的**汇合点**。
/// 同时记录两个代价：
/// - `pos_max`（mode 7 悬停，耦合失稳幅度）
/// - 北向阶跃 settle（4A 真值反馈，外环快不快）
#[test]
fn outer_joint_gain_scan() {
    let _g = lock();
    println!("\n=== 4.6 联合扫描 (att_kp, kv_xy)：耦合幅度 vs 外环速度 ===");
    println!(
        "{:>8} {:>8} {:>14} {:>14} {:>16}",
        "att_kp", "kv_xy", "悬停pos_max", "悬停pos_rmse", "阶跃settle(s)"
    );
    for att_kp in [2.0f32, 3.0, 4.0] {
        for kv in [0.2f32, 0.4, 0.8] {
            let vc = VehicleConfig::default_quad().with_gains(att_kp, 0.3, 0.3, kv);
            // 耦合：mode 7 悬停
            let sp_h = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
            let rh = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_h, 7);
            // 外环速度：4A 真值反馈的位置阶跃
            let sp_s = |t: f32, _tr: &VehicleState| {
                let n = if t >= 1.0 { 3.0 } else { 0.0 };
                hover_sp([n, 0.0, START_NED[2]])
            };
            let rs = run_outer(&vc, 15.0, SensorConfig::default(), 1.0, sp_s, false);
            println!(
                "{:>8.1} {:>8.1} {:>14.3} {:>14.3} {:>16.3}",
                att_kp,
                kv,
                rh.track.pos_max(),
                rh.track.pos_rmse(),
                rs.north.settle_s()
            );
            assert!(!rh.track.diverged(), "att_kp={att_kp} kv={kv} 发散");
        }
    }
}

/// 4.7 **传感器模型二分**：耦合失稳是由哪个传感器特性驱动的？
///
/// 假设：`vel` 估计的**相位滞后**是耦合的驱动 ⇒ 若成立，失稳应由
/// **GPS 0.15s 延迟 / 20Hz 帧率 / 噪声**中的某一项主导，逐项关掉应能定位。
/// 指标用**末段包络 RMS**（对相位不敏感，见 metrics 注释）。
#[test]
fn outer_sensor_model_bisect() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.7 传感器模型二分（悬停 20s，mode 7 全取估计）===");
    println!("{:>34} {:>14} {:>16}", "配置", "pos_rmse", "末段包络RMS");

    let mut configs: Vec<(&str, SensorConfig)> = vec![
        ("全清洁 default()", SensorConfig::default()),
        ("全逼真 realistic()", SensorConfig::realistic()),
    ];
    let mut c = SensorConfig::realistic();
    c.gps_delay = 0.0;
    configs.push(("realistic − GPS 延迟(0.15s→0)", c));
    let mut c = SensorConfig::realistic();
    c.gps_rate = 1000.0;
    configs.push(("realistic − GPS 降频(20→1000Hz)", c));
    let mut c = SensorConfig::realistic();
    c.accel_noise = 0.0;
    c.gyro_noise = 0.0;
    c.gyro_walk = 0.0;
    c.gyro_bias_inst = 0.0;
    c.vib_amp = 0.0;
    configs.push(("realistic − IMU 噪声/振动", c));
    let mut c = SensorConfig::realistic();
    c.accel_bias = [0.0; 3];
    c.gyro_bias = [0.0; 3];
    configs.push(("realistic − IMU 零偏", c));

    for (tag, scfg) in configs {
        let r = run_outer_mode(&vc, 20.0, scfg, 3.0, sp_fn, 7);
        println!(
            "{:>34} {:>14.3} {:>16.3}",
            tag,
            r.track.pos_rmse(),
            r.track.tail_pos_rms()
        );
        assert!(!r.track.diverged(), "{tag} 发散");
    }
}

/// 4.8 **稳定边界**：清洁 vs 逼真下 `kv_xy` 的临界值 ⇒ 量化"逼真度吃掉的裕度"。
#[test]
fn outer_kvxy_stability_boundary() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.8 kv_xy 稳定边界（悬停 20s，mode 7）===");
    println!(
        "{:>8} {:>16} {:>16}",
        "kv_xy", "清洁 末段RMS", "逼真 末段RMS"
    );
    for kv in [0.2f32, 0.3, 0.4, 0.6, 0.8, 1.2] {
        let vc = VehicleConfig::default_quad().with_gains(3.0, 0.3, 0.3, kv);
        let rc = run_outer_mode(&vc, 20.0, SensorConfig::default(), 3.0, sp_fn, 7);
        let rr = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 7);
        println!(
            "{:>8.2} {:>16.3} {:>16.3}",
            kv,
            rc.track.tail_pos_rms(),
            rr.track.tail_pos_rms()
        );
        assert!(!rc.track.diverged() && !rr.track.diverged(), "kv={kv} 发散");
    }
}

/// 4.9 **灵敏度量化**：逐通道把反馈从"真值"线性插值到"估计"，看逼真误差如何变化。
///
/// 目的：判断"补哪个通道的估计精度"最能把 4B 的误差拿回来。
#[test]
fn outer_error_sensitivity() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.9 误差灵敏度（悬停 20s，realistic，末段包络RMS）===");
    println!("{:>10} {:>14} {:>14} {:>14}", "alpha", "仅姿态", "仅速度", "仅位置");
    for a in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let r_att = run_outer_blend(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, a, 0.0, 0.0);
        let r_vel = run_outer_blend(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 0.0, a, 0.0);
        let r_pos = run_outer_blend(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 0.0, 0.0, a);
        println!(
            "{:>10.2} {:>14.3} {:>14.3} {:>14.3}",
            a,
            r_att.track.tail_pos_rms(),
            r_vel.track.tail_pos_rms(),
            r_pos.track.tail_pos_rms()
        );
    }
}

/// 4.10 **有界 vs 发散**：同一配置跑 20s 与 60s，比末段包络 RMS。
///
/// 若 60s 的末段 RMS 明显大于 20s ⇒ **发散**；若相近 ⇒ **有界的等比例放大**。
#[test]
fn outer_bounded_or_divergent() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.10 有界 vs 发散（mode 7，realistic，kv_xy=0.8）===");
    println!("{:>10} {:>16} {:>16}", "时长(s)", "pos_rmse", "末段包络RMS");
    for dur in [20.0f32, 60.0] {
        let r = run_outer_mode(&vc, dur, SensorConfig::realistic(), 3.0, sp_fn, 7);
        println!(
            "{:>10.0} {:>16.3} {:>16.3}",
            dur,
            r.track.pos_rmse(),
            r.track.tail_pos_rms()
        );
    }
    // 对照：清洁下同样对比（应几乎不变）
    for dur in [20.0f32, 60.0] {
        let r = run_outer_mode(&vc, dur, SensorConfig::default(), 3.0, sp_fn, 7);
        println!(
            "  清洁 {:>5.0}s {:>16.3} {:>16.3}",
            dur,
            r.track.pos_rmse(),
            r.track.tail_pos_rms()
        );
    }
}

/// 4.11 **跨阶段验证**：阶段 1 的平移补偿（`G_AW_GPS`）能否改善阶段 4 的耦合？
///
/// 这是把阶段 1/4 串起来的关键一问：A3 实测平移补偿能把姿态误差降 52~61%，
/// 而耦合是 `att × vel` —— 若补姿态精度有效，本测试应看到 4B 明显改善。
#[test]
fn outer_with_translation_compensation() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.11 平移补偿对阶段 4 耦合的作用（悬停 20s，mode 7，realistic）===");
    println!("{:>14} {:>16} {:>16}", "G_AW_GPS", "pos_rmse", "末段包络RMS");
    for aw in [0.0f32, 1.0] {
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_AW_GPS),
                aw,
            );
        }
        let r = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 7);
        println!(
            "{:>14.1} {:>16.3} {:>16.3}",
            aw,
            r.track.pos_rmse(),
            r.track.tail_pos_rms()
        );
    }
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_AW_GPS),
            0.0,
        );
    }
}

/// ① 诊断：垂向修复如何影响**估计器误差**，以及经哪条反馈路径传递。
///
/// `TrackMetrics` 量的是 **est vs truth**（估计器误差），非跟踪误差——
/// 所以“4B 悬停位置误差 1.744m”说的是**估计器**的误差。
/// 本诊断分解到三维（pos/horiz/**vert**）+ 反馈源位掩码。
#[test]
fn diagnose_kvz_pathway() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== ① 诊断：垂向修复对「估计器误差」的影响（30s realistic）===");
    println!("反馈源位掩码：bit0=位置估计 bit1=速度估计 bit2=姿态估计\n");
    println!(
        "{:>8} {:>6} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "kvz", "mode", "pos", "horiz", "vert", "vel", "尾段posRMS"
    );
    for kvz in [-1.0f32, 0.1] {
        set_kvz(kvz);
        for mode in [0u8, 1, 2, 4, 7] {
            let r = run_outer_mode(&vc, 30.0, SensorConfig::realistic(), 3.0, sp_fn, mode);
            println!(
                "{:>8} {mode:>6} {:>10.4} {:>10.4} {:>10.4} {:>10.4} {:>10.4}",
                if kvz < 0.0 { "历史".to_string() } else { format!("{kvz}") },
                r.track.pos_rmse(),
                r.track.horiz_rmse(),
                r.track.vert_rmse(),
                r.track.vel_rmse(),
                r.track.tail_pos_rms()
            );
        }
        println!();
    }
    set_kvz(0.0);
}

/// 垂向速度观测增益 `|k[5]|` 扫描（垂向专项 ① 的核心实验）。
///
/// 历史 `k[5]=0`（垂速纯 IMU 积分）→ 高度环漂移；但无上限地开大会“单拍 +24.6 → 爆炸”。
/// 本扫描找“既能补观测、又不推爆”的区间。
#[test]
fn outer_vertical_kvz_sweep() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    set_kvz(-1.0); // 历史对照（强制 k[5]=0）
    println!("\n=== ① 垂向速度观测增益扫描（60s 悬停，realistic，估计反馈）===");
    println!(
        "{:>12} {:>10} {:>12} {:>10} {:>10} {:>12}",
        "|k[5]|上限", "末高度", "净漂移m", "max|dz|", "水平RMSE", "pos RMSE"
    );
    for kvz in [-1.0f32, 0.02, 0.05, 0.1, 0.2, 1.0] {
        set_kvz(kvz);
        let r = run_outer(&vc, 60.0, SensorConfig::realistic(), 3.0, sp_fn, true);
        let y = &r.down.y;
        let n = y.len();
        let last = y[n - 1];
        let drift = last - START_NED[2];
        let maxdz = y
            .iter()
            .map(|v| (v - START_NED[2]).abs())
            .fold(0.0f32, f32::max);
        println!(
            "{:>12} {last:>10.3} {drift:>12.3} {maxdz:>10.3} {:>10.4} {:>12.4}",
            if kvz < 0.0 { "历史(0)".to_string() } else { format!("{kvz:.2}") },
            r.track.horiz_rmse(),
            r.track.pos_rmse()
        );
    }
    set_kvz(0.0);
}

/// 设 `G_KVZ_FROM_ALT`（0 = 历史行为：垂速不被位置观测修正）。
fn set_kvz(v: f32) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(flyctrl_core::estimator::ekf::G_KVZ_FROM_ALT),
            v,
        );
    }
}

/// **垂向通道专项：长时悬停的高度下沉**（H 场闭环）。
///
/// # 为何要它
///
/// M 场 `x_hover_noise` 在锁相修复后**只剩一项失败**：`dz=3.00m`（阈值 3.0）——
/// 60s 缓慢下沉 3.82m（真值 5→8.82m），而姿态（7.49°/11.81°）与水平（±2m）均通过。
/// 按纪律（M 场只验收），该问题**在 H 场先攻**。
///
/// 候选根因（待本测试确认）：EKF **不估计垂向速度**——
/// `update_alt` 里 `k[5] = 0.0` 刻意清零，垂速纯 IMU 积分
/// （清零原因：气压作为位置观测经非对角协方差会推爆垂速：
/// "恒定比力+气压下 vel_z 单拍 +24.6 → 爆炸"）。
/// `pos_est` 已记录后果：realistic 下 vD 误差 0.115→**1.015 m/s**。
///
/// # 度量
///
/// 高度（NED D，真值）随时间的**下沉量与速率**。两个反馈源对照：
/// `use_est=false`（真值反馈，外环本身）vs `true`（估计反馈，实际飞行）：
/// 若两者下沉相当 → 问题在外环/垂向估计的**共同上游**；若仅估计反馈下沉 → 是估计注入。
#[test]
fn outer_hover_vertical_sink_sixty_sec() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 垂向专项：60s 悬停高度（NED D，真值；起点 {START_NED:?}）===");
    println!(
        "{:>22} {:>10} {:>10} {:>12} {:>12}",
        "配置", "末高度", "下沉量m", "末10s速率", "max|dz|"
    );
    for (name, scfg, use_est) in [
        ("4A 真值反馈/精确", SensorConfig::default(), false),
        ("4B 估计反馈/精确", SensorConfig::default(), true),
        ("4A 真值反馈/realistic", SensorConfig::realistic(), false),
        ("4B 估计反馈/realistic", SensorConfig::realistic(), true),
    ] {
        let r = run_outer(&vc, 60.0, scfg, 3.0, sp_fn, use_est);
        let y = &r.down.y;
        let n = y.len();
        assert!(n > 100, "轨迹太短");
        let last = y[n - 1];
        let sink = last - START_NED[2]; // NED 向下为正 → 下沉为正
        let k = 2500.min(n - 1); // 末 10s（dt=4ms）
        let rate = (y[n - 1] - y[n - 1 - k]) / (k as f32 * r.down.dt);
        let maxdz = y
            .iter()
            .map(|v| (v - START_NED[2]).abs())
            .fold(0.0f32, f32::max);
        println!("{name:>22} {last:>10.3} {sink:>10.3} {rate:>12.4} {maxdz:>12.3}");
        assert!(last.is_finite(), "{name} 高度非有限");
        assert!(
            maxdz < 20.0,
            "{name} 高度发散（max|dz|={maxdz:.2}m）"
        );
    }
}

/// 4.12 **H2 专项**：扫描 `(tilt_max, kp_xy)`，看"倾角指令顶满"是否是极限环的驱动。
///
/// 假设（来自 M 场 [osc] 0.6–0.7Hz + 电机饱和 + 你的 WIP 诊断）：
/// `kp_xy` 偏大 ⇒ 速度误差稍大就命令大倾角 ⇒ 顶满 `tilt_max` ⇒ 姿态环到极限
/// ⇒ 电机饱和 ⇒ 极限环。若成立：**放宽 `tilt_max` 应显著改善**。
#[test]
fn h2_tilt_saturation_scan() {
    let _g = lock();
    let sp_fn = |_t: f32, _tr: &VehicleState| hover_sp(START_NED);
    println!("\n=== 4.12 H2 专项：(tilt_max, kp_xy) 扫描（悬停 20s，mode 7，realistic）===");
    println!(
        "{:>10} {:>8} {:>16} {:>14} {:>12}",
        "tilt_max", "kp_xy", "末段包络RMS", "pos_max", "倾角饱和率"
    );
    for tm in [0.35f32, 0.5, 0.7] {
        for kp in [0.3f32, 0.15] {
            let mut vc = VehicleConfig::default_quad().with_gains(3.0, 0.3, kp, 0.8);
            vc.tilt_max = tm;
            let r = run_outer_mode(&vc, 20.0, SensorConfig::realistic(), 3.0, sp_fn, 7);
            println!(
                "{:>10.2} {:>8.2} {:>16.3} {:>14.3} {:>11.1}%",
                tm,
                kp,
                r.track.tail_pos_rms(),
                r.track.pos_max(),
                r.tilt_sat * 100.0
            );
            assert!(!r.track.diverged(), "tm={tm} kp={kp} 发散");
        }
    }
}

/// 4.13 **复现 M 场的 ALT_HOLD 条件**（水平位置环旁路、只留速度环）。
///
/// M 场 `x_hover_noise` 是 ALT_HOLD（`rate_mode_xy`）：`des_v = clamp(sp.vel)` 摇杆直通，
/// 倾角指令**只**由 `kv_xy·(des_v − v)` 产生，且**没有位置反馈的阻尼**。
/// 这是与 H 场（完整位置保持）最大的模式差异 —— 若 H2 与它有关，这里应能复现。
#[test]
fn h2_reproduce_alt_hold_mode() {
    let _g = lock();
    println!("\n=== 4.13 ALT_HOLD（rate_mode_xy）复现尝试（20s，mode 7，realistic）===");
    println!(
        "{:>22} {:>16} {:>14} {:>12}",
        "配置", "末段包络RMS", "pos_max", "倾角饱和率"
    );
    for (tag, rate_mode, kv) in [
        ("完整位置保持 kv=0.8", false, 0.8f32),
        ("ALT_HOLD（位置旁路） kv=0.8", true, 0.8),
        ("ALT_HOLD kv=0.4", true, 0.4),
        ("ALT_HOLD kv=1.2", true, 1.2),
    ] {
        let vc = VehicleConfig::default_quad().with_gains(3.0, 0.3, 0.3, kv);
        // rate 模式下位置环旁路 ⇒ sp.pos 无意义；用真值构造无妨
        let sp_fn = |_t: f32, tr: &VehicleState| hover_sp([tr.pos[0].0, tr.pos[1].0, START_NED[2]]);
        let r = run_outer_full(
            &vc,
            20.0,
            SensorConfig::realistic(),
            3.0,
            sp_fn,
            Some(7),
            (0.0, 0.0, 0.0),
            rate_mode,
        );
        println!(
            "{:>22} {:>16.3} {:>14.3} {:>11.1}%",
            tag,
            r.track.tail_pos_rms(),
            r.track.pos_max(),
            r.tilt_sat * 100.0
        );
        assert!(!r.track.diverged(), "{tag} 发散");
    }
}
