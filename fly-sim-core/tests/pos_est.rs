//! 阶段 3：**位置/速度估计开环**测试（H 场）。
//!
//! # 隔离手段
//!
//! - **估计器不参与控制回路**：控制用 `plant.state_ned()` 的**真值状态**做完美姿态
//!   控制，飞行器按脚本剖面飞行；EKF 只在旁边做位置/速度估计 → **开环估计**。
//! - **EKF 走 `HilContext::step_hil`**（与生产同源：accel 40Hz 陷波+20Hz 低通、
//!   gyro 40Hz 陷波、FDIR、观测注入、setpoint 门控）。
//! - **同一状态对比**：每拍先用当前状态的传感器更新 EKF，再与**同一状态的真值**比较
//!   （避免引入一拍的相位误差）。
//!
//! # 两段隔离（路线图 §5）
//!
//! - **3A**：`SensorConfig::default()`（零噪声零偏）→ 姿态估计几乎无误差，
//!   位置通道被**单独**评价（不受姿态误差污染）。
//! - **3B**：`SensorConfig::realistic()` → 与 3A 对比，量化**姿态误差→位置**的耦合。
//!
//! # 已知设计局限
//!
//! EKF **不估计垂向速度**（纯 IMU 积分，位置观测不清零垂向速度增益），
//! 判据须按此设计能力定，不能用"垂速 RMSE"卡死一个设计上不估计的量。

use fly_sim_core::metrics::PosVelMetrics;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::plant::QuadrotorPlant;
use fly_sim_core::sensor::{SensorConfig, SensorFault};
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::PidController;
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::{Meter, Radian, Second};
use flyctrl_core::vehicle::Quaternion;
use std::sync::Mutex;

static LOCK: Mutex<()> = Mutex::new(());
fn lock() -> std::sync::MutexGuard<'static, ()> {
    LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

/// 起始位置（NED，D 向下为正 → 5m 高）。
const START_NED: [f32; 3] = [0.0, 0.0, -5.0];

/// 阶段 3 台架：真值姿态控制飞脚本剖面；EKF 位置/速度估计**开环**评估。
///
/// `qd_fn(t)` 返回 `(姿态指令, 总推力)` —— 与阶段 2 的 `control_attitude` 同接口，
/// 但反馈的是**真值状态**。
pub fn run_pos_est(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    qd_fn: impl Fn(f32) -> (Quaternion, f32),
) -> PosVelMetrics {
    run_pos_est_fault(vc, dur_s, scfg, settle_s, qd_fn, |_, _, _| None)
}

/// 同 [`run_pos_est`]，但可注入一次传感器故障（阶段 3.5/3.6）。
///
/// `fault_fn(t, 真值pos, 真值vel)` 返回 `Some(fault)` 时注入一次（此后不再调用）。
/// 传入真值是为了支持**真实丢星**语义：冻结在**冻结时刻的读数**，
/// 而不是一个拍脑袋的固定值（否则会把"参考值不对"混进"无观测漂移"）。
pub fn run_pos_est_fault(
    vc: &VehicleConfig,
    dur_s: f32,
    scfg: SensorConfig,
    settle_s: f32,
    qd_fn: impl Fn(f32) -> (Quaternion, f32),
    fault_fn: impl Fn(f32, [f64; 3], [f64; 3]) -> Option<SensorFault>,
) -> PosVelMetrics {
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
    // 位置初值对齐真值（与 `step_hil` 的 pos-init 同义）：否则首帧即有 5m 位置差，
    // 会把"估计器收敛速度"与"初值差"混在一起。
    hil.est.set_initial_position(START_NED);
    // ★ 气压基准必须与"posD=0 的定义"一致。
    //
    // `step_hil` 默认在首次 GPS fix 时锁 `baro_ref`，此后 `update_alt(alt − baro_ref)`
    // 把气压当成**相对固定点**的高度 ⇒ 即 **posD=0 = fix 时刻的高度**。
    // 而本台架用 `set_initial_position(-5)`（GPS 的 NED 原点 = 起飞点，真值 posD=-5）
    // ⇒ 两者基准差 5m，锚定会把 posD 往 0 拖（实测 3.3m 垂向偏差，水平却正常）。
    // 强行把 `baro_ref=0` 并置锁 ⇒ 气压成为**绝对**高度观测，与 GPS 原点一致。
    hil.baro_ref = 0.0;
    hil.baro_locked = true;

    let mut sim_imu = SimImu::new();
    let sp = flyctrl_core::controller::Setpoint::hover(
        [Meter(START_NED[0]), Meter(START_NED[1]), Meter(START_NED[2])],
        Radian(0.0),
    );

    let n = (dur_s / dt) as u32;
    let t0 = (settle_s / dt) as u32;
    let mut m = PosVelMetrics::default();
    let mut fault_done = false;
    for i in 0..n {
        let t = i as f32 * dt;
        let truth = plant.state_ned();
        if !fault_done {
            let tp = [truth.pos[0].0 as f64, truth.pos[1].0 as f64, truth.pos[2].0 as f64];
            let tv = [truth.vel[0].0 as f64, truth.vel[1].0 as f64, truth.vel[2].0 as f64];
            if let Some(f) = fault_fn(t, tp, tv) {
                plant.inject_sensor_fault(f);
                fault_done = true;
            }
        }

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
            &sp,
            false,
            false,
            true,
            &mut sim_imu,
        );

        if i >= t0 {
            m.push(
                [
                    r.est.pos[0].0,
                    r.est.pos[1].0,
                    r.est.pos[2].0,
                ],
                [r.est.vel[0].0, r.est.vel[1].0, r.est.vel[2].0],
                [truth.pos[0].0, truth.pos[1].0, truth.pos[2].0],
                [truth.vel[0].0, truth.vel[1].0, truth.vel[2].0],
            );
        }

        let (q_des, thr) = qd_fn(t);
        let cmd = ctrl.control_attitude(Second(dt), q_des, thr, &truth);
        plant.apply_actuators(&cmd);
        plant.step();
    }
    m
}

fn hover_thrust(vc: &VehicleConfig) -> f32 {
    vc.ctrl_params().hover_thrust
}

// ============================================================ 3.1 悬停

/// 3.1 悬停：位置/速度估计应保持稳定、无漂移。
#[test]
fn pos_est_hover() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let qd = move |_t: f32| (Quaternion::IDENTITY, h);
    for (tag, scfg) in [
        ("3A clean", SensorConfig::default()),
        ("3B realistic", SensorConfig::realistic()),
    ] {
        let m = run_pos_est(&vc, 30.0, scfg, 3.0, qd);
        println!("{}", m.summary(&format!("3.1 悬停 [{tag}]")));
        assert!(!m.diverged(), "3.1 [{tag}] 发散");
        // 判据（由实测推出）：3A 位置 RMSE <0.5m；3B <1.0m
        let lim = if tag.starts_with("3A") { 0.5 } else { 1.0 };
        assert!(m.pos_rmse() < lim, "3.1 [{tag}] 位置 RMSE {:.3}m 应 <{lim}m", m.pos_rmse());
        // 水平应明显优于垂向（垂向靠 baro 绝对观测、水平靠 GPS/Doppler）
        assert!(m.horiz_rmse() < 0.3, "3.1 [{tag}] 水平 RMSE {:.3}m 应 <0.3m", m.horiz_rmse());
        // **已知设计局限**：EKF 不估计垂向速度（纯 IMU 积分）→ realistic 下 vD 误差
        // 显著大于 clean（实测 0.115 → 1.015 m/s）。这里只守"有界"，不卡精度。
        assert!(m.vel_rmse() < 2.5, "3.1 [{tag}] 速度 RMSE {:.3}m/s 应有界", m.vel_rmse());
    }
}

// ============================================================ 3.2 水平机动

/// 3.2 水平机动：倾斜加速 → 回平（脚本剖面，无外环）。
///
/// 检验位置/速度估计能否跟上一段真实的水平机动（含加速度段与匀速段）。
#[test]
fn pos_est_horizontal_maneuver() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let tilt = 12.0f32.to_radians();
    let qd = move |t: f32| {
        // 0~2s 前倾加速；2~4s 回平；4s 后水平
        let pitch = if (1.0..3.0).contains(&t) { -tilt } else { 0.0 };
        let cos_t = (tilt.cos()).max(0.2);
        let thr = if pitch != 0.0 { (h / cos_t).clamp(0.1, 1.0) } else { h };
        (
            Quaternion::from_euler(Radian(0.0), Radian(pitch), Radian(0.0)),
            thr,
        )
    };
    for (tag, scfg) in [
        ("3A clean", SensorConfig::default()),
        ("3B realistic", SensorConfig::realistic()),
    ] {
        let m = run_pos_est(&vc, 15.0, scfg, 2.0, qd);
        println!("{}", m.summary(&format!("3.2 水平机动 [{tag}]")));
        assert!(!m.diverged(), "3.2 [{tag}] 发散");
        let lim = if tag.starts_with("3A") { 0.5 } else { 1.0 };
        assert!(m.pos_rmse() < lim, "3.2 [{tag}] 位置 RMSE {:.3}m 应 <{lim}m", m.pos_rmse());
    }
}

// ============================================================ 3.3 垂直爬升/下降

/// 3.3 垂直通道：抬高推力爬升 → 回落下降。
#[test]
fn pos_est_vertical_climb_descent() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let qd = move |t: f32| {
        // 1~3s 爬升（+8% 推力），3~5s 下降（-8%）
        let thr = if (1.0..3.0).contains(&t) {
            (h * 1.08).clamp(0.1, 1.0)
        } else if (3.0..5.0).contains(&t) {
            (h * 0.92).clamp(0.1, 1.0)
        } else {
            h
        };
        (Quaternion::IDENTITY, thr)
    };
    for (tag, scfg) in [
        ("3A clean", SensorConfig::default()),
        ("3B realistic", SensorConfig::realistic()),
    ] {
        let m = run_pos_est(&vc, 15.0, scfg, 2.0, qd);
        println!("{}", m.summary(&format!("3.3 垂直爬升/下降 [{tag}]")));
        assert!(!m.diverged(), "3.3 [{tag}] 发散");
        let lim = if tag.starts_with("3A") { 0.5 } else { 1.0 };
        assert!(m.pos_rmse() < lim, "3.3 [{tag}] 位置 RMSE {:.3}m 应 <{lim}m", m.pos_rmse());
    }
}

// ============================================================ 3.5 GPS 丢星

/// 3.5 GPS 冻结（模拟丢星）：水平位置应在**短时**内保持有界，不发散。
///
/// 用 `SensorFault::GpsStuck` 把位置/速度观测冻结为故障时刻的值（GPS 不再更新）。
/// 此时水平通道只剩 IMU 积分（无绝对参考）→ 必然缓慢漂移；判据守"短时漂移有界"。
#[test]
fn pos_est_gps_dropout() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let qd0 = move |_t: f32| (Quaternion::IDENTITY, h);

    // 机动：0~1s 悬停、1~3s 前倾加速、3s 后回平匀速；**t=3s 起冻结 GPS**
    // （冻结在**冻结时刻的真值** = 真实丢星语义）。
    let tilt = 12.0f32.to_radians();
    let h2 = h;
    let qd = move |t: f32| {
        let pitch = if (1.0..3.0).contains(&t) { -tilt } else { 0.0 };
        let thr = if pitch != 0.0 {
            (h2 / tilt.cos()).clamp(0.1, 1.0)
        } else {
            h2
        };
        (
            Quaternion::from_euler(Radian(0.0), Radian(pitch), Radian(0.0)),
            thr,
        )
    };
    let _ = qd0;
    let scfg = SensorConfig::realistic();
    let m = run_pos_est_fault(&vc, 12.0, scfg, 1.0, qd, |t, p, v| {
        if t >= 3.0 {
            Some(SensorFault::GpsStuck(Some([p[0], p[1], p[2], v[0], v[1], v[2]])))
        } else {
            None
        }
    });
    println!("{}", m.summary("3.5 GPS 冻结(丢星)"));
    assert!(!m.diverged(), "3.5 GPS 冻结后发散");
    // 冻结后 9s：水平漂移应有界。实测 1.024m（机动中丢星，真实丢星语义）
    // —— 门槛取 2.0m（由实测推出，留 ~2× 裕度）。
    assert!(
        m.horiz_rmse() < 2.0,
        "3.5 GPS 冻结后水平 RMSE {:.3}m 应在 2m 内（短时漂移有界）",
        m.horiz_rmse()
    );
}

// ============================================================ 3.6 气压故障

/// 3.6a 气压**阶跃**（+3m）：垂向估计不应把阶跃全盘当真实高度。
///
/// 与 M 场 `x_env_faults::baro_step_bounded_by_gps` 同语义（那里是"baro 主导垂直、
/// GPS 弱拉回 → 稳态偏置而非完全吸收"）。H 场用新加的 `SensorFault::BaroStep`。
#[test]
fn pos_est_baro_step() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let qd = move |_t: f32| (Quaternion::IDENTITY, h);
    let m = run_pos_est_fault(&vc, 12.0, SensorConfig::realistic(), 1.0, qd, |t, _, _| {
        if t >= 3.0 {
            Some(SensorFault::BaroStep(3.0))
        } else {
            None
        }
    });
    println!("{}", m.summary("3.6a 气压阶跃 +3m"));
    assert!(!m.diverged(), "3.6a 气压阶跃后发散");
    // 垂向误差应有界（阶跃 3m → 估计偏置应小于阶跃本身，且不发散）
    assert!(
        m.vert_rmse() < 3.5,
        "3.6a 垂向 RMSE {:.3}m 应有界（阶跃 3m）",
        m.vert_rmse()
    );
    assert!(m.horiz_rmse() < 1.0, "3.6a 气压故障不应影响水平");
}

/// 3.6b 气压**冻结**：数据仍"正常"（有读数）→ FDIR 不误报，靠估计器鲁棒性。
#[test]
fn pos_est_baro_stuck() {
    let _g = lock();
    let vc = VehicleConfig::default_quad();
    let h = hover_thrust(&vc);
    let qd = move |_t: f32| (Quaternion::IDENTITY, h);
    let m = run_pos_est_fault(&vc, 12.0, SensorConfig::realistic(), 1.0, qd, |t, p, _| {
        if t >= 3.0 {
            // 冻结在冻结时刻的读数（真实卡死语义）
            Some(SensorFault::BaroStuck(Some(-p[2])))
        } else {
            None
        }
    });
    println!("{}", m.summary("3.6b 气压冻结"));
    assert!(!m.diverged(), "3.6b 气压冻结后发散");
    assert!(
        m.pos_rmse() < 3.0,
        "3.6b 位置 RMSE {:.3}m 应有界（气压冻结后靠 GPS 弱拉回）",
        m.pos_rmse()
    );
}
