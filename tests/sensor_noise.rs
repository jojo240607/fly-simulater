//! P3-A2 噪声鲁棒性验收：`SensorConfig::realistic()`（= `--sensor-noise`）下，
//! 控制律必须保持 **TRU 真值有界**——不只 EST≈TRU，物理轨迹本身不许发散。
//!
//! 背景（PLAN 阶段 11 / 11-A，实测）：
//! - 根因定位：噪声消融（`zz_noise_ablation`）证明 **IMU 陀螺噪声/偏置是最小触发集**——
//!   单独叠加即让姿态环/电机指令饱和（GPS 位置外环次之），是"IMU 姿态抖动"主导而非 GPS。
//! - 控制律修复：PID 变体曾包一层 INDI 角加速度反馈，其有限差分（k≈I/dt=12.5）
//!   把陀螺噪声放大成饱和的非对称电机指令而翻滚发散；改纯 PID 后 realistic 噪声 +
//!   无地面约束下悬停稳定（TRU 有界）。
//! - EKF 无缺陷：噪声下 EST 仍贴合 TRU，发散的是物理真值本身（被控制律噪声放大）。
//!
//! 验收判据（FIDELITY_ROADMAP）：收敛判定必须同时断言 TRU 有界（不只 EST≈TRU）。
//!
//! 运行：
//!   cargo test --test sensor_noise                 # ToyWorld（默认）
//!   cargo test --features phy --test sensor_noise  # PhySdkWorld

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{ContactModel, RigidBodyWorld, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian};

#[cfg(feature = "phy")]
use fly_sim_core::physics::PhySdkWorld;

fn make_world() -> impl RigidBodyWorld + 'static {
    #[cfg(feature = "phy")]
    {
        PhySdkWorld::create_empty()
    }
    #[cfg(not(feature = "phy"))]
    {
        ToyWorld::new(9.81)
    }
}

/// 稳态悬停轨迹统计（TRU 真值）。
struct HoverStats {
    /// 采样窗口内 TRU 高度 d 范围（m，-5 为设定点）。
    d_min: f64,
    d_max: f64,
    /// 采样窗口内 TRU 水平漂移最大值（m）。
    h_max: f64,
    /// 采样窗口内 TRU 姿态偏离水平最大值（deg，2·acos(w)，NED 单位四元数=水平）。
    tilt_max_deg: f64,
    /// 是否出现 NaN/Inf。
    nan: bool,
    /// 结束时 TRU 高度 d。
    end_d: f64,
    /// 结束时 EST−TRU 位置误差（m，EKF 是否贴合真值）。
    est_err: [f64; 3],
}

impl Default for HoverStats {
    fn default() -> Self {
        Self {
            d_min: f64::MAX,
            d_max: f64::MIN,
            h_max: 0.0,
            tilt_max_deg: 0.0,
            nan: false,
            end_d: 0.0,
            est_err: [0.0; 3],
        }
    }
}

/// 在 realistic 噪声下跑稳态悬停（`sp = (0,0,-5)`），统计 TRU 真值轨迹。
fn run_steady(contact: Option<ContactModel>, secs: f64) -> HoverStats {
    let world = make_world();
    let cfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut sim = SimLoop::new(
        world,
        &cfg,
        dt,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        contact,
        Vec::new(),
    );
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (secs / dt) as u64;
    let mut st = HoverStats::default();
    for i in 0..steps {
        let truth = sim.step_frame(&sp).0;
        let d = truth.pos[2].0 as f64;
        let h = (truth.pos[0].0 as f64).hypot(truth.pos[1].0 as f64);
        // 姿态偏离水平 = att 相对单位四元数（水平）的旋转角 = 2·acos(w)。
        let tilt = 2.0 * (truth.att.w as f64).clamp(-1.0, 1.0).acos().to_degrees();
        if i % 25 == 0 {
            st.d_min = st.d_min.min(d);
            st.d_max = st.d_max.max(d);
            st.h_max = st.h_max.max(h);
            st.tilt_max_deg = st.tilt_max_deg.max(tilt);
        }
        if !d.is_finite() || !h.is_finite() {
            st.nan = true;
            return st;
        }
    }
    let (truth, _) = sim.snapshot();
    let est = sim.ctrl_debug_estimate();
    st.end_d = truth.pos[2].0 as f64;
    for k in 0..3 {
        st.est_err[k] = (est.pos[k].0 - truth.pos[k].0).abs() as f64;
    }
    st
}

fn assert_tru_bounded(st: &HoverStats, label: &str, max_h: f64) {
    assert!(!st.nan, "[{label}] TRU 出现 NaN/Inf");
    assert!(
        st.end_d.abs() < 10.0,
        "[{label}] TRU 高度 runaway：end d={:.1}（期望≈-5）",
        st.end_d
    );
    assert!(
        st.h_max < max_h,
        "[{label}] TRU 水平漂移过大：h_max={:.1}m（期望 < {max_h}）",
        st.h_max
    );
    assert!(
        st.tilt_max_deg < 45.0,
        "[{label}] TRU 姿态翻滚：max tilt={:.1}°（期望 < 45°，避免翻机）",
        st.tilt_max_deg
    );
}

/// P3-A2 核心验收：realistic 噪声 + **无地面约束**（最严苛，无掩盖）。
/// 纯 PID 控制律应保持 TRU 高度/水平/姿态全部有界。
#[test]
fn hover_realistic_none_contact_bounded() {
    let st = run_steady(None, 40.0);
    println!(
        "hover(none-contact) d=[{:.1},{:.1}] end={:.1} h_max={:.1} tilt_max={:.1}° est_err={:.2}/{:.2}/{:.2}",
        st.d_min, st.d_max, st.end_d, st.h_max, st.tilt_max_deg, st.est_err[0], st.est_err[1], st.est_err[2]
    );
    assert_tru_bounded(&st, "none-contact", 15.0);
}

/// realistic 噪声 + 默认地面约束（P1-2 掩盖态也应稳定，不 runaway）。
#[test]
fn hover_realistic_contact_bounded() {
    let st = run_steady(Some(ContactModel::default()), 30.0);
    println!(
        "hover(contact)    d=[{:.1},{:.1}] end={:.1} h_max={:.1} tilt_max={:.1}° est_err={:.2}/{:.2}/{:.2}",
        st.d_min, st.d_max, st.end_d, st.h_max, st.tilt_max_deg, st.est_err[0], st.est_err[1], st.est_err[2]
    );
    assert_tru_bounded(&st, "contact", 15.0);
}

/// 噪声下 EKF 必须贴合真值：EST 位置误差有界（证明发散的是控制律放大而非估计器）。
#[test]
fn est_tracks_truth_under_noise() {
    let st = run_steady(None, 40.0);
    println!("est_err = {:.2}/{:.2}/{:.2} m", st.est_err[0], st.est_err[1], st.est_err[2]);
    assert!(!st.nan, "TRU 出现 NaN/Inf");
    for k in 0..3 {
        assert!(
            st.est_err[k] < 5.0,
            "EST 偏离 TRU 过大：est_err[{k}]={:.2}m（期望 < 5m，EKF 应贴合真值）",
            st.est_err[k]
        );
    }
}
