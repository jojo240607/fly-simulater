//! P3-A2 噪声鲁棒性验收：`SensorConfig::realistic()`（= `--sensor-noise`）下，
//! 控制律必须保持 **TRU 真值有界**——不只 EST≈TRU，物理轨迹本身不许发散。
//!
//! 背景（PLAN 阶段 11 / 11-A，实测）：
//! - 根因定位：噪声消融诊断证明 **IMU 陀螺噪声/偏置是最小触发集**——
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

mod common;
use common::{assert_tru_bounded, TruStats};

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

/// 稳态悬停轨迹统计：TRU 真值（复用共享断言工具）+ EKF 贴合度。
struct HoverStats {
    /// TRU 真值统计（nan/水平漂移/高度范围/倾角，见 `common::assert_tru_bounded`）。
    tru: TruStats,
    /// 结束时 EST−TRU 位置误差（m，EKF 是否贴合真值）。
    est_err: [f64; 3],
}

impl Default for HoverStats {
    fn default() -> Self {
        Self {
            tru: TruStats::default(),
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
            st.tru.sample(h, d, tilt);
        }
        if st.tru.nan {
            return st;
        }
    }
    let (truth, _) = sim.snapshot();
    let est = sim.ctrl_debug_estimate();
    st.tru.end_d = truth.pos[2].0 as f64;
    for k in 0..3 {
        st.est_err[k] = (est.pos[k].0 - truth.pos[k].0).abs() as f64;
    }
    st
}

/// P3-A2 核心验收：realistic 噪声 + **无地面约束**（最严苛，无掩盖）。
/// 纯 PID 控制律应保持 TRU 高度/水平/姿态全部有界。
#[test]
fn hover_realistic_none_contact_bounded() {
    let st = run_steady(None, 40.0);
    println!(
        "hover(none-contact) d=[{:.1},{:.1}] end={:.1} h_max={:.1} tilt_max={:.1}° est_err={:.2}/{:.2}/{:.2}",
        st.tru.d_min, st.tru.d_max, st.tru.end_d, st.tru.h_max, st.tru.tilt_max_deg,
        st.est_err[0], st.est_err[1], st.est_err[2]
    );
    assert_tru_bounded(&st.tru, "none-contact", -5.0, 15.0, 45.0);
}

/// realistic 噪声 + 默认地面约束（P1-2 掩盖态也应稳定，不 runaway）。
#[test]
fn hover_realistic_contact_bounded() {
    let st = run_steady(Some(ContactModel::default()), 30.0);
    println!(
        "hover(contact)    d=[{:.1},{:.1}] end={:.1} h_max={:.1} tilt_max={:.1}° est_err={:.2}/{:.2}/{:.2}",
        st.tru.d_min, st.tru.d_max, st.tru.end_d, st.tru.h_max, st.tru.tilt_max_deg,
        st.est_err[0], st.est_err[1], st.est_err[2]
    );
    assert_tru_bounded(&st.tru, "contact", -5.0, 15.0, 45.0);
}

/// 噪声下 EKF 必须贴合真值：EST 位置误差有界（证明发散的是控制律放大而非估计器）。
#[test]
fn est_tracks_truth_under_noise() {
    let st = run_steady(None, 40.0);
    println!("est_err = {:.2}/{:.2}/{:.2} m", st.est_err[0], st.est_err[1], st.est_err[2]);
    assert!(!st.tru.nan, "TRU 出现 NaN/Inf");
    for k in 0..3 {
        assert!(
            st.est_err[k] < 5.0,
            "EST 偏离 TRU 过大：est_err[{k}]={:.2}m（期望 < 5m，EKF 应贴合真值）",
            st.est_err[k]
        );
    }
}
