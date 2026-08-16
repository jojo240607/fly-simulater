//! P1-2 闭环联动：障碍碰撞 + 避障控制器闭环验证（无头）。
//!
//! 两层验证：
//! 1) 单元层：[`AvoidanceConfig::avoidance_velocity`] 的触发/不触发/失效语义。
//! 2) 闭环层：装备前向测距 + 反应式避障的机体，朝障碍飞行时的最近逼近距离，
//!    应明显大于"裸飞（无避障）"的对照，证明其感知→决策→规避闭环有效。
//!
//! 用法：cargo test --test avoidance -- --nocapture

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{Obstacle, ToyWorld, ContactModel};
use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig};
use fly_simulater::airframe::load_airframe;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, Radian};

const DT: f64 = 0.004;

// ============================================================ 单元层

#[test]
fn avoid_velocity_triggers_when_close() {
    let cfg = AvoidanceConfig::new(4.0, 1.0, 0.5);
    let fwd = [1.0, 0.0, 0.0]; // 任意前向（单位向量）
    let right = [0.0, 1.0, 0.0];
    // 障碍在 2m（< danger 4m）：触发，制动沿 -fwd，横向沿 +right
    let s = fly_sim_core::sensor::RangeFinderSample { distance: 2.0, valid: true };
    let (v, trig) = cfg.avoidance_velocity(&s, fwd, right);
    assert!(trig, "近障碍应触发避障");
    // 制动分量 = severity(0.5)*brake(1.0)*danger(4.0)=2.0，沿 -fwd
    assert!((v[0] + 2.0).abs() < 1e-9, "前向制动应=-2.0，实得 {}", v[0]);
    assert!((v[1] - 0.5).abs() < 1e-9, "横向闪避应=+0.5，实得 {}", v[1]);
}

#[test]
fn avoid_velocity_silent_when_far() {
    let cfg = AvoidanceConfig::new(4.0, 1.0, 0.5);
    let fwd = [1.0, 0.0, 0.0];
    let right = [0.0, 1.0, 0.0];
    // 障碍在 6m（> danger 4m）：不触发
    let s = fly_sim_core::sensor::RangeFinderSample { distance: 6.0, valid: true };
    let (v, trig) = cfg.avoidance_velocity(&s, fwd, right);
    assert!(!trig, "远障碍不应触发避障");
    assert_eq!(v, [0.0, 0.0, 0.0]);
}

#[test]
fn avoid_velocity_conservative_on_invalid() {
    let cfg = AvoidanceConfig::new(4.0, 1.0, 0.5);
    let fwd = [1.0, 0.0, 0.0];
    let right = [0.0, 1.0, 0.0];
    // 失效读数（瞬断/近距盲区/量程饱和）：不介入，避免凭空闪避
    let s = fly_sim_core::sensor::RangeFinderSample { distance: 10.0, valid: false };
    let (v, trig) = cfg.avoidance_velocity(&s, fwd, right);
    assert!(!trig, "失效读数不应触发避障（保守不动作）");
    assert_eq!(v, [0.0, 0.0, 0.0]);
}

// ============================================================ 闭环层

/// 跑一个朝障碍飞行的闭环，返回 (最近逼近距离到障碍表面, 全程有限)。
///
/// 机体初始在引擎 (0,5,0)，前方（引擎 -X）= 障碍方向。障碍为球心 (-8,5,0) r=3，
/// 表面在 x=-5，距机体初始 5m。设定点指令 `vx_ned`（负值=朝障碍）。
/// `with_avoid`=true 时装备测距+避障。
fn run_toward_obstacle(with_avoid: bool, vx_ned: f64, seconds: f64) -> (f64, bool) {
    let cfg = load_airframe(None).expect("default airframe");
    let obstacle = Obstacle::Sphere { center: [-8.0, 5.0, 0.0], radius: 3.0 };
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        vec![obstacle],
    );
    if with_avoid {
        // 量程 12m > danger 4m；盲区 0.5m；无噪声/瞬断，保证稳定触发。
        let ranger = RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0xABCD);
        let av = AvoidanceConfig::new(4.0, 1.5, 0.8);
        ctrl.configure_avoidance(ranger, av);
    }

    // 设定点：高度 -5（NED），给定向前速度（朝障碍=负北向）。
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        yaw: Radian(0.0),
        vel: [MeterPerSecond(vx_ned as f32), MeterPerSecond(0.0), MeterPerSecond(0.0)],
    };
    let total = (seconds / DT) as u64;
    let mut min_gap = f64::INFINITY; // 机体 x - 障碍表面 x(-5)，越小越危险
    let mut finite = true;

    for _ in 0..total {
        let _st = ctrl.step(&sp);
        let (pos, quat) = ctrl.debug_up();
        if !quat[0].is_finite() {
            finite = false;
            break;
        }
        let gap = pos[0] - (-5.0); // 障碍表面在 x=-5
        if gap < min_gap {
            min_gap = gap;
        }
        if min_gap < -0.05 {
            // 已穿过障碍表面（数值上碰上），提前结束统计
            break;
        }
    }
    (min_gap, finite)
}

#[test]
fn avoidance_keeps_greater_clearance_than_bare() {
    // 朝障碍以 1.5 m/s 飞行 4 秒。
    let (gap_bare, ok_bare) = run_toward_obstacle(false, -1.5, 4.0);
    let (gap_av, ok_av) = run_toward_obstacle(true, -1.5, 4.0);

    println!(
        "[avoid] bare min_gap={:.3}m (finite={}) | avoidance min_gap={:.3}m (finite={})",
        gap_bare, ok_bare, gap_av, ok_av
    );

    assert!(ok_bare && ok_av, "两种配置都应数值稳定");
    // 避障闭环应让机体在更远离障碍处停下/转向：最近逼近距离明显更大。
    assert!(
        gap_av > gap_bare + 0.5,
        "避障应比裸飞留出更大安全间隙：avoid={:.3} bare={:.3}",
        gap_av, gap_bare
    );
    // 避障下不应撞击障碍（gap 不穿过表面进入负值过深）。
    assert!(gap_av > -0.05, "避障下机体不应穿过障碍表面，gap={:.3}", gap_av);
}

#[test]
fn avoidance_no_false_trigger_when_no_obstacle() {
    // 无避障装备（默认）时 run_toward_obstacle(false) 已覆盖裸飞；
    // 这里验证"装备了避障但前方无障"也不应产生异常横向漂移导致发散。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(), // 无障
    );
    let ranger = RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0xBEEF);
    let av = AvoidanceConfig::new(4.0, 1.5, 0.8);
    ctrl.configure_avoidance(ranger, av);

    // 悬停设定点（无前进指令）：无障时不应有任何介入。
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let total = (3.0 / DT) as u64;
    let mut finite = true;
    for _ in 0..total {
        let _st = ctrl.step(&sp);
        let (_pos, quat) = ctrl.debug_up();
        if !quat[0].is_finite() {
            finite = false;
            break;
        }
    }
    assert!(finite, "无障时装备避障不应导致发散");
}
