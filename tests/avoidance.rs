//! 反应式避障闭环集成测试（P1-2）。
//!
//! 设计要点：
//! - 机体在水平悬停下保持稳定（已知悬停稳定，见 `headless_hover_wind::hover_stability`），
//!   避免依赖"前向飞行"——前向飞行在当前控制律下会掉高发散（阶段结论），会让测距射线
//!   随姿态倾斜而失效，无法稳定触发避障。
//! - 改为让**障碍匀速逼近静止悬停的机体**：机体前向测距（机体 -X → NED 南向）在障碍进入
//!   危险距离时读出有效距离，触发"制动 + 横向闪避"速度指令，使机体向东侧移脱离航线；
//!   未装备避障的基准机体仍原地悬停，被逼近的障碍碰撞。
//! - 度量：模拟全程机体到障碍球面的最小净间隙（3D 距离 - 半径）。避障使间隙保持为正，
//!   基准则被侵入（间隙趋近 / 跌破 0）。

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{DynamicObstacle, Obstacle, ToyWorld, ContactModel};
use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig};
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;

/// 障碍基准中心（引擎世界系，Y-up：x=北, y=上, z=-东）。
/// 机体悬停在 (0, 5, 0)，前向（机体 -X）指向 NED 南（引擎 -x），故障碍放在机体南侧
/// （引擎 x 为负）才能在水平悬停时进入测距射线。
const OBSTACLE_BASE: [f64; 3] = [-26.0, 5.0, 0.0];
const OBSTACLE_RADIUS: f64 = 3.0;
/// 障碍以 2.0 m/s 沿 +x（北向）逼近机体。
const OBSTACLE_SPEED: f64 = 2.0;

/// 跑一个逼近场景，返回全程机体到障碍球面的最小净间隙（m，正=未接触）。
///
/// `with_avoid`：是否装备前向测距 + 反应式避障。
fn run_approach(with_avoid: bool, seconds: f64) -> f64 {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        vec![], // 静态障碍不放，全部走动态障碍
    );

    // 匀速逼近的动态球：基准在机体南侧 26m，向北 2.0 m/s 推进。
    let dyn_obs = DynamicObstacle {
        base: Obstacle::Sphere { center: OBSTACLE_BASE, radius: OBSTACLE_RADIUS },
        velocity: [OBSTACLE_SPEED, 0.0, 0.0],
    };
    ctrl.plant_set_dynamic_obstacles(vec![dyn_obs]);

    if with_avoid {
        // 量程 12m、危险距离 11m（< 量程，避免全量程误触发）；横向闪避 2.0 m/s。
        let ranger = RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0xABCD);
        ctrl.configure_avoidance(ranger, AvoidanceConfig::new(11.0, 1.5, 2.0));
    }

    // 机体原地悬停（稳定），不主动飞向障碍——障碍自己逼近。
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);

    let steps = (seconds / DT) as u64;
    let mut min_clear = f64::INFINITY;
    for i in 0..steps {
        ctrl.step(&hover_sp);
        let t = (i as f64) * DT;
        let center = [
            OBSTACLE_BASE[0] + OBSTACLE_SPEED * t,
            OBSTACLE_BASE[1],
            OBSTACLE_BASE[2],
        ];
        let (pos, _) = ctrl.debug_up();
        let dist = ((pos[0] - center[0]).powi(2)
            + (pos[1] - center[1]).powi(2)
            + (pos[2] - center[2]).powi(2))
        .sqrt();
        let clear = dist - OBSTACLE_RADIUS;
        if clear < min_clear {
            min_clear = clear;
        }
    }
    min_clear
}

#[test]
fn avoidance_keeps_greater_clearance_than_bare() {
    // 障碍从 26m 外逼近，约 13s 到达机体；留足时间让避障触发并侧移脱离。
    let gap_bare = run_approach(false, 14.0);
    let gap_av = run_approach(true, 14.0);

    // 基准：障碍抵达并侵入机体，净间隙应跌破 0（或极接近 0）。
    assert!(gap_bare < 0.5, "bare case should be contacted, gap_bare={}", gap_bare);
    // 避障：横向闪避应使机体全程保持与障碍的安全间隙（明显为正）。
    assert!(gap_av > 0.5, "avoidance should keep clearance, gap_av={}", gap_av);
    // 核心断言：避障净间隙显著大于基准。
    assert!(
        gap_av > gap_bare + 1.0,
        "avoidance clearance ({}) must exceed bare ({}) by >1m",
        gap_av,
        gap_bare
    );
}

#[test]
fn avoid_velocity_triggers_when_close() {
    // 单元级：危险距离内触发、外不触发；无效读数不触发。
    use fly_sim_core::sensor::{RangeFinderSample, AvoidanceConfig};
    let cfg = AvoidanceConfig::new(5.0, 1.5, 0.8);
    let fwd = [-1.0, 0.0, 0.0];
    let right = [0.0, 1.0, 0.0];

    let near = RangeFinderSample { distance: 3.0, valid: true };
    let (v, trig) = cfg.avoidance_velocity(&near, fwd, right);
    assert!(trig, "should trigger within danger distance");
    assert!(v[1] > 0.0, "should add lateral (right) evasion");

    let far = RangeFinderSample { distance: 9.0, valid: true };
    let (_, trig2) = cfg.avoidance_velocity(&far, fwd, right);
    assert!(!trig2, "should not trigger beyond danger distance");

    let invalid = RangeFinderSample { distance: 12.0, valid: false };
    let (_, trig3) = cfg.avoidance_velocity(&invalid, fwd, right);
    assert!(!trig3, "should not trigger on invalid reading");
}

#[test]
fn avoid_ranger_detects_obstacle_in_fov() {
    // 单元级：前向射线应命中正前方的球，返回有效读数。
    use fly_sim_core::sensor::RangeFinderModel;
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        vec![Obstacle::Sphere { center: [-8.0, 5.0, 0.0], radius: 3.0 }],
    );
    ctrl.set_ranger(Some(RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0xABCD)));
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..50 {
        ctrl.step(&hover_sp);
    }
    let s = ctrl.dbg_ranger().expect("ranger present");
    // 机体在 (0,5,0)，球心 (-8,5,0) r=3，表面距 5m，应在量程内且有效。
    assert!(s.valid, "ranger should see the forward sphere");
    assert!((s.distance - 5.0).abs() < 0.5, "ranger distance ~5m, got {}", s.distance);
}

#[test]
fn avoid_no_false_trigger_when_clear() {
    // 单元级：前方无障（量程外）时测距无效，避障不介入、不影响悬停。
    use fly_sim_core::sensor::{AvoidanceConfig, RangeFinderModel};
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        vec![Obstacle::Sphere { center: [-20.0, 5.0, 0.0], radius: 3.0 }],
    );
    let ranger = RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0xABCD);
    ctrl.configure_avoidance(ranger, AvoidanceConfig::new(11.0, 1.5, 2.0));
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..250 {
        ctrl.step(&hover_sp);
    }
    let (pos, q) = ctrl.debug_up();
    // 悬停应保持在 ~5m 高度、姿态接近水平（qw≈cos45≈0.707）。
    assert!(pos[1] > 4.0, "hover altitude should stay ~5m, got {}", pos[1]);
    assert!((q[0] - std::f64::consts::FRAC_PI_4.cos()).abs() < 0.15, "should stay level");
}
