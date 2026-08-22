//! 反应式避障闭环集成测试（P1-2 / P3-B2 多射线）。
//!
//! 设计要点：
//! - 机体在水平悬停下保持稳定（已知悬停稳定，见 `headless_hover_wind::hover_stability`），
//!   避免依赖"前向飞行"——前向飞行在当前控制律下会掉高发散（阶段结论），会让测距射线
//!   随姿态倾斜而失效，无法稳定触发避障。
//! - 改为让**障碍匀速逼近静止悬停的机体**：机体前向测距（机体 -X → NED 南向）在障碍进入
//!   危险距离时读出有效距离，触发"制动 + 横向闪避"速度指令，使机体向一侧侧移脱离航线；
//!   未装备避障的基准机体仍原地悬停，被逼近的障碍碰撞。
//! - P3-B2：测距改用**扇形多射线**（±60° × 5 条），决策按多射线聚合（排斥力求和的
//!   制动/横向分量，闪避方向随障碍横移连续翻转）。相比单射线 + `hold_time` 硬补：
//!   障碍横向滑出中央射线后侧向射线仍持续覆盖，无需"FOV 丢失后冻结指令 N 秒"，
//!   障碍彻底离开视场后避障自然释放。
//! - 度量：模拟全程机体到障碍球面的最小净间隙（3D 距离 - 半径）。避障使间隙保持为正，
//!   基准则被侵入（间隙趋近 / 跌破 0）。
use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{DynamicObstacle, Obstacle, ToyWorld};
use fly_sim_core::sensor::{
    AvoidanceConfig, RangeFinderFrame, RangeFinderModel, RayReading, SensorConfig,
};
use fly_simulater::airframe::load_airframe;

mod common;
use common::{assert_tru_bounded, TruStats};

const DT: f64 = 0.004;

/// 障碍基准中心（引擎世界系，Y-up：x=北, y=上, z=-东）。
/// 机体悬停在 (0, 5, 0)，前向（机体 -X）指向 NED 南（引擎 -x），故障碍放在机体南侧
/// （引擎 x 为负）才能在水平悬停时进入测距射线。
const OBSTACLE_BASE: [f64; 3] = [-26.0, 5.0, 0.0];
const OBSTACLE_RADIUS: f64 = 3.0;
/// 障碍以 2.0 m/s 沿 +x（北向）逼近机体。
const OBSTACLE_SPEED: f64 = 2.0;

/// 避障装备（`None` = 基准：不装测距/避障）。传 `&` 以便同一配置复用给多个场景。
type AvSetup = (RangeFinderModel, AvoidanceConfig);

/// P3-B2 多射线装备：±60° 半视场角扇形 × 5 条射线；量程 12m、危险距离 11m
/// （< 量程，避免全量程误触发）；横向闪避 2.0 m/s。
fn multi_ray_setup() -> AvSetup {
    (
        RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, std::f64::consts::PI / 3.0, 5, 0xABCD),
        AvoidanceConfig::new(11.0, 0.0, 2.0),
    )
}

/// P3-B2 对照装备：单条正前方射线（fov=0、ray_count=1 退化为单射线，即旧语义）。
fn single_ray_setup() -> AvSetup {
    (
        RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1, 0xABCD),
        AvoidanceConfig::new(11.0, 0.0, 2.0),
    )
}

/// 机体推力轴（机体 +Z）偏离世界竖直 (0,1,0) 的倾角（度），口径同 tecs/rc_modes。
fn tilt_deg(q: [f64; 4]) -> f64 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let v = [0.0f64, 0.0, 1.0];
    let qv = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
    let qqv = [
        y * qv[2] - z * qv[1],
        z * qv[0] - x * qv[2],
        x * qv[1] - y * qv[0],
    ];
    let up = [
        v[0] + 2.0 * w * qv[0] + 2.0 * qqv[0],
        v[1] + 2.0 * w * qv[1] + 2.0 * qqv[1],
        v[2] + 2.0 * w * qv[2] + 2.0 * qqv[2],
    ];
    let dot = up[1].clamp(-1.0, 1.0); // 与世界 +Y 点积
    f64::acos(dot).to_degrees()
}

/// 跑一个正对逼近场景，返回 (全程机体到障碍球面的最小净间隙(m), TRU 真值有界统计)。
///
/// `setup`：装备的测距 + 避障配置；`None` = 不装备（基准）。
fn run_approach(setup: Option<&AvSetup>, seconds: f64) -> (f64, TruStats) {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        // 场景测试默认真实噪声（FIDELITY_ROADMAP 收尾项：默认零噪声会屏蔽 EKF/控制律噪声行为）。
        SensorConfig::realistic(),
        ControllerKind::Pid,
        None,
        vec![], // 静态障碍不放，全部走动态障碍
    );

    // 匀速逼近的动态球：基准在机体南侧 26m，向北 2.0 m/s 推进。
    let dyn_obs = DynamicObstacle {
        base: Obstacle::Sphere { center: OBSTACLE_BASE, radius: OBSTACLE_RADIUS },
        velocity: [OBSTACLE_SPEED, 0.0, 0.0],
    };
    ctrl.plant_set_dynamic_obstacles(vec![dyn_obs]);

    if let Some((ranger, av)) = setup {
        ctrl.configure_avoidance(ranger.clone(), av.clone());
    }

    // 机体原地悬停（稳定），不主动飞向障碍——障碍自己逼近。
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);

    let steps = (seconds / DT) as u64;
    let mut min_clear = f64::INFINITY;
    let mut tru = TruStats::default();
    for i in 0..steps {
        ctrl.step(&hover_sp);
        let (pos, quat) = ctrl.debug_up();
        // TRU 真值归一化（引擎 Y-up → NED：h=hypot(x,z)、d=-y、tilt）。
        tru.sample(pos[0].hypot(pos[2]), -pos[1], tilt_deg(quat));
        let t = (i as f64) * DT;
        let center = [
            OBSTACLE_BASE[0] + OBSTACLE_SPEED * t,
            OBSTACLE_BASE[1],
            OBSTACLE_BASE[2],
        ];
        let dist = ((pos[0] - center[0]).powi(2)
            + (pos[1] - center[1]).powi(2)
            + (pos[2] - center[2]).powi(2))
        .sqrt();
        let clear = dist - OBSTACLE_RADIUS;
        if clear < min_clear {
            min_clear = clear;
        }
    }
    (min_clear, tru)
}

/// 跑一个横向偏置逼近场景（P3-B2 验收），返回
/// (最小净间隙(m), 末帧引擎 z 横向位置(m), TRU 真值有界统计)。
///
/// 球体沿 +x（北向）匀速逼近，但恒定偏出中央射线 `z_off`（引擎 z，世界 +z = NED 东）：
/// - `z_off` 略大于球半径(3m)时，单条中央射线（fov=0）到球心距离恒 > 半径 → 永远打不中
///   → 避障完全不触发 → 机体原地被"擦身而过"（净间隙≈z_off-3，近距擦碰）；
/// - 扇形多射线则从侧向射线早期命中 → 持续侧移闪避 → 净间隙显著为正；
///   障碍越过机体并离开视场后避障自然释放，位置环把机体拉回原点（末帧 z≈0）。
fn run_lateral_slide(setup: Option<&AvSetup>, z_off: f64, seconds: f64) -> (f64, f64, TruStats) {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        None,
        vec![],
    );

    let dyn_obs = DynamicObstacle {
        base: Obstacle::Sphere { center: [-26.0, 5.0, z_off], radius: OBSTACLE_RADIUS },
        velocity: [OBSTACLE_SPEED, 0.0, 0.0],
    };
    ctrl.plant_set_dynamic_obstacles(vec![dyn_obs]);

    if let Some((ranger, av)) = setup {
        ctrl.configure_avoidance(ranger.clone(), av.clone());
    }

    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);

    let steps = (seconds / DT) as u64;
    let mut min_clear = f64::INFINITY;
    let mut final_z = 0.0f64;
    let mut tru = TruStats::default();
    for i in 0..steps {
        ctrl.step(&hover_sp);
        let (pos, quat) = ctrl.debug_up();
        tru.sample(pos[0].hypot(pos[2]), -pos[1], tilt_deg(quat));
        let t = (i as f64) * DT;
        let center = [-26.0 + OBSTACLE_SPEED * t, 5.0, z_off];
        let dist = ((pos[0] - center[0]).powi(2)
            + (pos[1] - center[1]).powi(2)
            + (pos[2] - center[2]).powi(2))
        .sqrt();
        let clear = dist - OBSTACLE_RADIUS;
        if clear < min_clear {
            min_clear = clear;
        }
        final_z = pos[2];
    }
    (min_clear, final_z, tru)
}

/// 正对逼近：装备避障（多射线）净间隙显著大于基准（被碰撞）。
#[test]
fn avoidance_keeps_greater_clearance_than_bare() {
    // 障碍从 26m 外逼近，约 13s 到达机体；留足时间让避障触发并侧移脱离。
    let (gap_bare, tru_bare) = run_approach(None, 14.0);
    let (gap_av, tru_av) = run_approach(Some(&multi_ray_setup()), 14.0);

    // TRU 有界：全程物理真值不 NaN、高度不 runaway、姿态不翻滚（验收判据推广）。
    assert_tru_bounded(&tru_bare, "avoid bare", -5.0, 15.0, 45.0);
    assert_tru_bounded(&tru_av, "avoid evasive", -5.0, 15.0, 45.0);

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

/// 单元级决策：危险距离内触发/外不触发/无效读数不触发；P3-B2 闪避方向随障碍横移翻转。
#[test]
fn avoid_velocity_triggers_when_close() {
    let cfg = AvoidanceConfig::new(5.0, 1.5, 0.8);
    let fwd = [-1.0, 0.0, 0.0]; // 机体前（NED 南）
    let right = [0.0, 1.0, 0.0]; // 机体右（NED 东）
    let frame = |rays: Vec<RayReading>| RangeFinderFrame { rays };

    // 正前方近障（射线沿 -fwd）：触发 + 横向速度沿 +right（居中对称退化为固定向右）。
    let near = frame(vec![RayReading { dir_ned: [-1.0, 0.0, 0.0], distance: 3.0, valid: true }]);
    let (v, trig, _) = cfg.avoidance_velocity(&near, fwd, right);
    assert!(trig, "danger 内应触发");
    assert!(v[1] > 0.0, "居中障碍应向右闪避, v_y={}", v[1]);

    // 危险距离外不触发。
    let far = frame(vec![RayReading { dir_ned: [-1.0, 0.0, 0.0], distance: 9.0, valid: true }]);
    let (_, trig2, _) = cfg.avoidance_velocity(&far, fwd, right);
    assert!(!trig2, "危险距离外不触发");

    // 无效读数（量程饱和 / 近距盲区 / 瞬断）不触发（"看不到"≠"无障碍"，但也不凭空闪避）。
    let invalid = frame(vec![RayReading { dir_ned: [-1.0, 0.0, 0.0], distance: 12.0, valid: false }]);
    let (_, trig3, _) = cfg.avoidance_velocity(&invalid, fwd, right);
    assert!(!trig3, "无效读数不触发");

    // P3-B2 方向翻转：障碍偏右 → 向左闪避；障碍偏左 → 向右闪避。
    let right_obs = frame(vec![RayReading {
        dir_ned: [-0.8944, 0.4472, 0.0], // 南-东（右前方）
        distance: 3.0,
        valid: true,
    }]);
    let (v_r, trig_r, _) = cfg.avoidance_velocity(&right_obs, fwd, right);
    assert!(trig_r, "右偏障碍应触发");
    assert!(v_r[1] < 0.0, "右偏障碍应向左闪避, v_y={}", v_r[1]);

    let left_obs = frame(vec![RayReading {
        dir_ned: [-0.8944, -0.4472, 0.0], // 南-西（左前方）
        distance: 3.0,
        valid: true,
    }]);
    let (v_l, trig_l, _) = cfg.avoidance_velocity(&left_obs, fwd, right);
    assert!(trig_l, "左偏障碍应触发");
    assert!(v_l[1] > 0.0, "左偏障碍应向右闪避, v_y={}", v_l[1]);
}

/// 集成级：正前方球应在测距读数中命中（中央射线有效且距离接近真值）。
#[test]
fn avoid_ranger_detects_obstacle_in_fov() {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        None,
        vec![Obstacle::Sphere { center: [-8.0, 5.0, 0.0], radius: 3.0 }],
    );
    // 单条正前方射线（ray_count=1 退化）。
    ctrl.set_ranger(Some(RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1, 0xABCD)));
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..50 {
        ctrl.step(&hover_sp);
    }
    let frame = ctrl.dbg_ranger().expect("ranger present");
    let s = frame.center();
    // 机体在 (0,5,0)，球心 (-8,5,0) r=3，表面距 5m，应在量程内且有效。
    assert!(s.valid, "ranger should see the forward sphere");
    assert!((s.distance - 5.0).abs() < 0.5, "ranger distance ~5m, got {}", s.distance);
}

/// 集成级：前方无障（量程外）时测距无效，避障不介入、不影响悬停。
#[test]
fn avoid_no_false_trigger_when_clear() {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        None,
        vec![Obstacle::Sphere { center: [-20.0, 5.0, 0.0], radius: 3.0 }],
    );
    let ranger = RangeFinderModel::new(12.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1, 0xABCD);
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

/// P3-B2 验收：横向偏置逼近（偏出中央射线）时，扇形多射线持续覆盖并成功闪避；
/// 单射线漏检（净间隙≈擦碰级）；障碍越过并离开视场后避障自然释放，位置环接管回航。
#[test]
fn multi_ray_evades_lateral_slide_single_misses() {
    // 偏置略大于球半径(3m)：单条中央射线到球心距离恒 > 半径 → 永远打不中；
    // 扇形多射线（±60°）可早期命中。
    let z_off = 3.2;
    let (single_gap, _, tru_single) = run_lateral_slide(Some(&single_ray_setup()), z_off, 20.0);
    let (multi_gap, final_z, tru_multi) = run_lateral_slide(Some(&multi_ray_setup()), z_off, 20.0);

    // TRU 有界：全程物理真值不 NaN、高度不 runaway、姿态不翻滚（验收判据推广）。
    assert_tru_bounded(&tru_single, "slide single", -5.0, 8.0, 45.0);
    assert_tru_bounded(&tru_multi, "slide multi", -5.0, 15.0, 45.0);

    // 单射线：始终打不中 → 无规避，净间隙≈z_off-3（近距擦碰级）。
    assert!(
        single_gap < 1.0,
        "single-ray should graze (no evasion), gap_single={}",
        single_gap
    );
    // 多射线：侧向射线持续覆盖 → 成功闪避，净间隙显著为正。
    assert!(multi_gap > 1.0, "multi-ray should clear the slide, gap_multi={}", multi_gap);
    // 核心断言：多射线净间隙显著大于单射线。
    assert!(
        multi_gap > single_gap + 1.5,
        "multi clearance ({}) must beat single ({}) by >1.5m",
        multi_gap,
        single_gap
    );

    // 障碍越过并离开视场后：避障自然释放，位置环把机体拉回原点（末帧横向≈0）。
    assert!(
        final_z.abs() < 2.0,
        "after pass, position loop should reclaim home, final_z={}",
        final_z
    );
}

/// TEMP-DIAG：追踪横向滑移时每帧射线/闪避方向/机体横向位移。
#[test]
fn dbg_multi_lateral_slide() {
    let z_off = 3.2;
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        None,
        vec![],
    );
    let dyn_obs = DynamicObstacle {
        base: Obstacle::Sphere { center: [-26.0, 5.0, z_off], radius: OBSTACLE_RADIUS },
        velocity: [OBSTACLE_SPEED, 0.0, 0.0],
    };
    ctrl.plant_set_dynamic_obstacles(vec![dyn_obs]);
    let (ranger, av) = multi_ray_setup();
    ctrl.configure_avoidance(ranger.clone(), av.clone());
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);
    let steps = (20.0 / DT) as u64;
    let mut min_clear = f64::INFINITY;
    for i in 0..steps {
        ctrl.step(&hover_sp);
        let (pos, _q) = ctrl.debug_up();
        let t = (i as f64) * DT;
        let center = [-26.0 + OBSTACLE_SPEED * t, 5.0, z_off];
        let dist = ((pos[0] - center[0]).powi(2) + (pos[1] - center[1]).powi(2) + (pos[2] - center[2]).powi(2)).sqrt();
        let clear = dist - OBSTACLE_RADIUS;
        if clear < min_clear { min_clear = clear; }
        if (i as f64) * DT > 7.0 && (i as f64) * DT < 16.0 && i % 50 == 0 {
            let frame = ctrl.dbg_ranger().expect("ranger");
            let valid: Vec<String> = frame.rays.iter().filter(|r| r.valid).map(|r| format!("d={:.2}", r.distance)).collect();
            // 由四元数复算 fwd/right（与 plant 同款数学）
            let (_p, q) = ctrl.debug_up();
            let rot = |v: [f64; 3]| -> [f64; 3] {
                let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
                let r00 = 1.0 - 2.0 * (y*y + z*z); let r01 = 2.0*(x*y - w*z); let r02 = 2.0*(x*z + w*y);
                let r10 = 2.0*(x*y + w*z); let r11 = 1.0 - 2.0*(x*x + z*z); let r12 = 2.0*(y*z - w*x);
                let r20 = 2.0*(x*z - w*y); let r21 = 2.0*(y*z + w*x); let r22 = 1.0 - 2.0*(x*x + y*y);
                [r00*v[0]+r01*v[1]+r02*v[2], r10*v[0]+r11*v[1]+r12*v[2], r20*v[0]+r21*v[1]+r22*v[2]]
            };
            let ned = |u: [f64; 3]| [u[0], -u[2], -u[1]];
            let fwd = ned(rot([-1.0,0.0,0.0]));
            let right = ned(rot([0.0,1.0,0.0]));
            let (av_vel, trig, lat_comp) = av.avoidance_velocity(&frame, fwd, right);
            eprintln!("t={:.2}s pos=({:.2},{:.2},{:.2}) clear={:.2} fwd=({:.2},{:.2}) right=({:.2},{:.2}) trig={} lat_comp={:.3} av=({:.2},{:.2}) valid=[{}]",
                t, pos[0], pos[1], pos[2], clear, fwd[0], fwd[1], right[0], right[1], trig, lat_comp, av_vel[0], av_vel[1], valid.join(", "));
        }
    }
    eprintln!("MIN_CLEAR={:.4}", min_clear);
}

/// TEMP-DIAG：追踪正对逼近（z_off=0）时每帧射线/闪避速度/机体横向位移/方向锁存。
#[test]
fn dbg_multi_headon() {
    let z_off = 0.0;
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        None,
        vec![],
    );
    let dyn_obs = DynamicObstacle {
        base: Obstacle::Sphere { center: [-26.0, 5.0, z_off], radius: OBSTACLE_RADIUS },
        velocity: [OBSTACLE_SPEED, 0.0, 0.0],
    };
    ctrl.plant_set_dynamic_obstacles(vec![dyn_obs]);
    let (ranger, av) = multi_ray_setup();
    ctrl.configure_avoidance(ranger.clone(), av.clone());
    let hover_sp = hover_setpoint(0.0, 0.0, -5.0);
    let steps = (14.0 / DT) as u64;
    let mut min_clear = f64::INFINITY;
    for i in 0..steps {
        ctrl.step(&hover_sp);
        let (pos, _q) = ctrl.debug_up();
        let t = (i as f64) * DT;
        let center = [-26.0 + OBSTACLE_SPEED * t, 5.0, z_off];
        let dist = ((pos[0] - center[0]).powi(2) + (pos[1] - center[1]).powi(2) + (pos[2] - center[2]).powi(2)).sqrt();
        let clear = dist - OBSTACLE_RADIUS;
        if clear < min_clear { min_clear = clear; }
        if (i as f64) * DT > 7.0 && (i as f64) * DT < 14.0 && i % 40 == 0 {
            let frame = ctrl.dbg_ranger().expect("ranger");
            let valid: Vec<String> = frame.rays.iter().filter(|r| r.valid).map(|r| format!("d={:.2}", r.distance)).collect();
            let (fwd, right) = (ctrl.debug_fwd_ned(), ctrl.debug_right_ned());
            let (av_vel, trig, lat_comp) = av.avoidance_velocity(&frame, fwd, right);
            let ev_int = ctrl.debug_av_evade();
            let est = ctrl.debug_estimate_ned();   // NED：x=N y=E z=D
            let truth = ctrl.debug_truth_ned();    // NED
            // 机体横向 = NED E（y）。引擎 pos[2]（-东）→ NED E = -pos[2]。
            eprintln!("t={:.2}s pos=({:.2},{:.2},{:.2}) clear={:.2} trig={} lat_comp={:.3} av=({:.2},{:.2}) ev_int=({:.2},{:.2}) est_velNED=({:.2},{:.2}) tru_velNED=({:.2},{:.2}) valid=[{}]",
                t, pos[0], pos[1], pos[2], clear, trig, lat_comp, av_vel[0], av_vel[1], ev_int[0], ev_int[1],
                est.vel[0].0, est.vel[1].0, truth.vel[0].0, truth.vel[1].0, valid.join(", "));
        }
    }
    eprintln!("MIN_CLEAR={:.4}", min_clear);
}
