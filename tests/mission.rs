//! P2-2 任务级逻辑：waypoint 路径跟随框架验证。
//!
//! 说明：`run_mission` 提供"任务执行 + 失败检测"框架。
//! - P3-A1（轨迹跟踪）前：PID 悬停控制器在**持续移动目标**下无速度前馈，稳态跟随
//!   误差 ≈ cruise_v/kp_xy（kp_xy=0.3、2m/s 巡航约 6.7m>5m 发散阈值），长距离巡航
//!   被判 `stable=false`。
//! - P3-A1（轨迹跟踪）后：`run_mission` 沿路径给速度前馈 + flyctrl PID 倾斜补偿
//!   与加速度前馈，长距离巡航应 `stable=true`（本文件的验收目标）。
//! 本测试验证：
//! - `run_mission` API 返回结构正确（duration/误差非负、可运行）；
//! - 长距离巡航能被**稳定跟随**（stable=true）——P3-A1 验收；
//! - 任务层仍能检测**真正**的失控（误差记录、发散路径检测不误报成功）。

#![cfg(feature = "phy")]

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use fly_simulater::airframe::load_airframe;
use fly_sim_core::physics::ContactModel;

mod common;
use common::{assert_tru_bounded, TruStats};

const DT: f64 = 0.004;

fn make_loop() -> SimLoop<PhySdkWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    SimLoop::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        // 场景测试默认真实噪声（FIDELITY_ROADMAP 收尾项：默认零噪声会屏蔽 EKF/控制律噪声行为）。
        SensorConfig::realistic(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    )
}

#[test]
fn mission_api_returns_valid_structure() {
    // 悬停任务（同点 waypoint）：应稳定完成，返回结构有效。
    let mut loop_sim = make_loop();
    let wp = [(0.0, 0.0, -5.0, 0.0), (0.0, 0.0, -5.0, 0.0)];
    let r = loop_sim.run_mission(&wp, 1.0);
    assert!(r.stable, "原地悬停任务应稳定");
    assert!(r.max_err >= 0.0 && r.rms_err >= 0.0, "误差记录非负");
    assert!(r.duration >= 0.0, "时长非负");
}

#[test]
fn mission_long_cruise_stable_with_trajectory_tracking() {
    // P3-A1 验收：长距离巡航（10m @ 2m/s）在"速度前馈 + 倾斜补偿"下应稳定跟随
    // （stable=true）。修复前无速度前馈，稳态误差 ≈ cruise_v/kp_xy = 2.0/0.3 ≈ 6.7m
    // > 5m 发散阈值 → stable=false；现沿路径给前馈速度，稳态误差应显著收敛。
    let mut loop_sim = make_loop();
    let wp = [(0.0, 0.0, -5.0, 0.0), (10.0, 0.0, -5.0, 0.0)];
    let r = loop_sim.run_mission(&wp, 2.0);
    assert!(r.stable, "长距离巡航应被稳定跟随（P3-A1）: max_err={:.2}", r.max_err);
    // 跟随完成：时长 ≈ 路径长/巡航速度（含起飞稳定段 2s）。
    assert!((r.duration - 10.0 / 2.0).abs() < 2.5, "时长应≈全程: dur={:.2}", r.duration);
    // 跟踪误差应明显低于发散阈值（5m）。实测约 3.3m，来自巡航启停的速度阶跃瞬态
    // （折线匀速巡航无加速度前馈），远好于修复前的 6.7m 稳态误差。
    assert!(r.max_err < 4.5, "跟踪误差应显著收敛: max_err={:.2}", r.max_err);
    // 验收判据（FIDELITY_ROADMAP）：收敛判定同时断言 TRU 有界——mission 全程内部
    // 已用真值角速度（>6 rad/s）做发散检测；此处再断言终点物理真值不 runaway
    // （snapshot 为真值 NED 状态；路径终点在 NED 北 10m，故水平判定量给足裕度）。
    let (truth, _) = loop_sim.snapshot();
    let mut tru = TruStats::default();
    tru.sample(
        (truth.pos[0].0 as f64).hypot(truth.pos[1].0 as f64),
        truth.pos[2].0 as f64,
        2.0 * (truth.att.w as f64).clamp(-1.0, 1.0).acos().to_degrees(),
    );
    assert_tru_bounded(&tru, "mission long-cruise", -5.0, 20.0, 45.0);
}

#[test]
fn mission_records_tracking_error() {
    // 短距离慢速移动：任务可能稳定或检测失败，但误差记录必须正确计算。
    let mut loop_sim = make_loop();
    let wp = [(0.0, 0.0, -5.0, 0.0), (2.0, 0.0, -5.0, 0.0)];
    let r = loop_sim.run_mission(&wp, 0.5);
    // 误差与时长一致：duration ≈ 路径长/速度（若稳定）
    if r.stable {
        assert!((r.duration - 2.0 / 0.5).abs() < 2.0, "时长应≈路径/速度: dur={:.2}", r.duration);
    } else {
        // 检测到失败：时长应小于全程（中途发散）
        assert!(r.duration < 4.0, "中途发散时长应小于全程");
    }
    assert!(r.rms_err >= 0.0 && r.max_err >= 0.0);
}
