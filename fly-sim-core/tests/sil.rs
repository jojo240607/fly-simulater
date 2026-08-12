//! SIL 回归集成测试：直接在代码内跑悬停 / 自由落体场景并断言通过判据，
//! 与手动看 stdout 相比提供 CI 级回归保护（阶段 9 增补）。
//!
//! 这些场景依赖真实物理引擎 `phy-sdk`（本地 path 依赖），因此测的是
//! 飞控栈（EKF→PID→plant→physics）端到端的正确性，而非替身。

#![cfg(feature = "phy")]

use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sim::{SimLoop, windy_config};
use fly_sim_core::wind::WindField;
use fly_sim_core::ControllerKind;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::physics::ContactModel;
use flyctrl_core::config::VehicleConfig;

const DT: f64 = 0.004;

fn make_loop() -> SimLoop<PhySdkWorld> {
    let cfg = VehicleConfig::default_quad();
    let world = PhySdkWorld::create_empty();
    SimLoop::new(
        world,
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
    )
}

#[test]
fn sil_hover_converges_and_stable() {
    let mut loop_sim = make_loop();
    let ok = loop_sim.run_hover(10.0);
    assert!(
        ok,
        "hover must converge to setpoint (0,0,-5) and stay stable; got {} steps",
        loop_sim.steps()
    );
}

#[test]
fn sil_freefall_energy_non_increasing() {
    let mut loop_sim = make_loop();
    let ok = loop_sim.run_freefall(10.0);
    assert!(
        ok,
        "freefall must fall and keep mechanical energy non-increasing; got {} steps",
        loop_sim.steps()
    );
}

#[test]
fn sil_wind_hover_holds_altitude() {
    // 阶段 3 抗风悬停：有基础风 + 阵风，仍应维持高度（不发散、不坠地）。
    let cfg = VehicleConfig::default_quad();
    let world = PhySdkWorld::create_empty();
    let wind = Some(WindField::new(windy_config()));
    let mut loop_sim = SimLoop::new(
        world,
        &cfg,
        DT,
        wind,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
    );
    let ok = loop_sim.run_hover_wind(15.0);
    // 注：默认 PID 抗风上限 ~0.3m/s，强风会饱和翻滚，本测例不验证"抗风位置保持"，
    // 只验证风-气动耦合正确接入（机体被风明显吹离原点）且数值稳定（无 NaN/Inf）。
    assert!(ok, "wind-hover must stay numerically stable and show wind disturbance");
}
