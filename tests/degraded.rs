//! 阶段 5 增强集成测试：单电机效率系数故障模型 + 量化容错边界。
//!
//! 锁住：
//! - `set_motor_eff` 正确缩放电机指令（部分退化走缩放路径而非置零）；
//! - 单电机推力损失注入后姿控发散（当前控制律无重构分配，合法不可恢复）；
//! - 发散存活步数随效率系数单调（eff 越小、存活越短）——物理一致性回归。
//!
//! 运行：`cargo test --test degraded`

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;

fn make_ctrl() -> FlyController<PhySdkWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    FlyController::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
    )
}

#[test]
fn eff_scales_motor_command_not_zero() {
    // 部分退化（eff=0.6）应缩放而非置零：注入后该路指令 = 原指令 × 0.6（>0）。
    let mut ctrl = make_ctrl();
    // 先稳态几步让控制律产出非零指令。
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..200 {
        ctrl.step(&sp);
    }
    let base = ctrl.last_cmd();
    // 记录 m1 正常指令作为缩放基准对比。
    let m1_base = base.motor[1];

    let mut eff = [1.0f32; 4];
    eff[0] = 0.6;
    ctrl.set_motor_eff(eff);
    // 再跑一步，此时 m0 指令被缩放。
    ctrl.step(&sp);
    let cmd = ctrl.last_cmd();
    assert!(cmd.motor[0] > 0.0, "m0 部分退化指令应 >0（缩放而非置零），got {}", cmd.motor[0]);
    // m0 缩放后若原指令非零，应明显小于正常档（m1 未退化，可作参考）。
    if m1_base > 0.01 {
        assert!(
            cmd.motor[0] < cmd.motor[1] + 1e-6,
            "m0 缩放后推力应低于未退化路 (m0={}, m1={})",
            cmd.motor[0], cmd.motor[1]
        );
    }
    // 其余三路不受影响。
    assert!((cmd.motor[1] - m1_base).abs() < 1e-3, "m1 不应受 m0 退化影响");
}

#[test]
fn degraded_full_loss_diverges() {
    // eff=0.0（完全停转）注入后姿控应发散：存活步数有限（< 注入后 8s 窗口）。
    let mut ctrl = make_ctrl();
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    // 稳态 4s。
    for _ in 0..1000 {
        ctrl.step(&sp);
    }
    ctrl.set_motor_eff([0.0, 1.0, 1.0, 1.0]);
    let mut survived = 0u64;
    for _ in 0..2000 {
        ctrl.step(&sp);
        let w = ctrl.world_state().omega;
        let rate = (w[0].0 * w[0].0 + w[1].0 * w[1].0 + w[2].0 * w[2].0).sqrt();
        if rate > 1.0 {
            break;
        }
        survived += 1;
    }
    assert!(survived < 2000, "完全停转应致姿控发散（存活步数有限），got {}", survived);
}

#[test]
fn tolerance_boundary_monotonic() {
    // 存活步数应随效率系数降低而减少：eff=0.95 存活 > eff=0.50 存活。
    fn survived_steps(eff: f32) -> u64 {
        let mut ctrl = make_ctrl();
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        for _ in 0..1000 {
            ctrl.step(&sp);
        }
        let mut eff_arr = [1.0f32; 4];
        eff_arr[0] = eff;
        ctrl.set_motor_eff(eff_arr);
        let mut survived = 0u64;
        for _ in 0..2000 {
        ctrl.step(&sp);
        let w = ctrl.world_state().omega;
        let rate = (w[0].0 * w[0].0 + w[1].0 * w[1].0 + w[2].0 * w[2].0).sqrt();
        if rate > 1.0 {
            break;
        }
        survived += 1;
        }
        survived
        }

    let s95 = survived_steps(0.95);
    let s50 = survived_steps(0.50);
    assert!(s95 > s50, "eff=0.95 应比 eff=0.50 存活更久 (s95={}, s50={})", s95, s50);
}
