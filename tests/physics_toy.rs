//! 阶段 7 集成测试：用 `ToyWorld` 玩具级物理替身验证 `QuadrotorPlant` 与物理抽象层，
//! 无需启动 C 物理引擎。这些测试锁住 phase 2/3/4 的真实度回归与接口充分性：
//! - 满油门 -> 机体上升（旋翼推力经 `apply_impulse` 注入，ToyWorld 积分后上升）；
//! - 纯偏航力矩 -> 机体绕 Z 轴角速度变化（瞬态力矩语义）；
//! - 瞬态力/矩 `step` 后清零（不残留幽灵推力，防替换引擎出残留力 bug）；
//! - plant 在替身世界上连续 step 不产出 NaN（接口充分性）。
//!
//! 运行：`cargo test --test physics_toy`

use fly_sim_core::{RigidBodyWorld, ToyWorld};
use fly_sim_core::controller::actuator_full;
use fly_sim_core::QuadrotorPlant;
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;

fn make_plant(world: ToyWorld) -> QuadrotorPlant<ToyWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    QuadrotorPlant::new(world, &cfg, DT, None, Default::default())
}

#[test]
fn thrust_makes_it_climb() {
    // 四个电机满油门 -> 净竖直推力 > 重力 -> 高度应上升（ToyWorld Y 增加）。
    let world = ToyWorld::new(9.81);
    let mut plant = make_plant(world);
    let y0 = plant.debug_up().0[1];
    for _ in 0..200 {
        plant.apply_actuators(&actuator_full());
        plant.step();
    }
    let y1 = plant.debug_up().0[1];
    assert!(y1 > y0 + 0.1, "满油门应上升, y0={} y1={}", y0, y1);
}

#[test]
fn yaw_torque_spins_body() {
    // 对 ToyWorld 施加纯世界 Z 力矩 -> 绕 Z 角速度应出现（验证瞬态力矩语义）。
    let mut world = ToyWorld::new(9.81);
    let id = world.add_body(1.0, &[0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0], &[0.01, 0.01, 0.01]);
    let tz = 0.05;
    for _ in 0..100 {
        // 力矩冲量 = 力矩 × dt。
        world.apply_torque_impulse(id, &[0.0, 0.0, tz * DT], 0);
        world.step(DT);
    }
    let w = world.get_angular_velocity(id);
    assert!(w[2].abs() > 1e-3, "Z 力矩应产生绕 Z 角速度, wz={}", w[2]);
}

#[test]
fn transient_force_cleared_after_step() {
    // 施加一次力后 step，再 step 一次（不施加），机体不应继续被该力加速（力已清零）。
    let mut world = ToyWorld::new(9.81);
    let id = world.add_body(1.0, &[0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0], &[0.01, 0.01, 0.01]);
    // 线冲量 = 力 × dt（瞬态，step 后不残留）。
    world.apply_impulse(id, &[1.0 * DT, 0.0, 0.0], 0);
    world.step(DT);
    let v1 = world.get_velocity(id);
    // 第二帧不施加任何力（除重力）。
    world.step(DT);
    let v2 = world.get_velocity(id);
    // 第二帧无外力（除重力，仅影响 y），x 速度应保持（无残留力推进）。
    assert!(
        (v2[0] - v1[0]).abs() < 1e-9,
        "瞬态力 step 后应清零, v1x={} v2x={}",
        v1[0],
        v2[0]
    );
}

#[test]
fn plant_steps_without_nan_on_toy() {
    // 在替身上连续 step 不产出 NaN/Inf（验证 RigidBodyWorld 接口对 plant 的充分性）。
    let world = ToyWorld::new(9.81);
    let mut plant = make_plant(world);
    for _ in 0..500 {
        plant.apply_actuators(&actuator_full());
        plant.step();
        let (pos, quat) = plant.debug_up();
        for v in pos.iter().chain(quat.iter()) {
            assert!(v.is_finite(), "plant 状态应有限, got {}", v);
        }
    }
}
