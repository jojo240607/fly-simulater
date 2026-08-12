//! 阶段 7 集成测试：用 `ToyWorld` 玩具级物理替身验证 `QuadrotorPlant` 与物理抽象层，
//! 无需启动 C 物理引擎。这些测试锁住 phase 2/3/4 的真实度回归与接口充分性：
//! - 满油门 -> 机体上升（旋翼推力经 `apply_impulse` 注入，ToyWorld 积分后上升）；
//! - 纯偏航力矩 -> 机体绕 Z 轴角速度变化（瞬态力矩语义）；
//! - 瞬态力/矩 `step` 后清零（不残留幽灵推力，防替换引擎出残留力 bug）；
//! - plant 在替身世界上连续 step 不产出 NaN（接口充分性）。
//!
//! 运行：`cargo test --test physics_toy`

use fly_sim_core::{ContactModel, RigidBodyWorld, ToyWorld};
use fly_sim_core::physics::resolve_ground_contact;
use fly_sim_core::controller::actuator_full;
use fly_sim_core::QuadrotorPlant;
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;

fn make_plant(world: ToyWorld) -> QuadrotorPlant<ToyWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    QuadrotorPlant::new(world, &cfg, DT, None, Default::default(), Some(ContactModel::default()))
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

// ===================== P1-2 接触 / 碰撞模型（玩具世界验证） =====================
//
// 注意坐标约定：ToyWorld 自带一个 y=0 的停机坪钳制（pos.y<0 时清零）。为让本项目的
// 接触模型（独立弹簧-阻尼 + 库仑摩擦）真正起作用，而不被 ToyWorld 的钳制掩盖，
// 所有测试把接触面放在 y≈5 附近（远高于 0），使 ToyWorld 的 y=0 钳制永不被触发。

/// 把 body 放在接触面（pen=0），重力应建立极小的平衡穿透，机体被托住不飞出、速度收敛。
#[test]
fn contact_rests_on_ground_no_sink() {
    let mut world = ToyWorld::new(9.81);
    // 接触面 contact_y = ground_y + contact_half_h = 5.0 + 0.1 = 5.1。
    // body 起始正好在接触面（pen=0, v=0），重力自然压出微小平衡穿透，弹簧托住。
    let id = world.add_body(1.0, &[0.0, 5.1, 0.0, 1.0, 0.0, 0.0, 0.0], &[0.01, 0.01, 0.01]);
    let m = ContactModel {
        ground_y: 5.0,
        restitution: 0.0, // 纯非弹，应静止停在接触面附近
        friction: 0.8,
        penalty_k: 2000.0, // 较软弹簧，平衡穿透合理（m*g/k≈5mm），无弹飞
        contact_half_h: 0.1,
    };
    for _ in 0..200 {
        resolve_ground_contact(&mut world, id, 1.0, &m, DT);
        world.step(DT);
    }
    let mut tf = [0.0f64; 7];
    world.get_rigid_transforms(&mut tf);
    let y = tf[1];
    assert!(y.is_finite(), "接触后 y 应有限, got {}", y);
    // 停在接触面附近（平衡穿透 ~m*g/k≈5mm），不应飞出。
    assert!(y > 4.9 && y < 5.2, "应停在接触面(5.1)附近, y={}", y);
    let v = world.get_velocity(id);
    assert!(v[1].abs() < 0.5, "静止后应无大幅法向速度, vy={}", v[1]);
}

/// 从高处以恢复系数 e=0.6 落下，触地后法向速度应反向（反弹），且反弹幅度小于入射。
#[test]
fn contact_bounces_with_restitution() {
    let mut world = ToyWorld::new(9.81);
    // 接触面 = 5.1；body 放在 y=7.0（高于接触面），自由下落命中。
    let id = world.add_body(1.0, &[0.0, 7.0, 0.0, 1.0, 0.0, 0.0, 0.0], &[0.01, 0.01, 0.01]);
    let m = ContactModel {
        ground_y: 5.0,
        restitution: 0.6,
        friction: 0.8,
        penalty_k: 8000.0,
        contact_half_h: 0.1,
    };
    // 先自由下落若干步直到接近接触面，记录入射法向速度。
    let mut v_before = 0.0;
    for _ in 0..200 {
        world.step(DT);
        let v = world.get_velocity(id);
        if v[1] < v_before {
            v_before = v[1];
        }
        if v[1] < -1.0 {
            break;
        }
    }
    assert!(v_before < -0.5, "下落应获得负向速度, vy={}", v_before);
    // 继续步进，直到发生接触反弹（vy 由负转正）。
    let mut bounced = false;
    for _ in 0..800 {
        resolve_ground_contact(&mut world, id, 1.0, &m, DT);
        world.step(DT);
        let v = world.get_velocity(id);
        if v[1] > 0.1 {
            bounced = true;
            // 反弹速度应小于入射速度（能量不增）。
            assert!(v[1] < v_before.abs(), "反弹不应超入射, in={} out={}", v_before.abs(), v[1]);
            break;
        }
    }
    assert!(bounced, "应观察到反弹（法向速度反向）");
}

/// 给水平初速度 + 高摩擦，接触后水平速度应被库仑摩擦显著衰减（不打滑飞出）。
#[test]
fn contact_friction_stops_horizontal_slide() {
    let mut world = ToyWorld::new(9.81);
    // 接触面 = 5.1；body 放在 y=5.0（穿透，持续接触），给 +X 动量。
    let id = world.add_body(1.0, &[0.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0], &[0.01, 0.01, 0.01]);
    world.apply_impulse(id, &[2.0, 0.0, 0.0], 0); // 给 +X 动量
    world.step(DT);
    let v0 = world.get_velocity(id)[0];
    assert!(v0 > 0.5, "应获得正向水平速度, vx={}", v0);
    let m = ContactModel {
        ground_y: 5.0,
        restitution: 0.0,
        friction: 0.9, // 高摩擦
        penalty_k: 8000.0,
        contact_half_h: 0.1,
    };
    for _ in 0..200 {
        resolve_ground_contact(&mut world, id, 1.0, &m, DT);
        world.step(DT);
    }
    let v = world.get_velocity(id);
    assert!(v[0].abs() < v0 * 0.3, "高摩擦应显著衰减水平速度, v0={} v={}", v0, v[0]);
}

/// 完整 plant：启用接触、无推力，机体从高处落下后应停在接触面附近（不穿透、不飞出）。
#[test]
fn plant_contact_stops_falling_quad() {
    let world = ToyWorld::new(9.81);
    let cfg = load_airframe(None).expect("default airframe");
    // 注：plant 内部把机体放在 NED(0,0,0) → 引擎 y=5.0。
    // 把接触面正设在机体起始高度（contact_y = 4.95+0.05 = 5.0），重力压出微小平衡穿透被托住。
    // 用较软弹簧避免下落冲击失稳（真实着陆可用更硬弹簧 + 更小 dt）。
    let mut plant = QuadrotorPlant::new(
        world,
        &cfg,
        DT,
        None,
        Default::default(),
        Some(ContactModel {
            ground_y: 4.95,
            restitution: 0.1,
            friction: 0.9,
            penalty_k: 2000.0,
            contact_half_h: 0.05,
        }),
    );
    // 直接驱动 plant（无控制律），机体应被接触面托住在 y≈5.0 附近，不飞出、不穿透。
    for step in 0..1500 {
        plant.step();
        let (pos, _) = plant.debug_up();
        assert!(pos.iter().all(|v| v.is_finite()), "plant 状态应有限");
    }
    let (pos, _) = plant.debug_up();
    // 接触面 = 4.95+0.05 = 5.0 = 起始高度。机体应停在附近（平衡穿透极小）。
    assert!(pos[1] > 4.5 && pos[1] < 5.5, "机体应停在接触面附近, y={}", pos[1]);
}
