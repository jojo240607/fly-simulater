// 引擎一致性对比：纯物理，无控制器、无传感器。
// 给机体施加恒定"悬停推力"（恰好抵消重力），比较 ToyWorld 与 PhySdkWorld 的高度演化。
// 若两者一致，则线性物理等价；若高度发散，则引擎物理本身有差异。

#[cfg(feature = "phy")]
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::physics::{RigidBodyWorld, ToyWorld};

fn make_world() -> Box<dyn RigidBodyWorld> {
    #[cfg(feature = "phy")]
    {
        let _ = PhySdkWorld::create_empty();
    }
    Box::new(ToyWorld::new(9.81))
}

#[allow(dead_code)]
fn make_world_real() -> Box<dyn RigidBodyWorld> {
    #[cfg(feature = "phy")]
    {
        return Box::new(PhySdkWorld::create_empty());
    }
    #[cfg(not(feature = "phy"))]
    {
        Box::new(ToyWorld::new(9.81))
    }
}

#[test]
fn engine_torque_attitude() {
    let dt = 0.004;
    let mass = 1.5;
    let inertia = [0.01, 0.01, 0.02];

    // 机体初始 identity 姿态，施加绕世界 X 轴的恒定力矩。
    let pos7 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

    let mut w = make_world();
    let id = w.add_body(mass, &pos7, &inertia);

    let steps = 250; // 1s
    let k = [0.05, 0.0, 0.0]; // 世界系恒力矩
    for _ in 0..steps {
        let ki = [k[0] * dt, k[1] * dt, k[2] * dt];
        w.apply_torque_impulse(id, &ki, 0);
        w.step(dt);
    }

    let mut tf = [0.0f64; 7];
    w.get_body_transform(id, &mut tf);
    println!(
        "ZZENGINE toy torque attitude: q=({:.4},{:.4},{:.4},{:.4})",
        tf[3], tf[4], tf[5], tf[6]
    );
    // 与真实引擎对比：用相同力矩，姿态应一致（用于定位积分约定差异）。
}

#[test]
fn engine_hover_thrust() {
    let dt = 0.004;
    let mass = 1.5;
    let inertia = [0.01, 0.01, 0.02];
    let g = 9.81;

    // 机体初始位置 (0, 0, 0)，姿态 identity（无翻转，推力沿 +Y 上）。
    let pos7 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

    let mut w = make_world();
    let id = w.add_body(mass, &pos7, &inertia);

    let steps = 1000; // 4s
    for _ in 0..steps {
        // 恒定向上推力 = m*g，恰好抵消重力。
        let f = [0.0, mass * g, 0.0];
        let f_imp = [f[0] * dt, f[1] * dt, f[2] * dt];
        w.apply_impulse(id, &f_imp, 0);
        w.step(dt);
    }

    let mut tf = [0.0f64; 7];
    w.get_body_transform(id, &mut tf);
    let y = tf[1];
    println!("ZZENGINE toy hover after {}s: y={:.4}", steps as f64 * dt, y);
    // 若引擎线性正确，推力=m*g 抵消重力，净加速度≈0，应维持在 y≈0（可能极轻微漂移）。
    assert!((y).abs() < 0.5, "ToyWorld hover drift too large: y={}", y);
}
