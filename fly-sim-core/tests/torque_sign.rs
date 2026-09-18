//! 姿态控制符号链回归：控制器 → 混控 → 被控对象。
//!
//! 背景：`x_hover_demo` 60s 悬停 t≈44s 起滚转慢速自激发散，且把重力锚定门控
//! 收紧（打开姿态 P 项）后发散更快，一度怀疑姿态环存在**正反馈**（符号/坐标系
//! 不一致，见 fly-sim-core/src/plant.rs `read_sensors` 上方注释）。
//!
//! 本测试离线验证这条链的符号一致性，结论：**一致**（非正反馈）：
//! - 混控 `x4_mix(hover, +p/+q/+r)` 经臂力矩+反射后产生**同向**飞控系角速度；
//! - 机体 +roll 时，期望水平的姿态内环 `attitude_rates` 输出**反号** p_cmd（纠正）。
//!
//! 即"符号反号"假设被证伪；44s 发散不在 控制器→混控→被控对象 这一段。

use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::plant::QuadrotorPlant;
use fly_sim_core::sensor::SensorConfig;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::attitude::x4_mix;
use flyctrl_core::vehicle::ActuatorCmd;

fn axis_response(axis: [f32; 3]) -> [f64; 3] {
    let vc = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new_at(
        PhySdkWorld::create_empty(),
        &vc,
        0.004,
        None,
        SensorConfig::default(),
        None,
        vec![],
        [0.0, 0.0, -5.0],
    );
    // 先悬停稳定（电机一阶滞后）
    for _ in 0..150 {
        plant.apply_actuators(&ActuatorCmd { motor: [0.5; 4] });
        plant.step();
    }
    let before = plant.state_ned();
    // 施加带指令的混控 0.5s
    for _ in 0..125 {
        let m = x4_mix(0.5, axis);
        plant.apply_actuators(&ActuatorCmd { motor: m });
        plant.step();
    }
    let after = plant.state_ned();
    [
        (after.omega[0].0 - before.omega[0].0) as f64,
        (after.omega[1].0 - before.omega[1].0) as f64,
        (after.omega[2].0 - before.omega[2].0) as f64,
    ]
}

#[test]
fn mixer_matches_plant_omega_sign() {
    let p = axis_response([0.05, 0.0, 0.0]);
    let q = axis_response([0.0, 0.05, 0.0]);
    let r = axis_response([0.0, 0.0, 0.05]);
    eprintln!("[mix-sign] +p_cmd -> d_omega_fc=({:.3},{:.3},{:.3})", p[0], p[1], p[2]);
    eprintln!("[mix-sign] +q_cmd -> d_omega_fc=({:.3},{:.3},{:.3})", q[0], q[1], q[2]);
    eprintln!("[mix-sign] +r_cmd -> d_omega_fc=({:.3},{:.3},{:.3})", r[0], r[1], r[2]);
    assert!(p[0] > 0.0, "+p_cmd 应产生 +p(ωx) 角速度，实际 {p:?}");
    assert!(q[1] > 0.0, "+q_cmd 应产生 +q(ωy) 角速度，实际 {q:?}");
    assert!(r[2] > 0.0, "+r_cmd 应产生 +r(ωz) 角速度，实际 {r:?}");
}

/// [诊断] 控制器姿态 P 项符号：机体有 +roll 时，期望水平的 p_cmd 应反号（减小 roll）。
#[test]
fn attitude_p_term_corrects_tilt() {
    let vc = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new_at(
        PhySdkWorld::create_empty(), &vc, 0.004, None, SensorConfig::default(), None, vec![],
        [0.0, 0.0, -5.0],
    );
    for _ in 0..150 {
        plant.apply_actuators(&ActuatorCmd { motor: [0.5; 4] });
        plant.step();
    }
    for _ in 0..125 {
        plant.apply_torque_disturbance([0.02, 0.0, 0.0]);
        plant.apply_actuators(&ActuatorCmd { motor: [0.5; 4] });
        plant.step();
    }
    let att = plant.state_ned().att;
    let roll = att.roll().to_degrees();
    let out = flyctrl_core::controller::attitude::attitude_rates(
        att,
        flyctrl_core::vehicle::Quaternion::IDENTITY,
        1.0,
        0.0,
        [0.0, 0.0, 0.0],
    );
    eprintln!(
        "[att-sign] plant roll={roll:.2}° -> attitude_rates(des=level) p_cmd={:.4} q_cmd={:.4}",
        out.rates[0], out.rates[1]
    );
    assert!(
        (out.rates[0] * roll) < 0.0,
        "P 项与倾斜同向（正反馈）：roll={roll} p_cmd={}",
        out.rates[0]
    );
}
