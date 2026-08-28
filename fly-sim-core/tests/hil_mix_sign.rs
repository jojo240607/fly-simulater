//! HIL 混控符号探针：直接对游戏 plant（fly-sim-core）做开环力矩符号验证。
//!
//! 飞控侧 `x4_mix` 只对 flyctrl 自带的 toy 物理（`open_loop_torque_sign_probe`）
//! 验证过符号，从未对 HIL 使用的游戏 plant 验证。此处用与飞控完全相同的
//! `x4_mix` 输出注入游戏 plant，测量 1s 后 FRD 系角速度方向：
//!   +p_cmd（右滚）应产生 FRD 角速度 +p（gyro[0]>0）；
//!   +q_cmd（机头上仰）应产生 FRD 角速度 +q（gyro[1]>0）；
//!   +r_cmd（右偏航/顺时针，Z 向下俯视顺时针）应产生 FRD +r（gyro[2]>0）。
//! 任一轴反号 → 该轴姿态环正反馈发散（HIL 炸机根因）。

#![cfg(feature = "phy")]

use fly_sim_core::physics::{ContactModel, PhySdkWorld};
use fly_sim_core::plant::QuadrotorPlant;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::attitude::x4_mix;
use flyctrl_core::vehicle::ActuatorCmd;

const DT: f64 = 0.004;

fn probe(pqr: [f32; 3], steps: usize) -> ([f32; 3], [f32; 3]) {
    let cfg = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        Default::default(),
        Some(ContactModel::default()),
        Vec::new(),
    );
    let cmd = ActuatorCmd {
        motor: x4_mix(0.5, pqr),
    };
    plant.apply_actuators(&cmd);
    let mut omega_fc = [0.0f32; 3];
    let mut att = [0.0f32; 3];
    for _ in 0..steps {
        plant.step();
        let (imu, _pos) = plant.read_sensors();
        omega_fc = [
            imu.gyro[0].0,
            imu.gyro[1].0,
            imu.gyro[2].0,
        ];
        let (_, q) = plant.debug_up();
        let q = flyctrl_core::vehicle::Quaternion { w: q[0] as f32, x: q[1] as f32, y: q[2] as f32, z: q[3] as f32 };
        att = [q.roll().to_degrees(), q.pitch().to_degrees(), q.yaw().to_degrees()];
    }
    (omega_fc, att)
}

#[test]
fn hil_mix_sign_probe() {
    let probes: [(&str, [f32; 3]); 3] = [
        ("roll +p (右滚) ", [0.25, 0.0, 0.0]),
        ("pitch +q (上仰)", [0.0, 0.25, 0.0]),
        ("yaw +r (右偏航)", [0.0, 0.0, 0.25]),
    ];
    for (name, pqr) in probes {
        let (w, att) = probe(pqr, 250); // 1s
        println!(
            "{:<16} -> omega_fc@1s=({:+.3}, {:+.3}, {:+.3}) rad/s | R/P/Y=({:+.1}, {:+.1}, {:+.1}) deg",
            name, w[0], w[1], w[2], att[0], att[1], att[2]
        );
    }
}
