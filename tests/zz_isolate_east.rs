//! 阶段 11-A 帧符号诊断：命令固定东向速度设定点 vel[1]=+1 m/s，
//! 观察无人机物理上到底是向东(+e)还是向西(-e)运动，判定 roll→水平推力符号。
//! 若向东：控制器 roll 约定与物理一致；若向西：roll 符号反了（需修复）。

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, Radian};

#[test]
fn diagnose_east_velocity_sign() {
    let world = ToyWorld::new(9.81);
    let mut loop_sim = SimLoop::new(
        world,
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::default(), // 干净传感器，排除噪声干扰
        ControllerKind::Pid,
        None, // 无地面
        vec![],
    );
    // 悬停在原点，但设定东向速度 +1 m/s（NED +Y=东）。
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0), MeterPerSecond(1.0), MeterPerSecond(0.0)],
        yaw: Radian(0.0),
    };
    let mut last_e = 0.0f32;
    let mut last_ve = 0.0f32;
    let mut last_roll = 0.0f32;
    for step in 1..=5000 {
        let (wtrue, _cmd) = loop_sim.step_frame(&sp);
        let e = wtrue.pos[1].0; // NED East
        let ve = wtrue.vel[1].0; // NED East velocity
        if step % 500 == 0 {
            let est = loop_sim.ctrl_debug_estimate();
            let q = est.att;
            let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
            let fw = loop_sim.debug_f_world(); // 引擎世界系 x=北 y=上 z=西
            println!(
                "t={:.2}s e={:+.2}m ve={:+.2}m/s roll={:+.3}rad fw_N={:+.2}N fw_E={:+.2}N",
                step as f64 * 0.004, e, ve, roll, fw[0], -fw[2]
            );
            last_e = e; last_ve = ve; last_roll = roll;
        }
    }
    println!("RESULT_E: cmd_east(+1) -> e={:.2}m ve={:.2}m/s roll={:.3}", last_e, last_ve, last_roll);

    // 复位并命令北向速度 vel[0]=+1，验证 pitch 符号。
    let mut loop_sim = SimLoop::new(
        ToyWorld::new(9.81),
        &VehicleConfig::default_quad(),
        0.004,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        None,
        vec![],
    );
    let spn = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(1.0), MeterPerSecond(0.0), MeterPerSecond(0.0)],
        yaw: Radian(0.0),
    };
    let mut last_n = 0.0f32;
    let mut last_vn = 0.0f32;
    for step in 1..=5000 {
        let (wtrue, _cmd) = loop_sim.step_frame(&spn);
        let n = wtrue.pos[0].0;
        let vn = wtrue.vel[0].0;
        if step == 5000 { last_n = n; last_vn = vn; }
    }
    println!("RESULT_N: cmd_north(+1) -> n={:.2}m vn={:.2}m/s", last_n, last_vn);
    println!("INTERP: 东向 ve>0 且北向 vn>0 => 两轴符号均正确；否则对应轴需再修");
}
