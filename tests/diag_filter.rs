//! 临时诊断：realistic 噪声下滤波前后 est.omega / 姿态误差演化。
use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{RigidBodyWorld, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian};

/// 跑 10s realistic 仿真，返回 (最大 tilt°, 末尾 est_att roll/pitch, 末尾 truth tilt°)。
fn run_scenario(tag: &str, sc: SensorConfig) {
    let world = ToyWorld::new(9.81);
    let cfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut sim = SimLoop::new(world, &cfg, dt, None, sc, ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (10.0 / dt) as u64;
    let mut max_tilt = 0.0f32;
    let mut end_est = (0.0f32, 0.0f32);
    let mut end_tilt = 0.0f32;
    for i in 0..steps {
        let (truth, _) = sim.step_frame(&sp);
        let est = sim.ctrl_debug_estimate();
        let tilt = 2.0 * (truth.att.w as f64).clamp(-1.0, 1.0).acos().to_degrees() as f32;
        if tilt > max_tilt {
            max_tilt = tilt;
        }
        if i == steps - 1 {
            end_est = (est.att.roll().to_degrees() as f32, est.att.pitch().to_degrees() as f32);
            end_tilt = tilt;
        }
    }
    println!(
        "{}: max_tilt={:.1}° end_tilt={:.1}° end_est=({:.1},{:.1})",
        tag, max_tilt, end_tilt, end_est.0, end_est.1
    );
}

/// 噪声隔离：分别屏蔽陀螺/加计噪声路径，判定失稳触发源。
#[test]
fn diag_noise_isolation() {
    let mut base = SensorConfig::realistic();
    // 仅陀螺噪声/偏置（加计零噪声零振动）
    let mut gyro_only = base.clone();
    gyro_only.accel_bias = [0.0, 0.0, 0.0];
    gyro_only.accel_noise = 0.0;
    gyro_only.vib_amp = 0.0;
    // 仅加计噪声/偏置/振动（陀螺零噪声零偏置）
    let mut accel_only = base.clone();
    accel_only.gyro_bias = [0.0, 0.0, 0.0];
    accel_only.gyro_noise = 0.0;
    accel_only.gyro_walk = 0.0;
    accel_only.gyro_bias_inst = 0.0;
    // 仅静态偏置（无白噪声、无振动）
    let mut bias_only = base.clone();
    bias_only.accel_noise = 0.0;
    bias_only.gyro_noise = 0.0;
    bias_only.gyro_walk = 0.0;
    bias_only.gyro_bias_inst = 0.0;
    bias_only.vib_amp = 0.0;
    // 无振动（其余噪声保持）
    let mut no_vib = base.clone();
    no_vib.vib_amp = 0.0;
    // 无陀螺白噪声（其余保持）
    let mut no_gyro_wn = base.clone();
    no_gyro_wn.gyro_noise = 0.0;
    // 无加计白噪声（其余保持）
    let mut no_accel_wn = base.clone();
    no_accel_wn.accel_noise = 0.0;

    run_scenario("baseline      ", base.clone());
    run_scenario("gyro_only     ", gyro_only);
    run_scenario("accel_only    ", accel_only);
    run_scenario("bias_only     ", bias_only);
    run_scenario("no_vib        ", no_vib);
    run_scenario("no_gyro_wn    ", no_gyro_wn);
    run_scenario("no_accel_wn   ", no_accel_wn);
}

#[test]
fn diag_omega_evolution() {
    let world = ToyWorld::new(9.81);
    let cfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut sim = SimLoop::new(
        world,
        &cfg,
        dt,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        None,
        Vec::new(),
    );
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (10.0 / dt) as u64;
    use std::fs::OpenOptions;
    use std::io::Write;
    let mut f = OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open("_diag_filter.log")
        .unwrap();
    for i in 0..steps {
        let (truth, _) = sim.step_frame(&sp);
        let est = sim.ctrl_debug_estimate();
        let (err, pqr, ctrl_om) = sim.ctrl_debug_pid_pqr();
        let (raw_d, raw_vd, filt_d, filt_vd, ez, iz, des_vz, acc_d, des_thr) =
            sim.ctrl_debug_pid_internal();
        if i % 25 == 0 {
            let tilt = 2.0 * (truth.att.w as f64).clamp(-1.0, 1.0).acos().to_degrees();
            writeln!(
                f,
                "i={} truth_att=({:.2},{:.2},{:.2}) tilt={:.1}° est_att=({:.2},{:.2},{:.2}) ctrl_om=({:.3},{:.3},{:.3}) tru_om=({:.3},{:.3},{:.3}) pqr=({:.3},{:.3},{:.3}) err=({:.3},{:.3},{:.3})",
                i,
                truth.att.roll().to_degrees(),
                truth.att.pitch().to_degrees(),
                truth.att.yaw().to_degrees(),
                tilt,
                est.att.roll().to_degrees(),
                est.att.pitch().to_degrees(),
                est.att.yaw().to_degrees(),
                ctrl_om[0], ctrl_om[1], ctrl_om[2],
                truth.omega[0].0, truth.omega[1].0, truth.omega[2].0,
                pqr[0], pqr[1], pqr[2],
                err[0], err[1], err[2],
            )
            .unwrap();
            writeln!(
                f,
                "   est_pos=({:.2},{:.2},{:.2}) tru_pos=({:.2},{:.2},{:.2}) est_vel=({:.3},{:.3},{:.3}) ab=({:.3},{:.3},{:.3}) vz_des={:.3} acc_d={:.3} thr={:.3} ez={:.3} iz={:.3} d_est={:.2} d_tru={:.2}",
                est.pos[0].0, est.pos[1].0, est.pos[2].0,
                truth.pos[0].0, truth.pos[1].0, truth.pos[2].0,
                est.vel[0].0, est.vel[1].0, est.vel[2].0,
                est.accel_bias[0], est.accel_bias[1], est.accel_bias[2],
                des_vz, acc_d, des_thr, ez, iz, filt_d, raw_d,
            )
            .unwrap();
        }
    }
}
