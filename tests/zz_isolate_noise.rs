//! 阶段 11-A 诊断：捕捉初始瞬态（首 0.5s）的推力/姿态/加速度，定位 IMU 偏置致悬停发散触发点。
//! 诊断脚本，不参与 CI（zz_* 命名）。

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::VehicleState;

fn finite_state(s: &VehicleState) -> bool {
    s.pos.iter().all(|p| p.0.is_finite()) && s.vel.iter().all(|v| v.0.is_finite()) && s.att.w.is_finite()
}

fn run_initial(kind: &str, cfg: SensorConfig) {
    let world = ToyWorld::new(9.81);
    let contact = None;
    let mut loop_sim = SimLoop::new(
        world,
        &VehicleConfig::default_quad(),
        0.004,
        None,
        cfg,
        ControllerKind::Pid,
        contact,
        vec![],
    );
    let sp = fly_sim_core::controller::hover_setpoint(0.0, 0.0, -5.0);
    let sample: Vec<u64> = vec![1, 2, 5, 10, 20, 50, 100];
    let mut si = 0;
    println!("=== {kind} ===");
    for step in 1..=100 {
        let (wtrue, cmd) = loop_sim.step_frame(&sp);
        if !finite_state(&wtrue) { println!("  DIVERGED at {step}"); return; }
        if si < sample.len() && step == sample[si] {
            let est = loop_sim.ctrl_debug_estimate();
            let thr = cmd.motor.iter().map(|m| *m as f64).sum::<f64>() / 4.0;
            let (rd, rvd, fd, fvd, ez, iz, dvz, acd, dthr) = loop_sim.ctrl_debug_pid_internal();
            println!(
                "  step={:3} t={:5.3}s TRU_d={:7.3} TRU_vd={:6.2} EST_d={:7.3} EST_vd={:6.2} thr={:.3}",
                step,
                step as f64 * 0.004,
                wtrue.pos[2].0 as f64,
                wtrue.vel[2].0 as f64,
                est.pos[2].0 as f64,
                est.vel[2].0 as f64,
                thr
            );
            println!(
                "      [PID] raw_d={:.4} raw_vd={:.4} filt_d={:.4} filt_vd={:.4} ez={:.4} iz={:.4} des_vz={:.4} acc_d={:.4} des_thr={:.4}",
                rd as f64, rvd as f64, fd as f64, fvd as f64, ez as f64, iz as f64, dvz as f64, acd as f64, dthr as f64
            );
            si += 1;
        }
    }
}

#[test]
fn diagnose_initial_transient() {
    run_initial("baseline", SensorConfig::default());
    run_initial("realistic", SensorConfig::realistic());
    let mut bias_only = SensorConfig::default();
    bias_only.accel_bias = [0.02, -0.01, 0.05];
    bias_only.gyro_bias = [0.001, -0.0005, 0.002];
    run_initial("bias_only", bias_only);
}
