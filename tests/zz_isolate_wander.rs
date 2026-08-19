//! 阶段 11-A 水平游走诊断：高频采样 NED 水平位置(n,e)与速度，分析游走模式。
//! 判断：游走是"有界振荡"还是"有偏漂移"？是否随时间增长？触发源是否噪声/偏置？

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;

fn run(label: &str, cfg: SensorConfig) {
    run_kp(label, cfg, None);
}

fn run_kp(label: &str, cfg: SensorConfig, kp_xy: Option<f32>) {
    let mut vc = VehicleConfig::default_quad();
    if let Some(kp) = kp_xy {
        vc.kp_xy = kp;
    }
    let world = ToyWorld::new(9.81);
    let mut loop_sim = SimLoop::new(
        world,
        &vc,
        0.004,
        None,
        cfg,
        ControllerKind::Pid,
        None, // 无地面
        vec![],
    );
    let sp = fly_sim_core::controller::hover_setpoint(0.0, 0.0, -5.0);
    let mut max_h = 0.0f64; // 最大水平位移
    let mut max_vh = 0.0f64;
    println!("=== {label} ===");
    let steps = (40.0 / 0.004) as u64;
    for i in 0..steps {
        let (wtrue, _) = loop_sim.step_frame(&sp);
        let n = wtrue.pos[0].0 as f64;
        let e = wtrue.pos[1].0 as f64;
        let vh = (wtrue.vel[0].0 as f64).hypot(wtrue.vel[1].0 as f64);
        let h = n.hypot(e);
        if h > max_h { max_h = h; }
        if vh > max_vh { max_vh = vh; }
        let t = i as f64 * 0.004;
        // 每 2s 采样一次
        if i % 500 == 0 {
            println!("  t={:5.1}s n={:+7.2}m e={:+7.2}m vn={:+5.2} ve={:+5.2} vh={:.2}", t, n, e, wtrue.vel[0].0, wtrue.vel[1].0, vh);
        }
    }
    println!("  STAT: max_h={:.2}m max_vh={:.2}m/s", max_h, max_vh);
}

#[test]
fn diagnose_horizontal_wander() {
    run("clean", SensorConfig::default());
    run("realistic", SensorConfig::realistic());
    let mut bias = SensorConfig::default();
    bias.accel_bias = [0.02, -0.01, 0.05];
    bias.gyro_bias = [0.001, -0.0005, 0.002];
    run("bias_only", bias);

    // 消融：realistic 基础上逐个关掉可疑项，定位水平游走主因。
    let r = SensorConfig::realistic();
    // A) 关振动（40Hz 机体振动加速度）
    let mut a = r.clone();
    a.vib_amp = 0.0;
    run("realistic_no_vib", a);
    // B) 关 GPS 延迟
    let mut b = r.clone();
    b.gps_delay = 0.0;
    run("realistic_no_gpsdelay", b);
    // C) 关 GPS 位置噪声
    let mut c = r.clone();
    c.gps_pos_noise = 0.0;
    run("realistic_no_gpspos", c);
    // D) 关 accel 噪声
    let mut d = r.clone();
    d.accel_noise = 0.0;
    run("realistic_no_accelnoise", d);

    // 水平位置环增益扫描：延迟下过高的 kp_xy 会导致追滞后位置而振荡。
    run_kp("realistic_kp0.5", r.clone(), Some(0.5));
    run_kp("realistic_kp0.40", r.clone(), Some(0.40));
    run_kp("realistic_kp0.35", r.clone(), Some(0.35));
    run_kp("realistic_kp0.30", r.clone(), Some(0.30));
    run_kp("realistic_kp0.25", r.clone(), Some(0.25));
    run_kp("realistic_kp0.20", r.clone(), Some(0.20));
}
