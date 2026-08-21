//! 临时消融诊断（P3-A2）：从 zero 噪声出发，逐一叠加单一噪声分量，
//! 定位"最小触发集"——哪个传感器分量单独就能把悬停打爆。
//! 分析完即删，结论写入 PLAN 阶段 11-A。

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::ToyWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian};

fn run_cfg(name: &str, cfg: &SensorConfig, secs: f64) -> (f64, f64, f64, f64) {
    // 返回 (d_span, mean_d, hmax, end_h)
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut sim = SimLoop::new(world, &vcfg, dt, None, cfg.clone(), ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (secs / dt) as u64;
    let mut dmin = f64::MAX;
    let mut dmax = f64::MIN;
    let mut hmax = 0.0f64;
    let mut sum = 0.0;
    let mut n = 0u64;
    let mut end_h = 0.0f64;
    for i in 0..steps {
        let (w, _cmd) = sim.step_frame(&sp);
        let d = w.pos[2].0 as f64;
        let h = (w.pos[0].0 as f64).hypot(w.pos[1].0 as f64);
        if i % 50 == 0 {
            dmin = dmin.min(d);
            dmax = dmax.max(d);
            hmax = hmax.max(h);
            sum += d;
            n += 1;
        }
        end_h = h;
        if !d.is_finite() || !w.pos[0].0.is_finite() {
            return (f64::INFINITY, f64::INFINITY, h, h);
        }
    }
    (dmax - dmin, sum / n as f64, hmax, end_h)
}

fn base() -> SensorConfig {
    SensorConfig::default()
}

#[test]
fn ablation_minimal_trigger() {
    let b = base();
    let mut cfgs: Vec<(&str, SensorConfig)> = Vec::new();
    cfgs.push(("zero", b.clone()));

    // baro 单独
    let mut c = b.clone();
    c.baro_noise = 0.3;
    c.baro_drift = 0.05;
    cfgs.push(("baro", c.clone()));

    // GPS 位置噪声单独
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    cfgs.push(("gps-pos", c.clone()));

    // GPS 延迟单独
    let mut c = b.clone();
    c.gps_delay = 0.15;
    c.gps_rate = 20.0;
    cfgs.push(("gps-delay", c.clone()));

    // GPS 位置噪声 + 延迟 + 降频
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_delay = 0.15;
    c.gps_rate = 20.0;
    cfgs.push(("gps-pos+delay", c.clone()));

    // accel 噪声/偏置/振动单独
    let mut c = b.clone();
    c.accel_noise = 0.05;
    c.accel_bias = [0.02, -0.01, 0.05];
    cfgs.push(("accel", c.clone()));
    let mut c = b.clone();
    c.vib_amp = 0.1;
    cfgs.push(("vib", c.clone()));

    // gyro 单独
    let mut c = b.clone();
    c.gyro_noise = 0.003;
    c.gyro_bias = [0.001, -0.0005, 0.002];
    cfgs.push(("gyro", c.clone()));

    // mag 单独
    let mut c = b.clone();
    c.mag_hard_iron = [0.3, -0.2, 0.4];
    c.mag_soft_iron = [0.98, 1.03, 0.99];
    c.mag_noise = 0.05;
    cfgs.push(("mag", c.clone()));

    println!("== 40s no-contact steady hover (PID), d 设定 -5 ==");
    println!("  配置            d-span(m)   mean_d(m)  hmax(m)   end_h(m)  状态");
    for (name, c) in &cfgs {
        let (span, mean, hmax, endh) = run_cfg(name, c, 40.0);
        let ok = span.is_finite() && span < 2.0 && hmax < 10.0;
        println!(
            "  {name:16} {span:8.2}   {mean:8.2}   {hmax:8.2}   {endh:8.2}  {}",
            if ok { "OK" } else { "DIVERGE" }
        );
    }
}
