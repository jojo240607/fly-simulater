//! 风速 × 控制律 抗风能力扫描：量化 pid/indi/lqr 在不同恒定风速下的末态姿态。
//!
//! 用法：cargo test --test wind_scan --features phy -- --nocapture

#![cfg(feature = "phy")]

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;
use fly_sim_core::physics::ContactModel;
use fly_sim_core::wind::{WindConfig, WindField};

const DT: f64 = 0.004;

fn make_ctrl(kind: ControllerKind, speed: f64) -> FlyController<PhySdkWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    let wind = if speed > 0.0 {
        Some(WindField::new(WindConfig {
            base: [speed, 0.0, 0.0],
            ..Default::default()
        }))
    } else {
        None
    };
    FlyController::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        wind,
        SensorConfig::default(),
        kind,
        Some(ContactModel::default()),
    )
}

/// 四元数分量 -> (roll, pitch, yaw) 度（机体->世界，NED 近似）。
fn rpy_deg(w: f32, x: f32, y: f32, z: f32) -> (f64, f64, f64) {
    let (w, x, y, z) = (w as f64, x as f64, y as f64, z as f64);
    let roll = f64::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = f64::asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    let yaw = f64::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    (roll, pitch, yaw)
}

fn run(kind: ControllerKind, speed: f64, secs: f64) -> (f64, f64, bool) {
    let mut ctrl = make_ctrl(kind, speed);
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let total = (secs / DT) as u64;
    let mut finite = true;
    let mut last = (0.0f64, 0.0f64, 0.0f64);
    for i in 0..total {
        let st = ctrl.step(&sp);
        if !st.att.w.is_finite() || !st.att.x.is_finite() || !st.att.y.is_finite() || !st.att.z.is_finite() {
            finite = false;
            break;
        }
        if i == total - 1 {
            last = rpy_deg(st.att.w, st.att.x, st.att.y, st.att.z);
        }
    }
    (last.0, last.1, finite)
}

#[test]
fn wind_scan_all() {
    let kinds = [
        ("PID", ControllerKind::Pid),
        ("INDI", ControllerKind::Indi),
        ("LQR", ControllerKind::Lqr),
    ];
    let speeds = [0.0, 0.3, 0.6, 1.0, 2.0];
    println!("\n{:>6} | {:>8} | {:>10} | {:>10} | {:>6}", "kind", "wind", "roll_deg", "pitch_deg", "finite");
    println!("{}", "-".repeat(56));
    for (name, kind) in kinds.iter() {
        for &sp in speeds.iter() {
            let (r, p, f) = run(*kind, sp, 5.0);
            println!("{:>6} | {:>7.1} | {:>10.2} | {:>10.2} | {:>6}", name, sp, r, p, if f { "yes" } else { "NO" });
        }
    }
    println!("{}", "-".repeat(56));
    // 本测例只为打印量化对比，不作为通过性断言。
    assert!(true);
}
