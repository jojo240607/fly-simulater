//! 四旋翼 SIL/HIL 仿真器入口。
//!
//! 当前：`fly-simulater run --hover` 跑 SIL 悬停场景（物理引擎 FFI + flyctrl-core 控制律）。
//! 后续：HIL 模式经 USB CDC 接真实飞控（见 DESIGN.md §9）。

mod phy_ffi;
mod plant;
mod controller;
mod sim;
mod airframe;
mod wind;
mod sensor;

use flyctrl_core::config::VehicleConfig;
use phy_ffi::phy_ffi_abi_version;

/// CLI 解析结果。
struct Cli {
    airframe: Option<String>,
    scenario: String, // "hover" | "wind"
    sensor_noise: bool,
}

/// 极简 CLI：支持 `--airframe <path>`、`--scenario <hover|wind>`、`--sensor-noise`。
fn parse_args() -> Cli {
    let mut cli = Cli { airframe: None, scenario: "hover".to_string(), sensor_noise: false };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--airframe" => {
                if let Some(p) = args.next() {
                    cli.airframe = Some(p);
                } else {
                    eprintln!("[main] --airframe 需要跟一个路径");
                    std::process::exit(2);
                }
            }
            "--scenario" => {
                if let Some(s) = args.next() {
                    cli.scenario = s;
                } else {
                    eprintln!("[main] --scenario 需要跟 hover|wind");
                    std::process::exit(2);
                }
            }
            "--sensor-noise" => {
                cli.sensor_noise = true;
            }
            "--help" | "-h" => {
                println!("用法: fly-simulater [--airframe <path.toml>] [--scenario hover|wind] [--sensor-noise]");
                println!("  --airframe      外部机架 TOML（缺省用内置 default_quad）");
                println!("  --scenario      hover=无风悬停(默认) | wind=抗风悬停(阶段3)");
                println!("  --sensor-noise  开启真实 IMU/GPS 噪声（暴露 EKF 对噪声不耐受，见 PLAN 阶段5）");
                std::process::exit(0);
            }
            other => {
                eprintln!("[main] 未知参数: {}", other);
                std::process::exit(2);
            }
        }
    }
    cli
}

fn main() {
    // 1. 核对物理引擎 ABI 版本（期望 >= 2，含刚体单实例操控符号）。
    let abi = unsafe { phy_ffi_abi_version() };
    println!("[main] phy_ffi ABI version = {}", abi);
    assert!(abi >= 2, "物理引擎库过旧，需要 ABI >= 2（含 per-body FFI）");

    // 2. 解析 CLI + 加载机架（外部 TOML 或内置默认，阶段 0）。
    let cli = parse_args();
    let cfg = match airframe::load_airframe(cli.airframe.as_deref()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("[main] 机架加载失败: {}", e);
            std::process::exit(2);
        }
    };
    println!(
        "[main] airframe = {} | mass={}kg arm={}m Tcoef={:.3}N yawK={:.4}",
        cfg.name, cfg.mass, cfg.arm_length, cfg.thrust_coeff, cfg.torque_coeff
    );
    println!(
        "[main] inertia = [{:.4}, {:.4}, {:.4}] kg·m^2",
        cfg.inertia[0], cfg.inertia[1], cfg.inertia[2]
    );

    // 3. SIL 场景（dt=4ms 对齐 MCU 控制周期）。
    let dt = 0.004;
    let wind = if cli.scenario == "wind" {
        Some(wind::WindField::new(sim::windy_config()))
    } else {
        None
    };
    // 阶段 4：传感器噪声（默认零噪声保持 PASS；--sensor-noise 开启真实噪声）。
    let sensor_cfg = if cli.sensor_noise {
        println!("[main] 传感器真实噪声已开启（IMU/GPS 噪声+延迟+丢星）");
        crate::sensor::SensorConfig::realistic()
    } else {
        crate::sensor::SensorConfig::default()
    };
    let mut loop_sim = sim::SimLoop::new(&cfg, dt, wind, sensor_cfg);

    let ok = match cli.scenario.as_str() {
        "wind" => {
            println!("[main] running SIL anti-wind hover (15s, dt={}ms)...", dt * 1000.0);
            let r = loop_sim.run_hover_wind(15.0);
            println!("[main] SIL anti-wind {}", if r { "PASS" } else { "FAIL" });
            r
        }
        _ => {
            println!("[main] running SIL hover (10s, dt={}ms)...", dt * 1000.0);
            let r = loop_sim.run_hover(10.0);
            println!(
                "[main] SIL hover {} ({} steps)",
                if r { "PASS" } else { "FAIL" },
                loop_sim.steps()
            );
            r
        }
    };

    std::process::exit(if ok { 0 } else { 1 });
}
