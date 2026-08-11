//! 四旋翼 SIL/HIL 仿真器入口。
//!
//! 当前：`fly-simulater run --hover` 跑 SIL 悬停场景（物理引擎 FFI + flyctrl-core 控制律）。
//! 后续：HIL 模式经 USB CDC 接真实飞控（见 DESIGN.md §9）。

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor;
use fly_sim_core::sim;
use fly_sim_core::wind;
use fly_simulater::airframe;
use fly_simulater::log::CsvLogger;
use fly_simulater::view;

/// CLI 解析结果。
struct Cli {
    airframe: Option<String>,
    scenario: String, // "hover" | "wind" | "freefall"
    sensor_noise: bool,
    controller: ControllerKind,
    fail_motor: Option<u8>, // 阶段 5：电机故障注入（0..3）
    log_path: Option<String>, // 阶段 6：CSV 日志输出路径
    view: bool,             // 阶段 7：实时 3D 可视化
}

impl Cli {
    fn parse_controller(s: &str) -> ControllerKind {
        match s {
            "pid" => ControllerKind::Pid,
            "indi" => ControllerKind::Indi,
            "lqr" => ControllerKind::Lqr,
            other => {
                eprintln!("[main] --controller 需要 pid|indi|lqr，收到: {}", other);
                std::process::exit(2);
            }
        }
    }
}

/// 极简 CLI：支持 `--airframe <path>`、`--scenario <hover|wind|freefall>`、
/// `--sensor-noise`、`--controller <pid|indi|lqr>`、`--fail-motor <0..3>`。
fn parse_args() -> Cli {
    let mut cli = Cli {
        airframe: None,
        scenario: "hover".to_string(),
        sensor_noise: false,
        controller: ControllerKind::Pid,
        fail_motor: None,
        log_path: None,
        view: false,
    };
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
                    eprintln!("[main] --scenario 需要跟 hover|wind|freefall");
                    std::process::exit(2);
                }
            }
            "--controller" => {
                if let Some(c) = args.next() {
                    cli.controller = Cli::parse_controller(&c);
                } else {
                    eprintln!("[main] --controller 需要跟 pid|indi|lqr");
                    std::process::exit(2);
                }
            }
            "--fail-motor" => {
                if let Some(m) = args.next() {
                    match m.parse::<u8>() {
                        Ok(v @ 0..=3) => cli.fail_motor = Some(v),
                        _ => {
                            eprintln!("[main] --fail-motor 需要 0..3");
                            std::process::exit(2);
                        }
                    }
                } else {
                    eprintln!("[main] --fail-motor 需要跟 0..3");
                    std::process::exit(2);
                }
            }
            "--sensor-noise" => {
                cli.sensor_noise = true;
            }
            "--log" => {
                if let Some(p) = args.next() {
                    cli.log_path = Some(p);
                } else {
                    eprintln!("[main] --log 需要跟一个输出路径");
                    std::process::exit(2);
                }
            }
            "--view" => {
                cli.view = true;
            }
            "--help" | "-h" => {
                println!("用法: fly-simulater [--airframe <path.toml>] [--scenario hover|wind] [--controller pid|indi|lqr] [--fail-motor 0..3] [--sensor-noise] [--log <path.csv>]");
                println!("  --airframe      外部机架 TOML（缺省用内置 default_quad）");
                println!("  --scenario      hover=无风悬停(默认) | wind=抗风悬停(阶段3) | freefall=自由落体能量守恒(阶段9)");
                println!("  --controller    pid=PID(默认) | indi=INDI+PID基线 | lqr=LQR");
                println!("  --fail-motor    注入单电机故障 0..3（该电机停转，阶段5）");
                println!("  --sensor-noise  开启真实 IMU/GPS 噪声（暴露 EKF 对噪声不耐受，见 PLAN 阶段5）");
                println!("  --log           CSV 日志输出（真值/估计/指令/IMU，阶段6）");
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
    // 1. 物理引擎以 Rust 源码级依赖（phy-sdk rlib）接入，无 C-ABI / ABI 版本检查。
    println!("[main] 物理引擎: phy-sdk (Rust rlib, 源码级依赖)");

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
        Some(wind::WindField::new(sim::windy_config())) as Option<_>
    } else {
        None
    };
    // 阶段 4：传感器噪声（默认零噪声保持 PASS；--sensor-noise 开启真实噪声）。
    let sensor_cfg = if cli.sensor_noise {
        println!("[main] 传感器真实噪声已开启（IMU/GPS 噪声+延迟+丢星）");
        sensor::SensorConfig::realistic()
    } else {
        sensor::SensorConfig::default()
    };

    // 阶段 5：电机故障注入（单电机停转）。
    let mut fail_mask = [false; 4];
    if let Some(m) = cli.fail_motor {
        fail_mask[m as usize] = true;
        println!("[main] 注入电机故障: m{} 停转", m);
    }

    // 阶段 7：实时 3D 可视化（后台快跑仿真 + 采样渲染）。窗口关闭即退出。
    if cli.view {
        println!("[main] 启动实时 3D 可视化（后台仿真 + 渲染采样）...");
        view::run_view(
            &cfg,
            dt,
            wind,
            sensor_cfg,
            cli.controller,
            &cli.scenario,
            fail_mask,
        );
        return;
    }

    // 阶段 6：批处理式 SIL（命令行 + 可选 CSV）。
    let mut loop_sim =
        sim::SimLoop::new(PhySdkWorld::create_empty(), &cfg, dt, wind, sensor_cfg, cli.controller);

    // 阶段 6：CSV 日志（经 on_step 回调注入；runner 决定如何存储）。
    if let Some(ref p) = cli.log_path {
        match CsvLogger::new(p) {
            Ok(mut logger) => {
                println!("[main] CSV 日志 -> {}", p);
                loop_sim.set_on_step(Box::new(move |row| {
                    let _ = logger.write(row);
                }));
            }
            Err(e) => eprintln!("[main] 无法创建日志 {}: {}", p, e),
        }
    }

    loop_sim.set_motor_failure(fail_mask);

    let ok = match cli.scenario.as_str() {
        "freefall" => {
            println!("[main] running freefall energy-conservation (10s, dt={}ms)...", dt * 1000.0);
            let r = loop_sim.run_freefall(10.0);
            println!(
                "[main] freefall {} ({} steps)",
                if r { "PASS" } else { "FAIL" },
                loop_sim.steps()
            );
            r
        }
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
