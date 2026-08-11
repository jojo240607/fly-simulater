//! 四旋翼 SIL/HIL 仿真器入口。
//!
//! 当前：`fly-simulater run --hover` 跑 SIL 悬停场景（物理引擎 FFI + flyctrl-core 控制律）。
//! 后续：HIL 模式经 USB CDC 接真实飞控（见 DESIGN.md §9）。

mod phy_ffi;
mod plant;
mod controller;
mod sim;

use flyctrl_core::config::VehicleConfig;
use phy_ffi::phy_ffi_abi_version;

fn main() {
    // 1. 核对物理引擎 ABI 版本（期望 >= 2，含刚体单实例操控符号）。
    let abi = unsafe { phy_ffi_abi_version() };
    println!("[main] phy_ffi ABI version = {}", abi);
    assert!(abi >= 2, "物理引擎库过旧，需要 ABI >= 2（含 per-body FFI）");

    let cfg = VehicleConfig::default_quad();
    println!(
        "[main] airframe = {} | mass={}kg arm={}m Tcoef={:.3}N yawK={:.4}",
        cfg.name, cfg.mass, cfg.arm_length, cfg.thrust_coeff, cfg.torque_coeff
    );

    // 2. SIL 悬停场景（dt=4ms 对齐 MCU 控制周期，跑 10s）。
    let dt = 0.004;
    let mut loop_sim = sim::SimLoop::new(&cfg, dt);
    println!("[main] running SIL hover (10s, dt={}ms)...", dt * 1000.0);
    let ok = loop_sim.run_hover(10.0);
    println!(
        "[main] SIL hover {} ({} steps)",
        if ok { "PASS" } else { "FAIL" },
        loop_sim.steps()
    );

    std::process::exit(if ok { 0 } else { 1 });
}
