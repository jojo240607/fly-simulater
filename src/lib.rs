//! 四旋翼 SIL/HIL 仿真库。
//!
//! 把仿真层作为库暴露，使集成测试（`tests/`）能直接 `use fly_simulater::...` 验证
//! 公共接口（含物理引擎抽象 `physics` 与可替换的 `ToyWorld` 替身）。二进制入口
//! `main.rs` 复用本库。

pub mod phy_ffi;
pub mod physics;
pub mod plant;
pub mod controller;
pub mod sim;
pub mod airframe;
pub mod wind;
pub mod sensor;
pub mod log;
