//! `fly-simulater`：仿真 runner（薄壳）。
//!
//! 只装 runner 特有内容：机架加载（`airframe`）、CSV 日志写出（`log`）、CLI（`main`）。
//! 仿真内核全部来自 `fly-sim-core` 库 crate。

pub mod airframe;
pub mod log;
pub mod view;

// 再导出仿真内核常用类型，方便 runner 与测试直接引用。
pub use fly_sim_core::{
    ControllerKind, LogRow, RigidBodyWorld, RigidTransform, SensorConfig, SimLoop, ToyWorld,
    WindConfig, WindField, WindVec,
};
#[cfg(feature = "phy")]
pub use fly_sim_core::PhySdkWorld;
pub use fly_sim_core::{FlyController, QuadrotorPlant};
#[cfg(feature = "phy")]
pub use fly_sim_core::RealFlyController;
