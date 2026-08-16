//! `fly-sim-core`：四旋翼 SIL 仿真内核（纯计算，无 I/O 边界）。
//!
//! 包含：
//! - `physics`：物理引擎抽象层（`RigidBodyWorld` trait + `PhySdkWorld` 真实适配器 + `ToyWorld` 测试替身）。
//! - `wind`：风场与环境模型（基础风/阵风/Dryden 湍流）。
//! - `sensor`：传感器真实化模型（IMU 噪声/偏置/GPS 延迟丢星）。
//! - `plant`：四旋翼推进模型（`QuadrotorPlant<W>`），桥接物理引擎与控制律。
//! - `controller`：飞控包装（`FlyController<W>`），接入 `flyctrl-core` 的真实控制律。
//! - `sim`：仿真主循环与场景（`SimLoop<W>`），日志经 `on_step` 回调交给 runner。
//! - `log`：日志条目纯数据结构（`LogRow`）。
//!
//! runner（如 `fly-simulater` bin）依赖本 crate，负责 CLI、机型加载、CSV 写出等具体形态。

pub mod alloc;
pub mod controller;
pub mod log;
pub mod mavlink;
pub mod physics;
pub mod plant;
pub mod render;
pub mod sensor;
pub mod sim;
pub mod wind;

// 便捷再导出：runner 最常用到的类型。
pub use controller::{ControllerKind, FlyController};
#[cfg(feature = "phy")]
pub use controller::RealFlyController;
pub use log::LogRow;
pub use mavlink::{loopback_telemetry, MavlinkBridge, MavlinkStreamParser};
#[cfg(feature = "phy")]
pub use physics::PhySdkWorld;
pub use physics::{ContactInfo, ContactModel, resolve_ground_contact, RigidBodyWorld, RigidTransform, TerrainField, ToyWorld};
pub use plant::QuadrotorPlant;
pub use sensor::{SensorConfig, SensorModel};
pub use sim::{windy_config, SimLoop};
pub use wind::{WindConfig, WindField, WindVec};
