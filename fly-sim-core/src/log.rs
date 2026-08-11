//! 阶段 6：日志条目数据定义（纯数据，零 I/O）。
//!
//! `LogRow` 是仿真内核产出的单帧记录，不含任何文件/序列化逻辑——
//! 序列化（CSV/Parquet/二进制）由 runner（bin）侧决定。这样核心保持纯计算，
//! 便于复用（HIL、游戏集成、可视化）而不绑定具体存储格式。

use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, VehicleState};

/// 单帧日志条目（阶段 6 字段全集）。
pub struct LogRow {
    pub step: u64,
    pub t: f64,
    pub true_state: VehicleState, // 物理引擎真值（NED）
    pub est_state: VehicleState,  // 估计器输出（NED）
    pub cmd: ActuatorCmd,
    pub imu: ImuSample,
}
