//! fly-sim-hil：HIL 链路层（与 MCU 仿真平台共用的 HIL 闭环协议）。
//!
//! 从 fly-sim-server 抽取：USB-CDC 与飞控 MCU 的 MAVLink 数据交互
//! （HIL_SENSOR/HIL_GPS/SET_POSITION 注入真值，HIL_ACTUATOR_CONTROLS 回传
//! 执行器指令）。串口后端抽象为 [`hil_link::HilPort`] trait：
//!   - `realport` feature：真实 USB-CDC（serialport，VID 0483:5740）；
//!   - 默认：虚拟口后端由调用方（如 mcu_simulater）实现 [`hil_link::HilPort`]，
//!     经 [`hil_link::HilLink::open_virtual`] 接入，单进程 HIL 闭环。
pub mod hil_link;
