//! 机架数据输入：从外部 TOML 加载机型参数，构造 `flyctrl_core::VehicleConfig`。
//!
//! 设计约束：`flyctrl-core` 是 no_std crate，不能依赖 serde/toml。因此解析与
//! 反序列化放在 host 侧的本模块，解析后手动构造 `VehicleConfig`（core 保持
//! 不变，`name` 用字面量以满足 `&'static str` 字段）。
//!
//! 字段与 `flyctrl_core::config::VehicleConfig` 一一对应（见 core/src/config.rs）。

use flyctrl_core::config::VehicleConfig;
use serde::Deserialize;

/// TOML 反序列化结构（owned，允许任意字符串 name）。
#[derive(Debug, Clone, Deserialize)]
pub struct AirframeToml {
    pub name: String,
    pub mass: f32,
    pub arm_length: f32,
    pub thrust_coeff: f32,
    pub torque_coeff: f32,
    pub inertia: [f32; 3],
    pub motor_tau: f32,
    pub drag_coeff: [f32; 3],
    pub induced_drag_coeff: f32,
    pub disk_area: f32,
    pub air_density: f32,
    pub gravity: f32,
    pub tilt_max: f32,
    pub hover_thrust: f32,
    pub vmax_xy: f32,
    pub vmax_z: f32,
}

impl AirframeToml {
    /// 解析 TOML 文本。
    pub fn from_str(s: &str) -> Result<Self, String> {
        toml::from_str(s).map_err(|e| format!("机架 TOML 解析失败: {}", e))
    }

    /// 从文件路径加载并解析。
    pub fn from_path(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("读取机架文件 {} 失败: {}", path, e))?;
        Self::from_str(&text)
    }

    /// 转成 `VehicleConfig`（name 用静态字面量，core 保持 no_std）。
    pub fn into_config(self) -> VehicleConfig {
        // 悬停油门从物理推导（高保真关键）：
        //   悬停时总推力 = 重力 = mass·g，单电机满油门推力 = thrust_coeff，
        //   故 hover_thrust = mass·g / (4·thrust_coeff)。
        // 直接以物理值为准，覆盖 TOML 里可能手写出错的值（TOML 字段保留作文档参考）。
        let hover_thrust = self.mass * self.gravity / (4.0 * self.thrust_coeff);
        if (hover_thrust - self.hover_thrust).abs() > 0.05 {
            eprintln!(
                "[airframe] 警告: TOML 中 hover_thrust={:.3} 与物理推导 {:.3} 偏差大，已用物理值覆盖",
                self.hover_thrust, hover_thrust
            );
        }
        VehicleConfig {
            // 注意：core 的 name 是 &'static str，这里把 owned String 泄露为 'static
            // （仿真生命周期 = 进程生命周期，可接受）。
            name: Box::leak(self.name.into_boxed_str()),
            mass: self.mass,
            arm_length: self.arm_length,
            thrust_coeff: self.thrust_coeff,
            torque_coeff: self.torque_coeff,
            inertia: self.inertia,
            motor_tau: self.motor_tau,
            drag_coeff: self.drag_coeff,
            induced_drag_coeff: self.induced_drag_coeff,
            disk_area: self.disk_area,
            air_density: self.air_density,
            gravity: self.gravity,
            tilt_max: self.tilt_max,
            hover_thrust,
            vmax_xy: self.vmax_xy,
            vmax_z: self.vmax_z,
        }
    }
}

/// 加载机架：给定路径则解析 TOML，否则回退到内置 `default_quad()`。
pub fn load_airframe(path: Option<&str>) -> Result<VehicleConfig, String> {
    match path {
        Some(p) => {
            println!("[airframe] 从 {} 加载机架", p);
            AirframeToml::from_path(p).map(|a| a.into_config())
        }
        None => {
            println!("[airframe] 使用内置默认机架 default_quad()");
            Ok(VehicleConfig::default_quad())
        }
    }
}
