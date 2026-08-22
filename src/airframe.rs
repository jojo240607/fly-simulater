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
    // ---- 阶段 8 动力系统（可选，缺省用默认值）----
    #[serde(default = "default_battery_v")]
    pub battery_v_nom: f32,
    #[serde(default = "default_battery_r")]
    pub battery_r: f32,
    #[serde(default = "default_motor_kv")]
    pub motor_kv: f32,
    #[serde(default = "default_motor_r")]
    pub motor_r: f32,
    #[serde(default = "default_rotor_inertia")]
    pub rotor_inertia: f32,
    #[serde(default = "default_slipstream_drag")]
    pub slipstream_drag_coeff: f32,
    // ---- P3-C1 叶素理论参数（可选，缺省用默认值）----
    #[serde(default = "default_rotor_blades")]
    pub rotor_blades: f32,
    #[serde(default = "default_rotor_solidity")]
    pub rotor_solidity: f32,
    #[serde(default = "default_rotor_cl_alpha")]
    pub rotor_cl_alpha: f32,
    #[serde(default = "default_rotor_cd0")]
    pub rotor_cd0: f32,
    #[serde(default = "default_rotor_stall_alpha")]
    pub rotor_stall_alpha: f32,
    // ---- P3-C2 桨盘干扰（可选，缺省用默认值）----
    #[serde(default = "default_rotor_downwash_coupling")]
    pub rotor_downwash_coupling: f32,
}

fn default_rotor_downwash_coupling() -> f32 {
    0.25
}

fn default_rotor_blades() -> f32 {
    2.0
}
fn default_rotor_solidity() -> f32 {
    0.08
}
fn default_rotor_cl_alpha() -> f32 {
    6.2832
}
fn default_rotor_cd0() -> f32 {
    0.012
}
fn default_rotor_stall_alpha() -> f32 {
    0.24
}

fn default_rotor_inertia() -> f32 {
    1.2e-5
}
fn default_slipstream_drag() -> f32 {
    0.06
}
fn default_battery_v() -> f32 {
    14.8
}
fn default_battery_r() -> f32 {
    0.015
}
fn default_motor_kv() -> f32 {
    102.6
}
fn default_motor_r() -> f32 {
    0.12
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
            slipstream_drag_coeff: self.slipstream_drag_coeff,
            air_density: self.air_density,
            gravity: self.gravity,
            tilt_max: self.tilt_max,
            hover_thrust,
            vmax_xy: self.vmax_xy,
            vmax_z: self.vmax_z,
            att_kp: 3.0,
            att_kd: 0.3,
            kp_xy: 0.5,
            kv_xy: 0.8,
            vel_lpf_tau: 0.15,
            drag_fwd: 0.14, // P3-A3/C1/C2：空速拖拽前馈（对齐 default_quad 的 0.14，含 BET 桨盘阻力 + P3-C2 下洗耦合）
            battery_v_nom: self.battery_v_nom,
            battery_r: self.battery_r,
            motor_kv: self.motor_kv,
            motor_r: self.motor_r,
            rotor_inertia: self.rotor_inertia,
            rotor_blades: self.rotor_blades,
            rotor_solidity: self.rotor_solidity,
            rotor_cl_alpha: self.rotor_cl_alpha,
            rotor_cd0: self.rotor_cd0,
            rotor_stall_alpha: self.rotor_stall_alpha,
            rotor_downwash_coupling: self.rotor_downwash_coupling,
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
