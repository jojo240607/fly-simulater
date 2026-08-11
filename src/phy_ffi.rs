//! 手写 `phy_ffi` C-ABI 绑定（与 `d:/project/game/physics/pkg/release/phy_ffi.h` 对应）。
//!
//! 仅声明本仿真器用到的符号；完整契约见头文件 + `PHY_FFI_ABI_VERSION`。
//! 所有 `PhyWorldHandle` 为不透明指针，Rust 侧不解读其布局。

#![allow(non_snake_case)]

use std::os::raw::c_char;

/// 不透明世界句柄（对应 C 侧 `PhyWorldHandle *`）。
#[repr(C)]
pub struct PhyWorldHandle {
    _private: [u8; 0],
}

extern "C" {
    /// 返回库的 ABI 版本（与编译期期望核对，fail-fast）。
    pub fn phy_ffi_abi_version() -> u32;

    // ---- 场景工厂 ----
    pub fn phy_world_create_rigid() -> *mut PhyWorldHandle;
    pub fn phy_world_create_rigid_empty() -> *mut PhyWorldHandle;
    pub fn phy_world_create_fluid() -> *mut PhyWorldHandle;
    pub fn phy_world_create_granular() -> *mut PhyWorldHandle;
    pub fn phy_world_create_coupled() -> *mut PhyWorldHandle;

    // ---- 步进 ----
    pub fn phy_world_step(w: *mut PhyWorldHandle, dt: f64) -> i32;
    pub fn phy_world_step_checked(w: *mut PhyWorldHandle, dt: f64) -> i32;
    pub fn phy_world_time(w: *mut PhyWorldHandle) -> f64;

    // ---- 刚体读回 ----
    pub fn phy_world_rigid_count(w: *mut PhyWorldHandle) -> usize;
    pub fn phy_world_get_rigid_transforms(
        w: *mut PhyWorldHandle,
        buf: *mut f64,
        len: usize,
    ) -> usize;

    // ---- 刚体单实例操控（P-quad，ABI v2）----
    pub fn phy_world_rigid_add_body(
        w: *mut PhyWorldHandle,
        shape_kind: i32,
        mass: f64,
        pos7: *const f64,
        inertia3: *const f64,
    ) -> i64;
    pub fn phy_world_rigid_apply_force(
        w: *mut PhyWorldHandle,
        id: i64,
        f3: *const f64,
        dt: f64,
        mode: i32,
    ) -> i32;
    pub fn phy_world_rigid_apply_torque(
        w: *mut PhyWorldHandle,
        id: i64,
        t3: *const f64,
        dt: f64,
        mode: i32,
    ) -> i32;
    pub fn phy_world_rigid_get_velocity(
        w: *mut PhyWorldHandle,
        id: i64,
        out3: *mut f64,
    ) -> i32;
    pub fn phy_world_rigid_get_angular_velocity(
        w: *mut PhyWorldHandle,
        id: i64,
        out3: *mut f64,
    ) -> i32;

    // ---- 释放 ----
    pub fn phy_world_destroy(w: *mut PhyWorldHandle);

    // 序列化（暂未用，预留）。
    pub fn phy_world_save(w: *mut PhyWorldHandle, path: *const c_char) -> i32;
    pub fn phy_world_load(path: *const c_char) -> *mut PhyWorldHandle;
}
