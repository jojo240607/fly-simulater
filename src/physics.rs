//! 物理引擎抽象层（阶段 7：依赖倒置）。
//!
//! 仿真层（`plant.rs` 的四旋翼推进模型）只依赖本模块定义的 `RigidBodyWorld` trait，
//! 而不依赖具体物理引擎（C 库 FFI 或测试替身）。这样替换物理引擎实现（或注入玩具级
//! 替身做单元测试）无需改动任何仿真逻辑。
//!
//! **接口语义约定（实现者必须遵守）**：
//! - `apply_force` / `apply_torque` 施加的是**瞬态**力/力矩：引擎在 `step` 内消费后
//!   必须清零，下一帧不会残留。这保证四旋翼每帧重算重注入，不会出现幽灵推力。
//! - `step(dt)` 推进动力学并返回状态码（0=OK，非 0=检测到非有限状态，世界损坏）。
//! - `get_rigid_transforms` 批量读回所有刚体的 `(pos.xyz + quat.wxyz)` 7 元组，
//!   按 body_id 顺序排列。这是真实引擎 ABI 的接口形态，替身也实现它，保持统一。

use std::os::raw::c_double;
use crate::phy_ffi;

/// 刚体 7 元组：(pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z)。
/// 世界系与引擎一致（Y-up，右手）。
pub type RigidTransform = [f64; 7];

/// 物理引擎最小接口（仿真层依赖此 trait，而非具体引擎）。
///
/// 设计取舍（与"通用物理引擎接口"提案的差异，见 PLAN 阶段 7 讨论）：
/// - 不暴露 `set_state` / `at_point`：四旋翼推力沿机体过质心轴，纯力矩用 `apply_torque`，
///   `at_point` 传 None 即可，无需力臂；引擎持有状态，仿真层不写回。
/// - 保留批量 `get_rigid_transforms`：真实引擎 ABI 即此形态，替身对齐避免为单 body 改 ABI。
/// - 重力矢量由引擎内部持有（世界系 (0,-g,0)），不每次传入。
pub trait RigidBodyWorld {
    /// 当前刚体数量。
    fn body_count(&self) -> usize;

    /// 在世界中添加一个刚体，返回其 body_id（>=0）。
    /// `pos7` = (x,y,z, qw,qx,qy,qz)；`inertia3` = 主转动惯量 (Ixx,Iyy,Izz)。
    /// `mass == 0` 表示静态刚体（无限质量，不受力运动，用作地面）。
    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64;

    /// 施加瞬态力（世界系 3 向量，N）。`mode` 透传给底层（0=默认）。
    fn apply_force(&mut self, id: i64, f3: &[f64; 3], dt: f64, mode: i32);

    /// 施加瞬态力矩（世界系 3 向量，N·m）。
    fn apply_torque(&mut self, id: i64, t3: &[f64; 3], dt: f64, mode: i32);

    /// 读回线速度（世界系，m/s）。
    fn get_velocity(&self, id: i64) -> [f64; 3];

    /// 读回角速度（世界系，rad/s）。
    fn get_angular_velocity(&self, id: i64) -> [f64; 3];

    /// 批量读回所有刚体 7 元组到 `buf`（长度需 >= body_count()*7），返回写入个数。
    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize;

    /// 推进一个时间步，返回状态码（0=OK）。
    fn step(&mut self, dt: f64) -> i32;

    /// 世界累计时间（s）。
    fn time(&self) -> f64;
}

// ============================================================ FFI adapter

/// 真实物理引擎的 Rust 适配器：包装 `phy_ffi` C-ABI 调用。
///
/// 这是 `RigidBodyWorld` 的生产实现，把不透明指针 + 外部函数包成 trait 方法，
/// 让 `plant` 无需 `unsafe` 与外部符号纠缠。
pub struct PhyFfiWorld {
    handle: *mut phy_ffi::PhyWorldHandle,
}

impl PhyFfiWorld {
    /// 创建空刚体世界（无 demo 物体），由仿真层自行添加机体与地面。
    pub fn create_empty() -> Self {
        let handle = unsafe { phy_ffi::phy_world_create_rigid_empty() };
        assert!(!handle.is_null(), "phy_world_create_rigid_empty 失败");
        Self { handle }
    }
}

impl RigidBodyWorld for PhyFfiWorld {
    fn body_count(&self) -> usize {
        unsafe { phy_ffi::phy_world_rigid_count(self.handle) }
    }

    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64 {
        unsafe {
            phy_ffi::phy_world_rigid_add_body(
                self.handle,
                1, /* Box */
                mass,
                pos7.as_ptr(),
                inertia3.as_ptr(),
            )
        }
    }

    fn apply_force(&mut self, id: i64, f3: &[f64; 3], dt: f64, mode: i32) {
        unsafe {
            phy_ffi::phy_world_rigid_apply_force(self.handle, id, f3.as_ptr(), dt, mode);
        }
    }

    fn apply_torque(&mut self, id: i64, t3: &[f64; 3], dt: f64, mode: i32) {
        unsafe {
            phy_ffi::phy_world_rigid_apply_torque(self.handle, id, t3.as_ptr(), dt, mode);
        }
    }

    fn get_velocity(&self, id: i64) -> [f64; 3] {
        let mut v = [0.0f64; 3];
        unsafe {
            phy_ffi::phy_world_rigid_get_velocity(self.handle, id, v.as_mut_ptr());
        }
        v
    }

    fn get_angular_velocity(&self, id: i64) -> [f64; 3] {
        let mut a = [0.0f64; 3];
        unsafe {
            phy_ffi::phy_world_rigid_get_angular_velocity(self.handle, id, a.as_mut_ptr());
        }
        a
    }

    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize {
        unsafe { phy_ffi::phy_world_get_rigid_transforms(self.handle, buf.as_mut_ptr(), buf.len()) }
    }

    fn step(&mut self, dt: f64) -> i32 {
        unsafe { phy_ffi::phy_world_step_checked(self.handle, dt) }
    }

    fn time(&self) -> f64 {
        unsafe { phy_ffi::phy_world_time(self.handle) }
    }
}

impl Drop for PhyFfiWorld {
    fn drop(&mut self) {
        unsafe { phy_ffi::phy_world_destroy(self.handle) };
    }
}

// ============================================================ 玩具级替身（测试用）

/// 玩具级物理世界：半隐式欧拉刚体积分，固定重力 (0,-g,0)，简单地面碰撞。
///
/// 目的（见阶段 7 设计讨论）：
/// 1. 作为"测试替身"验证 `plant.rs` 的力/力矩施加逻辑正确，**无需启动 C 物理引擎**；
/// 2. 锁住 phase 2/3/4 真实度回归（推力→上升、力矩→角速度、地面效应→近地增益）；
/// 3. 验证 `RigidBodyWorld` 接口的充分性——若替身无法满足仿真需求，说明接口有缺口。
///
/// 简化假设（够用即可，非高保真）：
/// - 刚体无耦合惯量（对角惯量），角速度直接 `ω += I^{-1}·τ·dt`。
/// - 姿态四元数用 `q += 0.5·(0,ω)⊗q·dt` 后归一化（足够慢速姿态积分）。
/// - 地面：y <= 0 时位置钳制 y=0、线速度 y 分量置 0（无反弹，模拟停机坪）。
/// - 瞬态力/矩：每 `step` 消费后清零（遵守 trait 语义约定）。
pub struct ToyWorld {
    mass: Vec<f64>,
    inertia: Vec<[f64; 3]>,
    pos: Vec<[f64; 3]>,
    quat: Vec<[f64; 4]>, // (w,x,y,z)
    vel: Vec<[f64; 3]>,
    ang: Vec<[f64; 3]>,
    force: Vec<[f64; 3]>, // 本帧瞬态力（step 后清零）
    torque: Vec<[f64; 3]>, // 本帧瞬态力矩
    gravity: f64,
    time: f64,
}

impl ToyWorld {
    pub fn new(gravity: f64) -> Self {
        Self {
            mass: Vec::new(),
            inertia: Vec::new(),
            pos: Vec::new(),
            quat: Vec::new(),
            vel: Vec::new(),
            ang: Vec::new(),
            force: Vec::new(),
            torque: Vec::new(),
            gravity,
            time: 0.0,
        }
    }

    fn quat_mul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
        // (w,x,y,z) Hamilton 积。
        let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
        let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    }

    fn quat_norm(q: &mut [f64; 4]) {
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        if n > 1e-12 {
            for i in 0..4 {
                q[i] /= n;
            }
        }
    }
}

impl RigidBodyWorld for ToyWorld {
    fn body_count(&self) -> usize {
        self.mass.len()
    }

    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64 {
        let id = self.mass.len() as i64;
        self.mass.push(mass);
        self.inertia.push(*inertia3);
        self.pos.push([pos7[0], pos7[1], pos7[2]]);
        self.quat.push([pos7[3], pos7[4], pos7[5], pos7[6]]);
        self.vel.push([0.0; 3]);
        self.ang.push([0.0; 3]);
        self.force.push([0.0; 3]);
        self.torque.push([0.0; 3]);
        id
    }

    fn apply_force(&mut self, id: i64, f3: &[f64; 3], _dt: f64, _mode: i32) {
        let i = id as usize;
        for k in 0..3 {
            self.force[i][k] += f3[k];
        }
    }

    fn apply_torque(&mut self, id: i64, t3: &[f64; 3], _dt: f64, _mode: i32) {
        let i = id as usize;
        for k in 0..3 {
            self.torque[i][k] += t3[k];
        }
    }

    fn get_velocity(&self, id: i64) -> [f64; 3] {
        self.vel[id as usize]
    }

    fn get_angular_velocity(&self, id: i64) -> [f64; 3] {
        self.ang[id as usize]
    }

    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize {
        let n = self.mass.len();
        let need = n * 7;
        let len = buf.len().min(need);
        for i in 0..len / 7 {
            let off = i * 7;
            buf[off..off + 3].copy_from_slice(&self.pos[i]);
            buf[off + 3..off + 7].copy_from_slice(&self.quat[i]);
        }
        len
    }

    fn step(&mut self, dt: f64) -> i32 {
        for i in 0..self.mass.len() {
            let m = self.mass[i];
            if m <= 0.0 {
                // 静态刚体：不受力、不动。
                self.force[i] = [0.0; 3];
                self.torque[i] = [0.0; 3];
                continue;
            }
            // ---- 线运动：半隐式欧拉 ----
            let mut acc = [0.0f64; 3];
            for k in 0..3 {
                acc[k] = self.force[i][k] / m;
            }
            acc[1] -= self.gravity; // 重力（世界 -Y）
            for k in 0..3 {
                self.vel[i][k] += acc[k] * dt;
                self.pos[i][k] += self.vel[i][k] * dt;
            }
            // 地面碰撞：钳制 y>=0（停机坪，无反弹）。
            if self.pos[i][1] < 0.0 {
                self.pos[i][1] = 0.0;
                if self.vel[i][1] < 0.0 {
                    self.vel[i][1] = 0.0;
                }
            }

            // ---- 角运动：ω += I^{-1}·τ·dt ----
            let inv_i = [
                1.0 / self.inertia[i][0].max(1e-9),
                1.0 / self.inertia[i][1].max(1e-9),
                1.0 / self.inertia[i][2].max(1e-9),
            ];
            let mut ang_acc = [0.0f64; 3];
            for k in 0..3 {
                ang_acc[k] = self.torque[i][k] * inv_i[k];
            }
            for k in 0..3 {
                self.ang[i][k] += ang_acc[k] * dt;
            }
            // 姿态积分：q += 0.5·(0,ω)⊗q·dt，后归一化。
            let omega_q = [0.0, self.ang[i][0], self.ang[i][1], self.ang[i][2]];
            let dq = Self::quat_mul(&omega_q, &self.quat[i]);
            for k in 0..4 {
                self.quat[i][k] += 0.5 * dq[k] * dt;
            }
            Self::quat_norm(&mut self.quat[i]);

            // 瞬态力/矩清零（trait 语义约定）。
            self.force[i] = [0.0; 3];
            self.torque[i] = [0.0; 3];
        }
        self.time += dt;
        0
    }

    fn time(&self) -> f64 {
        self.time
    }
}

// 抑制未使用告警：c_double 仅用于明确 ABI 宽度一致性。
#[allow(dead_code)]
fn _assert_c_double(_: c_double) {}
