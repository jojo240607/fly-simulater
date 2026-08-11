//! 被控对象：把物理引擎（`phy_ffi` 预编译库）接入飞控闭环。
//!
//! 职责：
//! 1. 持有物理引擎世界句柄，index 0 = 四旋翼机体刚体。
//! 2. 旋翼推进模型：把 4 路归一化推力 `ActuatorCmd` 转成世界系力/力矩，
//!    经 FFI `apply_force`/`apply_torque` 注入（半隐式欧拉，step 前每帧一次）。
//! 3. 坐标桥接：物理引擎是 Y-up 世界系 / 机体(前-右-上)，飞控是 NED 世界系 /
//!    机体(前-右-下)。所有轴映射只在 `plant` 边界发生（见 `coord` 模块）。
//! 4. `read_sensors`：由刚体真值生成 `ImuSample`/`PosSample`（NED 语义）喂飞控。

use std::os::raw::c_double;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{
    ActuatorCmd, ImuSample, PosSample, Quaternion, VehicleState,
};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};

use crate::phy_ffi::{self, PhyWorldHandle};

// ============================================================ 坐标桥接
//
// 物理引擎（Y-up）：世界 x=北,y=上,z=-东（右手）；机体 前-X,右-Y,上-Z。
// 飞控（NED）     ：世界 北-X,东-Y,下-Z；机体 前-X,右-Y,下-Z。
//
// 世界位置：ned(n,e,d) <-> up(x,y,z): x=n, y=-d, z=-e
// 机体轴：  飞控机体下-Z = 引擎机体上+Z，故引擎机体四元数 = 飞控 q * q_flipX，
//           q_flipX = (w=0, x=1, y=0, z=0) 绕 X 转 180°。

/// 引擎机体(前-右-上)四元数 -> 飞控机体(前-右-下)四元数。
fn quat_up_to_ned(q_up: [f64; 4]) -> Quaternion {
    // q_flipX * q_up   (绕 X 转 180° 把上轴翻成下轴)
    let flip = Quaternion { w: 0.0, x: 1.0, y: 0.0, z: 0.0 };
    let q = Quaternion {
        w: q_up[0] as f32,
        x: q_up[1] as f32,
        y: q_up[2] as f32,
        z: q_up[3] as f32,
    };
    flip * q
}

/// 飞控机体(前-右-下)四元数 -> 引擎机体(前-右-上)四元数。
fn quat_ned_to_up(q_ned: Quaternion) -> [f64; 4] {
    let flip = Quaternion { w: 0.0, x: 1.0, y: 0.0, z: 0.0 };
    let q = flip * q_ned; // q_flipX * q_ned
    [q.w as f64, q.x as f64, q.y as f64, q.z as f64]
}

/// NED 世界向量 (n,e,d) -> 引擎世界向量 (x,y,z)。
fn vec_ned_to_up(ned: [f32; 3]) -> [f64; 3] {
    [ned[0] as f64, -ned[2] as f64, -ned[1] as f64]
}

/// 引擎世界向量 (x,y,z) -> NED 世界向量 (n,e,d)。
fn vec_up_to_ned(up: [f64; 3]) -> [f32; 3] {
    [up[0] as f32, -up[2] as f32, -up[1] as f32]
}

// ============================================================ 被控对象

pub struct QuadrotorPlant {
    world: *mut PhyWorldHandle,
    body_id: i64,
    cfg: VehicleConfig,
    dt: f64,
    /// 上一帧世界系线速度（数值微分算加速度）。
    prev_vel_up: [f64; 3],
    /// 当前帧缓存的 4 路推力（N），供 step 前注入。
    thrust_n: [f64; 4],
    gravity: f64,
    time: f64,
}

impl QuadrotorPlant {
    /// 在世界中创建机体刚体。mass / 转动惯量来自机型配置。
    pub fn new(cfg: &VehicleConfig, dt: f64) -> Self {
        // 空刚体世界（无 demo 地面/球），自行添加机体与可选地面。
        let world = unsafe { phy_ffi::phy_world_create_rigid_empty() };
        assert!(!world.is_null(), "phy_world_create_rigid_empty 失败");

        // 静态地面盒（质量 0 = 无限质量，不动）。位于 y=-5，半高 0.5。
        let ground_pos7: [f64; 7] = [0.0, -5.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let ground_inertia: [f64; 3] = [1.0, 1.0, 1.0];
        let _ground = unsafe {
            phy_ffi::phy_world_rigid_add_body(
                world, 1 /*Box*/, 0.0 /*mass=0 静态*/, ground_pos7.as_ptr(), ground_inertia.as_ptr(),
            )
        };

        // 初始位姿：NED (0,0,-5) = 悬停 5m 高 -> 引擎 (0, 5, 0)。
        // 初始姿态：机体"上"轴(+Z, 引擎机体系)对齐世界 +Y(上)，即绕 X 轴 +90°。
        // 这样旋翼推力(沿机体 +Z)初始向上托住重力（引擎重力沿 -Y）。
        let c = (std::f64::consts::FRAC_PI_4).cos(); // cos45
        let s = (std::f64::consts::FRAC_PI_4).sin(); // sin45
        // 绕 X 轴 -90°：使机体"上"轴(+Z, 引擎机体系)对齐世界 +Y(上)。
        // （验证：rotate_by_quat((c,-s,0,0),(0,0,1)) = (0,1,0) 向上）
        let pos7: [f64; 7] = [0.0, 5.0, 0.0, c, -s, 0.0, 0.0]; // q=(cos45, -sin45,0,0) 绕X-90°
        // 主转动惯量：用质量 * 臂长² 量级的近似对角（X/Y 对称）。
        let i = cfg.mass as f64 * cfg.arm_length as f64 * cfg.arm_length as f64;
        let inertia3: [f64; 3] = [i, i, 2.0 * i]; // Izz ~ 2*Ixx 对 X 四旋翼典型
        let body_id = unsafe {
            phy_ffi::phy_world_rigid_add_body(world, 1 /*Box*/, cfg.mass as f64, pos7.as_ptr(), inertia3.as_ptr())
        };
        assert!(body_id >= 0, "phy_world_rigid_add_body 失败");

        Self {
            world,
            body_id,
            cfg: cfg.clone(),
            dt,
            prev_vel_up: [0.0; 3],
            thrust_n: [0.0; 4],
            gravity: 9.81,
            time: 0.0,
        }
    }

    /// 保存本拍 4 路归一化推力，供 `step` 前注入为世界系力/力矩。
    pub fn apply_actuators(&mut self, cmd: &ActuatorCmd) {
        for i in 0..4 {
            self.thrust_n[i] = self.cfg.thrust_coeff as f64 * cmd.motor[i] as f64;
        }
    }

    /// 推进一个物理步：先把旋翼力/力矩注入机体，再 step。
    pub fn step(&mut self) {
        // ---- 旋翼推进模型（引擎世界系，f64）----
        // 取当前引擎姿态（按 body_id 偏移）用于把机体力/矩旋到世界系。
        let tf = self.read_body_tf();
        let q = [tf[3], tf[4], tf[5], tf[6]];
        // 机体合力（引擎机体系）：推力沿机体 +Z。
        let sum_t: f64 = self.thrust_n.iter().sum();
        let f_body = [0.0, 0.0, sum_t];
        let f_world = rotate_by_quat(q, f_body);

        // 机体合力矩（引擎机体系）：臂力矩 + 反扭矩。
        let l = self.cfg.arm_length as f64;
        // X 布局臂向量（前+X, 右+Y）：m0=前右, m1=后左, m2=前左, m3=后右
        let arms: [[f64; 2]; 4] = [
            [l, l],   // 0 前右
            [-l, -l], // 1 后左
            [l, -l],  // 2 前左
            [-l, l],  // 3 后右
        ];
        // spin: 0,1 CCW(+1), 2,3 CW(-1)
        let spin: [f64; 4] = [1.0, 1.0, -1.0, -1.0];
        let mut tau_body = [0.0f64; 3];
        for i in 0..4 {
            let (rx, ry) = (arms[i][0], arms[i][1]);
            let t = self.thrust_n[i];
            // r × (0,0,t) = (ry*t, -rx*t, 0)
            tau_body[0] += ry * t;
            tau_body[1] += -rx * t;
            // 反扭矩（绕推力轴，机体 Z）：CCW 正转 -> 机体受 CW 反扭矩
            let tq = spin[i] * self.cfg.torque_coeff as f64 * t;
            tau_body[2] += tq;
        }
        let tau_world = rotate_by_quat(q, tau_body);

        unsafe {
            phy_ffi::phy_world_rigid_apply_force(
                self.world, self.body_id, f_world.as_ptr(), self.dt, 0,
            );
            phy_ffi::phy_world_rigid_apply_torque(
                self.world, self.body_id, tau_world.as_ptr(), self.dt, 0,
            );
        }

        // ---- 步进物理引擎 ----
        let rc = unsafe { phy_ffi::phy_world_step_checked(self.world, self.dt) };
        assert_eq!(rc, 0, "物理引擎 step 检测到 NaN/Inf，世界已损坏");
        self.time += self.dt;
    }

    /// 由刚体真值生成传感器样本（NED 语义）喂飞控。
    pub fn read_sensors(&mut self) -> (ImuSample, Option<PosSample>) {
        let mut tf = self.read_body_tf();
        let mut vel = [0.0f64; 3];
        let mut ang = [0.0f64; 3];
        unsafe {
            phy_ffi::phy_world_rigid_get_velocity(self.world, self.body_id, vel.as_mut_ptr());
            phy_ffi::phy_world_rigid_get_angular_velocity(self.world, self.body_id, ang.as_mut_ptr());
        }

        let pos_up = [tf[0], tf[1], tf[2]];
        let q_up = [tf[3], tf[4], tf[5], tf[6]];
        let pos_ned = vec_up_to_ned(pos_up); // [n, e, d]

        // 角速度：引擎机体(前-右-上) -> 飞控机体(前-右-下)，Z 轴翻转。
        let omega_fc = [ang[0] as f32, ang[1] as f32, -ang[2] as f32];

        // 比力（机体，不含重力）：数值微分世界速度得 a_world，减重力项后旋到机体。
        let a_world = [
            (vel[0] - self.prev_vel_up[0]) / self.dt,
            (vel[1] - self.prev_vel_up[1]) / self.dt,
            (vel[2] - self.prev_vel_up[2]) / self.dt,
        ];
        self.prev_vel_up = vel;
        // 比力 = a_world - g_world(引擎系 (0,-g,0))
        let specific_force_world = [a_world[0], a_world[1] + self.gravity, a_world[2]];
        let sf_body_up = rotate_by_quat_conj(q_up, specific_force_world);
        // 引擎机体(上+Z) -> 飞控机体(下-Z)：z 翻转
        let accel_fc = [sf_body_up[0] as f32, sf_body_up[1] as f32, -sf_body_up[2] as f32];

        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(accel_fc[0]),
                MeterPerSecondSquared(accel_fc[1]),
                MeterPerSecondSquared(accel_fc[2]),
            ],
            gyro: [
                RadianPerSecond(omega_fc[0]),
                RadianPerSecond(omega_fc[1]),
                RadianPerSecond(omega_fc[2]),
            ],
        };
        let pos = PosSample {
            pos: [Meter(pos_ned[0]), Meter(pos_ned[1]), Meter(pos_ned[2])],
        };
        (imu, Some(pos))
    }

    /// 按 `body_id` 偏移读回该刚体的 7 元组 (pos.xyz + quat.wxyz)。
    /// `phy_world_get_rigid_transforms` 不带 id，故分配足够 buf 并取末尾 7 个。
    fn read_body_tf(&self) -> [f64; 7] {
        let len = (self.body_id as usize + 1) * 7;
        let mut buf = vec![0.0f64; len];
        unsafe {
            phy_ffi::phy_world_get_rigid_transforms(self.world, buf.as_mut_ptr(), len);
        }
        let off = self.body_id as usize * 7;
        [
            buf[off], buf[off + 1], buf[off + 2], buf[off + 3], buf[off + 4], buf[off + 5],
            buf[off + 6],
        ]
    }

    /// 调试：返回引擎世界系真实坐标 (x,y,z) 与四元数 (w,x,y,z)，绕开 NED 映射。
    pub fn debug_up(&self) -> ([f64; 3], [f64; 4]) {
        let tf = self.read_body_tf();
        ([tf[0], tf[1], tf[2]], [tf[3], tf[4], tf[5], tf[6]])
    }

    /// 取当前 NED 世界状态（供不变量检查 / 日志）。
    pub fn state_ned(&self) -> VehicleState {
        let tf = self.read_body_tf();
        let pos_up = [tf[0], tf[1], tf[2]];
        let q_up = [tf[3], tf[4], tf[5], tf[6]];
        let pos_ned = vec_up_to_ned(pos_up);
        let quat_ned = quat_up_to_ned(q_up);
        let mut vel = [0.0f64; 3];
        let mut ang = [0.0f64; 3];
        unsafe {
            phy_ffi::phy_world_rigid_get_velocity(self.world, self.body_id, vel.as_mut_ptr());
            phy_ffi::phy_world_rigid_get_angular_velocity(self.world, self.body_id, ang.as_mut_ptr());
        }
        VehicleState {
            pos: [Meter(pos_ned[0]), Meter(pos_ned[1]), Meter(pos_ned[2])],
            vel: [
                MeterPerSecond(vec_up_to_ned(vel)[0]),
                MeterPerSecond(vec_up_to_ned(vel)[1]),
                MeterPerSecond(vec_up_to_ned(vel)[2]),
            ],
            att: quat_ned,
            omega: [RadianPerSecond(ang[0] as f32), RadianPerSecond(ang[1] as f32), RadianPerSecond(-ang[2] as f32)],
        }
    }
}

impl Drop for QuadrotorPlant {
    fn drop(&mut self) {
        unsafe { phy_ffi::phy_world_destroy(self.world) };
    }
}

// ============================================================ 四元数工具（f64, 引擎系）

/// 引擎系单位四元数 (w,x,y,z) 旋转向量 v（机体->世界）。
fn rotate_by_quat(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - w * z);
    let r02 = 2.0 * (x * z + w * y);
    let r10 = 2.0 * (x * y + w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - w * x);
    let r20 = 2.0 * (x * z - w * y);
    let r21 = 2.0 * (y * z + w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

/// 引擎系单位四元数共轭（世界->机体）旋转向量 v。
fn rotate_by_quat_conj(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y + w * z);
    let r02 = 2.0 * (x * z - w * y);
    let r10 = 2.0 * (x * y - w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z + w * x);
    let r20 = 2.0 * (x * z + w * y);
    let r21 = 2.0 * (y * z - w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

// 抑制未使用告警：c_double 仅用于明确 ABI 宽度。
#[allow(dead_code)]
fn _assert_c_double(_: c_double) {}
