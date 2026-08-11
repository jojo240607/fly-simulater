//! 物理引擎抽象层（阶段 7：依赖倒置）。
//!
//! 仿真层（`plant.rs` 的四旋翼推进模型）只依赖本模块定义的 `RigidBodyWorld` trait，
//! 而不依赖具体物理引擎（Rust `phy-sdk` 或测试替身）。这样替换物理引擎实现（或注入玩具级
//! 替身做单元测试）无需改动任何仿真逻辑。
//!
//! **接口语义约定（实现者必须遵守）**：
//! - `apply_impulse` / `apply_torque_impulse` 施加的是**瞬态**线/角冲量（N·s / N·m·s）：
//!   这一步消耗的作用量，step 后不会残留。这保证四旋翼每帧重算重注入，不会出现幽灵推力。
//! - `step(dt)` 推进动力学并返回状态码（0=OK，非 0=检测到非有限状态，世界损坏）。
//! - `get_rigid_transforms` 批量读回所有刚体的 `(pos.xyz + quat.wxyz)` 7 元组，
//!   按 body_id 顺序排列。这是真实引擎 ABI 的接口形态，替身也实现它，保持统一。

use phy_math::Vec3 as V3;
use phy_sdk::rigid::{RigidSubsystem, RigidWorld};
use phy_sdk::{get_as, get_as_mut, PhysicsBuilder, World};

/// 刚体 7 元组：(pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z)。
/// 世界系与引擎一致（Y-up，右手）。
pub type RigidTransform = [f64; 7];

/// 物理引擎最小接口（仿真层依赖此 trait，而非具体引擎）。
///
/// 设计取舍（与"通用物理引擎接口"提案的差异，见 PLAN 阶段 7 讨论）：
/// - 不暴露 `set_state` / `at_point`：四旋翼推力沿机体过质心轴，纯力矩用 `apply_torque_impulse`，
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

    /// 施加瞬态线冲量（世界系 3 向量，N·s）。`mode` 透传给底层（0=默认）。
    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], mode: i32);

    /// 施加瞬态角冲量（世界系 3 向量，N·m·s）。
    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], mode: i32);

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

// ============================================================ phy-sdk adapter

/// 真实物理引擎的 Rust 适配器：包装 `phy-sdk`（`phy_sdk::World<f64>`）。
///
/// 这是 `RigidBodyWorld` 的生产实现，直接用 safe Rust 调 `phy-sdk` 的强类型 API，
/// 无需 `unsafe`、无 C-ABI 绑定、无运行时 DLL 依赖。物理引擎以 `rlib` 形态被
/// cargo 静态链入仿真二进制（`phy-sdk` 的 cdylib 仍保留给非 Rust 宿主，与此无关）。
pub struct PhySdkWorld {
    world: World<f64>,
    rigid_idx: usize,
    t: f64,
}

impl PhySdkWorld {
    /// 创建空刚体世界（仅启用刚体子系统，无 demo 物体），由仿真层自行添加机体与地面。
    pub fn create_empty() -> Self {
        let world: World<f64> = PhysicsBuilder::new().rigid().build();
        // rigid 是 build 时第一个（也是唯一）声明的子系统，索引 0。
        let mut s = Self { world, rigid_idx: 0, t: 0.0 };
        // 四旋翼仿真必须关闭引擎的"休眠"机制（B1 Sleeping）：
        // 悬停稳定后机体速度趋于 0，引擎会误判为"近静止"并将 body 置 sleeping、
        // 清零速度并冻结位置——这会让本应维持悬停/可坠落的机体被锁死，且 disarm
        // 后无法靠重力坠回。飞控需要世界始终积分。
        //
        // world.step 的休眠判定是"near_rest（速度低于阈值）且 sleep_time>=st 即休眠"。
        // - 不能把 sleep_time 设为 0：初始静止体 sleep_time 本为 0，0>=0 在**第一步**
        //   立即休眠（自由落体零推力场景正是如此，机体起步即冻结）。
        // - 也不能把速度阈值设为无穷大：那会让 near_rest 恒为真，sleep_time 照样累积、
        //   到默认 0.5s 后仍休眠（本 bug 第一次修复就踩中）。
        // 正确做法是把 sleep_time（休眠时长阈值 st）设为无穷大：sleep_time >= ∞ 永不
        // 成立 → 永不休眠。速度阈值保持默认即可。
        if let Some(rw) = get_as_mut::<RigidSubsystem<f64>>(&mut s.world, s.rigid_idx) {
            rw.world.params.sleep_time = f64::INFINITY;
        }
        s
    }

    fn rigid(&self) -> &RigidWorld<f64> {
        get_as(&self.world, self.rigid_idx)
            .map(|s: &RigidSubsystem<f64>| &s.world)
            .expect("rigid 子系统缺失")
    }

    fn rigid_mut(&mut self) -> &mut RigidWorld<f64> {
        get_as_mut(&mut self.world, self.rigid_idx)
            .map(|s: &mut RigidSubsystem<f64>| &mut s.world)
            .expect("rigid 子系统缺失")
    }
}

/// `Body` 的局部逆惯量对角阵（`inv_inertia_local` 为 `Mat3`，这里从主惯量构造）。
fn inv_inertia_mat3(ix: f64, iy: f64, iz: f64) -> phy_math::na::Matrix3<f64> {
    let sx = 1.0 / ix.max(1e-9);
    let sy = 1.0 / iy.max(1e-9);
    let sz = 1.0 / iz.max(1e-9);
    phy_math::na::Matrix3::from_diagonal(&V3::new(sx, sy, sz))
}

impl RigidBodyWorld for PhySdkWorld {
    fn body_count(&self) -> usize {
        self.rigid().bodies.len()
    }

    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64 {
        use phy_math::na::UnitQuaternion;
        let (px, py, pz) = (pos7[0], pos7[1], pos7[2]);
        let (qw, qx, qy, qz) = (pos7[3], pos7[4], pos7[5], pos7[6]);
        // 几何：用 Box 占位（半长 = 机臂长），贴近四旋翼体积；惯量由 inertia3 显式覆盖。
        let arm = (inertia3[0] + inertia3[1] + inertia3[2]).sqrt().max(0.1);
        let shape = phy_rigid::shape::Shape::Box {
            half: V3::new(arm, arm, arm),
        };
        let inv_mass = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        let mut b = phy_rigid::shape::Body::new(shape, V3::new(px, py, pz), inv_mass);
        b.rot = UnitQuaternion::new_normalize(phy_math::na::Quaternion::new(qw, qx, qy, qz));
        b.inv_inertia_local = inv_inertia_mat3(inertia3[0], inertia3[1], inertia3[2]);
        self.rigid_mut().add_body(b) as i64
    }

    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], _mode: i32) {
        let b = &mut self.rigid_mut().bodies[id as usize];
        b.apply_impulse(V3::new(j3[0], j3[1], j3[2]));
    }

    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], _mode: i32) {
        // 真引擎无"纯力矩"接口：角冲量 = I_world⁻¹ · k，直接注入 ang_vel。
        let b = &mut self.rigid_mut().bodies[id as usize];
        let dw = b.inv_inertia_world() * V3::new(k3[0], k3[1], k3[2]);
        b.ang_vel += dw;
    }

    fn get_velocity(&self, id: i64) -> [f64; 3] {
        let v = self.rigid().bodies[id as usize].vel;
        [v.x, v.y, v.z]
    }

    fn get_angular_velocity(&self, id: i64) -> [f64; 3] {
        let w = self.rigid().bodies[id as usize].ang_vel;
        [w.x, w.y, w.z]
    }

    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize {
        let bodies = &self.rigid().bodies;
        let n = bodies.len();
        let need = n * 7;
        let len = buf.len().min(need);
        for i in 0..len / 7 {
            let b = &bodies[i];
            let q = b.rot.quaternion(); // (w, i, j, k)
            let off = i * 7;
            buf[off] = b.pos.x;
            buf[off + 1] = b.pos.y;
            buf[off + 2] = b.pos.z;
            buf[off + 3] = q.w;
            buf[off + 4] = q.i;
            buf[off + 5] = q.j;
            buf[off + 6] = q.k;
        }
        len
    }

    fn step(&mut self, dt: f64) -> i32 {
        match self.world.step_checked(dt) {
            Ok(()) => {
                self.t += dt;
                0
            }
            Err(_) => -1,
        }
    }

    fn time(&self) -> f64 {
        self.t
    }
}

// ============================================================ 玩具级替身（测试用）

/// 玩具级物理世界：半隐式欧拉刚体积分，固定重力 (0,-g,0)，简单地面碰撞。
///
/// 目的（见阶段 7 设计讨论）：
/// 1. 作为"测试替身"验证 `plant.rs` 的冲量施加逻辑正确，**无需启动真实物理引擎**；
/// 2. 锁住 phase 2/3/4 真实度回归（推力→上升、力矩→角速度、地面效应→近地增益）；
/// 3. 验证 `RigidBodyWorld` 接口的充分性——若替身无法满足仿真需求，说明接口有缺口。
///
/// 简化假设（够用即可，非高保真）：
/// - 刚体无耦合惯量（对角惯量），角速度直接 `ω += I^{-1}·k`。
/// - 姿态四元数用 `q += 0.5·(0,ω)⊗q·dt` 后归一化（足够慢速姿态积分）。
/// - 地面：y <= 0 时位置钳制 y=0、线速度 y 分量置 0（无反弹，模拟停机坪）。
/// - 瞬态冲量：一步消耗，不跨帧残留（遵守 trait 语义约定）。
pub struct ToyWorld {
    mass: Vec<f64>,
    inertia: Vec<[f64; 3]>,
    pos: Vec<[f64; 3]>,
    quat: Vec<[f64; 4]>, // (w,x,y,z)
    vel: Vec<[f64; 3]>,
    ang: Vec<[f64; 3]>,
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
        id
    }

    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], _mode: i32) {
        let i = id as usize;
        let m = self.mass[i];
        if m <= 0.0 {
            return; // 静态刚体不受冲量。
        }
        // 线冲量直接是速度增量：Δv = j / m。
        for k in 0..3 {
            self.vel[i][k] += j3[k] / m;
        }
    }

    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], _mode: i32) {
        let i = id as usize;
        let m = self.mass[i];
        if m <= 0.0 {
            return;
        }
        // 角冲量 = I⁻¹ · k。
        let inv_i = [
            1.0 / self.inertia[i][0].max(1e-9),
            1.0 / self.inertia[i][1].max(1e-9),
            1.0 / self.inertia[i][2].max(1e-9),
        ];
        for k in 0..3 {
            self.ang[i][k] += k3[k] * inv_i[k];
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
                // 静态刚体：不动。
                continue;
            }
            // ---- 线运动：半隐式欧拉（重力 + 已注入的速度）----
            self.vel[i][1] -= self.gravity * dt; // 重力（世界 -Y）
            for k in 0..3 {
                self.pos[i][k] += self.vel[i][k] * dt;
            }
            // 地面碰撞：钳制 y>=0（停机坪，无反弹）。
            if self.pos[i][1] < 0.0 {
                self.pos[i][1] = 0.0;
                if self.vel[i][1] < 0.0 {
                    self.vel[i][1] = 0.0;
                }
            }

            // ---- 姿态积分：q += 0.5·(0,ω)⊗q·dt，后归一化 ----
            let omega_q = [0.0, self.ang[i][0], self.ang[i][1], self.ang[i][2]];
            let dq = Self::quat_mul(&omega_q, &self.quat[i]);
            for k in 0..4 {
                self.quat[i][k] += 0.5 * dq[k] * dt;
            }
            Self::quat_norm(&mut self.quat[i]);
        }
        self.time += dt;
        0
    }

    fn time(&self) -> f64 {
        self.time
    }
}
