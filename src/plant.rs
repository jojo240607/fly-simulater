//! 被控对象：把物理引擎（`phy_ffi` 预编译库）接入飞控闭环。
//!
//! 职责：
//! 1. 持有物理引擎世界句柄，index 0 = 四旋翼机体刚体。
//! 2. 旋翼推进模型（阶段 2 高保真）：
//!    - 电机一阶滞后：`thrust_actual` 指数趋近目标油门（tau=cfg.motor_tau）。
//!    - 旋翼推力：4 路实际推力 → 世界系力（沿机体 +Z）+ 臂力矩 + 反扭矩。
//!    - 机体气动阻力：型阻（0.5·ρ·Cd·|v|·v，三轴）+ 诱导阻力（随前飞速度，
//!      悬停≈0，前飞→k·T 沿 -Z），在机体坐标系施加。
//!    - 地面效应：近地（h < 1.5×桨径）推力增益 +30%。
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

use crate::physics::RigidBodyWorld;
use crate::wind::{WindField, WindVec};
use crate::sensor::{SensorConfig, SensorModel};

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

// ============================================================ 阶段 2 推进模型辅助

/// 阶段 2c：地面效应增益。机体高度 h（世界 Y，米）低于约 1.5×桨径时推力增强。
/// 模型：h < h0 时 gain = 1 + k*(1 - h/h0)，线性趋于地面；h>=h0 时 gain=1。
fn ground_effect_gain(h: f64, prop_diam: f64) -> f64 {
    let h0 = 1.5 * prop_diam; // 效应显著高度上限
    if h <= 0.0 {
        return 1.0 + 0.30; // 贴地最大 +30%（经验值）
    }
    if h >= h0 {
        return 1.0;
    }
    let k = 0.30;
    1.0 + k * (1.0 - h / h0)
}

/// 机体气动阻力返回结构（机体坐标系三轴力）。
struct AeroDrag {
    fx: [f64; 3],
}

// ============================================================ 被控对象

pub struct QuadrotorPlant<W> {
    world: W,
    body_id: i64,
    cfg: VehicleConfig,
    dt: f64,
    /// 上一帧世界系线速度（数值微分算加速度）。
    prev_vel_up: [f64; 3],
    /// 控制器下发的目标油门（4 路归一化，[0,1]），由电机一阶滞后趋近实际推力。
    cmd_motor: [f64; 4],
    /// 电机一阶滞后后的实际归一化推力（用于阶段 2 电机动态）。
    thrust_actual: [f64; 4],
    /// 当前帧缓存的 4 路实际推力（N），供 step 前注入。
    thrust_n: [f64; 4],
    gravity: f64,
    time: f64,
    /// 阶段 3：可选风场（世界系 UP 风速）。None = 无风。
    wind: Option<WindField>,
    /// 阶段 4：传感器真实化模型（IMU 噪声/偏置/GPS 延迟丢星）。
    sensor: SensorModel,
}

impl<W> QuadrotorPlant<W>
where
    W: RigidBodyWorld,
{
    /// 在世界中创建机体刚体。mass / 转动惯量来自机型配置。
    /// `world`：实现了 `RigidBodyWorld` 的物理世界（真实引擎或测试替身）。
    /// `wind`：可选风场（阶段 3 抗风/前飞场景）。
    /// `sensor_cfg`：传感器模型配置（阶段 4；默认零噪声保持场景 PASS）。
    pub fn new(world: W, cfg: &VehicleConfig, dt: f64, wind: Option<WindField>, sensor_cfg: SensorConfig) -> Self {
        let mut world = world;
        // 静态地面盒（质量 0 = 无限质量，不动）。位于 y=-5，半高 0.5。
        let ground_pos7: [f64; 7] = [0.0, -5.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let ground_inertia: [f64; 3] = [1.0, 1.0, 1.0];
        let _ground = world.add_body(0.0 /*mass=0 静态*/, &ground_pos7, &ground_inertia);

        // 初始位姿：NED (0,0,-5) = 悬停 5m 高 -> 引擎 (0, 5, 0)。
        // 初始姿态：机体"上"轴(+Z, 引擎机体系)对齐世界 +Y(上)，即绕 X 轴 +90°。
        // 这样旋翼推力(沿机体 +Z)初始向上托住重力（引擎重力沿 -Y）。
        let c = (std::f64::consts::FRAC_PI_4).cos(); // cos45
        let s = (std::f64::consts::FRAC_PI_4).sin(); // sin45
        // 绕 X 轴 -90°：使机体"上"轴(+Z, 引擎机体系)对齐世界 +Y(上)。
        // （验证：rotate_by_quat((c,-s,0,0),(0,0,1)) = (0,1,0) 向上）
        let pos7: [f64; 7] = [0.0, 5.0, 0.0, c, -s, 0.0, 0.0]; // q=(cos45, -sin45,0,0) 绕X-90°
        // 主转动惯量：使用机型配置的真实惯量（阶段 1，不再用 m·L² 近似）。
        // 断言正定，避免物理引擎收到非物理惯量。
        assert!(
            cfg.inertia[0] > 0.0 && cfg.inertia[1] > 0.0 && cfg.inertia[2] > 0.0,
            "机架惯量必须为正定对角: {:?}",
            cfg.inertia
        );
        let inertia3: [f64; 3] = [
            cfg.inertia[0] as f64,
            cfg.inertia[1] as f64,
            cfg.inertia[2] as f64,
        ];
        let body_id = world.add_body(cfg.mass as f64, &pos7, &inertia3);
        assert!(body_id >= 0, "add_body 失败");

        Self {
            world,
            body_id,
            cfg: cfg.clone(),
            dt,
            prev_vel_up: [0.0; 3],
            cmd_motor: [0.0; 4],
            thrust_actual: [0.0; 4],
            thrust_n: [0.0; 4],
            gravity: 9.81,
            time: 0.0,
            wind,
            sensor: SensorModel::new(sensor_cfg, dt),
        }
    }

    /// 保存本拍 4 路归一化油门指令（[0,1]），由电机一阶滞后在 step 内趋近实际推力。
    pub fn apply_actuators(&mut self, cmd: &ActuatorCmd) {
        for i in 0..4 {
            self.cmd_motor[i] = cmd.motor[i] as f64;
        }
    }

    /// 推进一个物理步：先把旋翼力/力矩注入机体，再 step。
    pub fn step(&mut self) {
        // ---- 阶段 2a：电机一阶滞后 ----
        // 实际推力指数趋近目标：thrust_actual += (cmd - actual)·(dt/tau)
        // （motor_tau 已过 airframe 注入 cfg；tau=0 退化为无滞后直接跟随）
        let tau = (self.cfg.motor_tau as f64).max(1e-6);
        let alpha = (self.dt / tau).min(1.0); // 数值稳定，dt>>tau 时整步跳变
        for i in 0..4 {
            let target = self.cmd_motor[i].clamp(0.0, 1.0);
            self.thrust_actual[i] += (target - self.thrust_actual[i]) * alpha;
        }

        // ---- 旋翼推进模型（引擎世界系，f64）----
        // 取当前引擎姿态（按 body_id 偏移）用于把机体力/矩旋到世界系。
        let tf = self.read_body_tf();
        let q = [tf[3], tf[4], tf[5], tf[6]];
        // 当前高度（世界系 Y 为"上"，NED 下向=-Y），用于地面效应。
        let h = tf[1];
        // 阶段 2c：地面效应增益（近地推力增强），桨径估为 0.4·臂长。
        let prop_diam = 0.4 * self.cfg.arm_length as f64;
        let ge = ground_effect_gain(h, prop_diam);

        // 4 路实际推力（含地面效应增益与电机滞后）。
        let mut t_n = [0.0f64; 4];
        for i in 0..4 {
            t_n[i] = self.cfg.thrust_coeff as f64 * self.thrust_actual[i] * ge;
        }
        self.thrust_n = t_n;

        // 机体合力（引擎机体系）：推力沿机体 +Z。
        let sum_t: f64 = t_n.iter().sum();
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
            let t = t_n[i];
            // r × (0,0,t) = (ry*t, -rx*t, 0)
            tau_body[0] += ry * t;
            tau_body[1] += -rx * t;
            // 反扭矩（绕推力轴，机体 Z）：CCW 正转 -> 机体受 CW 反扭矩
            let tq = spin[i] * self.cfg.torque_coeff as f64 * t;
            tau_body[2] += tq;
        }
        // 阶段 3：推进风场，取当前世界系（UP）风速。无风则为 0。
        let wind_up: WindVec = match &mut self.wind {
            Some(w) => w.sample(self.dt),
            None => [0.0; 3],
        };
        // 阶段 2b：机体气动阻力（含诱导阻力），基于相对风速（v_body - wind），在机体坐标系施加。
        let aero = self.aero_drag_body(q, &wind_up);
        // 机体合力 = 旋翼推力 + 气动阻力；合力矩 = 旋翼力矩（阻力矩略，量级小）
        let mut f_body_tot = [0.0, 0.0, 0.0];
        for k in 0..3 {
            f_body_tot[k] = f_body[k] + aero.fx[k];
        }
        let f_world_tot = rotate_by_quat(q, f_body_tot);
        let tau_world = rotate_by_quat(q, tau_body);

        self.world.apply_force(self.body_id, &f_world_tot, self.dt, 0);
        self.world.apply_torque(self.body_id, &tau_world, self.dt, 0);

        // ---- 步进物理引擎 ----
        let rc = self.world.step(self.dt);
        assert_eq!(rc, 0, "物理引擎 step 检测到 NaN/Inf，世界已损坏");
        self.time += self.dt;
    }

    /// 阶段 2b：机体坐标系气动阻力（含动量理论诱导阻力）。
    /// 基于相对风速 `v_rel = v_body - wind`（阶段 3 风场）。
    /// 返回机体坐标系三轴力 [fx,fy,fz]（N），与世界系推力叠加前先旋到世界系。
    fn aero_drag_body(&self, q: [f64; 4], wind_up: &WindVec) -> AeroDrag {
        // 机体线速度（世界系 -> 机体系）：用现成 rotate_by_quat_conj。
        let vel = self.world.get_velocity(self.body_id);
        // 相对风速：机体速度 - 风速（同世界系）。
        let mut v_rel = [0.0f64; 3];
        for i in 0..3 {
            v_rel[i] = vel[i] - wind_up[i];
        }
        let vb = rotate_by_quat_conj(q, v_rel);

        // 三项阻力（机体三轴，前/右/下方向）：
        //  (1) 型阻：0.5*rho*Cd_i*|v|*v_i （随速度平方，悬停时可忽略）
        //  (2) 诱导阻力：随前飞速度增大（悬停→0，前飞→k*T）。
        //      物理上诱导阻力是"为产生升力而伴随的前飞阻力"，与水平速度相关，
        //      悬停时无水平速度故≈0，不应是恒常下拉力（否则制造虚假稳态偏置）。
        let rho = self.cfg.air_density as f64;
        let sum_t: f64 = self.thrust_n.iter().sum();
        let vh = (vb[0] * vb[0] + vb[1] * vb[1]).sqrt(); // 机体水平速度
        let v_ref = 1.0; // 特征速度 (m/s)，sigmoid 拐点
        let induced_factor = (vh * vh) / (vh * vh + v_ref * v_ref);
        let di = self.cfg.induced_drag_coeff as f64 * sum_t * induced_factor;

        let mut fx = [0.0f64; 3];
        for i in 0..3 {
            let v = vb[i];
            let cd = self.cfg.drag_coeff[i] as f64;
            // 型阻（沿速度反向）
            fx[i] -= 0.5 * rho * cd * v * v.abs();
        }
        // 诱导阻力沿机体 -Z（与推力反向）。sanity clamp 防止异常值。
        fx[2] -= di.min(sum_t * 0.5);

        AeroDrag { fx }
    }

    /// 由刚体真值生成传感器样本（NED 语义）喂飞控。
    pub fn read_sensors(&mut self) -> (ImuSample, Option<PosSample>) {
        let mut tf = self.read_body_tf();
        let vel = self.world.get_velocity(self.body_id);
        let ang = self.world.get_angular_velocity(self.body_id);

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

        // NED 速度（真值）：世界 UP 系 vel 转 NED。
        let vel_ned = vec_up_to_ned(vel);

        // 阶段 4：真值过传感器模型（噪声/偏置/延迟/丢星）。
        let (imu, pos_sample) = self.sensor.process(
            self.dt,
            accel_fc,
            omega_fc,
            pos_ned,
            vel_ned,
        );
        (imu, pos_sample)
    }

    /// 按 `body_id` 偏移读回该刚体的 7 元组 (pos.xyz + quat.wxyz)。
    /// 通过 `get_rigid_transforms` 批量读回后取本 body 段（trait 接口形态，引擎/替身一致）。
    fn read_body_tf(&self) -> [f64; 7] {
        let len = (self.body_id as usize + 1) * 7;
        let mut buf = vec![0.0f64; len];
        self.world.get_rigid_transforms(&mut buf);
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
        let vel = self.world.get_velocity(self.body_id);
        let ang = self.world.get_angular_velocity(self.body_id);
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
