//! 被控对象：把物理引擎接入飞控闭环。
//!
//! 职责：
//! 1. 持有物理引擎世界句柄，index 0 = 四旋翼机体刚体。
//! 2. 旋翼推进模型（阶段 2 高保真）：
//!    - 电机一阶滞后：`thrust_actual` 指数趋近目标油门（tau=cfg.motor_tau）。
//!    - 旋翼推力：4 路实际推力 → 世界系力（沿机体 +Z）+ 臂力矩 + 反扭矩。
//!    - 机体气动阻力：型阻（0.5·ρ·Cd·|v|·v，三轴）+ 诱导阻力（随前飞速度，
//!      悬停≈0，前飞→k·T 沿 -Z），在机体坐标系施加。
//!    - 地面效应：近地（h < 1.5×桨径）推力增益 +30%。
//!    经 `apply_impulse`/`apply_torque_impulse` 注入（半隐式欧拉，step 前每帧一次）。
//! 3. 坐标桥接：物理引擎是 Y-up 世界系 / 机体(前-右-上)，飞控是 NED 世界系 /
//!    机体(前-右-下)。所有轴映射只在 `plant` 边界发生。
//! 4. `read_sensors`：由刚体真值生成 `ImuSample`/`PosSample`（NED 语义）喂飞控。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{
    rotate_vec_by_quat_inverse, ActuatorCmd, ImuSample, PosSample, Quaternion, VehicleState,
};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};

use crate::physics::{ContactInfo, ContactModel, Obstacle, RigidBodyWorld};
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
    /// 控制器下发的目标油门（4 路归一化，[0,1]）。
    cmd_motor: [f64; 4],
    /// 阶段 8 动力系统：电机实际转速（rad/s），由油门×电池电压的稳定转速经一阶滞后趋近。
    motor_speed: [f64; 4],
    /// 当前电池端电压（V），随总电流（∝ω²）跌落。
    battery_v: f64,
    /// 螺旋桨推力系数 k_t（N per (rad/s)²），使满油门/满电压时单电机推力=thrust_coeff。
    prop_kt: f64,
    /// 螺旋桨反扭矩系数 k_q（N·m per (rad/s)²）。
    prop_kq: f64,
    /// 电机电效率（电气功率→机械功率）。
    motor_eta: f64,
    /// 当前帧缓存的 4 路实际推力（N），供 step 前注入。
    thrust_n: [f64; 4],
    gravity: f64,
    time: f64,
    /// 阶段 3：可选风场（世界系 UP 风速）。None = 无风。
    wind: Option<WindField>,
    /// 阶段 4：传感器真实化模型（IMU 噪声/偏置/GPS 延迟丢星）。
    sensor: SensorModel,
    /// P1-2：地面接触模型。`None` 表示无地面（真空 / 自由落体能量守恒场景）。
    /// `Some` 时每步经 `resolve_ground_contact` 惩罚模型解算接触冲量（不注入原生地面刚体）。
    contact: Option<ContactModel>,
    /// P1-2 续：静态障碍列表（碰撞体）。非空时每步经 `resolve_obstacle_contact`
    /// 惩罚模型解算碰撞冲量。机体碰撞球半径取螺旋桨外周包络（≈1.2×臂长）。
    obstacles: Vec<Obstacle>,
    /// 最近一次接触解算结果（供日志 / 调试；`None` 表示本步未接触）。
    last_contact: Option<ContactInfo>,
    /// P0-2：动量理论诱导速度（m/s），含垂直气流耦合；每步在 step() 内刷新。
    induced_vel: f64,
    /// 调试：最近一次 apply_actuators 算出的机体力矩（引擎机体系）。
    last_tau_body: [f64; 3],
}

impl<W> QuadrotorPlant<W>
where
    W: RigidBodyWorld,
{
    /// 在世界中创建机体刚体。mass / 转动惯量来自机型配置。
    /// `world`：实现了 `RigidBodyWorld` 的物理世界（真实引擎或测试替身）。
    /// `wind`：可选风场（阶段 3 抗风/前飞场景）。
    /// `sensor_cfg`：传感器模型配置（阶段 4；默认零噪声保持场景 PASS）。
    /// `contact`：P1-2 地面接触模型。`Some` 时向世界注入静态地面盒并每步解算接触；
    ///   `None` 表示无地面（真空 / 能量守恒自由落体场景）。
    /// `obstacles`：P1-2 续静态障碍列表（碰撞体）。非空时每步解算碰撞冲量。
    pub fn new(
        world: W,
        cfg: &VehicleConfig,
        dt: f64,
        wind: Option<WindField>,
        sensor_cfg: SensorConfig,
        contact: Option<ContactModel>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        let mut world = world;
        // 注：P1-2 地面接触完全由 `resolve_ground_contact` 惩罚模型处理（读取机体位姿、
        // 施加弹簧-阻尼+库仑摩擦冲量），**不向物理世界注入原生地面刚体**。这样：
        // - 真实引擎与测试替身行为一致（无"原生碰撞求解器"与惩罚模型双重接触）；
        // - 真空 / 自由落体能量守恒场景（contact=None）天然无地面，机体自由下落不发散。

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

        // 阶段 8 动力系统：由配置派生螺旋桨系数。
        // 满油门+满电池电压 → 电机稳定转速 ω_max = motor_kv·battery_v_nom，
        // 使此时单电机推力 = thrust_coeff（保持既有满油门推力标定）。
        let omega_max = (cfg.motor_kv as f64 * cfg.battery_v_nom as f64).max(1e-6);
        let prop_kt = cfg.thrust_coeff as f64 / (omega_max * omega_max);
        // 反扭矩：原 torque_coeff 是"每 N 推力"的线性系数；∝ω² 下满油门反扭矩
        // = torque_coeff·thrust_coeff，故 prop_kq = torque_coeff·thrust_coeff / ω_max²。
        let prop_kq =
            cfg.torque_coeff as f64 * cfg.thrust_coeff as f64 / (omega_max * omega_max);
        let battery_v = cfg.battery_v_nom as f64;

        Self {
            world,
            body_id,
            cfg: cfg.clone(),
            dt,
            prev_vel_up: [0.0; 3],
            cmd_motor: [0.0; 4],
            motor_speed: [0.0; 4],
            battery_v,
            prop_kt,
            prop_kq,
            motor_eta: 0.80,
            thrust_n: [0.0; 4],
            gravity: 9.81,
            time: 0.0,
            wind,
            sensor: SensorModel::new(sensor_cfg, dt),
            contact,
            obstacles,
            last_contact: None,
            induced_vel: 0.0,
            last_tau_body: [0.0; 3],
        }
    }

    /// P1-2：设置 / 清除地面接触模型（纯惩罚模型，不涉及世界刚体增删）。
    ///
    /// - `Some(m)`：启用地面接触解算（`resolve_ground_contact` 每步施加冲量）。
    /// - `None`：关闭地面接触解算（真空场景）。
    pub fn set_contact(&mut self, contact: Option<ContactModel>) {
        self.contact = contact;
    }

    /// P1-2 续：设置 / 清除静态障碍列表（纯惩罚模型，不涉及世界刚体增删）。
    ///
    /// - `obs` 非空：启用障碍碰撞解算（`resolve_obstacle_contact` 每步施加冲量）。
    /// - `obs` 空：关闭障碍碰撞解算。
    pub fn set_obstacles(&mut self, obs: Vec<Obstacle>) {
        self.obstacles = obs;
    }

    /// P1-2：读取最近一次接触解算结果（未接触时为 `None`）。
    pub fn contact_info(&self) -> Option<ContactInfo> {
        self.last_contact
    }

    /// 保存本拍 4 路归一化油门指令（[0,1]），由电机一阶滞后在 step 内趋近实际推力。
    pub fn apply_actuators(&mut self, cmd: &ActuatorCmd) {
        for i in 0..4 {
            self.cmd_motor[i] = cmd.motor[i] as f64;
        }
    }

    /// 推进一个物理步：先把旋翼力/力矩注入机体，再 step。
    pub fn step(&mut self) {
        // ---- 阶段 8 动力系统（油门→电压→转速→推力，∝ω² + 电池掉压）----
        // 用上一拍电池电压算本拍电流/掉压（一拍延迟，250Hz 足够稳定）。
        let kv = self.cfg.motor_kv as f64;
        let tau = (self.cfg.motor_tau as f64).max(1e-6);
        let alpha = (self.dt / tau).min(1.0);
        let v_bat = self.battery_v.max(1e-3);
        let mut t_n = [0.0f64; 4];
        let mut q_n = [0.0f64; 4]; // 螺旋桨反扭矩（机体 Z 轴反扭矩）
        let mut i_bat = 0.0f64;
        for i in 0..4 {
            let u = self.cmd_motor[i].clamp(0.0, 1.0);
            // 控制器语义：归一化油门 u 表示期望推力 = thrust_coeff·u（线性，与控制律兼容）。
            // 由 ∝ω² 反解所需转速 ω_req = sqrt(thrust_coeff·u / prop_kt)。
            let t_req = self.cfg.thrust_coeff as f64 * u;
            let omega_req = (t_req / self.prop_kt).max(0.0).sqrt();
            // 电机稳定转速受电池电压限制：最大转速 = kv·V_bat。
            // 掉压时 omega_ss < omega_req → 推力不足，体现"大机动掉压"。
            let omega_ss = omega_req.min(kv * v_bat);
            // 转速一阶滞后
            self.motor_speed[i] += (omega_ss - self.motor_speed[i]) * alpha;
            let om = self.motor_speed[i].max(0.0);
            // 螺旋桨：推力与反扭矩均 ∝ ω²
            let t = self.prop_kt * om * om;
            let q = self.prop_kq * om * om;
            t_n[i] = t;
            q_n[i] = q;
            // 电机电流 ≈ 机械功率 Q·ω / (电效率·端电压) + 小空载电流。
            let v_m = (u * v_bat).max(1e-3);
            let i_mech = q * om / (self.motor_eta * v_m);
            i_bat += i_mech + 1.0; // 空载电流 ~1A/电机
        }
        // 电池电压跌落：目标 V = V_oc - I·R，但用低通平滑趋近（电池电压不能瞬时跳变，
        // 化学/电容动力学），避免"油门↑→电流↑→掉压↑→转速受限→推力不足→再加油门"的正反馈发散。
        let v_target = (self.cfg.battery_v_nom as f64 - i_bat * self.cfg.battery_r as f64).max(0.0);
        let alpha_bat = (self.dt / 0.30).min(1.0); // ~0.3s 时间常数
        self.battery_v += (v_target - self.battery_v) * alpha_bat;
        self.thrust_n = t_n;

        // ---- 旋翼推进模型（引擎世界系，f64）----
        // 取当前引擎姿态（按 body_id 偏移）用于把机体力/矩旋到世界系。
        let tf = self.read_body_tf();
        let q = [tf[3], tf[4], tf[5], tf[6]];
        // 当前高度（世界系 Y 为"上"，NED 下向=-Y），用于地面效应。
        let h = tf[1];
        // 阶段 2c：地面效应增益（近地推力增强），桨径估为 0.4·臂长。
        let prop_diam = 0.4 * self.cfg.arm_length as f64;
        let ge = ground_effect_gain(h, prop_diam);
        for i in 0..4 {
            t_n[i] *= ge;
        }

        // 机体合力（引擎机体系）：推力沿机体 +Z。
        let sum_t: f64 = t_n.iter().sum();
        let f_body = [0.0, 0.0, sum_t];
        let f_world = rotate_by_quat(q, f_body);

        // ---- P0-2：滑流 / 诱导速度（动量理论）----
        // 动量理论诱导速度（含垂直气流耦合）：
        //   vi² + v·vi - T/(2·ρ·A) = 0 ，v = 穿过桨盘的空气速度（机体 Z，上正）。
        //   机体上升 (vb_z>0) → 穿过桨盘空气上流 (v<0) → vi 增大（爬升更费劲）；
        //   机体下降 (vb_z<0) → vi 减小（下降更省力，直至涡环）。
        // 解：vi = (-v + sqrt(v² + 2·T/(ρ·A))) / 2 = (vb_z + sqrt(vb_z² + 2·T/(ρ·A))) / 2。
        let rho = self.cfg.air_density as f64;
        let a_disk = (self.cfg.disk_area as f64).max(1e-4);
        let vb_now = rotate_by_quat_conj(q, self.world.get_velocity(self.body_id));
        let vz = vb_now[2]; // 机体 Z 速度（上正）
        let disc_term = 2.0 * sum_t / (rho * a_disk);
        let vi = (vz + (vz * vz + disc_term).max(0.0).sqrt()) * 0.5;
        self.induced_vel = vi;
        // 滑流冲击机体下拉力：下洗气流 vi 作用在等效投影面积 a_disk 上的动量通量，
        // 按 slipstream_drag_coeff 比例耦合到机身（沿机体 -Z）。
        let f_slip = self.cfg.slipstream_drag_coeff as f64 * 0.5 * rho * a_disk * vi * vi;
        // 滑流同时把机体略微"后推"：机身以水平速度 vb_xy 切割下洗，产生小反向阻力分量，
        // 已包含在 aero_drag_body 的诱导阻力项中，这里只加沿轴的下洗冲击部分。

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
        // 陀螺进动效应：机体角速度（世界系→机体系）用于算螺旋桨角动量进动力矩。
        let omega_world = self.world.get_angular_velocity(self.body_id);
        let omega_body = rotate_by_quat_conj(q, omega_world);
        let rotor_i = self.cfg.rotor_inertia as f64;
        let mut tau_body = [0.0f64; 3];
        for i in 0..4 {
            let (rx, ry) = (arms[i][0], arms[i][1]);
            let t = t_n[i];
            // r × (0,0,t) = (ry*t, -rx*t, 0)
            tau_body[0] += ry * t;
            tau_body[1] += -rx * t;
            // 反扭矩（绕推力轴，机体 Z）：= 螺旋桨阻力矩 Q（∝ω²），CCW 正转 -> 机体受 CW 反扭矩
            let tq = spin[i] * q_n[i];
            tau_body[2] += tq;
        }
        // 陀螺进动：螺旋桨角动量 H = I_rotor·Ω·ẑ(机体)，机体以 ω 转动产生 M_gyro = H × ω。
        // 四旋翼等速反桨时净 H_z=0（悬停无净陀螺）；转速不对称（机动/偏航/故障）时
        // 产生俯仰↔滚转耦合力矩（陀螺稳定效应）。
        let gyro = gyro_torque(&self.motor_speed, &spin, rotor_i, omega_body);
        tau_body[0] += gyro[0];
        tau_body[1] += gyro[1];
        // 阶段 3：推进风场，取当前世界系（UP）风速（P2-B：传入机体位置以启用
        // 空间相关风场 / 风切变廓线）。无风则为 0。
        // 先读取机体位置（独立 immutable 借用），再可变借 wind，避免借用冲突。
        let wind_up: WindVec = if self.wind.is_some() {
            let tf = self.read_body_tf();
            let pos = [tf[0], tf[1], tf[2]];
            let w = self.wind.as_mut().unwrap();
            w.sample_at(self.dt, &pos)
        } else {
            [0.0; 3]
        };
        // 阶段 2b：机体气动阻力（含诱导阻力），基于相对风速（v_body - wind），在机体坐标系施加。
        let aero = self.aero_drag_body(q, &wind_up);
        // 机体合力 = 旋翼推力 + 气动阻力 + 滑流冲击（P0-2）；合力矩 = 旋翼力矩（阻力矩略，量级小）
        let mut f_body_tot = [0.0, 0.0, 0.0];
        for k in 0..3 {
            f_body_tot[k] = f_body[k] + aero.fx[k];
        }
        // 滑流下洗冲击机体的下拉力（机体 -Z）。
        f_body_tot[2] -= f_slip.min(sum_t * 0.4);
        let f_world_tot = rotate_by_quat(q, f_body_tot);
        let tau_world = rotate_by_quat(q, tau_body);

        // impulse 模型：把每帧"力 × dt"化为线冲量，"力矩 × dt"化为角冲量注入。
        let f_impulse = [
            f_world_tot[0] * self.dt,
            f_world_tot[1] * self.dt,
            f_world_tot[2] * self.dt,
        ];
        let tau_impulse = [
            tau_world[0] * self.dt,
            tau_world[1] * self.dt,
            tau_world[2] * self.dt,
        ];
        self.world.apply_impulse(self.body_id, &f_impulse, 0);
        self.world.apply_torque_impulse(self.body_id, &tau_impulse, 0);
        self.last_tau_body = tau_body;

        // ---- 步进物理引擎 ----
        let rc = self.world.step(self.dt);
        assert_eq!(rc, 0, "物理引擎 step 检测到 NaN/Inf，世界已损坏");
        self.time += self.dt;

        // ---- P1-2：地面接触解算（惩罚弹簧-阻尼 + 库仑摩擦）----
        // 仅在启用接触时解算并注入法向/切向冲量；`None` 表示真空（不做接触）。
        self.last_contact = match &self.contact {
            Some(m) => {
                let info = crate::physics::resolve_ground_contact(
                    &mut self.world,
                    self.body_id,
                    self.cfg.mass as f64,
                    m,
                    self.dt,
                );
                if info.touching { Some(info) } else { None }
            }
            None => None,
        };

        // ---- P1-2 续：障碍碰撞解算（惩罚模型，与地面接触同源）----
        // 障碍列表非空时，每步经 `resolve_obstacle_contact` 解算最深穿透障碍并注入冲量。
        // 仅当本步未触地（避免地面/障碍双接触叠加冲量）且障碍非空时解算。
        if !self.obstacles.is_empty() {
            let body_radius = 1.2 * self.cfg.arm_length as f64; // 螺旋桨外周包络
            let cm = self.contact.clone().unwrap_or_default();
            let info = crate::physics::resolve_obstacle_contact(
                &mut self.world,
                self.body_id,
                self.cfg.mass as f64,
                &self.obstacles,
                body_radius,
                &cm,
                self.dt,
            );
            // 障碍接触与地面接触取"或"：任一接触即标记 last_contact（优先障碍，更显著）。
            if info.touching {
                self.last_contact = Some(info);
            }
        }
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

        // 角速度：Rapier 的 get_angular_velocity 返回【世界系】角速度，必须先旋到
        // 引擎机体(前-右-上)系，再按飞控混控端 (p,-q,-r) 的约定翻转 Y、Z 得到飞控机体
        // (前-右-下)系。该翻转必须与 actuator 端的 (p,-q,-r) 互为逆，否则俯仰/偏航轴的
        // 阻尼项符号反掉，姿态环发散（悬停近水平时世界系≈机体系故无碍，倾斜后炸机）。
        let q_up_q = Quaternion { w: q_up[0] as f32, x: q_up[1] as f32, y: q_up[2] as f32, z: q_up[3] as f32 };
        let ang_body = rotate_vec_by_quat_inverse(q_up_q, [ang[0] as f32, ang[1] as f32, ang[2] as f32]);
        let omega_fc = [ang_body[0], -ang_body[1], -ang_body[2]];

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

    /// 阶段 8：取动力系统状态（电池端电压 V，4 路电机转速 rad/s），供测试/日志。
    pub fn powertrain_state(&self) -> (f64, [f64; 4]) {
        (self.battery_v, self.motor_speed)
    }

    /// P0-2：取当前动量理论诱导速度（m/s，含垂直气流耦合），供测试/日志。
    pub fn induced_velocity(&self) -> f64 {
        self.induced_vel
    }

    /// 调试：最近一次 apply_actuators 算出的机体力矩（引擎机体系，[x,y,z]）。
    pub fn debug_tau_body(&self) -> [f64; 3] {
        self.last_tau_body
    }

    /// 调试：取引擎世界系真实角速度 (rad/s)。
    pub fn debug_ang_world(&self) -> [f64; 3] {
        self.world.get_angular_velocity(self.body_id)
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
        let q_up_q = Quaternion { w: q_up[0] as f32, x: q_up[1] as f32, y: q_up[2] as f32, z: q_up[3] as f32 };
        let ang_body = rotate_vec_by_quat_inverse(q_up_q, [ang[0] as f32, ang[1] as f32, ang[2] as f32]);
        VehicleState {
            time_boot_ms: (self.time * 1000.0) as i32,
            pos: [Meter(pos_ned[0]), Meter(pos_ned[1]), Meter(pos_ned[2])],
            vel: [
                MeterPerSecond(vec_up_to_ned(vel)[0]),
                MeterPerSecond(vec_up_to_ned(vel)[1]),
                MeterPerSecond(vec_up_to_ned(vel)[2]),
            ],
            att: quat_ned,
            omega: [RadianPerSecond(ang_body[0]), RadianPerSecond(-ang_body[1]), RadianPerSecond(-ang_body[2])],
            airspeed: MeterPerSecond((vel[0] * vel[0] + vel[1] * vel[1]).sqrt() as f32),
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

/// 螺旋桨陀螺进动力矩（机体系）：
/// M_gyro = Σ_i (H_i × ω)，H_i = I_rotor·Ω_i·spin_i·ẑ（机体 Z 轴角动量）。
/// 四旋翼等速反桨时 ΣH_z=0 → 悬停无净陀螺；转速不对称（机动/偏航/故障）时
/// 俯仰角速度在滚转轴、滚转角速度在俯仰轴产生耦合力矩。
pub fn gyro_torque(
    motor_speed: &[f64; 4],
    spin: &[f64; 4],
    rotor_i: f64,
    omega_body: [f64; 3],
) -> [f64; 3] {
    let mut tau = [0.0f64; 3];
    for i in 0..4 {
        let hz = rotor_i * motor_speed[i] * spin[i];
        tau[0] += -hz * omega_body[1]; // M_x = -H_z·ω_y
        tau[1] += hz * omega_body[0]; // M_y = +H_z·ω_x
    }
    tau
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
