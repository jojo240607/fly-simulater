//! 飞控包装：把 `flyctrl-core` 的 `HilContext`（SIL/HIL 共享控制律）接入仿真闭环。
//!
//! 关键：这里实现**真实**（非 mock）传感器/执行器 trait，数据来自物理引擎世界
//! （经 `QuadrotorPlant`），控制输出回写 `plant`。这样跑的是与 MCU 上 `control.rs`
//! 同一份 `EkfEstimator` + `PidController` + `Fdir` 算法。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{Controller, IndiController, LqrController, PidController, Setpoint, TecsController};
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::hal::actuator::{MotorActuator, OutputProtocol};
use flyctrl_core::hal::sensor::{AirspeedSensor, GpsSensor, ImuSensor, MagSensor};
use flyctrl_core::hil::HilContext;
use flyctrl_core::units::{Meter, MeterPerSecondSquared, Radian, RadianPerSecond, Second};
use flyctrl_core::vehicle::{ActuatorCmd, AirspeedSample, ImuSample, PosSample, VehicleState};

use crate::alloc::allocate_eff;
use crate::plant::QuadrotorPlant;
use crate::physics::{ContactInfo, ContactModel, DynamicObstacle, Obstacle, RigidBodyWorld};
use crate::wind::WindField;
use crate::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig};

/// 生产路径便捷别名：用真实物理引擎（phy-sdk）的控制器。
#[cfg(feature = "phy")]
pub type RealFlyController = FlyController<crate::physics::PhySdkWorld>;

/// 测试/调试便捷：返回四路满油门（归一化 1.0）执行器指令。
pub fn actuator_full() -> ActuatorCmd {
    let mut c = ActuatorCmd::zero();
    for i in 0..4 {
        c.motor[i] = 1.0;
    }
    c
}

/// 控制器种类（阶段 5：高级控制律对比；P3-A3：TECS 总能量控制）。
#[derive(Clone, Copy, Debug)]
pub enum ControllerKind {
    Pid,
    Indi, // INDI 包装 PID 基线
    Lqr,
    Tecs, // TECS 总能量控制（能量保持 + 空速拖拽前馈）
}

/// 不同控制器类型的 HIL 闭环（泛型单态化）。
enum CtrlVariant {
    Pid(HilContext<EkfEstimator, IndiController<PidController>>),
    Indi(HilContext<EkfEstimator, IndiController<PidController>>),
    Lqr(HilContext<EkfEstimator, IndiController<LqrController>>),
    Tecs(HilContext<EkfEstimator, TecsController>),
}

// ---- 真实传感器：把物理引擎真值喂给控制律 ----

/// 物理引擎提供的 IMU（真值，可在此基础上叠加噪声，首版用干净值）。
pub struct SimImu {
    last: ImuSample,
}

/// 物理引擎提供的 GPS/NED 位置。
pub struct SimGps {
    last: Option<PosSample>,
}

// 说明：为绕开 `plant` 同时被 FlyController（可变）与传感器 trait（可变）借用的冲突，
// 这里改为"推模式"：FlyController.step 先把当帧样本存入 SimImu/SimGps，再调 HilContext。
// 因此 trait 实现只读 last，无 plant 引用。

impl ImuSensor for SimImu {
    fn read(&mut self) -> ImuSample { self.last }
    fn healthy(&self) -> bool { true }
}
impl GpsSensor for SimGps {
    fn read(&mut self) -> Option<PosSample> { self.last }
    fn healthy(&self) -> bool { self.last.is_some() }
}

/// 物理引擎提供的空速（真空速，不含风）：由世界真值水平速度幅值换算。
pub struct SimAirspeed {
    last: Option<AirspeedSample>,
}

impl AirspeedSensor for SimAirspeed {
    fn read(&mut self) -> Option<AirspeedSample> { self.last }
    fn healthy(&self) -> bool { self.last.is_some() }
}

/// 物理引擎提供的磁力计（机体系三轴磁场）。
pub struct SimMag {
    last: [f32; 3],
}

impl MagSensor for SimMag {
    fn read(&mut self) -> [f32; 3] { self.last }
    fn healthy(&self) -> bool { true }
}

// ---- 真实执行器：仅缓存控制输出，由 FlyController.step 显式回写 plant ----
//
// 注意：不能用裸指针缓存 `plant` 地址——`FlyController::new` 里 `plant` 构造后会被
// 移入 `Self`，原栈地址作废，经裸指针写会落到一个已失效的位置（表现为推力恒为 0）。
// 因此 SimMotors 只记录最近指令，推力注入在 FlyController::step 内显式完成。

pub struct SimMotors {
    last: ActuatorCmd,
}

impl MotorActuator for SimMotors {
    fn apply(&mut self, cmd: &ActuatorCmd) {
        self.last = *cmd;
    }
    fn disarm(&mut self) {
        self.last = ActuatorCmd::zero();
    }
    fn healthy(&self) -> bool { true }
}

// ---- 控制器主结构 ----

pub struct FlyController<W> {
    hil: CtrlVariant,
    plant: QuadrotorPlant<W>,
    imu: SimImu,
    gps: SimGps,
    air: SimAirspeed,
    mag: SimMag,
    motors: SimMotors,
    cfg: VehicleConfig,
    /// 解锁态：true=电机可转（默认 true，保证既有悬停/任务测试行为不变）；
    /// MAVLink DISARM 置 false 时停转。MAVLink ARM 置 true。
    armed: bool,
    /// 当前飞行模式（MAV custom mode 低字节），由 MAVLink DO_SET_MODE / 内部状态机更新。
    mode: u8,
    /// 最近一次 MAVLink 起飞指令请求的高度（m，绝对），0 表示无。
    takeoff_alt: f32,
    /// 阶段 5：故障注入——每路电机推进效率系数（1.0=正常，0.0=完全停转，
    /// 中间值=部分效率退化）。实测：四旋翼在当前无重构控制律下，单电机推力
    /// 损失（无论完全还是部分）均致姿控发散、不可恢复（见 sim.rs run_hover_degraded）。
    fail_mask: [f32; 4],
    /// P1-2 闭环联动：反应式避障配置。`None`=不装避障（默认，行为同前）。
    /// 装备后，step 内读前向测距，危险时改写速度设定点（制动+横向闪避）。
    avoidance: Option<AvoidanceConfig>,
    /// 控制周期（s），用于避障保持逻辑的计时。
    dt: f64,
    /// 仿真已推进时间（s），每次 [`FlyController::step`] 累加 `dt`。
    time: f64,
    /// 最近一次**确认危险**的时刻与避障指令（NED，m/s）：
    /// `Some((time, vel))` 表示"已确认危险且尚在 `hold_time` 保持窗口内"。
    /// 用于单射线 FOV 丢失后继续维持避障，避免位置环立即把机体拉回航线。
    av_last: Option<(f64, [f64; 3])>,
}

impl<W> FlyController<W>
where
    W: RigidBodyWorld,
{
    pub fn new(
        world: W,
        cfg: &VehicleConfig,
        dt: f64,
        wind: Option<WindField>,
        sensor_cfg: SensorConfig,
        kind: ControllerKind,
        contact: Option<ContactModel>,
        obstacles: Vec<Obstacle>,
    ) -> Self {
        let mut ekf = EkfEstimator::default_quad();
        // 阶段 11-A：EKF 初始位置估计必须与机体真实初始位置一致（NED d=-5，即引擎 y=5），
        // 否则 GPS/气压首次校正前 PID 看到 ~5m 位置误差全油门弹射（见 PLAN 阶段 11-A）。
        // 注意：此处硬编码需与 `QuadrotorPlant::new` 的初始位置（pos7=[0,5,0]→NED d=-5）保持一致。
        ekf.set_initial_position([0.0, 0.0, -5.0]);
        let dt_s = Second(dt as f32);
        let hil = match kind {
            ControllerKind::Pid => {
                // PID 基线姿态环。注：早期曾包一层 INDI 角加速度反馈（gain_scale=0.5）以
                // 对抗 EKF 纯陀螺积分漂移，但实测在 realistic 传感器噪声（gyro_noise=0.003）
                // 下，INDI 的角加速度误差反馈（k_inv≈I/dt=12.5，对 dt=0.004 的有限差分）把
                // 陀螺噪声放大成饱和的剧烈非对称电机指令（cmd 从 [0.5×4] 跳成 [1,0,0,1]），
                // 驱动机体翻滚 → none-contact 场景垂直/整体发散（PLAN 阶段 11-A）。
                // 修复：Pid 变体不再包 INDI（gain_scale=0=纯 PID）。实测纯 PID 在 realistic
                // 噪声下悬停稳定（姿态误差<0.01rad、d 稳定在 -5±2m、电机指令平衡）。
                let base = PidController::from_config(&cfg.ctrl_params());
                let indi = IndiController::with_inertia(base, cfg.inertia, dt as f32, 0.0);
                CtrlVariant::Pid(HilContext::new(ekf, indi, dt_s))
            }
            ControllerKind::Indi => {
                let base = PidController::from_config(&cfg.ctrl_params());
                let indi = IndiController::with_inertia(base, cfg.inertia, dt as f32, 0.8);
                CtrlVariant::Indi(HilContext::new(ekf, indi, dt_s))
            }
            ControllerKind::Lqr => {
                // LQR 姿态环单独使用时会因 EKF 姿态估计动态误差发散；包一层轻量 INDI
                // 角加速度反馈（gain_scale=0.5）稳定姿态，与 PID 变体同策略。
                let base = LqrController::from_config(&cfg.ctrl_params());
                let indi = IndiController::with_inertia(base, cfg.inertia, dt as f32, 0.5);
                CtrlVariant::Lqr(HilContext::new(ekf, indi, dt_s))
            }
            ControllerKind::Tecs => {
                // TECS 总能量控制（P3-A3）：垂直通道对总能量高度做 PI + 水平空速拖拽
                // 前馈，姿态内环复用 attitude.rs（纯 TECS，同 PID 变体不包 INDI）。
                let tecs = TecsController::from_config(&cfg.ctrl_params());
                CtrlVariant::Tecs(HilContext::new(ekf, tecs, dt_s))
            }
        };

        let plant = QuadrotorPlant::new(world, cfg, dt, wind, sensor_cfg, contact, obstacles);

        let imu = SimImu {
            last: ImuSample {
                accel: [MeterPerSecondSquared(0.0); 3],
                gyro: [RadianPerSecond(0.0); 3],
            },
        };
        let gps = SimGps { last: None };
        let air = SimAirspeed { last: None };
        let mag = SimMag { last: [0.0; 3] };
        let motors = SimMotors { last: ActuatorCmd::zero() };

        Self {
            hil,
            plant,
            imu,
            gps,
            air,
            mag,
            motors,
            cfg: cfg.clone(),
            armed: true,
            mode: 0,
            takeoff_alt: 0.0,
            fail_mask: [1.0; 4],
            avoidance: None,
            dt,
            time: 0.0,
            av_last: None,
        }
    }

    /// P1-2 闭环联动：装备/卸下反应式避障控制器。
    ///
    /// 装备后，[`FlyController::step`] 会在控制律之前读前向测距传感器，
    /// 当检测到前方障碍进入危险距离时，把避障速度指令（制动+横向闪避）并入
    /// 速度设定点，实现"感知→决策→规避"闭环。
    pub fn set_avoidance(&mut self, cfg: Option<AvoidanceConfig>) {
        self.avoidance = cfg;
    }

    /// P1-2 闭环联动：装备前向测距传感器（障碍反射来源）。
    /// 不装备则 [`FlyController::step`] 不会触发避障（即使已装避障配置）。
    pub fn set_ranger(&mut self, ranger: Option<RangeFinderModel>) {
        self.plant.set_ranger(ranger);
    }

    /// P1-2 闭环联动：一次性装备"测距传感器 + 避障控制器"闭环，
    /// 是 [`FlyController::set_ranger`] 与 [`FlyController::set_avoidance`] 的便捷组合。
    pub fn configure_avoidance(&mut self, ranger: RangeFinderModel, avoidance: AvoidanceConfig) {
        self.plant.set_ranger(Some(ranger));
        self.avoidance = Some(avoidance);
    }

    /// 阶段 5：设置电机完全失效掩码（true=该电机效率置 0，停转）。
    /// 索引 0..3 对应 m0..m3。四旋翼单电机完全停转不可恢复（阶段 5 结论）。
    pub fn set_motor_failure(&mut self, mask: [bool; 4]) {
        for i in 0..4 {
            self.fail_mask[i] = if mask[i] { 0.0 } else { 1.0 };
        }
    }

    /// 阶段 5（增强）：设置每路电机效率系数（1.0=正常，0.0=完全停转，
    /// 中间值=部分效率退化）。索引 0..3 对应 m0..m3。
    pub fn set_motor_eff(&mut self, eff: [f32; 4]) {
        for i in 0..4 {
            // 夹紧到 [0,1]，防御非法 CLI 输入。
            self.fail_mask[i] = eff[i].clamp(0.0, 1.0);
        }
    }

    /// 推模式：先让 plant 产出当帧样本，存入传感器 trait，再跑控制律，最后 step 世界。
    pub fn step(&mut self, setpoint: &Setpoint) -> VehicleState {
        // 1) 取世界真值（NED 语义）。
        let (imu_sample, pos_sample) = self.plant.read_sensors();
        self.imu.last = imu_sample;
        self.gps.last = pos_sample;
        // P3-A3：空速计观测用**地速幅值** |v_ground|——EKF 空速模型 `h(x)=|v_ground|`，
        // 喂地速才一致，避免逆风下风相对空速（<地速）把水平速度估计拉偏（曾致逆风
        // 被吹回 + 拖拽前馈用错方向）。真实相对空速矢量经 set_measured_airspeed_vec
        // 单独注入 TECS 做拖拽前馈（见下），两者互不干扰。
        let vg = self.plant.ground_airspeed_ned();
        let vh = (vg[0] * vg[0] + vg[1] * vg[1]).sqrt();
        self.air.last = Some(AirspeedSample {
            speed: flyctrl_core::units::Airspeed(vh as f32),
            timestamp_s: 0.0,
        });
        // 相对空速矢量（NED 水平）= v_ground - wind：每周期注入控制器，供 TECS
        // 空速拖拽前馈定向（其它控制器默认忽略）。
        let v_rel = self.plant.relative_airspeed_ned();
        match &mut self.hil {
            CtrlVariant::Tecs(h) => h.ctrl.set_measured_airspeed_vec(v_rel),
            _ => {}
        }
        // 磁力计：取机体磁场（含硬铁/软铁/噪声），喂给 EKF yaw 约束。
        let (mag_sample, baro) = self.plant.read_sensors_attitude();
        self.mag.last = [
            mag_sample.field[0] as f32,
            mag_sample.field[1] as f32,
            mag_sample.field[2] as f32,
        ];

        // 2) 跑控制律（SIL/HIL 共享闭环）。motors.apply 只记录指令。
        // 2.0) P1-2 闭环联动：装备避障时，先读前向测距并把避障速度并入设定点。
        //      setpoint 是 immutable 引用，这里构造一个叠加过避障的本地副本。
        let mut sp = setpoint.clone();
        if let Some(av_cfg) = &self.avoidance {
            if let Some(sample) = self.plant.read_ranger() {
                let fwd = self.plant.forward_dir_ned();
                let right = self.plant.right_dir_ned();
                let (av_vel, triggered) = av_cfg.avoidance_velocity(&sample, fwd, right);
                // 记录最近一次"确认危险"的时刻与避障指令（供 FOV 丢失后的保持窗口用）。
                if triggered {
                    self.av_last = Some((self.time, av_vel));
                }
                // 单射线 FOV 丢失（障碍滑出射线 → 读数失效）时，避障不会立即释放，
                // 而是在最近一次确认危险后的 hold_time 内继续维持指令，让横向分离
                // 距离积累足够；否则位置外环会把机体拉回原航线，净间隙不足（实测
                // 机体在射线边缘形成极限环，横向位移被封顶在障碍半径附近）。
                let held = triggered
                    || self
                        .av_last
                        .map_or(false, |(t, _)| self.time - t <= av_cfg.hold_time);
                if held {
                    // 保持窗口内沿用最近一次确认危险的避障指令（方向恒定，避免随
                    // 失效读数抖动）；本轮触发时即为新计算的指令。
                    let vel = self.av_last.map_or(av_vel, |(_, v)| v);
                    // 速度设定点（NED，m/s）叠加避障指令。
                    sp.vel[0].0 += vel[0] as f32;
                    sp.vel[1].0 += vel[1] as f32;
                    // 同时偏移位置设定点的水平分量，使位置外环目标跟随横向闪避
                    // 脱离航线——否则串级 PID 的位置环会把机体拉回原点，抵消避障
                    // 速度指令（诊断实测：仅注入速度时机体横向位移被压制，避障几乎无效）。
                    sp.pos[0].0 += vel[0] as f32 * 10.0;
                    sp.pos[1].0 += vel[1] as f32 * 10.0;
                    // 注意：setpoint 竖向/偏航不动，仅水平被规避层接管。
                }
            }
        }
        // 推进仿真时钟（避障保持窗口的计时基准）。
        self.time += self.dt;
        let setpoint_ref = &sp;
        let state = match &mut self.hil {
            CtrlVariant::Pid(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
            CtrlVariant::Indi(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
            CtrlVariant::Lqr(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
            CtrlVariant::Tecs(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
        };

        // 2.4) 气压计高度融合：锚定 EKF 垂直通道，抑制开环加计积分导致的高度漂移
        // （真实掉高/控制抖动的根因之一）。baro.altitude 为向上高度（m），EKF 用向下为负 D，
        // 通过 update_alt 把气压测高作为 D 位置观测（见 ekf::update_alt：y = -alt - x[2]）。
        // baro 含噪声/漂移（见 SensorModel::process_baro），经独立 r_alt 观测约束 D 位置。
        // 三种控制律底层都是 EkfEstimator，逐一调用。
        let baro_alt = baro.altitude as f32;
        match &mut self.hil {
            CtrlVariant::Pid(h) => h.est.update_alt(baro_alt),
            CtrlVariant::Indi(h) => h.est.update_alt(baro_alt),
            CtrlVariant::Lqr(h) => h.est.update_alt(baro_alt),
            CtrlVariant::Tecs(h) => h.est.update_alt(baro_alt),
        }

        // 2.5) 阶段 5/8：故障处理。
        // - 全有效：按效率系数缩放每路指令（原行为，不变）。
        // - 存在退化/失效电机：用控制分配器(P1-1)把期望动作(推力/滚转/俯仰/偏航)
        //   最小二乘重分配到剩余有效电机，实现容错（牺牲偏航等最弱轴）。
        let has_degraded = self.fail_mask.iter().any(|&e| e < 1.0);
        let mut cmd = self.motors.last;
        if has_degraded {
            // 由固定混控反解期望动作 des = M⁻¹·cmd。
            let m: [f64; 4] = [
                cmd.motor[0] as f64,
                cmd.motor[1] as f64,
                cmd.motor[2] as f64,
                cmd.motor[3] as f64,
            ];
            let des = [
                (m[0] + m[1] + m[2] + m[3]) / 4.0,
                (m[0] - m[1] - m[2] + m[3]) / 2.0,
                (m[0] - m[1] + m[2] - m[3]) / 2.0,
                (m[0] + m[1] - m[2] - m[3]) / 2.0,
            ];
            let u = allocate_eff(des, &self.fail_mask);
            for i in 0..4 {
                cmd.motor[i] = u[i].clamp(0.0, 1.0) as f32;
            }
        } else {
            for i in 0..4 {
                cmd.motor[i] = (cmd.motor[i] as f32) * self.fail_mask[i];
            }
        }

        // 2.6) 把控制指令显式回写被控对象（注入推力/力矩）。
        // 解锁门控：未解锁（MAVLink DISARM）时强制零推力，模拟电机停转/安全上锁。
        if self.armed {
            self.plant.apply_actuators(&cmd);
        } else {
            self.plant.apply_actuators(&ActuatorCmd::zero());
        }

        // 3) 推进物理世界（已注入本拍推力）。
        self.plant.step();

        state
    }

    /// 解锁（MAVLink ARM）。电机恢复可转。
    pub fn arm(&mut self) { self.armed = true; }
    /// 上锁（MAVLink DISARM）。本拍起电机停转（零推力）。
    pub fn disarm(&mut self) { self.armed = false; }
    /// 设置飞行模式（MAV custom mode 低字节），由 MAVLink DO_SET_MODE / 内部状态机写入。
    pub fn set_mode(&mut self, mode: u8) { self.mode = mode; }
    /// 请求起飞到指定绝对高度（m）。仅记录意图，实际目标由上层任务逻辑消费。
    pub fn request_takeoff(&mut self, alt_m: f32) { self.takeoff_alt = alt_m; }
    /// 当前是否解锁。
    pub fn is_armed(&self) -> bool { self.armed }
    /// 当前飞行模式码。
    pub fn mode(&self) -> u8 { self.mode }
    /// 最近一次请求的起飞高度（m，0=无）。
    pub fn takeoff_alt(&self) -> f32 { self.takeoff_alt }

    /// 取当前世界状态（NED）用于日志/不变量检查。
    pub fn world_state(&self) -> VehicleState {
        self.plant.state_ned()
    }

    /// 阶段 9：直接把执行器指令写入被控对象（不跑控制律，供自由落体等无控场景）。
    pub fn plant_apply(&mut self, cmd: &ActuatorCmd) {
        self.plant.apply_actuators(cmd);
    }

    /// 阶段 9：直接推进物理世界一步（不跑控制律，供自由落体等无控场景）。
    pub fn plant_step(&mut self) {
        self.plant.step();
    }

    /// P1-2：运行时设置/清除地面接触（自由落体能量守恒场景用 None 关闭地面）。
    pub fn plant_set_contact(&mut self, contact: Option<ContactModel>) {
        self.plant.set_contact(contact);
    }

    /// P1-2 续：运行时设置/清除静态障碍列表。
    pub fn plant_set_obstacles(&mut self, obstacles: Vec<Obstacle>) {
        self.plant.set_obstacles(obstacles);
    }

    /// P-动态障碍：运行时设置/清除匀速平移动态障碍。
    pub fn plant_set_dynamic_obstacles(&mut self, obstacles: Vec<DynamicObstacle>) {
        self.plant.set_dynamic_obstacles(obstacles);
    }

    /// P1-2：读取最近一次接触解算结果（未接触时为 `None`）。
    pub fn contact_info(&self) -> Option<ContactInfo> {
        self.plant.contact_info()
    }

    /// 调试：返回引擎世界系真实坐标与四元数。
    pub fn debug_up(&self) -> ([f64; 3], [f64; 4]) {
        self.plant.debug_up()
    }

    /// 调试：返回最近一次测距采样（避障诊断用）。
    pub fn dbg_ranger(&mut self) -> Option<crate::sensor::RangeFinderSample> {
        self.plant.read_ranger()
    }

    /// 阶段 8：动力系统状态（电池端电压 V，4 路电机转速 rad/s）。
    pub fn powertrain_state(&self) -> (f64, [f64; 4]) {
        self.plant.powertrain_state()
    }

    /// 调试：最近一次 step 实际产生的总推力（N）。
    pub fn debug_thrust_sum(&self) -> f64 {
        self.plant.debug_thrust_sum()
    }

    /// 调试：当前电池端电压（V）。
    pub fn debug_battery_v(&self) -> f64 {
        self.plant.debug_battery_v()
    }

    /// 调试：最近一次归一化油门指令 [0,1]×4。
    pub fn debug_cmd_motor(&self) -> [f64; 4] {
        self.plant.debug_cmd_motor()
    }

    /// 调试：最近一次电机实际归一化油门 [0,1]×4。
    pub fn debug_thrust_actual_u(&self) -> [f64; 4] {
        self.plant.debug_thrust_actual_u()
    }

    /// 最近一次控制指令。
    pub fn last_cmd(&self) -> ActuatorCmd {
        self.motors.last
    }

    /// 最近一次 IMU 样本（阶段 6 日志用）。
    pub fn last_imu(&self) -> ImuSample {
        self.imu.last
    }

    /// 调试：返回最近一次姿态控制器输出（`(err[3], pqr[3], om[3])`）。
    pub fn dbg_att(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        match &self.hil {
            CtrlVariant::Pid(h) => h.ctrl.inner().dbg_last(),
            CtrlVariant::Indi(h) => h.ctrl.inner().dbg_last(),
            CtrlVariant::Lqr(_) => ([0.0; 3], [0.0; 3], [0.0; 3]),
            CtrlVariant::Tecs(h) => h.ctrl.dbg_last(),
        }
    }

    /// P3-A3 调试：TECS 最近一次（能量高度[向下], 能量误差[向下正]）与（真空速, 拖拽前馈加速度幅值）。
    pub fn dbg_tecs(&self) -> ((f32, f32), (f32, f32)) {
        match &self.hil {
            CtrlVariant::Tecs(h) => (h.ctrl.debug_energy(), h.ctrl.debug_drag()),
            _ => ((0.0, 0.0), (0.0, 0.0)),
        }
    }

    /// P3-A3 调试：当前机体位置风速（NED，m/s）。
    pub fn wind_ned(&mut self) -> [f32; 3] {
        self.plant.wind_ned()
    }

    /// P3-A3 调试：真实相对空速矢量（NED 系，水平分量，m/s）= v_ground - wind。
    pub fn relative_airspeed_ned(&mut self) -> [f32; 2] {
        self.plant.relative_airspeed_ned()
    }

    /// 调试：取引擎世界系真实角速度 (rad/s)。
    pub fn debug_ang_world(&self) -> [f64; 3] {
        self.plant.debug_ang_world()
    }

    /// 调试：返回真实 NED 状态（位置/速度），用于诊断 EKF 估计误差。
    /// 调试：返回当前 EKF 估计状态（已含气压计融合），用于诊断估计误差。
    pub fn debug_estimate_ned(&self) -> VehicleState {
        match &self.hil {
            CtrlVariant::Pid(h) => h.estimate(),
            CtrlVariant::Indi(h) => h.estimate(),
            CtrlVariant::Lqr(h) => h.estimate(),
            CtrlVariant::Tecs(h) => h.estimate(),
        }
    }

    /// 调试：返回当前估计器估计的加计零偏（机体系，m/s²）。
    pub fn debug_accel_bias(&self) -> [f32; 3] {
        match &self.hil {
            CtrlVariant::Pid(h) => h.est.accel_bias(),
            CtrlVariant::Indi(h) => h.est.accel_bias(),
            CtrlVariant::Lqr(h) => h.est.accel_bias(),
            CtrlVariant::Tecs(h) => h.est.accel_bias(),
        }
    }

    /// 调试：返回 PID 控制器的垂向位置积分项（用于诊断抗下沉效果）。
    pub fn debug_pid_iz(&self) -> f32 {
        match &self.hil {
            CtrlVariant::Pid(h) => h.ctrl.inner().debug_iz(),
            CtrlVariant::Indi(h) => h.ctrl.inner().debug_iz(),
            _ => 0.0,
        }
    }

    /// 阶段 11-A 诊断：返回 PID 控制律内部量（绕开 no_std 无打印）。
    /// 元组：(raw_d, raw_vd, filt_d, filt_vd, ez, iz, des_vz, acc_d, des_thr)。
    pub fn debug_pid_internal(&self) -> (f32, f32, f32, f32, f32, f32, f32, f32, f32) {
        match &self.hil {
            CtrlVariant::Pid(h) => h.ctrl.inner().debug_pid_internal(),
            CtrlVariant::Indi(h) => h.ctrl.inner().debug_pid_internal(),
            _ => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        }
    }

    /// 诊断：返回基线 PID 的姿态误差向量、期望机体角速度(pqr)、实测角速度。
    /// 元组：(err[3], pqr[3], omega[3])。区分"姿态误差"与"INDI 增量"哪个主导发散。
    pub fn debug_pid_pqr(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        match &self.hil {
            CtrlVariant::Pid(h) => h.ctrl.inner().dbg_last(),
            CtrlVariant::Indi(h) => h.ctrl.inner().dbg_last(),
            _ => ([0.0; 3], [0.0; 3], [0.0; 3]),
        }
    }

    /// 调试：最近一次 apply_actuators 算出的机体力矩（引擎机体系）。
    pub fn debug_tau_body(&self) -> [f64; 3] {
        self.plant.debug_tau_body()
    }

    /// 调试：电机转速/电压/系数诊断。
    pub fn debug_motor_diag(&self) -> (f64, f64, f64, f64, f64) {
        self.plant.debug_motor_diag()
    }

    /// 调试：最近一次 step 的世界系合力（引擎世界系，x=北/y=上/z=东）。
    pub fn debug_f_world(&self) -> [f64; 3] {
        self.plant.debug_f_world()
    }

    /// 调试：返回引擎世界系真实状态（NED），供传感器噪声鲁棒性诊断对比 EKF 估计。
    pub fn debug_truth_ned(&self) -> VehicleState {
        self.plant.state_ned()
    }
}

// 辅助：构造悬停设定点。
pub fn hover_setpoint(n: f32, e: f32, d: f32) -> Setpoint {
    Setpoint::hover([Meter(n), Meter(e), Meter(d)], Radian(0.0))
}
