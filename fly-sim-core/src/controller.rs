//! 飞控包装：把 `flyctrl-core` 的 `HilContext`（SIL/HIL 共享控制律）接入仿真闭环。
//!
//! 关键：这里实现**真实**（非 mock）传感器/执行器 trait，数据来自物理引擎世界
//! （经 `QuadrotorPlant`），控制输出回写 `plant`。这样跑的是与 MCU 上 `control.rs`
//! 同一份 `EkfEstimator` + `PidController` + `Fdir` 算法。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{Controller, IndiController, LqrController, PidController, Setpoint};
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

/// 控制器种类（阶段 5：高级控制律对比）。
#[derive(Clone, Copy, Debug)]
pub enum ControllerKind {
    Pid,
    Indi, // INDI 包装 PID 基线
    Lqr,
}

/// 不同控制器类型的 HIL 闭环（泛型单态化）。
enum CtrlVariant {
    Pid(HilContext<EkfEstimator, IndiController<PidController>>),
    Indi(HilContext<EkfEstimator, IndiController<PidController>>),
    Lqr(HilContext<EkfEstimator, IndiController<LqrController>>),
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
        let ekf = EkfEstimator::default_quad();
        let dt_s = Second(dt as f32);
        let hil = match kind {
            ControllerKind::Pid => {
                // PID 姿态环单独使用时会因 EKF 姿态估计误差（纯陀螺积分噪声下漂移）
                // 产生慢性下沉；包一层轻量 INDI 角加速度反馈（gain_scale=0.5）打破正反馈，
                // 与纯 INDI（0.8）区别开，保留 PID 主体特性。
                let base = PidController::from_config(&cfg.ctrl_params());
                let indi = IndiController::with_inertia(base, cfg.inertia, dt as f32, 0.5);
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
        // 真空速 = 水平速度幅值（无风假设下，世界速度即相对空气速度）。
        let st = self.plant.state_ned();
        let vh = (st.vel[0].0 * st.vel[0].0 + st.vel[1].0 * st.vel[1].0).sqrt();
        self.air.last = Some(AirspeedSample {
            speed: flyctrl_core::units::Airspeed(vh as f32),
            timestamp_s: 0.0,
        });
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
                if triggered {
                    // 速度设定点（NED，m/s）叠加避障指令。
                    sp.vel[0].0 += av_vel[0] as f32;
                    sp.vel[1].0 += av_vel[1] as f32;
                    // 注意：setpoint 竖向/偏航不动，仅水平速度被规避层接管。
                }
            }
        }
        let setpoint_ref = &sp;
        let state = match &mut self.hil {
            CtrlVariant::Pid(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
            CtrlVariant::Indi(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
            CtrlVariant::Lqr(h) => h.step(&mut self.imu, &mut self.gps, &mut self.air, setpoint_ref, &mut self.motors, &self.cfg),
        };

        // 2.4) 气压计高度融合：baro.altitude 为向上高度（m），EKF 用向下为负 D，
        // 通过 update_alt 把气压测高作为 D 位置观测，抑制定高下沉。
        let baro_alt = baro.altitude as f32;
        match &mut self.hil {
            CtrlVariant::Pid(h) => h.est.update_alt(baro_alt),
            CtrlVariant::Indi(h) => h.est.update_alt(baro_alt),
            CtrlVariant::Lqr(h) => h.est.update_alt(baro_alt),
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

    /// 阶段 8：动力系统状态（电池端电压 V，4 路电机转速 rad/s）。
    pub fn powertrain_state(&self) -> (f64, [f64; 4]) {
        self.plant.powertrain_state()
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
        }
    }

    /// 调试：取引擎世界系真实角速度 (rad/s)。
    pub fn debug_ang_world(&self) -> [f64; 3] {
        self.plant.debug_ang_world()
    }

    /// 调试：返回真实 NED 状态（位置/速度），用于诊断 EKF 估计误差。
    pub fn debug_truth_ned(&self) -> VehicleState {
        self.world_state()
    }

    /// 调试：返回当前 EKF 估计状态（已含气压计融合），用于诊断估计误差。
    pub fn debug_estimate_ned(&self) -> VehicleState {
        match &self.hil {
            CtrlVariant::Pid(h) => h.estimate(),
            CtrlVariant::Indi(h) => h.estimate(),
            CtrlVariant::Lqr(h) => h.estimate(),
        }
    }

    /// 调试：返回当前估计器估计的加计零偏（机体系，m/s²）。
    pub fn debug_accel_bias(&self) -> [f32; 3] {
        match &self.hil {
            CtrlVariant::Pid(h) => h.est.accel_bias(),
            CtrlVariant::Indi(h) => h.est.accel_bias(),
            CtrlVariant::Lqr(h) => h.est.accel_bias(),
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

    /// 调试：最近一次 apply_actuators 算出的机体力矩（引擎机体系）。
    pub fn debug_tau_body(&self) -> [f64; 3] {
        self.plant.debug_tau_body()
    }
}

// 辅助：构造悬停设定点。
pub fn hover_setpoint(n: f32, e: f32, d: f32) -> Setpoint {
    Setpoint::hover([Meter(n), Meter(e), Meter(d)], Radian(0.0))
}
