//! 飞控包装：把 `flyctrl-core` 的 `HilContext`（SIL/HIL 共享控制律）接入仿真闭环。
//!
//! 关键：这里实现**真实**（非 mock）传感器/执行器 trait，数据来自物理引擎世界
//! （经 `QuadrotorPlant`），控制输出回写 `plant`。这样跑的是与 MCU 上 `control.rs`
//! 同一份 `EkfEstimator` + `PidController` + `Fdir` 算法。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{manual, Controller, IndiController, LqrController, ManualParams, PidController, Setpoint, TecsController};
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::fdir::Health;
use flyctrl_core::flightmode::{FlightMode, ModeContext, ModeGovernor};
use flyctrl_core::hal::actuator::{MotorActuator, OutputProtocol};
use flyctrl_core::hal::sensor::{AirspeedSensor, GpsSensor, ImuSensor, MagSensor, RtkSensor, VioSensor};
use flyctrl_core::hil::{HilContext, SimImu as FallbackImu};
use flyctrl_core::units::{Airspeed, Meter, MeterPerSecondSquared, Radian, RadianPerSecond, Second};
use flyctrl_core::vehicle::{ActuatorCmd, AirspeedSample, ImuSample, PosSample, RcInput, RtkSample, VehicleState, VioSample};

use crate::alloc::allocate_eff;
use crate::plant::QuadrotorPlant;
use crate::physics::{BodyCollider, ContactInfo, ContactModel, DynamicObstacle, Obstacle, RigidBodyWorld};
use crate::wind::WindField;
use crate::sensor::{AvoidanceConfig, RangeFinderModel, SensorConfig, SensorFault};

/// P3-B2 避障位置设定点横向偏移积分增益（m/s 每 m/s 横向避障速度）。
/// 威胁期内位置目标随避障速度侧移的平滑速率；稳态偏移 ≈ av_lat * K / λ。
/// 取值权衡：过小 → 位置外环把机体拉回原点、净位移不足；过大 → 位置目标骤移致翻滚掉高。
const AV_EVADE_POS_K: f32 = 3.0;
/// P3-B2 避障位置偏移泄漏率（1/s）。威胁结束后按 e^(-λt) 回零，
/// 使位置外环把机体拉回原航线（自然释放）；时间常数 ≈ 1/λ。
const AV_EVADE_POS_LAMBDA: f32 = 1.0;

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
    /// 定位锁定标志：收到过有效样本即为 true（仿真 GPS 一经锁定持续有效）。
    /// realistic GPS 为 20Hz、控制率更高，非 GPS 帧 `last=None` 属正常；
    /// `healthy()` 若直接看 `last.is_some()` 会让 position_available 逐帧抖动，
    /// 模式治理器在非 GPS 帧请求定点/任务会被误拒。用"锁定状态"而非"本帧有样本"
    /// 表征可用性（EKF 融合仍走 `read()` 的 `Some` 帧，不受影响）。
    has_fix: bool,
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
    fn healthy(&self) -> bool { self.has_fix }
}

/// 物理引擎提供的空速（真空速，不含风）：由世界真值水平速度幅值换算。
pub struct SimAirspeed {
    last: Option<AirspeedSample>,
}

impl AirspeedSensor for SimAirspeed {
    fn read(&mut self) -> Option<AirspeedSample> { self.last }
    fn healthy(&self) -> bool { self.last.is_some() }
}

/// 物理引擎提供的 VIO（视觉里程计）：高频相对位置/速度（短期准、长期漂移）。
pub struct SimVio {
    last: Option<VioSample>,
}

impl VioSensor for SimVio {
    fn read(&mut self) -> Option<VioSample> { self.last }
    fn healthy(&self) -> bool { self.last.is_some() }
}

/// 物理引擎提供的 RTK-GPS：厘米级高精度绝对位置。
pub struct SimRtk {
    last: Option<RtkSample>,
}

impl RtkSensor for SimRtk {
    fn read(&mut self) -> Option<RtkSample> { self.last }
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
    vio: SimVio,
    rtk: SimRtk,
    mag: SimMag,
    motors: SimMotors,
    /// P3-B1：GPS 可用开关（默认 true）。置 false 模拟 GPS 失锁（read 返回 None），
    /// 用于验收"VIO/RTK 融合在 GPS 中断期间兜底位置估计"。`has_fix` 同步置 false，
    /// 模式治理器据此在失锁期间拒绝定位/任务等依赖绝对位置的模式。
    gps_ok: bool,
    /// P3-B1：VIO 可用开关（默认 true）。置 false 模拟 VIO 失锁（不注入位置/速度观测），
    /// 用于对比"仅 GPS"与"GPS+VIO+RTK"在 GPS 中断时的位置保持能力。
    vio_ok: bool,
    /// P3-B1：RTK 可用开关（默认 true）。置 false 模拟 RTK 无固定解（不注入厘米级位置观测），
    /// 用于对比 VIO 单独与 VIO+RTK 的长期漂移抑制效果。
    rtk_ok: bool,
    cfg: VehicleConfig,
    /// 解锁态：true=电机可转（默认 true，保证既有悬停/任务测试行为不变）；
    /// MAVLink DISARM 置 false 时停转。MAVLink ARM 置 true。
    armed: bool,
    /// 当前飞行模式（MAV custom mode 低字节），由 MAVLink DO_SET_MODE / 内部状态机更新。
    mode: u8,
    /// 最近一次 MAVLink 起飞指令请求的高度（m，绝对），0 表示无。
    takeoff_alt: f32,
    /// P3-D1：飞行模式治理器（RC 模式开关 → 经合法性校验生效），与 `mode` 字段保持同步。
    mode_gov: ModeGovernor,
    /// P3-D1：手动/增稳控制参数（从机型配置提取，供 RC 直通档使用）。
    manual: ManualParams,
    /// P3-D1：返航基准点（起飞点，NED m）；初始即原点 (0,0,-5)。
    home: [f32; 3],
    /// P3-D1：进入自主保持模式时锚定的保持点（NED m），供 Position/Altitude/Mission 使用。
    hold_pos: [f32; 3],
    /// 最近一次传感器采样周期的气压计高度（m，向上为正），`finalize` 用它做 EKF 高度融合。
    baro_alt: f32,
    /// 阶段 5：故障注入——每路电机推进效率系数（1.0=正常，0.0=完全停转，
    /// 中间值=部分效率退化）。实测：四旋翼在当前无重构控制律下，单电机推力
    /// 损失（无论完全还是部分）均致姿控发散、不可恢复（见 sim.rs run_hover_degraded）。
    fail_mask: [f32; 4],
    /// P1-2 闭环联动：反应式避障配置。`None`=不装避障（默认，行为同前）。
    /// 装备后，step 内读扇形多射线测距（P3-B2），危险时改写速度设定点（制动+横向闪避）。
    avoidance: Option<AvoidanceConfig>,
    /// P3-B2 避障横向锁存方向：0=未锁存；±1=威胁事件内保持的闪避方向（沿 `right_ned`，
    /// +1=右，-1=左）。事件结束（`triggered` 变 false）重置。用于避免障碍近正前时
    /// `lateral_comp≈0` 逐帧翻号 → 净横向位移为零（P3-B2 排查结论）。
    av_evade_dir: f32,
    /// P3-B2 避障位置设定点横向偏移（NED 水平，m）。威胁期内随
    /// `锁存方向 × 横向避障速度幅度` 泄漏积分累积，威胁结束后指数泄漏回零，
    /// 使位置外环把机体拉回原航线（自然释放）。
    av_evade_int: [f32; 2],
    /// 控制周期（s），用于仿真推进计时。
    dt: f64,
    /// 仿真已推进时间（s），每次 [`FlyController::step`] 累加 `dt`。
    time: f64,
    /// 共享单步回退 IMU（`flyctrl_core::hil::SimImu`）：SIL 本拍恒有真实 IMU 帧，
    /// 该实例仅在 [`step_hil`](flyctrl_core::hil::HilContext::step_hil) 收到 `None`
    /// 时启用，与实机 HIL 注入饥饿时的回退数据同源，保证算法/数据完全一致。
    sim_imu: FallbackImu,
    /// 本拍是否走了共享单步 `step_hil`：`step_hil` 内部已做气压观测融合
    /// （`update_alt`），`finalize` 必须跳过，否则同一观测被双重注入。
    used_step_hil: bool,
    /// HIL 反相发散复现（诊断）：IMU 节流注入周期。0=关闭（SIL 默认每拍真实 IMU）；
    /// N>0 时每 N 拍才喂一次真实 IMU，其余拍传 None → `step_hil` 回退 sample-and-hold
    /// 陈旧 IMU（`last_real_imu`），复刻 HIL 中「MCU 4ms 控制拍 vs PC ~11ms 物理步」
    /// 导致的注入饥饿。复现完可移除。
    imu_throttle: u32,
    /// 节流注入计数器（每拍 +1，用于周期判定）。
    imu_step: u32,
    /// SIL/HIL 流程一致性：GPS/位置观测节流注入周期。0=每拍（SIL 默认 250Hz）；
    /// N>0 时每 N 拍才注入一次 GPS，其余拍传 None → EKF 位置仅靠 IMU 积分 + 气压
    /// 锚定，复刻 HIL 中 HIL_GPS/SET_POSITION 每 `HIL_NAV_EVERY=8` 步注入一次
    /// （≈31Hz）的节流节奏（见 fly-sim-server/src/main.rs）。
    gps_throttle: u32,
    /// GPS 节流计数器（每拍 +1，用于周期判定）。
    gps_step: u32,
    /// HIL 反相发散复现（诊断）：物理步节流周期。0=关闭（SIL 每拍推进物理）；
    /// N>0 时每 N 个控制拍才推进一次物理（`finalize` 中 apply_actuators +
    /// plant.step，每次仍推进 1×dt），期间 plant 冻结而控制律照常每拍运行，
    /// 复刻 HIL 中「MCU 4ms 控制拍 vs PC 物理步更慢」的双时钟失配：控制时钟
    /// 快于物理时钟 → sample-and-hold 陈旧 IMU 外推 12ms 而真值仅走 4ms，
    /// 姿态估计超前物理真值 → 控制器反向修正 → 反相发散。复现完可移除。
    plant_throttle: u32,
    /// 物理步节流计数器（每拍 +1，用于周期判定）。
    plant_step: u32,
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
        // 默认初始位姿：NED (0,0,-5) = 悬停 5m 高（与 QuadrotorPlant::new 一致）。
        Self::new_at(world, cfg, dt, wind, sensor_cfg, kind, contact, obstacles, [0.0, 0.0, -5.0])
    }

    /// 与 [`FlyController::new`] 相同，但可指定机体初始 NED 位置 `pos_ned`（[n,e,d] m）。
    ///
    /// P3-D5 多机共世界：每架机在不同初始位置创建，EKF 初始位置与机体真实位置对齐，
    /// 避免 GPS/气压首次校正前看到 ~5m 级位置误差全油门弹射（与 PLAN 阶段 11-A 同因）。
    pub fn new_at(
        world: W,
        cfg: &VehicleConfig,
        dt: f64,
        wind: Option<WindField>,
        sensor_cfg: SensorConfig,
        kind: ControllerKind,
        contact: Option<ContactModel>,
        obstacles: Vec<Obstacle>,
        pos_ned: [f32; 3],
    ) -> Self {
        let mut ekf = EkfEstimator::default_quad();
        // 阶段 11-A：EKF 初始位置估计必须与机体真实初始位置一致（NED），
        // 否则 GPS/气压首次校正前 PID 看到 ~5m 位置误差全油门弹射（见 PLAN 阶段 11-A）。
        // 注意：此处需与 `QuadrotorPlant::new_at` 的初始位置保持一一对应。
        ekf.set_initial_position(pos_ned);
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

        let plant = QuadrotorPlant::new_at(world, cfg, dt, wind, sensor_cfg, contact, obstacles, pos_ned);

        let imu = SimImu {
            last: ImuSample {
                accel: [MeterPerSecondSquared(0.0); 3],
                gyro: [RadianPerSecond(0.0); 3],
            },
        };
        let gps = SimGps { last: None, has_fix: false };
        let air = SimAirspeed { last: None };
        let vio = SimVio { last: None };
        let rtk = SimRtk { last: None };
        let mag = SimMag { last: [0.0; 3] };
        let motors = SimMotors { last: ActuatorCmd::zero() };

        Self {
            hil,
            plant,
            imu,
            gps,
            air,
            vio,
            rtk,
            mag,
            motors,
            gps_ok: true,
            vio_ok: true,
            rtk_ok: true,
            cfg: cfg.clone(),
            armed: true,
            mode: 0,
            takeoff_alt: 0.0,
            mode_gov: ModeGovernor::new(FlightMode::Manual),
            manual: ManualParams::from_ctrl(&cfg.ctrl_params()),
            home: [0.0, 0.0, -5.0],
            hold_pos: [0.0, 0.0, -5.0],
            baro_alt: 5.0,
            fail_mask: [1.0; 4],
            avoidance: None,
            av_evade_dir: 0.0,
            av_evade_int: [0.0, 0.0],
            dt,
            time: 0.0,
            sim_imu: FallbackImu::new(),
            used_step_hil: false,
            imu_throttle: 0,
            imu_step: 0,
            gps_throttle: 0,
            gps_step: 0,
            plant_throttle: 0,
            plant_step: 0,
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

    /// HIL 反相发散复现（诊断）：设置 IMU 节流注入周期。N>0 时每 N 拍喂一次真实 IMU，
    /// 其余拍传 None（复用共享单步的 sample-and-hold 回退），用于在 SIL 复现 HIL
    /// 注入饥饿导致的姿态估计反相发散。0=关闭（SIL 默认每拍真实 IMU）。复现完移除。
    pub fn set_imu_throttle(&mut self, every: u32) {
        self.imu_throttle = every;
        self.imu_step = 0;
    }

    /// SIL/HIL 流程一致性：设置 GPS/位置观测节流注入周期。N>0 时每 N 拍才注入一次
    /// GPS，其余拍传 None（EKF 位置靠 IMU 积分 + 气压锚定），复刻 HIL 中 HIL_GPS/
    /// SET_POSITION 每 `HIL_NAV_EVERY=8` 步（≈31Hz）注入一次的节流节奏。0=每拍
    /// （SIL 默认 250Hz）。
    pub fn set_gps_throttle(&mut self, every: u32) {
        self.gps_throttle = every;
        self.gps_step = 0;
    }

    /// HIL 反相发散复现（诊断）：设置物理步节流周期。N>0 时每 N 个控制拍才
    /// 推进一次物理（`finalize` 跳过中间拍的 apply_actuators + plant.step），
    /// 模拟 HIL 双时钟失配（MCU 4ms 控制拍快于 PC 物理步）。与 `set_imu_throttle`
    /// 配套使用（IMU 注入与物理步同频）。0=关闭（SIL 每拍推进物理）。复现完移除。
    pub fn set_plant_throttle(&mut self, every: u32) {
        self.plant_throttle = every;
        self.plant_step = 0;
    }

    /// 推模式：先让 plant 产出当帧样本，存入传感器 trait，再跑控制律，最后 step 世界。
    ///
    /// P3-D1：与 [`FlyController::step_rc`]（RC 直通）共用 `sample_sensors` /
    /// `finalize`，仅"控制指令来源"不同：这里消费外部给定设定点（自主轨迹），
    /// `step_rc` 消费遥控摇杆（手动/增稳）或由模式生成的保持设定点。
    pub fn step(&mut self, setpoint: &Setpoint) -> VehicleState {
        // 1) 采集当帧传感器样本（imu/gps/空速/磁力计/气压），与 step_rc 共用。
        self.sample_sensors();

        // 2) 按设定点闭环（含避障叠加 + 推进时钟），与 step_rc 共用。
        let state = self.run_setpoint(setpoint);

        // 3) 收尾：气压融合 / 故障处理 / 解锁门控 / 推进物理世界（与 step_rc 共用）。
        self.finalize();

        state
    }

    /// 设定点闭环骨架：避障叠加 → 推进仿真时钟 → HIL 单步。
    ///
    /// `step` / `step_rc` 共用，保证两条路径（自主设定点 / RC 直通）的
    /// 设定点型控制律（Position/Mission/Rtl/Land 等）行为一致。
    fn run_setpoint(&mut self, setpoint: &Setpoint) -> VehicleState {
        // P1-2/P3-B2 闭环联动：装备避障时，先读扇形多射线测距并把避障速度并入设定点。
        // setpoint 是 immutable 引用，这里构造一个叠加过避障的本地副本。
        let mut sp = setpoint.clone();
        if let Some(av_cfg) = &self.avoidance {
            if let Some(frame) = self.plant.read_ranger() {
                let fwd = self.plant.forward_dir_ned();
                let right = self.plant.right_dir_ned();
                // 多射线聚合决策（P3-B2）：障碍横向滑出中央射线后侧向射线仍持续覆盖，
                // 闪避方向随障碍横移连续翻转、危险度随距离连续衰减，无需单射线的
                // `hold_time` 冻结保持；障碍彻底离开视场后避障自然释放。
                let (av_vel, triggered, lat_comp) = av_cfg.avoidance_velocity(&frame, fwd, right);
                if triggered {
                    // P3-B2 横向投影（av_vel 沿 `right_ned` 的分量）用于取**幅度**：
                    // 已乘完整闪避幅度（≈evade_lateral·√sev，量级 0.5~2.0），其符号不可靠。
                    let lat_proj = av_vel[0] as f32 * right[0] as f32 + av_vel[1] as f32 * right[1] as f32;
                    let lat_mag = lat_proj.abs();

                    // 方向锁存：基于**原始排斥力横向分量** `lat_comp`（居中障碍时 ≈0，
                    // 幅度远小于 lat_proj，符号才真正代表"障碍偏哪侧"）。障碍明显偏侧
                    // （|lat_comp|>阈值）才更新方向；障碍近正前（≈0）时保持已锁存方向，
                    // 首次触发且无明确方向时默认向右——正对场景左右对称，任何单侧都等价
                    // 于远离障碍，关键是要**坚定单侧闪避**，避免逐帧翻号净位移为零
                    // （P3-B2 排查：用 lat_proj 判据时 |lat_proj| 恒 >0.5，锁存形同虚设，
                    // av_evade_dir 随噪声翻号 → 速度指令与 ev_int 净积累≈0，机体几乎不动）。
                    if lat_comp.abs() > 0.25 {
                        self.av_evade_dir = if lat_comp > 0.0 { 1.0 } else { -1.0 };
                    } else if self.av_evade_dir == 0.0 {
                        self.av_evade_dir = 1.0;
                    }

                    // 速度设定点（NED，m/s）：制动分量原样保留；横向分量改为沿
                    // **锁存方向 × 横向幅度**规范化——与下方位置偏移积分同向，
                    // 避免 av_vel 逐帧翻号时速度指令与位置设定点互相打架。
                    sp.vel[0].0 += av_vel[0] as f32 - lat_proj * right[0] as f32
                        + self.av_evade_dir * lat_mag * right[0] as f32;
                    sp.vel[1].0 += av_vel[1] as f32 - lat_proj * right[1] as f32
                        + self.av_evade_dir * lat_mag * right[1] as f32;

                    // 位置偏移泄漏积分：威胁期内按"锁存方向 × 横向幅度"沿 `right_ned`
                    // 累积位置设定点偏移，使串级 PID 的位置外环跟随侧移而非把机体拉回
                    // 原点；取代旧的 `sp.pos += av_vel*10` 硬放大（后者每帧放大速度级
                    // 目标、姿态/高度剧烈波动）。
                    //
                    // 一阶低通（加法同时乘泄漏）：稳态偏移收敛到 ≈ lateral_speed*K/λ，
                    // 与注释口径一致。纯积分（只加不泄）会在长威胁期无限制累积
                    // （实测 7s 威胁 ev_int 冲到 16.9m，机体被推到侧向 8m+，威胁结束后
                    // 回航距离过大，20s 内无法回原点）。
                    let dt = self.dt as f32;
                    let decay = (-AV_EVADE_POS_LAMBDA * dt).exp();
                    let lateral_speed = self.av_evade_dir * lat_mag;
                    self.av_evade_int[0] = self.av_evade_int[0] * decay
                        + AV_EVADE_POS_K * lateral_speed * right[0] as f32 * dt;
                    self.av_evade_int[1] = self.av_evade_int[1] * decay
                        + AV_EVADE_POS_K * lateral_speed * right[1] as f32 * dt;
                } else {
                    // 威胁结束：重置方向锁存，位置偏移按 e^(-λt) 指数泄漏回零，
                    // 位置外环把机体拉回原航线（自然释放）。
                    self.av_evade_dir = 0.0;
                    let dt = self.dt as f32;
                    let decay = (-AV_EVADE_POS_LAMBDA * dt).exp();
                    self.av_evade_int[0] *= decay;
                    self.av_evade_int[1] *= decay;
                }

                // 应用位置偏移（叠加到原设定点水平分量）。
                sp.pos[0].0 += self.av_evade_int[0];
                sp.pos[1].0 += self.av_evade_int[1];
            }
        }
        // 推进仿真时钟。
        self.time += self.dt;
        // 共享单步（方案 A）：喂入当拍原始样本（SIL 恒有 IMU/GPS/气压），
        // 由 `step_hil` 完成 SimImu 回退、姿态/位置初始化门控、EKF 估计 +
        // 气压观测、FDIR 健康闸、控制律与执行器限幅，与实机 HIL 完全同一份编排。
        // 与旧 `step` 的差异：VIO/RTK 融合（MCU HIL 无此通道）不再注入，垂直通道
        // 气压融合由 `step_hil` 内部完成（`finalize` 据此跳过 update_alt）。
        // 诊断：IMU 节流注入（HIL 反相发散复现）。默认 0=关闭（SIL 每拍真实 IMU）。
        // N>0 时每 N 拍喂一次真实 IMU，其余 None → `step_hil` 回退 sample-and-hold
        // 陈旧 IMU，复刻 HIL 中「MCU 4ms 控制拍 vs PC ~11ms 物理步」的注入饥饿。
        let imu = if self.imu_throttle > 0 {
            self.imu_step = self.imu_step.wrapping_add(1);
            if self.imu_step % self.imu_throttle == 0 {
                Some(self.imu.last)
            } else {
                None
            }
        } else {
            Some(self.imu.last)
        };
        let gps = if self.gps_throttle > 0 {
            self.gps_step = self.gps_step.wrapping_add(1);
            if self.gps_step % self.gps_throttle == 0 {
                self.gps.last
            } else {
                None
            }
        } else {
            self.gps.last
        };
        let baro_alt = Some(self.baro_alt);
        let armed = self.armed;
        // 共享单步（方案 A）：与实机 HIL（MCU control.rs）同一份编排——喂入当拍
        // 原始样本，由 `step_hil` 完成 SimImu 回退、姿态/位置初始化门控、EKF 估计
        // + 气压观测、FDIR 健康闸、控制律与执行器限幅。垂直通道气压融合由
        // `step_hil` 内部完成（`finalize` 据此跳过 update_alt）。VIO/RTK 融合见
        // `step_hil` 尾部（SIL 注入、MCU HIL 无此通道则传 None）。
        self.used_step_hil = true;
        let r = match &mut self.hil {
            CtrlVariant::Pid(h) => h.step_hil(imu, gps, baro_alt, self.vio.last, self.rtk.last, &sp, true, armed, true, &mut self.sim_imu),
            CtrlVariant::Indi(h) => h.step_hil(imu, gps, baro_alt, self.vio.last, self.rtk.last, &sp, true, armed, true, &mut self.sim_imu),
            CtrlVariant::Lqr(h) => h.step_hil(imu, gps, baro_alt, self.vio.last, self.rtk.last, &sp, true, armed, true, &mut self.sim_imu),
            CtrlVariant::Tecs(h) => h.step_hil(imu, gps, baro_alt, self.vio.last, self.rtk.last, &sp, true, armed, true, &mut self.sim_imu),
        };
        // 记录本拍执行器指令供 `finalize` 回写 plant（与旧 `h.step` 内部
        // `motors.apply` 等价）。
        self.motors.apply(&r.cmd);
        r.est
    }

    /// RC 直通闭环骨架：推进仿真时钟 → 按调用方给出的即时指令 HIL 单步。
    ///
    /// 供手动（角速率）/ 增稳（姿态保持）等**非设定点型**控制律使用，
    /// 与 `run_setpoint` 共用采集/估计/FDIR 骨架（见 [`HilContext::step_with_cmd`]）。
    fn run_cmd(&mut self, cmd_fn: impl FnOnce(&VehicleState) -> ActuatorCmd) -> VehicleState {
        self.time += self.dt;
        // 非共享单步路径：气压融合仍由 `finalize` 完成（`step_with_cmd` 内部不含 update_alt）。
        self.used_step_hil = false;
        match &mut self.hil {
            CtrlVariant::Pid(h) => h.step_with_cmd(&mut self.imu, &mut self.gps, &mut self.air, &mut self.vio, &mut self.rtk, cmd_fn, &mut self.motors, &self.cfg),
            CtrlVariant::Indi(h) => h.step_with_cmd(&mut self.imu, &mut self.gps, &mut self.air, &mut self.vio, &mut self.rtk, cmd_fn, &mut self.motors, &self.cfg),
            CtrlVariant::Lqr(h) => h.step_with_cmd(&mut self.imu, &mut self.gps, &mut self.air, &mut self.vio, &mut self.rtk, cmd_fn, &mut self.motors, &self.cfg),
            CtrlVariant::Tecs(h) => h.step_with_cmd(&mut self.imu, &mut self.gps, &mut self.air, &mut self.vio, &mut self.rtk, cmd_fn, &mut self.motors, &self.cfg),
        }
    }

    /// P3-D1：RC 直通步进——遥控输入驱动解锁、模式切换与手动/增稳控制。
    ///
    /// 完整流程："解锁 → 手动（角速率直通）→ 增稳（姿态保持）→ 定点/返航/降落"。
    /// - 解锁门控：`rc.armed` 直接决定 `armed`（链路过时 `rc.fresh=false` 时保持现状，
    ///   安全裁决仍由 FDIR 兜底）。
    /// - 模式治理：`rc.mode` 槽位 → 目标模式，经 [`ModeGovernor`] 合法性校验后生效，
    ///   健康恶化时自动降级（Critical→Land、Mission→Rtl），`self.mode` 与治理器同步。
    /// - 手动/增稳：直接映射摇杆（消费 [`HilContext::step_with_cmd`]，不跟踪位置设定点）。
    /// - 定点/返航/降落：沿用设定点闭环，锚定 `hold_pos` / `home`。
    pub fn step_rc(&mut self, rc: &RcInput) -> VehicleState {
        // 1) 解锁门控同步：遥控解锁开关（链路过时保持当前，交给 FDIR 裁决）。
        if rc.fresh {
            self.armed = rc.armed;
        }

        // 2) 模式治理：RC 模式开关槽位 → 目标模式（0/1/2/3/4/5 = 手动/增稳/
        //    定高/定点/返航/降落；6 位档位开关）。
        //    经合法性校验后生效；健康恶化主动降级；`self.mode` 与治理器保持同步。
        let slots: [FlightMode; 6] = [
            FlightMode::Manual,
            FlightMode::Stabilize,
            FlightMode::Altitude,
            FlightMode::Position,
            FlightMode::Rtl,
            FlightMode::Land,
        ];
        let target = slots[(rc.mode as usize).min(slots.len() - 1)];
        let ctx = ModeContext::new(self.armed, self.health(), self.gps.healthy());
        self.mode_gov.request(target, &ctx);
        self.mode_gov.degrade_on_health(&ctx);
        self.sync_mode();

        // 3) 采集当帧传感器样本（与 step 共用）。
        self.sample_sensors();

        // 4) 按当前模式分派控制：手动/增稳直通摇杆，其余锚定保持点。
        let state = match self.mode_gov.mode() {
            FlightMode::Manual => {
                let p = self.manual;
                self.run_cmd(|est| manual::manual_rates(rc, est, &p))
            }
            FlightMode::Stabilize => {
                let p = self.manual;
                self.run_cmd(|est| manual::stabilize(rc, est, &p))
            }
            FlightMode::Altitude | FlightMode::Position | FlightMode::Mission => {
                let sp = hover_setpoint(self.hold_pos[0], self.hold_pos[1], self.hold_pos[2]);
                self.run_setpoint(&sp)
            }
            FlightMode::Rtl => {
                let sp = hover_setpoint(self.home[0], self.home[1], self.home[2]);
                self.run_setpoint(&sp)
            }
            FlightMode::Land => {
                // 降落：锚定保持点水平位置，目标高度降至地面（NED d=0）。
                let sp = hover_setpoint(self.hold_pos[0], self.hold_pos[1], 0.0);
                self.run_setpoint(&sp)
            }
        };

        // 5) 收尾：气压融合 / 故障处理 / 解锁门控 / 推进物理世界（与 step 共用）。
        self.finalize();

        state
    }

    /// P3-D1：请求切换飞行模式（经模式治理器校验，非法请求保持当前模式不变）。
    pub fn request_mode(&mut self, target: FlightMode) -> bool {
        let ctx = ModeContext::new(self.armed, self.health(), self.gps.healthy());
        let ok = self.mode_gov.request(target, &ctx);
        self.sync_mode();
        ok
    }

    /// 当前生效的飞行模式（与 `mode` 字段同源）。
    pub fn flight_mode(&self) -> FlightMode {
        self.mode_gov.mode()
    }

    /// P3-D1：设置定点/定高/降落模式的锚定保持点（NED，m）。供地面站任务或测试平移。
    pub fn set_hold_pos(&mut self, n: f32, e: f32, d: f32) {
        self.hold_pos = [n, e, d];
    }

    /// 当前 FDIR 健康状态。
    pub fn health(&self) -> Health {
        match &self.hil {
            CtrlVariant::Pid(h) => h.fdir.health(),
            CtrlVariant::Indi(h) => h.fdir.health(),
            CtrlVariant::Lqr(h) => h.fdir.health(),
            CtrlVariant::Tecs(h) => h.fdir.health(),
        }
    }

    /// 把治理器当前模式同步到 MAV custom mode 低字节（`self.mode`）。
    fn sync_mode(&mut self) {
        self.mode = match self.mode_gov.mode() {
            FlightMode::Manual => 0,
            FlightMode::Stabilize => 1,
            FlightMode::Altitude => 2,
            FlightMode::Position => 3,
            FlightMode::Rtl => 4,
            FlightMode::Land => 5,
            FlightMode::Mission => 6,
        };
    }

    /// 推模式第一步：采集当帧传感器样本并写入传感器 trait。
    ///
    /// `step` / `step_rc` 共用，保证两条路径（自主设定点 / RC 直通）的测量通道一致：
    /// - IMU/GPS：物理真值经噪声模型；
    /// - 空速计：观测用**地速幅值** |v_ground|——EKF 空速模型 `h(x)=|v_ground|`，
    ///   喂地速才一致，避免逆风下风相对空速（<地速）把水平速度估计拉偏（曾致逆风
    ///   被吹回 + 拖拽前馈用错方向）。真实相对空速矢量经 set_measured_airspeed_vec
    ///   单独注入 TECS 做拖拽前馈（见下），两者互不干扰；
    /// - 磁力计：机体磁场（含硬铁/软铁/噪声），喂给 EKF yaw 约束；
    /// - 气压计：向上高度（m），存 `baro_alt` 供收尾做 EKF 高度观测。
    fn sample_sensors(&mut self) {
        let (imu_sample, pos_sample) = self.plant.read_sensors();
        self.imu.last = imu_sample;
        // P3-B1：GPS 可用门控——`gps_ok=false` 时强制失锁（read 返回 None），
        // 验证 VIO/RTK 融合在 GPS 中断期间兜底位置估计。
        self.gps.last = if self.gps_ok { pos_sample } else { None };
        // GPS 定位锁定：收到有效样本即置位（仿真 GPS 一经锁定持续有效）。
        if self.gps.last.is_some() {
            self.gps.has_fix = true;
        }
        let vg = self.plant.ground_airspeed_ned();
        let vh = (vg[0] * vg[0] + vg[1] * vg[1]).sqrt();
        self.air.last = Some(AirspeedSample {
            speed: Airspeed(vh as f32),
            timestamp_s: 0.0,
        });
        // P3-B1：VIO（高频相对位置/速度）与 RTK-GPS（厘米级绝对位置）样本，
        // 由 plant 真值 + 噪声模型生成，经 HilContext 注入 EKF 多源融合。
        // 可用开关用于验收对比（GPS 中断时 VIO 桥接 / VIO 单独 vs VIO+RTK 漂移）。
        self.vio.last = if self.vio_ok { self.plant.read_vio() } else { None };
        self.rtk.last = if self.rtk_ok { self.plant.read_rtk() } else { None };
        // 相对空速矢量（NED 水平）= v_ground - wind：每周期注入控制器，供 TECS
        // 空速拖拽前馈定向（其它控制器默认忽略）。
        let v_rel = self.plant.relative_airspeed_ned();
        match &mut self.hil {
            CtrlVariant::Tecs(h) => h.ctrl.set_measured_airspeed_vec(v_rel),
            _ => {}
        }
        let (mag_sample, baro) = self.plant.read_sensors_attitude();
        self.mag.last = [
            mag_sample.field[0] as f32,
            mag_sample.field[1] as f32,
            mag_sample.field[2] as f32,
        ];
        self.baro_alt = baro.altitude as f32;
    }

    /// 推模式收尾：气压计融合 → 故障处理 → 解锁门控 → 注入执行器 → 推进物理世界。
    ///
    /// `step` / `step_rc` 共用。控制指令已在 hil 步内经 `motors.apply` 记录于
    /// `self.motors.last`，这里做显式回写与安全门控：
    /// - 气压计高度融合：锚定 EKF 垂直通道，抑制开环加计积分导致的高度漂移
    ///   （真实掉高/控制抖动的根因之一）。baro.altitude 为向上高度（m），EKF 用
    ///   向下为负 D，通过 update_alt 把气压测高作为 D 位置观测（y = -alt - x[2]）。
    ///   baro 含噪声/漂移，经独立 r_alt 观测约束 D 位置。三种控制律底层都是 EkfEstimator。
    /// - 故障处理：全有效 → 按效率系数缩放每路指令；存在退化/失效电机 → 用控制
    ///   分配器(P1-1)最小二乘重分配到剩余有效电机（容错，牺牲偏航等最弱轴）。
    /// - 解锁门控：未解锁（DISARM）时强制零推力，模拟电机停转/安全上锁。
    fn finalize(&mut self) {
        let baro_alt = self.baro_alt;
        // 气压计高度融合：仅在非共享单步路径（`run_cmd` 手动/增稳）执行——
        // `step_hil`（`run_setpoint`）内部已做 update_alt，这里跳过避免双重注入。
        if !self.used_step_hil {
            match &mut self.hil {
                CtrlVariant::Pid(h) => h.est.update_alt(baro_alt),
                CtrlVariant::Indi(h) => h.est.update_alt(baro_alt),
                CtrlVariant::Lqr(h) => h.est.update_alt(baro_alt),
                CtrlVariant::Tecs(h) => h.est.update_alt(baro_alt),
            }
        }

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

        // 把控制指令显式回写被控对象（注入推力/力矩）。
        // 解锁门控：未解锁（MAVLink DISARM）时强制零推力，模拟电机停转/安全上锁。
        // 诊断：物理步节流（HIL 双时钟复现）。N>0 时每 N 个控制拍才推进一次物理，
        // 中间拍跳过 apply_actuators + plant.step（plant 冻结、指令被下一拍覆盖，
        // 控制律照常每拍运行）——复刻 HIL 中「MCU 4ms 控制拍 vs PC 物理步更慢」
        // 的时间基失配。复现完可移除。
        let do_plant_step = if self.plant_throttle > 0 {
            self.plant_step = self.plant_step.wrapping_add(1);
            self.plant_step % self.plant_throttle == 0
        } else {
            true
        };
        if do_plant_step {
            if self.armed {
                self.plant.apply_actuators(&cmd);
            } else {
                self.plant.apply_actuators(&ActuatorCmd::zero());
            }
            // 推进物理世界（已注入本拍推力）。仍推进 1×dt：HIL 的 PC 每物理步
            // 固定 `world.step(dt)`（dt=4ms），物理时钟相对控制时钟变慢正是本场景。
            self.plant.step();
        }
    }

    /// 解锁（MAVLink ARM）。电机恢复可转。
    pub fn arm(&mut self) { self.armed = true; }
    /// 上锁（MAVLink DISARM）。本拍起电机停转（零推力）。
    pub fn disarm(&mut self) { self.armed = false; }
    /// 设置飞行模式（MAV custom mode 低字节），由 MAVLink DO_SET_MODE / 内部状态机写入。
    /// 经模式治理器校验后生效（未解锁/无位置/健康不足时拒绝，保持当前模式）。
    pub fn set_mode(&mut self, mode: u8) {
        let target = match mode {
            0 => FlightMode::Manual,
            1 => FlightMode::Stabilize,
            2 => FlightMode::Altitude,
            3 => FlightMode::Position,
            4 => FlightMode::Rtl,
            5 => FlightMode::Land,
            6 => FlightMode::Mission,
            _ => return, // 未知模式码：忽略，保持当前。
        };
        let _ = self.request_mode(target);
    }
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

    /// 诊断：施加外部角冲量扰动（N·m·s，世界系 NED），模拟 HIL 释放/风等外部瞬态。
    /// 在 `step` 前调用，冲量随本步物理步生效；不受电机效率分配器补偿，用于制造
    /// 真实旋转以检验注入饥饿/时间基失配下的姿态估计反相发散。复现完移除。
    pub fn disturb_torque_impulse(&mut self, tau_impulse_ned: [f64; 3]) {
        self.plant.apply_torque_disturbance(tau_impulse_ned);
    }

    /// 阶段 9：直接推进物理世界一步（不跑控制律，供自由落体等无控场景）。
    pub fn plant_step(&mut self) {
        self.plant.step();
    }

    /// HIL：外部执行器指令驱动一物理步，并刷新传感器真值样本。
    ///
    /// 与 [`step`]（PC 控制律闭环）相对：HIL 下控制律在真实 MCU，PC 只施加
    /// `HIL_ACTUATOR_CONTROLS` 回传的电机指令并推进物理，然后重新采样 IMU/GPS 真值，
    /// 使 [`last_imu`](Self::last_imu) 返回本步最新比力/角速度，供 `HIL_SENSOR` 注入。
    pub fn plant_step_hil(&mut self, cmd: &ActuatorCmd) {
        self.plant_apply(cmd);
        self.plant.step();
        self.sample_sensors();
    }

    /// P3-B1：运行时注入 GPS 失锁（false）或恢复（true）。
    /// 失锁期间 `read()` 返回 None、`has_fix=false`（模式治理器拒绝依赖绝对位置的
    /// 模式），位置估计由 VIO/RTK 融合兜底——验收场景（vio_rtk.rs）用。
    pub fn set_gps_available(&mut self, ok: bool) {
        self.gps_ok = ok;
        if !ok {
            self.gps.last = None;
            self.gps.has_fix = false;
        }
    }

    /// P3-B1：运行时注入 VIO 失锁（false）或恢复（true）。
    /// 失锁期间不注入位置/速度观测，用于验收对比（GPS 中断时 VIO 桥接能力）。
    pub fn set_vio_available(&mut self, ok: bool) {
        self.vio_ok = ok;
        if !ok {
            self.vio.last = None;
        }
    }

    /// P3-B1：运行时注入 RTK 无固定解（false）或恢复（true）。
    /// 无固定解期间不注入厘米级位置观测，用于验收对比（VIO+RTK vs VIO 单独漂移）。
    pub fn set_rtk_available(&mut self, ok: bool) {
        self.rtk_ok = ok;
        if !ok {
            self.rtk.last = None;
        }
    }

    /// P3-B3：运行时向传感器模型注入硬/软故障（偏置突变/卡死/漂移，见 [`SensorFault`]）。
    /// 故障在"物理真值 → 传感器读数"处生效，随后喂给 EKF 与 FDIR，实现全链路故障注入。
    /// 例子：`AccelBias([0.3,0,0])` 软偏置；`AccelStuck(Some([0,0,0]))` 硬卡死（FDIR 判
    /// Critical → 失控保护归零执行器）；`GpsStuck(Some([...]))` GPS 冻结（软/硬均不退化为
    /// 失锁，`has_fix` 保持 true，位置估计被卡死值牵制）。
    pub fn inject_sensor_fault(&mut self, fault: SensorFault) {
        self.plant.inject_sensor_fault(fault);
    }

    /// P3-B3：是否已触发失控保护（FDIR Critical 单向置位，执行器归零）。
    /// 供验收测试确认硬故障（IMU 卡死）被 FDIR 检测并降级。
    pub fn failsafe_engaged(&self) -> bool {
        match &self.hil {
            CtrlVariant::Pid(h) => h.failsafe_engaged,
            CtrlVariant::Indi(h) => h.failsafe_engaged,
            CtrlVariant::Lqr(h) => h.failsafe_engaged,
            CtrlVariant::Tecs(h) => h.failsafe_engaged,
        }
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

    /// P3-D5：切换外部世界步进模式（多机共享世界用，见 [`QuadrotorPlant::set_external_world_step`]）。
    pub fn set_external_world_step(&mut self, external: bool) {
        self.plant.set_external_world_step(external);
    }

    /// P3-C3/P3-D5：注册/清除参与机体-机体碰撞的其他动态刚体碰撞体列表。
    ///
    /// 多机场景由 `MultiDroneSim` 用各机 `body_id()` 组装 peers 后调用，
    /// 每步解算双刚体碰撞（等大反向冲量、动量守恒）。
    pub fn plant_set_peer_colliders(&mut self, peers: Vec<BodyCollider>) {
        self.plant.set_peer_colliders(peers);
    }

    /// P3-D5：返回本机在物理世界中的刚体 id（`RigidBodyWorld::add_body` 返回值）。
    pub fn body_id(&self) -> i64 {
        self.plant.body_id()
    }

    /// P1-2：读取最近一次接触解算结果（未接触时为 `None`）。
    pub fn contact_info(&self) -> Option<ContactInfo> {
        self.plant.contact_info()
    }

    /// P3-D3：返回当前生效的障碍列表（引擎世界系，静态 + 动态展平后），供渲染。
    pub fn current_obstacles(&self) -> Vec<Obstacle> {
        self.plant.current_obstacles()
    }

    /// P3-D3：在引擎世界系任意点读取风场（可视化采样，只读，不扰动物理风）。
    pub fn wind_at(&self, pos: [f64; 3]) -> [f64; 3] {
        self.plant.wind_at(pos)
    }

    /// 调试：返回引擎世界系真实坐标与四元数。
    pub fn debug_up(&self) -> ([f64; 3], [f64; 4]) {
        self.plant.debug_up()
    }

    /// 调试：返回最近一次扇形多射线测距帧（避障诊断用）。
    pub fn dbg_ranger(&mut self) -> Option<crate::sensor::RangeFinderFrame> {
        self.plant.read_ranger()
    }

    /// 调试：机体前向 / 右向 NED 单位向量（P3-B2 诊断用）。
    pub fn debug_fwd_ned(&self) -> [f64; 3] {
        self.plant.forward_dir_ned()
    }

    /// 调试：机体右向 NED 单位向量（P3-B2 诊断用）。
    pub fn debug_right_ned(&self) -> [f64; 3] {
        self.plant.right_dir_ned()
    }

    /// 调试：P3-B2 避障位置偏移泄漏积分（NED 水平，m）。
    pub fn debug_av_evade(&self) -> [f32; 2] {
        self.av_evade_int
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
