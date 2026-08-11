//! 飞控包装：把 `flyctrl-core` 的 `HilContext`（SIL/HIL 共享控制律）接入仿真闭环。
//!
//! 关键：这里实现**真实**（非 mock）传感器/执行器 trait，数据来自物理引擎世界
//! （经 `QuadrotorPlant`），控制输出回写 `plant`。这样跑的是与 MCU 上 `control.rs`
//! 同一份 `EkfEstimator` + `PidController` + `Fdir` 算法。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{Controller, IndiController, LqrController, PidController, Setpoint};
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::hal::actuator::{MotorActuator, OutputProtocol};
use flyctrl_core::hal::sensor::{GpsSensor, ImuSensor};
use flyctrl_core::hil::HilContext;
use flyctrl_core::units::{Meter, MeterPerSecondSquared, Radian, RadianPerSecond, Second};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, VehicleState};

use crate::plant::QuadrotorPlant;
use crate::wind::WindField;
use crate::sensor::SensorConfig;

/// 控制器种类（阶段 5：高级控制律对比）。
#[derive(Clone, Copy, Debug)]
pub enum ControllerKind {
    Pid,
    Indi, // INDI 包装 PID 基线
    Lqr,
}

/// 不同控制器类型的 HIL 闭环（泛型单态化）。
enum CtrlVariant {
    Pid(HilContext<EkfEstimator, PidController>),
    Indi(HilContext<EkfEstimator, IndiController<PidController>>),
    Lqr(HilContext<EkfEstimator, LqrController>),
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

pub struct FlyController {
    hil: CtrlVariant,
    plant: QuadrotorPlant,
    imu: SimImu,
    gps: SimGps,
    motors: SimMotors,
    cfg: VehicleConfig,
    /// 阶段 5：故障注入——电机失效掩码（true=该电机停转/0% 效率）。
    fail_mask: [bool; 4],
}

impl FlyController {
    pub fn new(
        cfg: &VehicleConfig,
        dt: f64,
        wind: Option<WindField>,
        sensor_cfg: SensorConfig,
        kind: ControllerKind,
    ) -> Self {
        let ekf = EkfEstimator::default_quad();
        let dt_s = Second(dt as f32);
        let hil = match kind {
            ControllerKind::Pid => {
                let ctrl = PidController::from_config(&cfg.ctrl_params());
                CtrlVariant::Pid(HilContext::new(ekf, ctrl, dt_s))
            }
            ControllerKind::Indi => {
                let base = PidController::from_config(&cfg.ctrl_params());
                let indi = IndiController::with_inertia(base, cfg.inertia, dt as f32, 0.8);
                CtrlVariant::Indi(HilContext::new(ekf, indi, dt_s))
            }
            ControllerKind::Lqr => {
                let ctrl = LqrController::from_config(&cfg.ctrl_params());
                CtrlVariant::Lqr(HilContext::new(ekf, ctrl, dt_s))
            }
        };

        let plant = QuadrotorPlant::new(cfg, dt, wind, sensor_cfg);

        let imu = SimImu {
            last: ImuSample {
                accel: [MeterPerSecondSquared(0.0); 3],
                gyro: [RadianPerSecond(0.0); 3],
            },
        };
        let gps = SimGps { last: None };
        let motors = SimMotors { last: ActuatorCmd::zero() };

        Self {
            hil,
            plant,
            imu,
            gps,
            motors,
            cfg: cfg.clone(),
            fail_mask: [false; 4],
        }
    }

    /// 阶段 5：设置电机失效掩码（阶段 5 故障注入）。索引 0..3 对应 m0..m3。
    pub fn set_motor_failure(&mut self, mask: [bool; 4]) {
        self.fail_mask = mask;
    }

    /// 推模式：先让 plant 产出当帧样本，存入传感器 trait，再跑控制律，最后 step 世界。
    pub fn step(&mut self, setpoint: &Setpoint) -> VehicleState {
        // 1) 取世界真值（NED 语义）。
        let (imu_sample, pos_sample) = self.plant.read_sensors();
        self.imu.last = imu_sample;
        self.gps.last = pos_sample;

        // 2) 跑控制律（SIL/HIL 共享闭环）。motors.apply 只记录指令。
        let state = match &mut self.hil {
            CtrlVariant::Pid(h) => h.step(&mut self.imu, &mut self.gps, setpoint, &mut self.motors, &self.cfg),
            CtrlVariant::Indi(h) => h.step(&mut self.imu, &mut self.gps, setpoint, &mut self.motors, &self.cfg),
            CtrlVariant::Lqr(h) => h.step(&mut self.imu, &mut self.gps, setpoint, &mut self.motors, &self.cfg),
        };

        // 2.5) 阶段 5：故障注入——失效电机指令强行置 0。
        let mut cmd = self.motors.last;
        for i in 0..4 {
            if self.fail_mask[i] {
                cmd.motor[i] = 0.0;
            }
        }

        // 2.6) 把控制指令显式回写被控对象（注入推力/力矩）。
        self.plant.apply_actuators(&cmd);

        // 3) 推进物理世界（已注入本拍推力）。
        self.plant.step();

        state
    }

    /// 取当前世界状态（NED）用于日志/不变量检查。
    pub fn world_state(&self) -> VehicleState {
        self.plant.state_ned()
    }

    /// 调试：返回引擎世界系真实坐标与四元数。
    pub fn debug_up(&self) -> ([f64; 3], [f64; 4]) {
        self.plant.debug_up()
    }

    /// 最近一次控制指令。
    pub fn last_cmd(&self) -> ActuatorCmd {
        self.motors.last
    }

    /// 最近一次 IMU 样本（阶段 6 日志用）。
    pub fn last_imu(&self) -> ImuSample {
        self.imu.last
    }
}

// 辅助：构造悬停设定点。
pub fn hover_setpoint(n: f32, e: f32, d: f32) -> Setpoint {
    Setpoint::hover([Meter(n), Meter(e), Meter(d)], Radian(0.0))
}
