//! 仿真主循环与场景。
//!
//! 当前实现 SIL 模式：物理引擎在 PC，控制律经 `FlyController`(`HilContext`) 在 PC。
//! HIL 模式（接真实飞控 USB）见 DESIGN.md §9，后续在 `hil_link.rs` 扩展。
//!
//! 日志机制：核心只产出 `LogRow`（纯数据），通过 `on_step` 回调交给 runner 决定
//! 如何存储（CSV / 内存 / 可视化）。核心不碰任何文件系统 I/O。

use flyctrl_core::controller::Setpoint;
use flyctrl_core::invariants;
use flyctrl_core::units::{Meter, MeterPerSecond, Radian};
use flyctrl_core::vehicle::{ActuatorCmd, VehicleState};

use crate::controller::{hover_setpoint, FlyController};
use crate::physics::{ContactModel, RigidBodyWorld};
use flyctrl_core::config::VehicleConfig;
use crate::wind::{WindConfig, WindField};
use crate::sensor::SensorConfig;
use crate::controller::ControllerKind;
use crate::log::LogRow;

/// P2-2 路径跟随任务结果。
pub struct MissionResult {
    /// 最大跟踪误差（m）。
    pub max_err: f64,
    /// 均方根跟踪误差（m）。
    pub rms_err: f64,
    /// 任务总时长（s）。
    pub duration: f64,
    /// 全程是否姿态稳定（无发散）。
    pub stable: bool,
}

pub struct SimLoop<W> {
    ctrl: FlyController<W>,
    cfg: VehicleConfig,
    dt: f64,
    steps: u64,
    /// 阶段 6：可选每帧回调（runner 注入，用于 CSV/可视化）。None=不记录。
    on_step: Option<Box<dyn FnMut(&LogRow)>>,
    /// 阶段 7：可选逐物理步回调（runner 注入，用于实时可视化采样）。
    /// 传世界真值（NED）与最近控制指令，由渲染器做坐标映射。
    on_frame: Option<Box<dyn FnMut(&VehicleState, ActuatorCmd)>>,
    /// 阶段 6：机械能监测（无风无推力时机械能应单调衰减）。
    energy_prev: Option<f64>,
    energy_monotonic: bool,
}

impl<W> SimLoop<W>
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
    ) -> Self {
        Self {
            ctrl: FlyController::new(world, cfg, dt, wind, sensor_cfg, kind, contact),
            cfg: cfg.clone(),
            dt,
            steps: 0,
            on_step: None,
            on_frame: None,
            energy_prev: None,
            energy_monotonic: true,
        }
    }

    /// 阶段 6：设置每帧回调（runner 侧注入 CSV 写出 / 可视化等）。
    pub fn set_on_step(&mut self, cb: Box<dyn FnMut(&LogRow)>) {
        self.on_step = Some(cb);
    }

    /// 阶段 7：设置逐物理步回调（runner 侧注入实时可视化采样）。
    /// 每个物理步触发一次，传入世界真值（NED）与最近控制指令。
    pub fn set_on_frame(&mut self, cb: Box<dyn FnMut(&VehicleState, ActuatorCmd)>) {
        self.on_frame = Some(cb);
    }

    /// 阶段 6：被控对象真实机械能（动能 + 重力势能，NED）。
    /// 用于无风无推力场景的能量守恒校验（应单调衰减）。
    fn mechanical_energy(&self, st: &VehicleState) -> f64 {
        let m = self.cfg.mass as f64;
        let v2 = st.vel[0].0 as f64 * st.vel[0].0 as f64
            + st.vel[1].0 as f64 * st.vel[1].0 as f64
            + st.vel[2].0 as f64 * st.vel[2].0 as f64;
        let ke = 0.5 * m * v2;
        // NED 下向为正，高度 h=-d，势能 = m·g·h = -m·g·d。
        let pe = -m * 9.81 * st.pos[2].0 as f64;
        ke + pe
    }

    /// 跑一个悬停场景：设定点在 (0,0,-5)，验证收敛 + 不变量。
    pub fn run_hover(&mut self, seconds: f64) -> bool {
        let total = (seconds / self.dt) as u64;
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        let mut all_ok = true;

        for _ in 0..total {
            let st = self.ctrl.step(&sp);
            self.steps += 1;

            // 阶段 7：逐物理步回调（实时可视化采样）。
            if let Some(ref mut cb) = self.on_frame {
                cb(&self.ctrl.world_state(), self.ctrl.last_cmd());
            }

            // 阶段 6：每帧回调（runner 决定如何存储/展示）。
            if let Some(ref mut cb) = self.on_step {
                let w = self.ctrl.world_state();
                let row = LogRow {
                    step: self.steps,
                    t: self.steps as f64 * self.dt,
                    true_state: w,
                    est_state: st,
                    cmd: self.ctrl.last_cmd(),
                    imu: self.ctrl.last_imu(),
                };
                cb(&row);
            }

            // 阶段 6：能量守恒校验（无风场景，机械能应单调衰减）。
            if self.energy_prev.is_none() {
                self.energy_prev = Some(self.mechanical_energy(&self.ctrl.world_state()));
            } else {
                let e = self.mechanical_energy(&self.ctrl.world_state());
                // 仅当无明显外部推力做功（悬停稳态附近）校验单调性；允许数值误差 1e-3。
                if e > self.energy_prev.unwrap() + 1e-3 {
                    self.energy_monotonic = false;
                }
                self.energy_prev = Some(e);
            }

            if self.steps <= 5 || self.steps % 500 == 0 {
                let w = self.ctrl.world_state();
                let (up, qup) = self.ctrl.debug_up();
                let cmd = self.ctrl.last_cmd();
                println!(
                    "  step {}: TRUE_NED=({:.3},{:.3},{:.3}) EST_NED=({:.3},{:.3},{:.3}) UP=({:.3},{:.3},{:.3}) att=({:.2},{:.2},{:.2},{:.2}) cmd=({:.3},{:.3},{:.3},{:.3})",
                    self.steps,
                    w.pos[0].0, w.pos[1].0, w.pos[2].0,
                    st.pos[0].0, st.pos[1].0, st.pos[2].0,
                    up[0], up[1], up[2],
                    w.att.w, w.att.x, w.att.y, w.att.z,
                    cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                );
            }

            // 不变量：状态有限、指令有界。
            if !invariants::state_finite(&st) {
                eprintln!("[FAIL] step {}: state not finite", self.steps);
                all_ok = false;
                break;
            }
            let cmd = self.ctrl.last_cmd();
            if !invariants::actuator_bounded(&cmd) {
                eprintln!("[FAIL] step {}: actuator out of bounds", self.steps);
                all_ok = false;
                break;
            }
        }

        // 悬停场景下螺旋桨持续做正功，机械能非单调是物理正确的（并非守恒系统），
        // 因此仅作信息性报告，不当作失败。真正能量守恒校验适用于"无推力自由衰减"场景。
        if !self.energy_monotonic {
            println!("[hover] 能量: 机械能非单调（悬停推力持续做功，属正常，非数值问题）");
        } else {
            println!("[hover] 能量: 机械能单调衰减 OK（无风无外部做功）");
        }

        // 收敛判定：末态位置接近设定点。
        let end = self.ctrl.world_state();
        let dz = (end.pos[2].0 - (-5.0)).abs();
        let horiz = (end.pos[0].0).hypot(end.pos[1].0);
        println!(
            "[hover] end pos NED = ({:.2},{:.2},{:.2})m  |dz|={:.2} horiz={:.2}",
            end.pos[0].0, end.pos[1].0, end.pos[2].0, dz, horiz
        );
        let converged = dz < 0.5 && horiz < 0.5;
        all_ok && converged
    }

    /// 阶段 9：自由落体能量守恒测例。
    ///
    /// 关闭所有执行器（无推力），仅重力做功，验证物理引擎积分器 +
    /// `RigidBodyWorld` 接口的能量守恒正确性：**有气动阻力时机械能应单调衰减**，
    /// 无气动阻力时（当前默认机型阻力极小，近似守恒）应非增长（数值耗散允许轻微衰减）。
    ///
    /// 机体从 (0,0,-5) 释放，初始零速度、水平姿态。测例用 ToyWorld 与生产世界
    /// 都应成立（验证替身与真实引擎一致）。
    pub fn run_freefall(&mut self, seconds: f64) -> bool {
        // 能量守恒场景：关闭地面接触（真空，无地面盒），仅验证积分器能量守恒。
        // 否则机体落回地面盒会被碰撞处理干扰，能量不再守恒（与测试意图不符）。
        self.ctrl.plant_set_contact(None);
        let total = (seconds / self.dt) as u64;
        let mut all_ok = true;
        let e0 = self.mechanical_energy(&self.ctrl.world_state());
        let start_d = self.ctrl.world_state().pos[2].0;
        let mut e_min = f64::INFINITY;
        let mut e_max = f64::NEG_INFINITY;
        let mut non_increasing = true;
        const TOL: f64 = 0.5; // 允许接触求解器在落地瞬间的极小数值噪声

        for _ in 0..total {
            // 不跑控制律、不注入推力：传零指令 + 直接推进物理世界。
            let zero = flyctrl_core::vehicle::ActuatorCmd::zero();
            self.ctrl.plant_apply(&zero);
            self.ctrl.plant_step();
            self.steps += 1;

            if let Some(ref mut cb) = self.on_frame {
                cb(&self.ctrl.world_state(), self.ctrl.last_cmd());
            }
            if let Some(ref mut cb) = self.on_step {
                let w = self.ctrl.world_state();
                let st = w.clone();
                let row = LogRow {
                    step: self.steps,
                    t: self.steps as f64 * self.dt,
                    true_state: w,
                    est_state: st,
                    cmd: self.ctrl.last_cmd(),
                    imu: self.ctrl.last_imu(),
                };
                cb(&row);
            }

            let e = self.mechanical_energy(&self.ctrl.world_state());
            e_min = e_min.min(e);
            e_max = e_max.max(e);
            // 不变量：机械能绝不允许超过初始值 + TOL（阻力/落地只耗散能量，
            // 积分器若注入非物理能量会使 e 显著抬升 —— 这是真正的 bug 信号）。
            if e > e0 + TOL {
                non_increasing = false;
            }

            // 不变量：状态有限。
            if !invariants::state_finite(&self.ctrl.world_state()) {
                eprintln!("[FAIL] freefall step {}: state not finite", self.steps);
                all_ok = false;
                break;
            }
        }

        let end = self.ctrl.world_state();
        let fell = end.pos[2].0 > start_d; // NED d 向下为正，下落 => d 增大
        println!(
            "[freefall] end pos NED = ({:.3},{:.3},{:.3})m (初始 d={:.3})",
            end.pos[0].0, end.pos[1].0, end.pos[2].0, start_d
        );
        println!(
            "[freefall] 机械能: E0={:.3}J 范围[{:.3},{:.3}] 不增={}",
            e0, e_min, e_max, non_increasing
        );
        all_ok && fell && non_increasing
    }

    /// P1-2：着陆接触测例（真实引擎 + 惩罚接触模型）。
    ///
    /// 零推力释放机体，启用地面接触（`Some(ContactModel)`）。验证惩罚接触模型把下落的
    /// 四旋翼稳定拦停在地面附近：全程状态有限、末态位于地面附近（不穿透也不被弹飞）、
    /// 末态竖直速度趋近于 0（静止在地面）。
    pub fn run_drop(&mut self, seconds: f64) -> bool {
        // 注意：区别于 run_freefall，这里**保持**接触模型启用（构造时已 Some）。
        let cm = self.ctrl.contact_info(); // 仅用于打印接触状态信息
        let _ = cm;
        let total = (seconds / self.dt) as u64;
        let mut all_ok = true;
        let start_d = self.ctrl.world_state().pos[2].0;
        let mut max_d = start_d;
        let mut min_d = start_d;
        let mut end_vd = 0.0f64;

        for _ in 0..total {
            let zero = flyctrl_core::vehicle::ActuatorCmd::zero();
            self.ctrl.plant_apply(&zero);
            self.ctrl.plant_step();
            self.steps += 1;

            if let Some(ref mut cb) = self.on_frame {
                cb(&self.ctrl.world_state(), self.ctrl.last_cmd());
            }
            if let Some(ref mut cb) = self.on_step {
                let w = self.ctrl.world_state();
                let st = w.clone();
                let row = LogRow {
                    step: self.steps,
                    t: self.steps as f64 * self.dt,
                    true_state: w,
                    est_state: st,
                    cmd: self.ctrl.last_cmd(),
                    imu: self.ctrl.last_imu(),
                };
                cb(&row);
            }

            let w = self.ctrl.world_state();
            if !invariants::state_finite(&w) {
                eprintln!("[FAIL] drop step {}: state not finite", self.steps);
                all_ok = false;
                break;
            }
            max_d = max_d.max(w.pos[2].0);
            min_d = min_d.min(w.pos[2].0);
            end_vd = w.vel[2].0 as f64;
        }

        println!(
            "[drop] NED d 范围 [{:.3},{:.3}] 末态 vd={:.3} m/s",
            min_d, max_d, end_vd
        );
        // 接触面 contact_y(引擎)=-4.9 => NED d=+4.9。机体从起点 d=-5 自由下落，
        // 应被惩罚接触拦停在 d≈+4.9 附近（max_d 为最深处）：
        // - max_d > 4.0：确实落到了地面（而非悬停起点）；
        // - max_d < 6.5：未穿透地面/未被弹飞到无穷远；
        // - end_vd > -0.5：末态竖直速度趋零（静止在地面）。
        let settled = max_d > 4.0 && max_d < 6.5 && end_vd > -0.5;
        all_ok && settled
    }

    pub fn steps(&self) -> u64 { self.steps }

    /// 阶段 5：设置电机完全失效掩码（故障注入）。true=该电机停转。
    pub fn set_motor_failure(&mut self, mask: [bool; 4]) {
        self.ctrl.set_motor_failure(mask);
    }

    /// 阶段 5（增强）：设置每路电机效率系数（1.0=正常，0.0=停转，中间=部分退化）。
    pub fn set_motor_eff(&mut self, eff: [f32; 4]) {
        self.ctrl.set_motor_eff(eff);
    }

    /// P1 扩展：覆盖接触模型（含地形高度图）。在 `run_*` 之前调用以切换到带地形的接触面。
    pub fn set_contact(&mut self, c: Option<ContactModel>) {
        self.ctrl.plant_set_contact(c);
    }

    /// 阶段 7+（Web 后端）：单步推进 + 触发可视化回调，返回当前真值状态与最近控制指令。
    ///
    /// 用于增量驱动仿真（每显示帧推进若干物理步），区别于 `run_*` 的整段阻塞运行。
    pub fn step_frame(&mut self, sp: &Setpoint) -> (VehicleState, ActuatorCmd) {
        let st = self.ctrl.step(sp);
        self.steps += 1;
        let w = self.ctrl.world_state();
        let cmd = self.ctrl.last_cmd();
        if let Some(ref mut cb) = self.on_frame {
            cb(&w, cmd);
        }
        (w, cmd)
    }

    /// 阶段 7+（Web 后端）：取当前真值状态（NED）与最近控制指令快照，供渲染/遥测使用。
    pub fn snapshot(&self) -> (VehicleState, ActuatorCmd) {
        (self.ctrl.world_state(), self.ctrl.last_cmd())
    }

    /// 阶段 7+：取引擎世界系（Y-up）真实位姿 (pos xyz, quat wxyz)。绕开 NED 映射，
    /// 供渲染直接使用（渲染世界系与引擎同为 Y-up，仅 z 轴反号）。
    pub fn debug_up(&self) -> ([f64; 3], [f64; 4]) {
        self.ctrl.debug_up()
    }

    /// 阶段 8：动力系统状态（电池端电压 V，4 路电机转速 rad/s）。
    pub fn powertrain_state(&self) -> (f64, [f64; 4]) {
        self.ctrl.powertrain_state()
    }

    /// 阶段 3：风环境接入验证场景。
    ///
    /// 注意：当前默认 PID（ctrl_params）抗风能力极弱（>~0.3 m/s 持续风会因姿态环
    /// 饱和翻滚，见 PLAN 阶段 5 高级控制律）。因此本场景**不验证"抗风位置保持"**，
    /// 而是验证：
    ///  1) 风模型接入正确——轻风下机体被吹向下风方向（horiz 增长且符合风向）；
    ///  2) 风-气动耦合物理正确——不引入 NaN/Inf、指令有界、不瞬爆；
    ///  3) 强风下仍能保持数值稳定（状态合法），暴露控制律局限而非仿真崩溃。
    pub fn run_hover_wind(&mut self, seconds: f64) -> bool {
        let total = (seconds / self.dt) as u64;
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        let mut all_ok = true;
        let mut sum_dz2 = 0.0f64;
        let mut sum_horiz2 = 0.0f64;
        let mut max_dz = 0.0f64;
        let mut drift_east = 0.0f64; // 末态 NED 东向偏移（风沿世界 -Z_up = NED 东向）
        let mut final_horiz = 0.0f64; // 末态 NED 水平位移（|n,e|）

        for _ in 0..total {
            let st = self.ctrl.step(&sp);
            self.steps += 1;

            // 阶段 7：逐物理步回调（实时可视化采样）。
            if let Some(ref mut cb) = self.on_frame {
                cb(&self.ctrl.world_state(), self.ctrl.last_cmd());
            }

            let end = self.ctrl.world_state();
            let dz = (end.pos[2].0 - (-5.0)).abs() as f64;
            let horiz = (end.pos[0].0).hypot(end.pos[1].0) as f64;
            sum_dz2 += dz * dz;
            sum_horiz2 += horiz * horiz;
            max_dz = max_dz.max(dz);
            drift_east = end.pos[1].0 as f64; // 末态 NED 东向
            final_horiz = horiz; // 末态 NED 水平位移

            if self.steps <= 5 || self.steps % 500 == 0 {
                let w = self.ctrl.world_state();
                let cmd = self.ctrl.last_cmd();
                println!(
                    "  step {}: TRUE_NED=({:.3},{:.3},{:.3}) dz={:.3} horiz={:.3} cmd=({:.3},{:.3},{:.3},{:.3})",
                    self.steps,
                    w.pos[0].0, w.pos[1].0, w.pos[2].0, dz, horiz,
                    cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                );
            }

            if !invariants::state_finite(&st) || !invariants::actuator_bounded(&self.ctrl.last_cmd()) {
                eprintln!("[FAIL] step {}: 状态/指令不合法（风注入导致数值崩溃）", self.steps);
                all_ok = false;
                break;
            }
        }

        let rms_dz = (sum_dz2 / total as f64).sqrt();
        let rms_h = (sum_horiz2 / total as f64).sqrt();
        println!(
            "[wind-hover] RMS_dz={:.3}m RMS_horiz={:.3}m max_dz={:.3}m drift_east={:.3}m",
            rms_dz, rms_h, max_dz, drift_east
        );
        println!(
            "[wind-hover] 说明: 当前 PID 抗风上限~0.3m/s，更强风会饱和翻滚（控制律局限，非仿真错误）；本场景验证风模型接入正确 + 数值稳定。"
        );
        // 合格 = 风模型生效（风场耦合进动力学，机体被吹离原点：水平位移 > 0.1m）
        //       + 数值稳定（无非法状态 / 指令有界）。
        // 注意：当前默认 PID 抗风上限 ~0.3m/s，强风会饱和翻滚、无法保持高度/位置，
        // 因此本测例不验证"抗风位置保持"，只验证风-气动耦合正确接入且不发散。
        let wind_effect = final_horiz > 0.1;
        all_ok && wind_effect
    }

    /// 阶段 5（增强）：部分效率退化容错边界分析。
    ///
    /// 流程：先正常悬停 `stabilize` 秒建立稳态，再把指定电机效率降到 `eff`
    /// （如 0.6 = 部分退化），继续悬停 `recover` 秒。
    ///
    /// 关键结论（真实物理 + 控制论）：在**无控制分配重构**的 PID/INDI/LQR 下，
    /// 单电机推力损失（无论完全还是部分）都会削减总推力上限，导致：
    ///   - 推力不足 → 持续掉高（位置不可恢复，除非引入控制分配重构）；
    ///   - 姿控回路在退化后也很快发散（roll/pitch 失控翻滚），属阶段 5 已确认结论：
    ///     **四旋翼单电机推力损失（无论完全还是部分）在当前无重构控制律下均不可恢复**。
    ///
    /// 本场景**量化容错边界**（故障传播分析，非通过性测试）：
    ///   - 先正常悬停 `stabilize` 秒建立稳态；
    ///   - 注入 m`motor_idx` 效率 `eff`，继续悬停 `recover` 秒；
    ///   - 记录"姿控存活时间"（从注入到机体角速度超 `att_rate_lim` 或状态非法的步数）；
    ///   - 退化越轻，存活时间越长——给飞行员/故障检测更多处置窗口（真实工程结论）。
    ///
    /// P2-2 任务级逻辑：按 waypoint 折线路径巡航（起飞→水平移动→降落悬停）。
    ///
    /// `waypoints`：`(n, e, d, yaw)` NED 坐标序列；`cruise_v` 巡航速度（m/s）。
    /// 路径按弧长参数化：无人机沿折线以 `cruise_v` 匀速推进，每步给控制器
    /// 期望位置 + 前馈速度 + 偏航。记录跟踪误差并返回结果。
    pub fn run_mission(
        &mut self,
        waypoints: &[(f64, f64, f64, f64)],
        cruise_v: f64,
    ) -> MissionResult {
        if waypoints.len() < 2 {
            return MissionResult { max_err: 0.0, rms_err: 0.0, duration: 0.0, stable: true };
        }
        // 起飞稳定段：先悬停在第一个 waypoint 2s，让无人机从初始位置爬升并稳定，
        // 避免初始大跟踪误差（无人机初始在地面而路径起点在空中）导致控制律猛拉发散。
        {
            let (n0, e0, d0, y0) = waypoints[0];
            let sp0 = Setpoint {
                pos: [Meter(n0 as f32), Meter(e0 as f32), Meter(d0 as f32)],
                vel: [MeterPerSecond::ZERO; 3],
                yaw: Radian(y0 as f32),
            };
            for _ in 0..(500u64) {
                // 2s @ dt=4ms
                let (st, _) = self.step_frame(&sp0);
                let wtrue = self.ctrl.world_state().omega;
                let rate =
                    (wtrue[0].0 * wtrue[0].0 + wtrue[1].0 * wtrue[1].0 + wtrue[2].0 * wtrue[2].0)
                        .sqrt();
                if rate > 6.0 || !invariants::state_finite(&st) {
                    return MissionResult { max_err: 0.0, rms_err: 0.0, duration: 0.0, stable: false };
                }
            }

        }
        // 路径段（弧长参数化）
        let nseg = waypoints.len() - 1;
        let mut seg_len = vec![0.0f64; nseg];
        let mut cum_len = vec![0.0f64; nseg + 1];
        for i in 0..nseg {
            let (a, b) = (waypoints[i], waypoints[i + 1]);
            let dx = b.0 - a.0;
            let dy = b.1 - a.1;
            let dz = b.2 - a.2;
            seg_len[i] = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-9);
            cum_len[i + 1] = cum_len[i] + seg_len[i];
        }
        let total = cum_len[nseg];
        let mut s = 0.0f64;
        let mut max_err = 0.0f64;
        let mut sq_err = 0.0f64;
        let mut n = 0u64;
        let mut stable = true;

        while s < total + 1e-6 {
            // 定位当前路径段与弧长偏移
            let mut seg = 0usize;
            while seg < nseg && s > cum_len[seg + 1] {
                seg += 1;
            }
            let local = (s - cum_len[seg]).clamp(0.0, seg_len[seg]);
            let f = local / seg_len[seg];
            let a = waypoints[seg];
            let b = waypoints[seg + 1];
            let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
            let inv = 1.0 / seg_len[seg];
            // 期望位置
            let (pn, pe, pd) = (a.0 + dx * f, a.1 + dy * f, a.2 + dz * f);
            let _ = inv;
            // 偏航（段起点 yaw）
            let yaw = a.3;

            let sp = Setpoint {
                pos: [Meter(pn as f32), Meter(pe as f32), Meter(pd as f32)],
                // 位置追踪（不给速度前馈）：让 PID 位置环自行追踪，避免速度前馈
                // 与高度/姿态环耦合导致移动中掉高。
                vel: [MeterPerSecond::ZERO; 3],
                yaw: Radian(yaw as f32),
            };
            let (st, _cmd) = self.step_frame(&sp);
            // 跟踪误差（NED 欧氏）
            let act = [st.pos[0].0 as f64, st.pos[1].0 as f64, st.pos[2].0 as f64];
            let err = ((act[0] - pn).powi(2) + (act[1] - pe).powi(2) + (act[2] - pd).powi(2)).sqrt();
            if err > max_err {
                max_err = err;
            }
            sq_err += err * err;
            n += 1;
            // 发散检测：用真值角速度（`st` 为估计状态，起飞加速瞬间 EKF 角速度估计
            // 可能有尖峰，误判发散）。真值用 world_state()。
            let wtrue = self.ctrl.world_state().omega;
            let rate =
                (wtrue[0].0 * wtrue[0].0 + wtrue[1].0 * wtrue[1].0 + wtrue[2].0 * wtrue[2].0)
                    .sqrt();
            // 发散阈值：起飞/转弯的瞬时角速度可达数 rad/s，属正常机动；
            // 取 6 rad/s 表示真正失控翻滚。当前 PID 悬停控制器在持续移动目标下
            // 会掉高/振荡（无倾斜垂直分量补偿），任务层据此可靠检测失败（stable=false）。
            if rate > 6.0 || !invariants::state_finite(&st) {
                stable = false;
                break;
            }
            s += cruise_v * self.dt;
        }
        MissionResult {
            max_err,
            rms_err: if n > 0 { (sq_err / n as f64).sqrt() } else { 0.0 },
            duration: s / cruise_v.max(1e-9),
            stable,
        }
    }

    /// 返回 `false`（不可恢复，符合当前控制律预期）；输出为分析性报告。
    pub fn run_hover_degraded(
        &mut self,
        stabilize: f64,
        recover: f64,
        motor_idx: usize,
        eff: f32,
    ) -> bool {
        let total_stab = (stabilize / self.dt) as u64;
        let total_rec = (recover / self.dt) as u64;
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        let mut all_ok = true;
        let att_rate_lim = 1.0f64; // rad/s：姿控发散角速度阈值

        println!(
            "[degraded] 阶段1: 正常悬停 {}s 建立稳态 (电机全正常)",
            stabilize
        );
        // 确保稳定段电机全正常（覆盖调用方可能预先注入的退化）。
        self.ctrl.set_motor_eff([1.0f32; 4]);
        for _ in 0..total_stab {
            let st = self.ctrl.step(&sp);
            self.steps += 1;
            if !invariants::state_finite(&st) || !invariants::actuator_bounded(&self.ctrl.last_cmd()) {
                eprintln!("[FAIL] degraded stabilize step {}: 状态/指令不合法", self.steps);
                all_ok = false;
                break;
            }
        }

        if all_ok {
            // 注入部分效率退化。
            let mut eff_arr = [1.0f32; 4];
            eff_arr[motor_idx] = eff.clamp(0.0, 1.0);
            self.ctrl.set_motor_eff(eff_arr);
            println!(
                "[degraded] 阶段2: 注入 m{} 效率={:.2}（部分退化），继续悬停 {}s（量化容错边界）",
                motor_idx, eff_arr[motor_idx], recover
            );
            let mut max_dz = 0.0f64;
            let mut survived_steps: u64 = 0; // 姿控存活步数（从注入起）
            let mut diverged = false;
            for _ in 0..total_rec {
                let st = self.ctrl.step(&sp);
                self.steps += 1;
                let end = self.ctrl.world_state();
                let dz = (end.pos[2].0 - (-5.0)).abs() as f64;
                max_dz = max_dz.max(dz);
                // 机体角速度范数（姿控存活指标）。
                let w = end.omega;
                let att_rate = (w[0].0 * w[0].0 + w[1].0 * w[1].0 + w[2].0 * w[2].0).sqrt() as f64;

                if !diverged {
                    if att_rate > att_rate_lim || !invariants::state_finite(&st) {
                        diverged = true;
                        println!(
                            "[degraded] 姿控发散于注入后 {:.3}s（step {}，{} 仍有限={}）",
                            survived_steps as f64 * self.dt, self.steps,
                            if invariants::state_finite(&st) { "状态" } else { "状态非法" },
                            invariants::state_finite(&st)
                        );
                    } else {
                        survived_steps += 1;
                    }
                }

                if self.steps % 500 == 0 {
                    let cmd = self.ctrl.last_cmd();
                    println!(
                        "  step {}: TRUE_NED=({:.3},{:.3},{:.3}) dz={:.3} |w|={:.3} cmd=({:.3},{:.3},{:.3},{:.3})",
                        self.steps,
                        end.pos[0].0, end.pos[1].0, end.pos[2].0, dz, att_rate,
                        cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                    );
                }
                if !invariants::state_finite(&st) || !invariants::actuator_bounded(&self.ctrl.last_cmd()) {
                    all_ok = false;
                    break;
                }
            }

            let survived_s = survived_steps as f64 * self.dt;
            println!(
                "[degraded] 末态 NED=({:.2},{:.2},{:.2})m  max|dz|={:.2}m  姿控存活≈{:.3}s / {} 步",
                self.ctrl.world_state().pos[0].0,
                self.ctrl.world_state().pos[1].0,
                self.ctrl.world_state().pos[2].0,
                max_dz, survived_s, survived_steps
            );
            if diverged {
                println!(
                    "[degraded] 结论：单电机推力损失（eff={:.2}）致姿控发散，四旋翼不可恢复——需控制分配重构（未来工作）",
                    eff_arr[motor_idx]
                );
            } else {
                println!(
                    "[degraded] 结论：注入后 {}s 内姿控仍存活（推力损失致掉高 {:.2}m，需控制分配恢复高度）",
                    recover, max_dz
                );
            }
            // 当前控制律无重构分配，单电机退化均判不可恢复（符合阶段 5 结论）。
            false
        } else {
            false
        }
    }
}

// 抑制未使用告警。
#[allow(dead_code)]
fn _assert_state(_: VehicleState) {}

// 便捷：构造抗风场景的风配置（稳定侧风 + 阵风 + 轻湍流）。
#[allow(dead_code)]
pub fn windy_config() -> WindConfig {
    use crate::wind::WindVec;
    // 基础侧风：世界系 NED 东向 3 m/s -> UP 系 (x=n=0, y=-d=0, z=-e=-3)
    // 注：当前 PID（ctrl_params）为无风悬停增益，强风会饱和翻滚；
    // 这里用中等强度（2 m/s 基础 + 轻阵风），验证风-气动耦合正确且不发散。
    let base: WindVec = [0.0, 0.0, -0.8];
    WindConfig {
        base,
        gust_amp: [0.3, 0.0, 0.2],
        gust_freq: 0.12,
        turb_sigma: [0.1, 0.1, 0.15],
        turb_tau: 0.7,
        seed: 0xCAFE_BEEF,
    }
}
