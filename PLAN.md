# 高保真四旋翼仿真环境构建计划

> 目标：把 `fly-simulater` 从"闭环通路验证器"升级为"接近真实环境水平"的 SIL 仿真。
> 真实度优先级：机架真实化 > 气动/推进真实化 > 环境（风/扰动）> 传感器真实化 > 高级控制律对比 > 一致性/可复现。

## 当前基线（已具备）

- 闭环打通：物理引擎(`phy_ffi.dll`) 刚体 + RK4 + 旋翼推力/反扭矩注入 → 传感器(NED) → EKF → PID → mixer → 回注。
- `flyctrl-core::VehicleConfig` 已含 `inertia/drag_coeff/induced_drag_coeff/disk_area/air_density/motor_tau` 等真实字段，且有 `default_quad()`。
- 物理引擎已暴露：逐体 `add_body(inertia3)`、`apply_force/apply_torque`、`get_velocity/get_angular_velocity/get_rigid_transforms`、流体 SPH、颗粒系统。

## 已知缺口（必须修）

1. **惯量被近似覆盖（BUG）**：`plant.rs:100` 用 `I = mass·arm²` 近似，丢弃了 `VehicleConfig.inertia` 真实值。
2. **机架硬编码**：只能 `default_quad()`，无外部输入机制。
3. **气动阻力在 fly-simulater 侧未接**：`plant.rs` 没有 `drag_coeff` 项（flyctrl 工程的 `sim/src/physics.rs` 已接，但本工程没接）。
4. **电机无滞后、推力线性、无地面效应/桨干扰**。
5. **传感器理想、无风、无故障注入、无日志回放**。

---

## 阶段 0 — 机架数据输入机制 ⭐ 先做

**目的**：让"输入机架数据"成为显式动作，取代硬编码 `default_quad()`。

- 新建 `fly-simulater/airframes/450quad_x.toml`（+ 一个更小的 `5inch.toml` 示例）。
- 字段：`name, mass, arm_length, thrust_coeff, torque_coeff, inertia[3], motor_tau, drag_coeff[3], induced_drag_coeff, disk_area, air_density, gravity, tilt_max, hover_thrust, vmax_xy, vmax_z`。
- 在 `fly-simulater` 加 `toml` + `serde` 依赖，`src/airframe.rs`：
  - `struct AirframeToml`(serde) → 手动构造 `flyctrl_core::VehicleConfig`（用字面量 `"loaded"` 作 name，core 保持 no_std 不改）。
  - `load_airframe(path) -> Result<VehicleConfig, String>`。
- `main.rs` 支持 `--airframe <path>`，缺省回退 `default_quad()`。

## 阶段 1 — 真实惯量（修 BUG）

- `plant.rs:100-101` 删除 `m·L²` 近似，改用 `cfg.inertia`（引擎需 f64：直接 cast）。
- 校验：惯量对角且正定，断言 `Ixx,Iyy,Izz > 0`。

## 阶段 2 — 高保真推进模型 ✅ 已完成

- **气动阻力接入 `plant.rs`**：机体三轴型阻 `0.5·ρ·Cd·|v|·v` + 诱导阻力（随前飞速度 sigmoid，悬停≈0，前飞→`k·T` 沿机体 -Z），在机体坐标系施加（`aero_drag_body`）。
- **电机一阶滞后**：`plant` 维护 `thrust_actual[i]`，`step` 前 `thrust_actual += (cmd-actual)·(dt/tau)`，`tau=cfg.motor_tau`（alpha 做数值稳定 clamp）。
- **地面效应**：机体高度 < 1.5×桨径时推力增益 +30%（`ground_effect_gain`）。
- **推力非线性**：`T = thrust_coeff · u`（线性，u∈[0,1] 即满油门；预留 `ct·ω²` 开关）。
- **桨盘干扰（进阶，未做）**：相邻反向旋转桨下洗耦合修正项，留待精度需求更高时加。

**验证**：450quad/5inch 悬停均收敛（|dz|≤0.04m），且悬停态气动≈0 不引入虚假偏置（模型物理正确）；前飞场景将体现气动阻力。

## 阶段 3 — 风场与环境 ✅ 已完成

- `src/wind.rs`：基础风 + 阵风(多频正弦脉冲) + 湍流(Dryden 简化一阶低通白噪声)，全部种子化(确定性可复现)。世界系(引擎 Y-up)风速表示。
- **注入方式(高保真且物理一致)**：风以"相对气流"融入气动模型——`v_rel = v_body - wind`，气动阻力基于相对风速，无需额外外力接口(`plant::aero_drag_body`)。
- 场景：`--scenario wind` 跑抗风悬停(15s)，验证风模型生效(机体被吹向下风方向) + 数值稳定。

**重要发现(真实仿真价值)**：当前默认 `PidController`(flyctrl-core `ctrl_params`)**抗风能力极弱**——持续风 >~0.3 m/s 即因姿态环饱和翻滚发散(0.8m/s 第 500 步即 `cmd=(0,0,1,1)` 饱和)。这是**控制律局限**非仿真错误，将在阶段 5(INDI/LQR)解决。阶段 3 场景判定 = 风模型接入正确 + 数值稳定 + 风效应可观测，不要求位置保持。

## 阶段 4 — 传感器真实化 ✅ 已完成（模型接入）

- `src/sensor.rs`：`SensorModel` 确定性种子化。
  - IMU：零偏、高斯白噪声、陀螺随机游走(bias drift)、机体振动耦合(40Hz)。
  - GPS/NED：固定延迟环形缓冲(0.15s)、降频(5Hz)、位置/速度噪声、偶发丢星(NaN→None)。
- 接入 `plant::read_sensors`：真值过 `SensorModel::process` 转带噪/延迟读数。
- CLI `--sensor-noise` 开启真实噪声(`SensorConfig::realistic`)；默认零噪声保持场景 PASS。

**重要发现（真实仿真价值）**：开启 `--sensor-noise` 后 hover 立即发散(dz=102m)。
诊断确认：即使极小噪声(accel 0.5mg / gyro 0.017°/s)也发散，而零噪声时 PASS →
**当前 EKF 测量噪声协方差 R 疑似为 0**，任何 IMU 噪声被当成真实信号积分 → 位置误差指数累积。
这是**估计器缺陷**（非传感器模型 bug），将在阶段 5（高级控制律/估计器修复）解决。
阶段 4 已正确实现传感器模型，默认零噪声保证可用性，真实噪声开关用于暴露 EKF 问题。

## 阶段 5 — 高级控制律对比 + 故障注入 ✅ 已完成（控制律对比 + 故障注入机制）

- **控制器种类切换**：`controller.rs` 抽象 `ControllerKind { Pid, Indi, Lqr }` + `CtrlVariant`
  （枚举承载 `HilContext<EkfEstimator, Ctrl>`，泛型单态化）。`SimLoop::new` 接收 `kind`，
  `main.rs` 加 `--controller pid|indi|lqr`（缺省 pid）。
- **INDI**：`IndiController::with_inertia(base_pid, cfg.inertia, dt, 0.8)`，包 PID 基线。
- **LQR**：`LqrController::from_config(&cfg.ctrl_params())`。
- **故障注入（二元）**：`FlyController::set_motor_failure([bool;4])`；`step` 内对失效电机指令强制置 0
  再回写 plant。`main.rs` 加 `--fail-motor 0..3`（单电机停转）。
- **故障注入（增强：效率系数模型）**：`fail_mask` 从 `[bool;4]` 升级为 `[f32;4]` 效率系数
  （1.0=正常，0.0=完全停转，中间=部分效率退化）。`set_motor_eff([f32;4])` 夹紧到 [0,1]；
  `step` 第 2.5 步按系数**缩放**每路电机指令（`cmd.motor[i] *= fail_mask[i]`）而非置零。
  `set_motor_failure` 保留为 `true→0.0` 的便捷封装。`main.rs` 加 `--degrade-motor <0..3> <0..1>`。

**验证（无风悬停 10s, dt=4ms, 450quad）**：
- PID / INDI / LQR **三者均 PASS**（|dz|≈0.04m, horiz≈0）。INDI/LQR 悬停行为与 PID 一致收敛，
  证明 `ControllerKind` 抽象与控制律接线正确、可复现对比。
- `--fail-motor 0`：m0 指令强制置 0 → `cmd=(1,0,1,0)` 饱和、机体爬升漂移(dz=107m) → **合法 FAIL**
  （四旋翼单旋翼失效不可恢复，无冗余自由度）。仿真数值稳定（无 NaN/Inf），故障注入路径正确。

**量化容错边界（阶段 5 增强，`--scenario degraded` + `--degrade-motor 0 <eff>`，先 4s 正常悬停
建立稳态再注入）**：实测单电机推力损失（无论完全/部分）均致姿控发散、四旋翼不可恢复——
轻退化仅给飞行员稍长处置窗口，无重构控制律下无法重配平。存活时间（注入后角速度范数 >1.0 rad/s
即判发散）随效率系数变化：

| eff  | 存活时间 | 存活步数 |
|------|---------|---------|
| 0.95 | 0.416s  | 104     |
| 0.90 | 0.368s  | 92      |
| 0.80 | 0.332s  | 83      |
| 0.60 | 0.120s  | 30      |
| 0.50 | 0.096s  | 24      |
| 0.00 | 0.052s  | 13      |

结论：**当前 PID 控制律无控制分配重构，单电机推力损失不可恢复**（与二元失效同结论，但部分退化
量化了"容错边界"——即便 5% 推力损失也仅 ~0.4s 处置窗口）。真正可恢复需控制分配（control
allocation）重排剩余 3 路电机推力/力矩，属未来工作（见 PLAN 待办）。

**重要发现（真实仿真价值）**：
1. 无风干净悬停下 PID/INDI/LQR 轨迹几乎重合——差异仅在抗扰/动态场景显现，需阶段 6 日志量化。
2. `--sensor-noise` 暴露的 EKF 发散（阶段 4 发现，R 协方差疑似 0）**仍未修复**：`--controller lqr/indi`
   搭配 `--sensor-noise` 同样发散（EKF 共享，控制律不修估计器）。限为已知缺陷，待 flyctrl-core EKF 修正。
3. 单电机故障（完全/部分）均合法不可恢复——四旋翼无冗余自由度，可恢复容错需控制分配重构或
   六旋翼配置，留待精度需求更高时加。

**回归锁**（新增 `tests/degraded.rs`，`cargo test --test degraded`）：
- `eff_scales_motor_command_not_zero`：部分退化（eff=0.6）走缩放路径而非置零，m0 指令 >0、m1 不
  受影响。
- `degraded_full_loss_diverges`：eff=0.0 注入后姿控发散，存活步数有限（< 注入后 8s 窗口）。
- `tolerance_boundary_monotonic`：eff=0.95 存活步数 > eff=0.50（量化边界单调，物理一致性）。

## 阶段 6 — 闭环一致性与可复现 ✅ 已完成

- 时间步对齐：控制周期(100/250Hz) 与物理子步明确分离（`dt` 单源，主循环统一推进）。
- 日志/回放：`src/log.rs` 导出 28 列 CSV（step,t / TRUE_NED(3) / EST_NED(3) / vel(3) /
  att_quat(4) / omega(3) / cmd(4) / IMU accel(3) / gyro(3)），`--log <path>` 开启，`flush` 刷盘。
- 不变量监控：保留 NaN/有界检查（`invariants::state_finite` / `actuator_bounded`）；
  机械能监测（`mechanical_energy` = 动能 + 重力势能 NED）作为信息性指标——悬停推力做功
  下非单调属正常，不当作失败，仅"无推力自由衰减"场景才要求单调。
- 控制律量化对比：基于日志 RMS_dz / RMS_cmd / 抗风余量（阶段 3 `run_hover_wind` 已输出
  RMS_horiz / drift），可横向对比 PID/INDI/LQR 实测表现。

**验证**：`cargo build` 通过；`--log sim_out.csv` 悬停 10s 输出 2501 行（含表头）、28 列齐全；
能量项信息性正常打印，无 NaN/越界。

**后续可选增强（非阻塞）**：
1. 日志分析脚本（`tools/cmp_controllers.py`）自动算 RMS_dz/RMS_cmd 并出 PID/INDI/LQR 对比表。
2. 真·能量守恒测例：无推力自由落体/抛掷场景，校验机械能单调衰减（需新增 scenario `freefall`）。
3. 回放器：读 CSV 重渲染轨迹（matplotlib），用于回归与论文配图。

---

## 阶段 7 — 物理引擎解耦（依赖倒置 + 可替换替身）✅ 已完成

**动机**（来自用户提供的"物理引擎接口设计"讨论）：仿真层不应直接耦合具体物理引擎
（我们原本在 `plant.rs` 里裸调 `phy_ffi` C-ABI 函数），应依赖 `RigidBodyWorld` trait，
使引擎可替换、且能注入玩具级替身做单元测试（无需启动 C 引擎、加速测试、验证接口充分性）。

**实现**：
- `src/physics.rs`：定义 `RigidBodyWorld` trait（最小充分接口：`body_count/add_body/
  apply_force/apply_torque/get_velocity/get_angular_velocity/get_rigid_transforms/step/time`）。
  **关键语义约定（写进文档）**：`apply_force/apply_torque` 为**瞬态**，引擎 `step` 后必清零，
  防替换引擎出"幽灵残留推力"。重力由引擎内部持有（世界系 (0,-g,0)），不每次传入。
- `PhyFfiWorld`：真实引擎适配器，包 `phy_ffi` 不安全调用为 trait 方法（`Drop` 自管销毁）。
- `ToyWorld`：半隐式欧拉刚体积分 + 简单地面碰撞的测试替身，确定性、无外部依赖。
- `QuadrotorPlant<W>` / `FlyController<W>` / `SimLoop<W>` 全部静态泛型化（`W: RigidBodyWorld`），
  生产路径用 `PhyFfiWorld`（便捷别名 `RealFlyController = FlyController<PhyFfiWorld>`）。
- 引入 `src/lib.rs`，使集成测试 `use fly_simulater::...` 验证**公共接口**（可替换性验证）。
- `tests/physics_toy.rs`：4 个集成测试（满油门上升 / 纯力矩生角速度 / 瞬态力 step 后清零 /
  plant 替身连续 step 无 NaN），全 PASS。

**与"通用物理引擎接口"提案的取舍**（已在模块文档记录）：
- 不暴露 `set_state`/`at_point`：四旋翼推力沿机体过质心轴，纯力矩用 `apply_torque`，
  `at_point` 对四旋翼无意义；引擎持有状态，仿真层不写回。
- 保留批量 `get_rigid_transforms`：真实引擎 ABI 即此形态，替身对齐避免为单 body 改 ABI。
- 静态泛型（`impl RigidBodyWorld`）而非 `dyn`：避免 vtable 跨 C-FFI 边界的 ABI 风险。

**验证**：`cargo build` 通过；生产悬停 PASS（dz=0.04，与重构前一致，行为零回归）；
`cargo test --test physics_toy` 4/4 PASS。

---

## 阶段 8 — 修复悬停冻结 + FDIR 误判两个真实 BUG ✅

**背景**：阶段 5/7 的悬停"PASS"是虚假的——悬停稳定后真实机体在 ~1.2s 内会突然丢失推力并锁死。
批处理 CSV 暴露：`m0` 在 ~325 步突降为 0，`true_d` 永久冻结在 -4.778m（未达设定点 -5）。

**根因 1（物理引擎休眠机制，致命）**：`phy-rigid` 的 `RigidWorld::step` 带 B1 休眠管理——
速度低于阈值（`sleep_lin_vel2=0.01`）持续 `sleep_time=0.5s` 即把 body 置 `sleeping`、
清零速度并**冻结位置**。四旋翼悬停稳定后速度趋于 0，引擎误判"近静止"→ 机体被锁死，
且 disarm 后无法靠重力坠回（休眠体不积分）。表现即 `true_d` 冻结。

**根因 2（FDIR 冻结检测误判，连锁）**：`Fdir::update` 仅用"IMU 加速度三轴连续 20 步完全相等"
判 IMU 冻结。但稳定悬停时比力恒定（含 ~9.81 m/s² 重力分量）属正常 → 误判 `Critical` →
`motors.disarm()`（指令全 0）且 `failsafe_engaged` 单向置位。表现即 `m0` 突降为 0。

**修复**：
- `fly-sim-core/src/physics.rs` `PhySdkWorld::create_empty`：取 `RigidSubsystem` 并设
  `rw.world.params.sleep_time = f64::INFINITY`，为飞控仿真世界关闭休眠。**不能设 `0.0`**
  （初始静止体 `sleep_time` 本为 0，`0>=0` 第一步即休眠，自由落体零推力场景会起步即冻结）；
  **也不能把速度阈值 `sleep_lin_vel2/ang_vel2` 设无穷大**（那会让 `near_rest` 恒真、`sleep_time`
  照常累积、到默认 0.5s 后仍休眠）。正确做法是把休眠时长阈值 `sleep_time` 抬到无穷大。
- `flyctrl-core/src/fdir.rs` `Fdir::update`：冻结判据增加"加速度范数明显偏离合理静态重力
  区间 `[6,14] m/s²`"条件——真实 IMU 卡死常输出恒定为 0 或异常值（断流/失重/过载），
  稳定悬停（|a|≈9.81）不再误判。

**验证**：`cargo build` 通过；批处理收敛到 `true_d=-5.000000`、`m0≈0.50`（无 disarm/冻结），
两次运行确定性一致（无 true_d==0 步）；`--view` 背景仿真线程 PASS（cmd 稳定 ~0.50）；
`cargo test` 4/4 PASS 零回归。

---

## 实施顺序

**0 → 1 → 2 → 4 → 3 → 5 → 6 → 7 → 8 → 9**

0/1/2 是"机架真实 + 气动可信"的地基，最先做；4 独立于风场但真实 EKF 验证离不开；3 与 4 耦合；
5 验收；6 保证可持续迭代；7 把物理引擎依赖倒置，解锁替身单测与未来换引擎；
8 修复阶段 5/7 遗留的悬停冻结/误判潜伏 BUG（此前 PASS 系假象）。

---

## 阶段 9 — 自由落体能量守恒测例 ✅ 已完成

**目的**：在 PLAN §6 可选 #2 基础上，加一个无推力场景，验证物理引擎积分器 + `RigidBodyWorld`
接口的能量守恒正确性——有气动阻力时机械能应**不增**（阻力/落地只耗散，积分器不得注入能量）。

**实现**：
- `fly-sim-core/src/sim.rs` `SimLoop::run_freefall(seconds)`：不跑控制律、传零指令、直接
  `plant_apply(&zero)` + `plant_step()` 推进物理世界。每步算 `mechanical_energy`
  （`0.5·m·v² − m·g·d`，NED d 向下为正），断言：(1) 机械能从不超过初始值 + 0.5J 容差
  （catch 积分器能量注入的真实 bug）；(2) 机体确实下落（NED d 增大）；(3) 状态有限。
- `fly-sim-core/src/controller.rs`：新增 `plant_apply` / `plant_step` 公开方法（无控场景用）。
- `src/main.rs`：`--scenario freefall` 接入；help 文本与错误提示补全。

**修复（依赖库 `phy-rigid`，独立仓库 `d:/project/game/physics`）**：
- `scene.rs:17` 从 `crate::world` 导入 `gravity` 失败（world.rs 未重导出，已改为从 `phy_math` 直引）。
- `scene.rs:46` `SceneDesc.gravity: Vec3<T>` 缺 `#[serde(with="crate::shape::serde_geom")]`，
  导致 derive 走到 nalgebra 自带 serde（nalgebra 版本漂移后 `Vec3: Serialize` 不再满足）→ 编译失败。
  已补 `serde_geom`，与 `RigidWorld.gravity` 一致。

**验证**：`cargo build` 通过（顺带修好 phy-rigid 编译）；`--scenario freefall` PASS
（机体从 d=-5 自由下坠、触地静止于 d≈+2.985，机械能 E0=58.86J 全程不增）；`--scenario hover`
仍 PASS（收敛 d≈-4.96m、cmd≈0.50）；`cargo test` 4/4 PASS。

**后续可选**（未做）：日志分析脚本、CSV 回放渲染器、传感器噪声下 EKF 鲁棒性回归（已知 `--sensor-noise`
发散，待 EKF 调参或噪声门限处理）。

---

## 阶段 10 — 日志分析 + 轨迹回放工具链 ✅ 已完成

**目的**：PLAN §6 可选 #1/#3 落地。SIL 跑完 CSV 后，能自动量化控制律差异、并把轨迹渲染成 3D 图，
用于回归对比与论文/汇报配图，不依赖人工看数。

**新增**：
- `tools/cmp_controllers.py`：读多个 `--log` 产出的 CSV，自动算 `RMS_dz`(高度跟踪误差) /
  `RMS_horiz`(水平偏离) / `RMS_cmd`(平均油门幅度) / `final_dz`(末态高度残差) / `drift_e`(末态水平偏移)，
  打印对齐对比表，附带 NED 坐标解读（d 向下为正：`final_dz<0` = 停在设定点上方）。
  用法：`python tools/cmp_controllers.py logs/pid.csv logs/indi.csv logs/lqr.csv --labels pid indi lqr`
- `tools/replay.py`：读 CSV 把 NED 转回"上为正"Z 轴，matplotlib 画 3D 真值轨迹 + 设定点星标 +
  末态机体姿态箭头（可叠加估计轨迹）。用法：`python tools/replay.py logs/pid.csv --save traj.png`
  依赖 `matplotlib` + `numpy`（已 `pip install`，纯工具侧依赖，不影响 Rust 工程）。

**修复（同批）**：
- `src/log.rs` CSV 写出 bug：原 `writeln!` 直接写 `BufWriter`，但 `main.rs` 末尾 `std::process::exit`
  **跳过 `CsvLogger` 析构** → 末尾若干帧缓冲未 flush，导致日志文件缺失最后 ~16 行（2499 步场景丢 16 行）。
  修复：`CsvLogger::write` 每帧末尾显式 `self.w.flush()`，保证 `process::exit` 下日志完整。
  同时加字段数自检（正常 38 列，异常时补零并 WARN），避免下游解析错位。
- `fly-sim-core/tests/sil.rs` 集成测试修正：`default_quad` 是 `VehicleConfig` 的关联函数，
  改为 `VehicleConfig::default_quad()`（此前误写成 `flyctrl_core::config::default_quad` 导致编译失败）。

**验证**：三个控制律（pid/indi/lqr）悬停日志均 2500 行 × 38 列完整；
`cmp_controllers.py` 给出对比表（三者 RMS_dz≈0.116m，收敛一致）；`replay.py` 正常产出 PNG；
`cargo test` 4/4 PASS。
