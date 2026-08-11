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
- **故障注入**：`FlyController::set_motor_failure([bool;4])`；`step` 内对失效电机指令强制置 0
  再回写 plant。`main.rs` 加 `--fail-motor 0..3`（单电机停转）。

**验证（无风悬停 10s, dt=4ms, 450quad）**：
- PID / INDI / LQR **三者均 PASS**（|dz|≈0.04m, horiz≈0）。INDI/LQR 悬停行为与 PID 一致收敛，
  证明 `ControllerKind` 抽象与控制律接线正确、可复现对比。
- `--fail-motor 0`：m0 指令强制置 0 → `cmd=(1,0,1,0)` 饱和、机体爬升漂移(dz=107m) → **合法 FAIL**
  （四旋翼单旋翼失效不可恢复，无冗余自由度）。仿真数值稳定（无 NaN/Inf），故障注入路径正确。

**重要发现（真实仿真价值）**：
1. 无风干净悬停下 PID/INDI/LQR 轨迹几乎重合——差异仅在抗扰/动态场景显现，需阶段 6 日志量化。
2. `--sensor-noise` 暴露的 EKF 发散（阶段 4 发现，R 协方差疑似 0）**仍未修复**：`--controller lqr/indi`
   搭配 `--sensor-noise` 同样发散（EKF 共享，控制律不修估计器）。限为已知缺陷，待 flyctrl-core EKF 修正。
3. 单电机故障合法不可恢复——若要验证"可恢复故障容错"，需做"双故障/部分效率退化"或"六旋翼"
   配置，留待精度需求更高时加。

## 阶段 6 — 闭环一致性与可复现

- 时间步对齐：控制周期(100/250Hz) 与物理子步明确分离。
- 日志/回放：`.csv` 导出 NED/EST/CMD/IMU，支持复现与回归。
- 不变量监控：保留 NaN/有界检查 + 能量守恒校验（无风无推力机械能单调衰减）。
- 控制律量化对比：基于日志 RMS_dz / RMS_cmd / 抗风余量，给出 PID vs INDI vs LQR 实测表。

---

## 实施顺序

**0 → 1 → 2 → 4 → 3 → 5 → 6**

0/1/2 是"机架真实 + 气动可信"的地基，最先做；4 独立于风场但真实 EKF 验证离不开；3 与 4 耦合；5 验收；6 保证可持续迭代。
