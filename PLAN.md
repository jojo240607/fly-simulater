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

## 阶段 4 — 传感器真实化

- `read_sensors` 增加：
  - IMU：零偏（缓变/温度相关）、高斯噪声、随机游走、振动耦合（机体高频）。
  - GPS：固定延迟(100~200ms)、5~10Hz、位置/速度噪声、偶发丢星。
  - 磁罗盘：硬铁/软铁干扰 + 倾角误差（若启用航向）。
- 噪声种子化（可复现）。

## 阶段 5 — 高级控制律对比 + 故障注入

- 接入 `flyctrl-core` 已有 `IndiController/LqrController/MpcController`，同机架同风场对比 PID vs INDI vs LQR。
- 故障注入：单电机 0% 效率、传感器掉线、磁扰爆发。

## 阶段 6 — 闭环一致性与可复现

- 时间步对齐：控制周期(100/250Hz) 与物理子步明确分离。
- 日志/回放：`.csv` 导出 NED/EST/CMD/IMU，支持复现与回归。
- 不变量监控：保留 NaN/有界检查 + 能量守恒校验（无风无推力机械能单调衰减）。

---

## 实施顺序

**0 → 1 → 2 → 4 → 3 → 5 → 6**

0/1/2 是"机架真实 + 气动可信"的地基，最先做；4 独立于风场但真实 EKF 验证离不开；3 与 4 耦合；5 验收；6 保证可持续迭代。
