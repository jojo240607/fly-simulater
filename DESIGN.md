# 四旋翼无人机仿真器设计方案（SIL）

> 工程位置：`d:/project/game/fly-simulater`（当前为空，本方案在此落地）
> 物理引擎：`d:/project/game/physics`（自研 `phy-rigid` 刚体动力学）
> 飞控核心：`d:/project/mcu/oop/flyctrl/core`（`flyctrl-core`，含 SIL/HIL 共享闭环）
> 飞控 App（参考，不进本工程）：`d:/project/mcu/oop/joc-app-rust`

## 0. 目标

构建一个运行在 **host（PC）** 上的软件在环（SIL）四旋翼仿真器：

- 用自研物理引擎 `phy-rigid` 充当"真实世界"——含机体刚体、旋翼空气动力、重力、地面碰撞、扰动。
- 用飞控核心 `flyctrl-core` 的 `HilContext::step`（已在 `hil.rs` 实现、经 `flyctrl-core` 单测验证）充当"飞控算法"，与 MCU 上跑的是**同一份控制律**（EKF + PID + FDIR）。
- 二者通过 `ActuatorCmd{motor:[f32;4]}`（飞控 → 各电机归一化推力）与 `ImuSample`/`PosSample`（世界 → 飞控传感器）形成闭环。

收益：在 PC 上以 1kHz 仿真时间步长时间验证飞控不变量（无 NaN、指令有界、悬停稳定、失控保护归零），再烧录到 STM32F4，行为一致（这正是 `hil.rs` 的设计意图）。

## 1. 三个工程的能力盘点（已读源码确认）

### 1.1 物理引擎 `phy-rigid` / `phy-ffi`（作为预编译库集成，**非源码 path 依赖**）
> **集成方式（重要）**：物理引擎以 **C-ABI 预编译库** 形态集成进本工程，不是把 `phy-rigid` 源码当 Rust path 依赖。证据：`d:/project/game/physics/crates/phy-ffi/` 是 `crate-type = ["cdylib","rlib"]` 的 FFI 层，`d:/project/game/physics/pkg/release/` 已产出 `phy_ffi.dll` / `phy_ffi.a` / `phy_ffi.rlib` / `phy_ffi.def` 与自动生成的 `phy_ffi.h`（cbindgen，ABI 版本 `PHY_FFI_ABI_VERSION=1`）。本工程应 **链接该库 + 包含 `phy_ffi.h`**，通过 `extern "C"` FFI 调用，而非重新编译物理引擎源码。

- **FFI 边界已暴露的能力**（读 `phy_ffi.h` 确认）：
  - 创建世界：`phy_world_create_rigid(void)`（刚体世界，含平动+转动）、`phy_world_create_fluid/granular/coupled`。
  - 步进：`phy_world_step(w, f64 dt)` / `phy_world_step_f32(w, f32 dt)`，内部恒 `f64`；`phy_world_step_checked` 带数值有限看门狗（NaN/Inf 返回非 0，可回滚）。
  - 读回：`phy_world_rigid_count(w)`、`phy_world_get_rigid_transforms(w, f64*buf, len)`（每刚体 **7×f64 交错 = pos.xyz + quat.wijk**），另有 f32 通道 `phy_world_get_rigid_transforms_f32`；流体有 `*_positions`/`*_velocities`。
  - 序列化：`phy_world_save` / `phy_world_load`（JSON，用于场景存档与 S8 回放契约）。
  - 释放：`phy_world_destroy(w)`（空指针安全 no-op）。
- **FFI 边界当前缺口（必须补齐才能做四旋翼）**：`phy_ffi.h` 只暴露「整世界级」API——**没有「按 id 施加刚体推力/力矩」「按 id 取速度/角速度」「创建/索引指定 body」的符号**。即 `get_rigid_transforms` 只能回读位姿，无法把旋翼推力作用到机体，也无法读 `vel`/`ang_vel` 生成 IMU 真值。这是集成形态下比 §5「给 `phy-rigid` 提 `apply_body_wrench`」更下层的缺口：**需在 `phy-ffi` 工程新增一组 FFI 符号**（见 §5-FFI）。
- 物理引擎内部（`phy-rigid` crate）已完整支持平动+转动（`Body` 含 `ang_vel`、`inv_inertia_local`），CCD、顺序冲量接触求解、关节、射线均已具备；**没有四旋翼空气动力模型**（只有 `RaycastVehicle` 地面车），推进模型由本工程在 FFI 之上实现（把 4 路推力 → 调用 FFI「施加力/力矩」符号）。

### 1.2 飞控核心 `flyctrl-core`（已具备，直接复用）
- `hil::HilContext::step<I,G,M>`：泛型单步闭环 `传感器→估计→控制→FDIR→执行器`，返回 `VehicleState`。host 用 mock HAL、MCU 用 RTOS 驱动，算法代码完全共享。
- `estimator::EkfEstimator::default_quad()`：四旋翼默认 EKF。
- `controller::{PidController, LqrController, IndiController, MpcController}`：控制律 trait，默认 `PidController::from_config`。
- `fdir::Fdir`：健康监控（IMU 冻结/GPS dropout → `Critical` → 执行器归零）。
- `vehicle::{VehicleState, ImuSample, PosSample, ActuatorCmd, RcInput, Quaternion, Ned}`、`units::*`（类型安全单位）。
- `hal::sensor::{ImuSensor, GpsSensor}` + `hal::actuator::MotorActuator`：trait，host 侧需实现真实（非 mock）版本——由物理引擎喂数据。
- `config::VehicleConfig::default_quad()`：机型参数（质量、臂长、电机布局、PID 增益）。

### 1.3 飞控 App `joc-app-rust`（仅参考接口，不进本工程）
- MCU 上 `control.rs` 周期 4ms：`EkfEstimator`+`Fdir`+`PidController`→ `PWM ioctl` 输出 4 路占空比（1000–2000us）。
- SIL 中我们**直接调用 `HilContext`**，绕开 RTOS device vtable，因此不依赖 App 工程。

## 2. 整体架构

```
┌─────────────────────────── fly-simulater (host binary) ───────────────────────────┐
│                                                                                     │
│   ┌─────────────────┐    ActuatorCmd{motor[4]}     ┌──────────────────────────────┐ │
│   │  FlyController   │ ───────────────────────────▶ │   QuadrotorPlant (被控对象)  │ │
│   │ (flyctrl-core    │                              │  ┌────────────────────────┐  │ │
│   │  HilContext)     │ ◀─────────────────────────── │  │ phy-ffi 预编译库世界   │  │ │
│   │                  │   ImuSample / PosSample      │  │  - 机体 Body(刚体)     │  │ │
│   │  EKF+PID+FDIR    │   (由世界生成)               │  │  - 旋翼推进模型        │  │ │
│   └─────────────────┘                              │  │  - 地面/障碍 Body      │  │ │
│                                                    │  │  - 重力/扰动           │  │ │
│   SimLoop: 固定 dt (1ms / 4ms) 推进                 │  └────────────────────────┘  │ │
│     phy_world_step(dt)  [FFI]                      │                              │ │
│     plant.read_sensors() ──▶ controller.step()      │                              │ │
│     plant.apply_actuators(cmd)                     │                              │ │
│                                                    └──────────────────────────────┘ │
│                                                                                     │
│   Telemetry/Log: 记录 VehicleState 轨迹 → CSV / 实时 plotter                        │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

**坐标约定对齐**（关键坑）：
- 飞控 `VehicleState.pos/vel` 为 **NED**（北-X 东-Y **下-Z**），`att` 为机体→世界四元数，`omega` 机体角速度 (p,q,r)。
- 物理引擎 `phy-rigid` 默认重力沿 **-Y**（见 `world.rs::new` 的 `gravity::<T>()`），且 `pos/vel/ang_vel` 为世界系、**Y 向上**。
- **必须做坐标桥接**：仿真器内维护物理引擎的"真实世界"（Y-up），在生成 `ImuSample`/`PosSample` 与写入 `VehicleState` 时做 NED↔Y-up 转换（pos.y↔-pos.z、vel 同理；姿态四元数做轴交换）。建议在 `plant` 层集中处理，控制器层完全不知情。

## 3. 模块设计（新增代码都在 `fly-simulater`）

### 3.1 `Cargo.toml` / 库链接方式（**集成预编译物理引擎库，非源码依赖**）
- 新建二进制 crate（host, std），`edition 2021`。
- **物理引擎以预编译 C-ABI 库集成**：
  - 复制 `d:/project/game/physics/pkg/release/` 下的 `phy_ffi.h` + `phy_ffi.dll`（Windows 动态链）/ `phy_ffi.a`+`phy_ffi.rlib`（静态链）到本工程 `vendor/phy-ffi/`（或 `libs/`）。
  - 用 `build.rs` + `#[link(name="phy_ffi")]` 链接；或 `bindgen` 由 `phy_ffi.h` 生成 `src/phy_ffi_bind.rs`（推荐，避免手写出错）。
  - **不**把 `d:/project/game/physics` 当 cargo workspace 成员 / path 依赖（保持物理引擎作为独立发布库，ABI 版本契约 `PHY_FFI_ABI_VERSION` 约束两侧）。
  - 加载后先 `phy_ffi_abi_version()` 断言 `>=` 期望版本，fail-fast 拒绝不匹配的库。
- **飞控核心仍可用源码 path 依赖**（它是 Rust 库，非 FFI 库）：
  - `flyctrl-core = { path = "../../mcu/oop/flyctrl/core" }`（注意：它是 `#![no_std]`，host std 环境可依赖，编译通过）。
- 其他依赖：`nalgebra`（坐标桥接/矩阵）、`serialport`（HIL USB）、`csv`/`plotters`（可选可视化）。
- `Cargo.toml` 示例（节选）：
  ```toml
  [dependencies]
  flyctrl-core = { path = "../../mcu/oop/flyctrl/core" }
  nalgebra = "0.32"
  serialport = "4"
  csv = "1"

  [build-dependencies]
  bindgen = "0.69"   # 由 phy_ffi.h 生成绑定（或预生成提交到 src/）
  ```
  `build.rs` 关键动作：`bindgen` 读 `vendor/phy-ffi/phy_ffi.h` → 写 `src/phy_ffi_bind.rs`；`cargo:rustc-link-search=vendor/phy-ffi`；`cargo:rustc-link-lib=phy_ffi`（动态）或 `static=phy_ffi`（静态）。

### 3.2 `src/plant.rs` —— 被控对象（核心新增，走 FFI 调用物理引擎）
把物理引擎接入飞控闭环（**全程经 `phy_ffi` 预编译库，不触碰源码**）：
- `QuadrotorPlant` 持有一个 `PhyWorldHandle *`（经 §3.1 的 bindgen 绑定调用 `phy_world_create_rigid()` 创建），其中 **index 0 = 机体刚体**。机体质量 m / 转动惯量来自 `VehicleConfig`，经新增的 FFI 符号创建刚体并设定（`phy_world_rigid_add_body` + `phy_world_rigid_set_inertia`，见 §5-FFI）。
- **旋翼推进模型**（本工程最关键的物理增量，在 FFI 之上实现）：
  - X 型四旋翼布局（与 `ActuatorCmd` 注释一致）：motor[0]=前右(CCW) 1=后左(CCW) 2=前左(CW) 3=后右(CW)。
  - 每路推力 `T_i = k_T * motor[i]`（归一化推力→牛顿，k_T 使 hover 时 `ΣT_i ≈ m*g`）。
  - 机体合力 `F_body = Σ T_i * 机体-Z 轴`；机体合力矩 `τ_body = Σ (r_i × T_i * 机体-Z) + 反扭矩`（`r_i` 电机臂向量，反扭矩 `Q_i = k_Q*motor[i]*spin_sign_i`）。
  - 每帧 `phy_world_step` **前**，把世界系 `F_world`/`τ_world` 经新增 FFI 符号施加到机体：调用 `phy_world_rigid_apply_force(0, F_world)` + `phy_world_rigid_apply_torque(0, τ_world)`（见 §5-FFI）。力在引擎积分阶段消费（与 `phy-rigid` 内部 gravity 积分同路径）。**注意**：若 `phy-ffi` 暂未提供持续力累积（仅一次性脉冲），推进模型需每帧重施加（清+设），具体语义以 §5-FFI 落地的符号为准。
- `apply_actuators(&ActuatorCmd)`：保存本拍 4 路推力，供 `step` 前注入；建议用 `phy_world_step_checked` 带 NaN 看门狗，返回非 0 时回滚到上一已知良好状态并报警（对齐 S8 回放契约）。
- `read_sensors() -> (ImuSample, Option<PosSample>)`：
  - **位姿真值**：调 `phy_world_get_rigid_transforms(buf, len)`，取 index 0 的 7×f64（pos.xyz + quat.wijk）。**速度/角速度真值**：调新增 FFI `phy_world_rigid_get_velocity(0,..)` / `phy_world_rigid_get_angular_velocity(0,..)`（见 §5-FFI）——当前 `phy_ffi.h` 没有该符号，必须补齐。
  - **IMU（真值 + 噪声）**：由机体速度/角速度 + 姿态数值微分得比力（机体加速度 = 世界加速度经 `quat_conj` 旋到机体，再减重力分量 → 比力语义，对齐 `virtual imu.rs`）；`gyro` = 机体角速度。注入高斯噪声/偏置（可配置，验证 EKF 鲁棒性）。
  - **GPS/Baro**：由机体 `pos` 转 NED 生成 `PosSample`；可模拟 dropout（喂 `None`）触发 FDIR。
- **NED↔Y-up 桥接**：在 `read_sensors`（读回 Y-up → 转 NED 给飞控）与施加力/矩（飞控 NED 推力意图 → 转 Y-up 世界系）处集中转换，见 §4。该转换只发生在本工程 `plant.rs` 边界，飞控层无感知。

### 3.3 `src/controller.rs` —— 飞控包装
薄封装 `flyctrl_core::hil::HilContext`：
- `FlyController::new(cfg)`：`HilContext::new(EkfEstimator::default_quad(), PidController::from_config(&cfg.ctrl_params()), dt)`。
- 实现**真实**（非 mock）`ImuSensor`/`GpsSensor`：把 `plant.read_sensors()` 的结果喂给 `HilContext::step`。
- 实现 `MotorActuator`：把 `ActuatorCmd` 回写 `plant.apply_actuators`。
- `step(&mut self, setpoint)`：调用 `HilContext::step(&mut imu, &mut gps, &setpoint, &mut motors, &cfg)` 返回 `VehicleState`。

### 3.4 `src/sim.rs` —— 仿真主循环
- `SimLoop`：固定 `dt`（默认 4ms，对齐 MCU 控制周期；或 1ms 世界步 + 4ms 控制步可选）。
- 场景：
  1. **悬停**：`Setpoint::hover(pos0, yaw=0)`，验证高度/姿态收敛、指令有界。
  2. **阶跃/轨迹**：给定 NED 轨迹，验证跟踪。
  3. **扰动**：施加风（世界系常力/周期力）或 IMU 冻结（触发 FDIR→归零），验证不变量。
  4. **碰撞**：把机体 `pos` 初始化贴近地面/障碍，验证碰撞后 FDIR 与失控保护。
- 每步记录 `VehicleState` + `ActuatorCmd` + `Health` 到环形缓冲 / 直接写 CSV。

### 3.5 `src/main.rs` —— 入口与 CLI
- 参数：`--scenario hover|step|disturb|collide`、`--dt`、`--secs`、`--out trace.csv`、`--plot`。
- 跑完打印不变量检查（`state_finite`、`actuator_bounded`，复用 `flyctrl_core::invariants`），给出 PASS/FAIL。

### 3.6 可选 `src/viz.rs` —— 实时可视化
- 用 `plotters` 画 3D 轨迹，或输出 CSV 供外部（Python/Matplotlib）绘制。
- 不阻塞核心仿真（独立线程消费轨迹缓冲）。

## 4. 坐标与单位对齐清单（落地前必查）

| 项 | 飞控 (`flyctrl-core`) | 物理 (`phy-rigid`) | 桥接动作 |
|---|---|---|---|
| 位置 | NED，D 向下 | Y 向上 | `z_ned = -y_up`；`x_ned=x_up`,`y_ned=-z_up` |
| 速度 | 同上 | 同上 | 同位置（经 FFI `get_velocity` 读回世界系再转） |
| 姿态 | 机体→世界四元数 | 机体→世界四元数 | 轴交换：`q_up = swap_yz(q_ned)`（FFI `get_rigid_transforms` 回读 Y-up 四元数） |
| 角速度 | 机体 (p,q,r) | 世界 `ang_vel` | 经姿态旋到机体；轴交换（FFI `get_angular_velocity` 读回） |
| 重力 | 飞控内部处理（IMU 比力已减重力） | 引擎 `-Y` 重力 | 由 `read_sensors` 的比力计算统一 |
| 推力 | `ActuatorCmd.motor∈[0,1]` | 牛顿力 | `T = k_T * motor`，沿机体上轴；经 FFI `apply_force/torque` 注入 |

> 飞控用的 `vehic‮le.rs::Quaternion` 是自定义结构（`w,x,y,z`），`phy-math` 用 `nalgebra::UnitQuaternion`。桥接层做显式字段拷贝，避免静默错轴。

## 5. 物理引擎 FFI 缺口与补齐（**必须在 `phy-ffi` 工程做，不在本工程**）

> 因为物理引擎以**预编译 C-ABI 库**集成（§1.1/§3.1），本工程**不能**直接改 `phy-rigid` 源码或加 `apply_body_wrench`。四旋翼需要的「单刚体施力/读速」能力，必须回到 `d:/project/game/physics/crates/phy-ffi/` 工程，新增 FFI 符号并重新构建出库（`PHY_FFI_GEN_HEADER=1 cargo build -p phy-ffi` 重新生成 `phy_ffi.h`），再更新本工程的 `vendor/phy-ffi/`。

### 5.1 需在 `phy-ffi` 新增的符号（FFI 层，已落地，ABI v2）
```c
// 创建刚体：shape 类型(0=Sphere/1=Box) + 质量 + 初始位姿(pos7=pos.xyz+quat.wijk) + 主转动惯量(inertia3=Ixx,Iyy,Izz)；
// 返回 body id（>=0），失败 -1。
int64_t phy_world_rigid_add_body(PhyWorldHandle *w, int32_t shape_kind,
                                 double mass, const double *pos7, const double *inertia3);

// 施加世界系力(牛顿)/力矩(N·m)到指定刚体，按半隐式欧拉直接积分进 vel/ang_vel(×inv_mass×dt / ×I_world⁻¹×dt)。
// dt 与同帧 step 一致；mode 当前按累加(0)处理(引擎 step 不消费外力字段，故在 FFI 层积分)。
// 返回 0 成功，-1 失败(空指针/id 越界/panic)。
int32_t phy_world_rigid_apply_force(PhyWorldHandle *w, int64_t id, const double *f3, double dt, int32_t mode);
int32_t phy_world_rigid_apply_torque(PhyWorldHandle *w, int64_t id, const double *t3, double dt, int32_t mode);

// 读回指定刚体的线速度 / 角速度(世界系,各 3×f64)，返回 0 成功，-1 失败。
int32_t phy_world_rigid_get_velocity(PhyWorldHandle *w, int64_t id, double *out3);
int32_t phy_world_rigid_get_angular_velocity(PhyWorldHandle *w, int64_t id, double *out3);
```
- **已落地**：`d:/project/game/physics/crates/phy-ffi/src/lib.rs` 已加上述 5 个符号，`PHY_FFI_ABI_VERSION` 自增到 **2**，并 `build_package.py` 重新发布 `pkg/release/`（`.dll`/`.lib`/`.rlib`/`.h` 已含新符号）。
- 积分语义说明：引擎 `RigidWorld::step` 仅对 `vel` 加重力、**不动 `ang_vel`**、也不消费外力字段，故 `apply_force/torque` 在 `step` 前由本工程每帧调用一次完成积分；`PhyWorldHandle` 不存"上次施加量"，`mode` 保留但当前统一按累加（调用方每帧只调一次即可）。
- Rust 侧实现：在 `phy-ffi` 里从 `PhyWorldHandle` 取回 `&mut World<f64>`，调用 `phy-rigid` 既有的 `Body` 字段（`vel`/`ang_vel`/`inv_inertia_local`）与新增的 `apply_body_wrench`（若 `phy-rigid` 也缺则一并补，但这属于物理引擎内部演进，由 `physics` 工程维护）。

### 5.2 给 `phy-ffi` 提 PR 的内存/安全约定
- 所有 `const double *`/`double *` 缓冲由调用方(`fly-simulater`)分配，`len` 由调用方保证；`phy-ffi` 内部做空指针 + 长度校验，越界返回 -1（fail-fast，对齐现有 `phy_world_step_checked` 风格）。
- `id` 越界返回 -1；句柄空指针所有符号 no-op 返回 -1（与 `phy_world_destroy` 一致）。
- 因引擎内部恒 `f64`，FFI 边界不引入 f32 通道（本工程用 f64 精度，回放确定性更重要，见 S8）。

### 5.3 落地前的临时 workaround（若暂不发布新库）
若 `phy-ffi` 新符号暂未发布，可先用 **方案 B 临时绕行**：把 `fly-simulater` 临时切到 `phy-rigid` 源码 path 依赖跑通 SIL（§3.2 旧写法），待 `phy-ffi` 补齐并发布后再切回 FFI 集成。这是**开发期临时态**，最终交付必须是 FFI 集成形态。

## 6. 验证计划（对齐 `flyctrl-core` 既有单测精神）

1. **悬停收敛**：初始 `pos=(0,0,-5)` NED，跑 10s，`|pos - setpoint| < 0.5m`、姿态接近单位四元数、4 路推力 `∈[0.3,0.7]`（hover 附近）。
2. **不变量全程成立**：复用 `invariants::state_finite` / `actuator_bounded`，每步断言，导出 CSV 供离线核查。
3. **失控保护**：冻结 IMU 30 拍 → FDIR `Critical` → `ActuatorCmd=[0,0,0,0]`（与 `hil.rs::hil_loop_failsafe_zeroes_motors` 同结论）。
4. **GPS dropout**：周期性喂 `gps=None` → EKF 仅靠 IMU，位置估计漂移但有界、不 NaN。
5. **碰撞存活**：把机体贴地初始化 → `phy_world_step`（FFI）接触求解托住，不穿透、不发散。
6. **与 MCU 一致性（HIL 闭环）**：相同 `VehicleConfig` + 相同 `HilContext` 算法，SIL 跑出的控制指令序列，与 MCU 上 `control.rs` 在同等输入下应一致（因 `hil.rs` 已保证代码共享）。可用 `joc-app-rust` 的 `dataset_data.rs` 回放作为 SIL 的输入源做对拍。

## 7. 文件结构（落地后）

```
fly-simulater/
├── Cargo.toml
├── DESIGN.md                # 本文件
├── src/
│   ├── main.rs              # CLI 入口 + 不变量判定
│   ├── plant.rs             # QuadrotorPlant：物理引擎 + 旋翼推进 + 传感器 + 坐标桥接
│   ├── controller.rs        # FlyController：包装 HilContext + 真实 HAL 实现
│   ├── sim.rs               # SimLoop + 场景（hover/step/disturb/collide）
│   └── viz.rs               # 可选：轨迹导出 / 实时绘图
└── traces/                  # 仿真输出 CSV（gitignore）
```

## 8. 实施步骤（建议顺序）

1. 建 `Cargo.toml`，确认 `flyctrl-core` + `phy-rigid` 能在 std host 下编译通过（先写个最小 `main` 打印两边版本）。
2. 实现 `plant.rs`：经 `phy_ffi` 建世界 + `phy_world_rigid_add_body` 建机体 + 旋翼力/矩经 FFI `apply_force/torque` 注入 + NED↔Y-up 桥接 + `read_sensors`。先用 §5.3 临时 path 依赖跑通，再切回 FFI。
3. 实现 `controller.rs`：真实 `ImuSensor`/`GpsSensor`/`MotorActuator` 接 `HilContext`。
4. 实现 `sim.rs` 悬停场景，跑通并调 `k_T` 使 hover 推力≈0.5。
5. 加扰动/碰撞/GPS-dropout 场景 + `main.rs` 不变量判定。
6. （可选）方案 B 临时绕行：若 `phy-ffi` 新符号暂未发布，临时切 `phy-rigid` 源码依赖跑通 SIL；最终回到 §5 的 FFI 集成形态（依赖 `phy-ffi` 预编译库）。
7. （可选）`viz.rs` 轨迹可视化。
8. **HIL 接入**（接真实飞控，见 §9）：加 `hil_link.rs` + `SimLoop::mode` 双模；给 `joc-app-rust` 加 `--features hil` 分支（传感器来源切 USB、回传 `ActuatorCmd`、设定点来自 USB、ARM 由指令置位）；先用 §9.3 方案 B 紧凑帧跑通，再补 §9.3 方案 A 的 `HIL_SENSOR` 编码接 QGC。先在 SIL 跑通 §6 全部场景，再在 HIL 下复跑 §9.8 验证回路对齐。

## 9. USB-HIL 接入（接真实飞控做硬件在环）

> 结论先行：**可以，而且下行通道已基本现成**。把 `fly-simulater` 从「SIL（PC 算控制器）」切换为「HIL（MCU 算控制器、PC 算世界、USB CDC 传真值/指令）」即可。本工程跑在 host 的 `phy-rigid` 不动，只是把 §3.3 的 `controller.rs`（`HilContext`）替换成 USB 读写。

### 9.1 为什么几乎现成（已读源码确认）

- **MCU→PC 下行已打通**：`joc-app-rust/src/flyctrl/telemetry.rs` 明确「USB CDC(`usb0`) 是遥测主通道，电脑端免 USB-TTL 直接收 MAVLink」，每 20ms 经 `usb0` 发 `HEARTBEAT` / `LOCAL_POSITION_NED`(msg 32) / `SYS_STATUS`。PC 侧只要用 `serialport` crate 打开 `usb0` 对应的 COM 口（Windows 为 CDC-ACM 虚拟串口），按 `flyctrl-core::comm::mavlink` 的 v2 帧格式解析即可拿到 NED 位置/速度真值。
- **坐标系天然对齐**：下行 `LOCAL_POSITION_NED` 已经是 NED，与飞控内部 `VehicleState.pos/vel`(NED) 一致，PC 侧收到后做 NED↔Y-up 桥接（§4）即可喂给 `plant`，**无需额外轴交换**。
- **HAL 抽象友好**：`joc-app-rust` 控制回路经 `flyctrl-core::hal` trait 取传感器；HIL 模式只需「传感器来源」从真实 IMU 换成 USB 灌入的仿真真值，控制律（`EkfEstimator`+`PidController`+`Fdir`）**二进制级不变**——这正是 `hil.rs` 注释里 SIL/HIL 共享闭环的设计意图。

### 9.2 HIL 闭环架构（开环 HIL，推荐先做）

```
        USB CDC-ACM (虚拟串口, COMx)
  ┌──────────────────┐  ◀── HIL_SENSOR/SET_POSITION (PC→MCU) ──┐
  │  真实飞控 MCU      │  ─── LOCAL_POSITION_NED/ATTITUDE ──▶   │
  │ (joc-app-rust)    │                                         │
  │  control.rs 4ms   │                                         │  PC 端 fly-simulater
  │  Ekf+Pid+Fdir      │                                         │  (host)
  └──────────────────┘                                         │  ┌──────────────────┐
         │ ActuatorCmd(4路推力) 经 USB 回传                      │  │ phy-rigid 世界    │
         │ (或 MAVLink 自定义 / PWM 回采)                         │  │ QuadrotorPlant   │
         └──────────────────────────────────────────────────────┘  └──────────────────┘
```

- **MCU 端**：照常每 4ms 跑 `control.rs` → 输出 4 路 PWM 占空比。`fly-simulater` 需要这 4 路推力注入 `plant`。两种回采方式：
  1. **MCU 上行回传 `ActuatorCmd`**（推荐）：在 `telemetry.rs` 的下行帧里加一个 HIL 专用消息，把 `control.rs` 算出的 `ActuatorCmd.motor[4]` 随帧发出（或单独消息）。PC 直接读。
  2. **PC 侧 PWM 回采**：若 MCU 用真实 ESC/PWM 板，PC 经 USB 读 PWM 占空比——需额外硬件，不推荐纯 HIL。
- **PC 端**：`SimLoop` 在 `mode=HIL` 下，每帧：(a) `phy_world_step`(FFI) 推进世界；(b) 由 `plant.read_sensors()` 得到仿真 `ImuSample`/`PosSample` 真值；(c) 经 USB 上行发给 MCU（取代 SIL 里的 `controller.step`）；(d) 等 MCU 回传 `ActuatorCmd`；(e) `plant.apply_actuators(cmd)` 注入下拍。

### 9.3 上行协议选型（二选一，推荐 A）

**A. 标准 MAVLink `HIL_SENSOR`(msg 107) + `SET_POSITION_TARGET_LOCAL_NED`(msg 85)**（协议化、可被 QGC 复用）
- 优点：与现有 `mavlink.rs` 风格一致，PC 端用标准 MAVLink 解析库即可。
- 改动：给 `flyctrl-core::comm::mavlink.rs` 增加 `encode_hil_sensor`（字段：time_usec, xacc/yacc/zacc(比力 mG), xgyro/ygyro/zgyro(rad/s), abs_pressure, ...）、`encode_set_position_target_local_ned`（字段：x/y/z NED, vx/vy/vz, afx/afy/afz, yaw, type_mask）；并加对应 `CRC_EXTRA[107]=90`、`[85]=140`（标准 common.xml 值）。
- MCU 侧：在链路层（`comm/link` 或 `joc-app-rust` 的接收分支）解析 107/85，写入 `SENSOR_FRAME`（替代真实 IMU 采样），并把 85 的设定点写入 `Setpoint`。

**B. 自定义紧凑 HIL 二进制帧**（低延迟、最省字节）
- 帧格式（建议 ≤32 字节，定长，带 magic+seq+crc）：
  ```
  [0x48 'H'][0x49 'I'][0x4C 'L'][seq:u8]
  [imu: 6×f32 LE]  // ax,ay,az (比力, m/s², 机体) ; gx,gy,gz (角速度 rad/s, 机体)
  [pos: 3×f32 LE]  // NED, m
  [vel: 3×f32 LE]  // NED, m/s
  [quat:4×f32 LE]  // 机体->世界(飞控约定 w,x,y,z)
  [setpoint: 4×f32/或 type_mask+vec]  // 见 9.4
  [crc16:u16 LE]
  ```
- 优点：单帧同时含「仿真传感器真值 + 设定点」，比 MAVLink 分两条更省、延迟更低，适合 HIL 紧环。
- 缺点：需 MCU 侧写一个小型二进制解析分支（受 `hil` feature 门控）。

> 推荐：先用 **B** 把 HIL 跑通（最快、可控），稳定后再补 **A** 的 `HIL_SENSOR` 以便接 QGC 做可视化对比。

### 9.4 设定点（Setpoint）下发

- HIL 时真实接收机（`RcInput`）不接，设定点来自 PC。两种方式：
  - **PC 直接指定**：`SimLoop` 场景（hover/step/disturb）本就生成 `Setpoint`，随上行帧一起发给 MCU。
  - **复用 MAVLink `SET_POSITION_TARGET_LOCAL_NED`**：若用方案 A，PC 用标准消息下发，MCU `control.rs` 的 `Setpoint` 来源从 `RcInput` 切到该消息（feature 门控）。
- **ARM/DISARM**：用 `COMMAND_LONG` + `MAV_CMD_COMPONENT_ARM_DISARM`(400)（已在 `enums` 定义），PC 发令、MCU 置 `armed`——HIL 必须 ARM 后 `control.rs` 才输出非零推力（与正常飞行一致，防误触）。

### 9.5 MCU 侧最小改动（feature 门控，不影响正常飞行固件）

在 `joc-app-rust` 增加 `--features hil`：
1. `Cargo.toml`：`hil = []` feature；依赖 `flyctrl-core` 的 `hil` 对应能力（如需 `HIL_SENSOR` encode 则 `flyctrl-core` 也加 `hil` feature 暴露编码函数）。
2. `sensors_task.rs`：当 `cfg(feature="hil")` 时，IMU/GPS/Baro 的真值来源从真实驱动改为「USB 上行帧解析写入的 `SENSOR_FRAME`」；真实传感器采样分支被 `#[cfg(not(feature="hil"))]` 关掉。控制律完全不变。
3. `telemetry.rs`：HIL 时除现有下行帧外，额外回传 `ActuatorCmd.motor[4]`（方案 A 的回采方式 1），让 PC 拿到推力注入 `plant`。
4. `control.rs`：当 `cfg(feature="hil")` 时，`Setpoint` 来源从 `RcInput` 改为 USB 设定点；`armed` 由 `COMMAND_LONG` ARM 指令置位。
5. **安全闸**：HIL feature 下强制 `Fdir` 的失控保护仍生效；且 HIL 固件在 `HEARTBEAT` 置 `MAV_MODE_FLAG_HIL_ENABLED`(0x20)，地面站/QGC 能识别「当前为仿真模式」，避免把仿真当真飞。

### 9.6 实时性与步长（USB 非实时，必须处理）

- USB CDC-ACM 是**虚拟串口，无确定性时延**，做不了硬实时紧环（4ms 往返不可靠）。
- **步长策略**：
  - **开环 HIL（推荐）**：PC 世界步长放宽为 **20ms**（对齐 `telemetry` 现有周期），MCU 仍按自身 4ms 控制；PC 每 20ms 收一帧状态、发一帧真值/设定点，MCU 在中间拍用上一帧真值。闭环延迟 ≤ 一个 PC 步长（20ms），对悬停/轨迹足够。
  - **紧耦合 HIL（进阶）**：PC 侧对 MCU 回传做**时间戳 + 插值/预测**补偿 USB 抖动；仅当需验证高带宽控制律（如 IND/INDI）才上。
- **不变量仍适用**：`state_finite` / `actuator_bounded` 在 MCU 端由 `invariants` 校验，PC 端 `SimLoop` 同样校验世界状态，两侧任一侧 NaN/越界即停仿真并报警。

### 9.7 PC 端新增模块 `src/hil_link.rs`

```rust
// USB CDC-ACM 双向链路（host, std）
use serialport::{SerialPort, SerialPortSettings, DataBits, StopBits, FlowControl};
use flyctrl_core::comm::mavlink; // 若走方案 A
use flyctrl_core::vehicle::{ImuSample, PosSample, ActuatorCmd, Setpoint};

pub struct HilLink {
    port: Box<dyn SerialPort>,
    buf: Vec<u8>,
}
impl HilLink {
    pub fn open(com: &str, baud: u32) -> std::io::Result<Self> { /* 开 usb0 对应 COM */ }
    /// PC→MCU：把仿真真值(来自 plant.read_sensors) + setpoint 编码下发。
    pub fn send_truth(&mut self, imu: &ImuSample, pos: &PosSample, sp: &Setpoint) { /* 方案 B 紧凑帧 */ }
    /// MCU→PC：解析下行帧，返回最新 NED 状态 + 4路推力（供 plant 注入）。
    pub fn recv(&mut self) -> Option<(VehicleState, ActuatorCmd)> { /* 解析 LOCAL_POSITION_NED + HIL 回传 */ }
}
```

`SimLoop` 改造（§3.4）加 `mode: SimMode { Sil, Hil(hil_link) }`：
- `Sil`：调 `controller.step`（原 SIL）。
- `Hil(link)`：每帧 `phy_world_step`(FFI) → `plant.read_sensors` → `link.send_truth` → `link.recv` → `plant.apply_actuators`。

### 9.8 HIL 验证计划（对齐 §6）

1. **回路对齐**：SIL 悬停 10s 的轨迹 vs HIL（`--features hil` 固件 + USB）悬停 10s 轨迹，位置偏差 < 0.5m（证明 MCU 算法与 SIL 同一份代码、闭环一致）。
2. **USB 下行解析**：PC 收到 `LOCAL_POSITION_NED`，NED↔Y-up 还原后与 `plant` 世界真值偏差 < 0.05m（验证 §4 桥接 + 帧解析无错位）。
3. **ARM 安全**：未发 ARM 指令前，MCU 回传 `ActuatorCmd=[0,0,0,0]`；发 ARM 后才有推力（防误触）。
4. **失控保护跨环**：PC 冻结上行 IMU 真值 30 拍 → MCU `Fdir` 判 `Critical` → 回传推力归零 → PC `plant` 机体坠落但不发散（与 §6.3 同结论，且走真实芯片）。
5. **GPS dropout 跨环**：PC 周期性不发 `pos` 真值 → MCU EKF 仅用 IMU，位置估计漂移有界、不 NaN。

## 10. 风险与注意

- **角速度注入**：依赖 FFI `phy_world_rigid_apply_torque` 在 `phy_world_step` 前生效、且引擎积分阶段不丢弃 `ang_vel`；需在 `phy-ffi` 发布新符号后跑 §6.5 碰撞测试实证。若引擎只在积分阶段加重力不改 `ang_vel`，则安全。
- **坐标桥接 bug 是头号风险**：四元数轴交换写错会表现为"姿态镜像/偏航反向"，建议在 `plant.rs` 写专门的单元测试（已知姿态→已知 NED 输出）。
- **`flyctrl-core` 是 `#![no_std]`**：host 用 std 依赖它没问题，但它若内部用到 `cortex-m` 等硬件特性会编译失败——已确认其 `hal` 用 trait 抽象、host 侧实现真实版本而非 mock 硬件版，应无硬件依赖泄漏。若编译报缺 `core::arch` 等，需在 `flyctrl-core` 加 `cfg(not(thumbv7em))` 门控（超出本工程范围，先验证）。
- **`phy-ffi` 物理引擎以预编译库集成**：host 有堆，无妨；由 `physics` 工程独立发布，`fly-simulater` 只经 FFI 调用，不重编译物理源码。MCU 侧无堆约束与 SIL/HIL 无关（HIL 时世界仍在 PC）。
