# 四旋翼建模仿真 · 商用对标差距与推进路线图

> 目标：让 `fly-sim-core` 的动力学/传感/动力保真度对标商用建模仿真框架
> （Gazebo+ArduPilot/PX4 SITL、Drake、MATLAB/Simulink Quadcopter、RotorPy、JSBSim）。
> 本文档 = 差距清单 + 优先级计划 + 进度跟踪。

## 一、当前已具备（对标商用的基础盘）

| 维度 | 现状 |
|---|---|
| 刚体动力学 | 六自由度刚体（phy-sdk），真实惯量（非 m·L² 近似） |
| 旋翼推进 | 电机一阶滞后 + 推力/反扭矩/臂力矩，X 布局混控 |
| 气动 | 型阻（0.5ρCd\|v\|v）+ 诱导阻力 + 地面效应（近地 +30%） |
| 环境 | 风场（可注入）、静态地面 |
| 传感器 | IMU（偏置/白噪声/随机游走/振动耦合）、GPS（延迟/降频/丢星） |
| 控制/估计 | PID / INDI / LQR 三种 + EKF，**与 MCU 共享同一份代码**（真 HIL 语义） |
| 故障 | 电机效率系数注入（完全/部分退化） |
| 保真 | 确定性可复现、数值不变量检查 |

这套在"控制器 + 传感器数据通路 + 基础动力学"层面已能对标中型 SITL 框架的**控制闭环验证**能力。

## 二、对标商用的主要差距（按重要度排序）

### 1. 螺旋桨/旋翼气动 —— 当前最大简化（差距最大）
- **现状**：油门指令 → 线性推力（`thrust_coeff × 油门`）+ 定比反扭矩。地面效应是经验公式。
- **商用**：动量理论 / 叶素理论（Blade Element + Momentum Theory，如 RotorPy / Gazebo `gazebo_motor_model` / PX4 ESC 模型）：推力与转速平方成正比（`T = k·ω²`），反扭矩与 `ω²` 相关，滑流（downwash）影响前后串列，陀螺进动效应（螺旋桨转动惯量的陀螺力矩）。
- **影响**：大油门/大机动/前飞时推力误差大，无法模拟转速响应、螺旋桨饱和、机动中陀螺耦合。

### 2. 动力系统（电池 + ESC + 电机）—— 完全缺失
- **现状**：`motor_tau` 一阶滞后到推力，无电压/电流概念。
- **商用**：电池内阻/容量/放电曲线 → 电压随电流跌落；ESC 电流/发热/饱和；电机 KV + 负载 → 转速；油门非线性。
- **影响**：无法模拟电池电压跌落导致推力下降、续航、大机动掉压——真实飞行最常见的失效。

### 3. 碰撞/接触/地形 —— 当前极简
- **现状**：仅静态地面盒（y=-5 平面），无真实接触（无摩擦/反弹/碰撞响应）。
- **商用**：完整刚体碰撞（多刚体、凸包/网格、摩擦、接触力）、地形高度图、多机/障碍物。
- **影响**：无法做坠落/着陆/撞击场景，也无法做有地形的循迹/避障。

### 4. 传感器/感知覆盖不足
- **现状**：IMU + GPS。磁力计仅预留接口。
- **商用**：气压计、空速计、磁力计（硬铁/软铁校准）、视觉里程计/VIO、激光雷达测距、光流、RTK-GPS。
- **影响**：无法验证依赖这些传感器的控制律/估计器分支；EKF 融合维度受限。

### 5. 风/大气模型过于简单
- **现状**：一个可注入的风矢量（`WindField`），无结构。
- **商用**：Dryden/Gust 湍流谱（与空速相关）、风梯度（随高度）、阵风/侧风、热气流。
- **影响**：抗风/扰动场景真实性受限。

### 6. 控制分配重构 / 冗余容错（已验证的痛点）
- **现状**：固定 X 布局 mixer，单电机失效即不可恢复。
- **商用**：在线控制分配（加权伪逆/序列二次规划）在冗余/部分失效时重排剩余电机。
- **影响**：`run_hover_degraded` 已验证"无重构分配则不可恢复"——这是与商用的关键差距之一。

### 7. 执行器/机体更多故障与退化模型
- **现状**：电机效率系数（推力缩放）。无舵机卡滞、无传感故障注入到估计器。
- **商用**：传感器硬/软故障（偏置突变/卡死/漂移）、执行器卡死/效率损失、结构损伤（惯量突变）。

### 8. 系统/生态差距（非纯仿真算法）
- 无标准消息协议（MAVLink）、无 HIL（真实板子）、无 RT 调度模拟、无多机/通信链路、
  无任务规划器（mission）、无基准数据集/真值回放、无 Monte Carlo 统计、无更完善的可视化分析。

## 三、优先级计划

| 优先级 | 项目 | 理由 | 状态 |
|---|---|---|---|
| **P0** | 动力系统（油门→电流→电压→转速→推力 + 反扭矩∝ω²） | 最常见失效来源，改动集中、收益大 | ✅ 已完成（推力∝ω² + 反扭矩 + 电池掉压 + 陀螺进动 + 滑流/诱导速度） |
| **P0** | 螺旋桨动量/叶素理论（推力∝ω² + 滑流 + 陀螺效应） | 大机动/前飞真实性的根基 | ✅ 已完成 |
| **P1** | 在线控制分配重构（冗余容错） | 直接回应"退化不可恢复"痛点 | ✅ 已完成（实现+独立验证） |
| **P1** | 碰撞/接触（着陆/撞击） | 解锁坠落与地形场景 | ✅ 已完成（平面+地形高度图惩罚接触；多刚体待续） |
| **P1** | 障碍碰撞（静态球/盒） | 解锁避障/撞击场景 | ✅ 已完成（Obstacle 枚举 + resolve_obstacle_contact 惩罚模型；多刚体待续） |
| **P1** | Dryden 湍流风场 | 抗风场景真实化，成本低 | ✅ 已完成 |
| **P1** | 地面效应（近地推力增强，标准增益曲线） | 近地悬停/着陆真实化，验证简单 | ✅ 已完成（标准 zhang/Phillips 增益 + 单调性） |
| **P2** | 更多传感器（磁力计+气压计已做；空速计已做；VIO/RTK 待续） | 丰富 EKF 融合验证 | 部分完成 |
| **P2** | 任务级逻辑（mission/路径）✅；MAVLink 待续 | 对标"能跑真机流程" | 部分完成 |
| **P2** | 触地翻滚力矩（倾斜撞地产生滚转力矩） | 坠地姿态演化真实化 | ✅ 已完成（翻滚力矩 + 机体-地形接触；凸包/障碍碰撞待续） |
| **P2** | 空间相关风场 + 阵风突风注入 | 机身不同部位风速不同 + 确定性突风 | ✅ 已完成（风切变廓线 + 空间相关场 + 1-cos 确定性突风） |
| **P2** | 热气流（thermal）上升气流模型 | 滑翔/长航时场景真实化 | ✅ 已完成（高斯径向衰减 + 高度封顶 + 确定性水平漂移） |

## 四、推进记录（进度跟踪）

### P0-1：动力系统 ✅ 已完成
- 目标：建立「油门 → 电流 → 电池电压跌落 → 电机转速 → 推力」完整动力链，
  推力与转速平方成正比（`T = k_t·ω²`），反扭矩与转速平方相关。
- 实现：
  - `VehicleConfig`/`DynParams` 增加 `battery_v_nom` / `battery_r` / `motor_kv` / `motor_r`。
  - `plant.rs` 新增动力链：油门→期望推力(线性, 兼容控制律)→∝ω²反解转速→电机转速滞后→
    推力 `T=prop_kt·ω²`、反扭矩 `Q=prop_kq·ω²`（机身 Z 轴）；电流=机械功率 Q·ω/(η·V)+空载；
    电池电压 `V=V_oc-I·R`（低通平滑趋近，破坏正反馈）。
  - 关键设计：油门→推力**稳态线性**（控制器兼容，悬停收敛不变），
    但**掉压使大油门时转速受 `kv·V_bat` 限制** → 推力不足（体现真实掉压）。
- 验证（`tests/powertrain.rs` 2 项）：推力∝ω² 恒定；大油门掉压>5%、小油门<3%、大油门电压更低。
- 回归：workspace 全 15 项测试通过，`sil_hover` 悬停收敛 `|dz|=0.04m`。
- 注：同时修复了 `phy-optics`/`phy-fluid`/`phy-demo` 的 `Shape::Capsule` 非穷尽匹配
  （他人未提交的 shape 改动导致的编译断裂）。

### P0-2：螺旋桨气动（陀螺进动效应 + 滑流/诱导速度）✅ 已完成
- 目标：螺旋桨转动惯量在机动时的陀螺进动耦合力矩（滚转↔俯仰耦合），
  与动量理论诱导速度/滑流（downwash）共同构成更真实的旋翼气动。
- 实现（陀螺进动，前次）：
  - `VehicleConfig`/`DynParams`/airframe 增 `rotor_inertia`（≈1.2e-5 kg·m²）。
  - `plant.rs` 新增 `gyro_torque()`：`M_gyro = Σ H_i×ω_body`，
    `H_i = I_rotor·Ω_i·spin_i·ẑ`（机体 Z 轴角动量）。
- 实现（滑流/诱导速度，本次）：
  - `VehicleConfig`/`DynParams`/airframe 增 `slipstream_drag_coeff`（≈0.06 无量纲，
    旋翼下洗冲击机体附加下拉力比例）。复用既有 `disk_area`/`air_density`。
  - `plant.rs` 每步按动量理论解诱导速度（含垂直气流耦合）：

    ```
    vi = (vb_z + sqrt(vb_z² + 2·T/(ρ·A))) / 2
    ```

    - `vb_z` = 机体竖直速度（上正）。上升（vb_z>0）→ vi 增大（爬升更费劲）；
      下降（vb_z<0）→ vi 减小（直至涡环）。静悬 vi = sqrt(T/(2·ρ·A))。
  - 滑流下洗冲击机体下拉力：`f_slip = slipstream_drag_coeff · 0.5·ρ·A·vi²`，
    沿机体 -Z，叠加到机体合力（上限 0.4·T，防失稳）。
  - 既有 `aero_drag_body` 诱导阻力项（水平前飞下洗耦合）保留，与滑流冲击正交。
  - 新增 `Plant::induced_velocity()` 读数接口（供日志/测试）。
- 验证（`tests/powertrain.rs` 增 2 项）：
  - `induced_velocity_momentum_theory`：静悬 vi≈6 m/s（450quad 动量理论）；
    上升 vi>悬停、下降 0<vi<悬停（垂直耦合符号正确）。
  - `slipstream_force_scales_with_induced_velocity`：稳态悬停 plant 内部 vi
    与理论值偏差 <15%；大油门 vi > 小油门 vi（滑流随推力单调）。
- 回归：workspace 全 22 项测试通过（含新增 2 项）。

### P1-1：在线控制分配重构 ✅ 已完成（实现+独立验证）
- 目标：把控制器期望动作（推力/滚转/俯仰/偏航）最优分配给电机，部分失效时
  重排剩余电机，实现冗余容错（牺牲优先级最低轴）。
- 实现（`fly-sim-core/src/alloc.rs`）：
  - 控制矩阵 M（X 布局，行=推力/滚转/俯仰/偏航，列=电机，与 pid.rs 固定混控一致）。
  - `allocate_eff(des, eff)`：全有效→原固定混控（完全兼容）；
    部分失效→**最小二乘** `u_n=(M_aᵀM_a)⁻¹M_aᵀdes`，失效电机油门=0。
- 验证（独立 `tools/alloc_check.rs`，rustc 直跑，绕过 phy-rigid 阻塞）：
  全效=原混控；单失效解析解 0.5/3.25 精确；双失效解析解 0.2；全失效=0；
  混合需求满足最小二乘法方程（残差与有效列正交，解析最优）。
  - 残差 0.0277 为欠执行器（3电机4需求）物理本质：四旋翼单失效后只能保
    部分轴、偏航等最弱轴妥协。
- 闭环接入 ✅：`FlyController::step` 在存在退化/失效电机时，由固定混控反解期望动作
  （des=M⁻¹·cmd），经 `allocate_eff` 重分配后注入 plant；全有效走原路径（行为不变）。
- 闭环验证（`tests/degraded.rs`，workspace 全 25 项通过）：
  - 单电机完全失效：存活 13→28 步（2 倍）。
  - 部分退化 eff=0.6：存活 24→60+ 步（2.5 倍）。
  - eff=0.50/0.95：固定混控下 24/104 步发散，分配器下稳定维持（满窗 8s）——
    **"退化即失控"→"可维持"**，直接兑现本项容错目标。
- 注：`phy-rigid` 编译断裂期间用独立验证；已修复后 cargo test 全过。

### P1-3：Dryden 湍流风场 ✅ 已完成（实现+独立验证）
- 增强（`fly-sim-core/src/wind.rs`）：
  - 阵风多频叠加（freq/2freq/3freq 递减幅度），频谱更丰富。
  - 各轴 Dryden 尺度差异：纵向时间常数=turb_tau、横向/垂直=0.5·turb_tau
    （纵向谱低频强、相关长；横向谱衰减缓）。
  - **修复湍流增益**：一阶低通稳态输出方差 = α/(2-α)·σw²，原实现时间常数大时
    湍流被过度平滑（σw=1 实际输出仅±0.05m/s）。加增益 `√((2-α)/α)` 补偿，
    使输出标准差≈turb_sigma。
- 验证（独立 `tools/wind_check.rs`，rustc 直跑）：确定性；湍流均值≈0、std≈σ；
  阵风周期均值≈0、振幅存在；纵向比横向平滑（Dryden 尺度）。
- 注：wind.rs 改动因 phy-rigid（他人进行中）编译阻塞，用独立编译验证等价逻辑。

### P1-2：碰撞/接触（着陆/撞击）✅ 完成

- 实现（`fly-sim-core/src/physics.rs`）：
  - `ContactModel { ground_y, restitution, friction, penalty_k, contact_half_h, terrain }`：惩罚弹簧-阻尼
    + 库仑摩擦的地面接触模型。恢复系数 `e` 经 `ζ = -ln(e)/(2π)` 推导临界阻尼比（clamp [0,1]），
    临界阻尼 `c_crit = 2√(k·m)`；法向冲量**仅在接近时**（`v_n < 0`）施加，避免回弹泵能。
  - `TerrainField`（`Flat(h)` / `HeightMap{origin,spacing,nx,nz,heights}`）：地形高度场。接触判定面
    由 `ground_y + contact_half_h` 扩展为 `ground_y + terrain_height_at(x,z) + contact_half_h`，HeightMap
    块内双线性插值、范围外边缘钳制。`resolve_ground_contact` 经 `terrain_normal`（HeightMap 用中心差分
    梯度 `normalize(-∂h/∂x, 1, -∂h/∂z)`）构造局部法向，使斜坡上重力切向分量驱动机体沿坡下滑、接触法向
    冲量方向正确——与既有平面惩罚模型正交，不引入原生刚体。
  - `ContactInfo { touching, penetration, normal_force, friction_impulse }` + `resolve_ground_contact(...)`
    （经 `RigidBodyWorld::get_body_transform(id)` 读取目标刚体位姿，不依赖原生地面刚体）。
- 接入（`fly-sim-core/src/plant.rs`）：`QuadrotorPlant` 持有 `Option<ContactModel>`，每步 `world.step`
  后调用 `resolve_ground_contact` 施加接触冲量（**纯惩罚模型，不向物理世界注入原生地面盒**，避免与
  引擎原生碰撞求解器双重接触）。`set_contact(None)` 即切到真空（自由落体能量守恒场景）。
- 验证：
  - `tests/physics_toy.rs` 4 项（ToyWorld 替身）：静止不下陷、按恢复系数回弹、摩擦止滑、plant 拦停下落四旋翼。
  - `tests/sil.rs` `sil_landing_settles_on_ground`（真实引擎 `PhySdkWorld`）：零推力释放 → 被惩罚接触
    稳定拦停在地面附近（全程状态有限、末态竖直速度趋零），`run_drop` 场景同步提供。
  - 全量测试 `--features phy` 与默认构建均通过。
- **下一步（未做）**：多刚体/凸包碰撞、机体间/障碍物碰撞；地形高度图已完成（见 `TerrainField` + `terrain` 场景）。

### P1-2 续：障碍碰撞（静态球/盒）✅ 完成

- 目标：在 P1-2 地面接触的同款"惩罚模型"哲学下，支持静态障碍物（球/轴对齐盒）碰撞，
  解锁避障/撞击场景，且**不向物理世界注入原生刚体**（避免与引擎原生碰撞求解器冲突）。
- 实现（`fly-sim-core/src/physics.rs`）：
  - `Obstacle` 枚举：`Sphere { center:[f64;3], radius:f64 }` + `Box { min:[f64;3], max:[f64;3] }`
    （引擎世界系 Y-up：x=北，y=上，z=-东）。`closest_point_and_normal(p)` 返回机体中心 `p`
    到障碍的最近接触点与碰撞法向（指向机体、离开障碍的单位向量；盒内取最近面方向以推开）。
  - `resolve_obstacle_contact(world, id, mass, obstacles, body_radius, cm, dt) -> ContactInfo`：
    - `body_radius` 取螺旋桨外周包络（plant 层默认 `1.2·arm_length`），把"中心 vs 障碍"间隙
      转为"表面 vs 表面"接触。
    - 球：间隙 `= |p-center| - (radius+body_radius)`；盒：间隙 `= |p-cp| - body_radius`。
    - 取**最深穿透**障碍解算（单点接触近似，稳定）。法向冲量用与地面接触同款
      `ζ=-ln(e)/(2π)` → `c_crit=2√(k·m)` → `jn=(k·pen - c_n·v_n⁺)·dt`（仅在接近时，`vn<0`）。
    - 切向摩擦（库仑）：预算 `μ·jn`，抵消切向速度（`scale = min(1, budget/(m·|vt|))`）。
    - 复用 `ContactInfo`（已扩展 `normal`/`point`/`impulse` 字段）返回接触信息。
- 接入（`fly-sim-core/src/plant.rs`）：`QuadrotorPlant` 持有 `Vec<Obstacle>`，`new` 增加 `obstacles`
  参数、新增 `set_obstacles(...)`；每步 `world.step` 后、地面接触解算之后调用
  `resolve_obstacle_contact`（仅当障碍非空）。障碍接触与地面接触取"或"（`last_contact` 优先障碍）。
  同步 `SimLoop::new` / `FlyController::new` 透传 `obstacles`（默认 `Vec::new()`，零回归）。
- 验证：
  - `tests/physics_toy.rs` 2 项（ToyWorld 替身）：`obstacle_sphere_bounces_body_away`
    （球顶下落被推离，停在球表面附近）、`obstacle_box_stops_horizontal_penetration`
    （水平冲撞盒面被法向推开、水平速度削减，不穿入盒内）。
  - `tests/powertrain.rs` 1 项：`obstacle_collision_plant_integration`（经 `set_obstacles`
    注入球障碍，1000 步自由下落后状态有限且未深穿透）。
  - 全量 `--features phy` 与默认构建均通过。
- **下一步（未做）**：多障碍同时深穿透的精确多接触解算；凸包/动态障碍；障碍反射到传感器
  （如避障雷达/视觉失效）。障碍碰撞与避障控制器尚未闭环联动。

### P2-2：任务级逻辑（waypoint 路径跟随）✅ 框架完成
- 实现（`fly-sim-core/src/sim.rs`）：`run_mission(waypoints, cruise_v) -> MissionResult`
  - 折线路径按弧长参数化，起飞稳定段（悬停在首 waypoint 2s）+ 逐段位置追踪。
  - 记录最大/RMS 跟踪误差、时长、姿态稳定判据（真值角速度 >6 rad/s 判失控）。
- 验证（`tests/mission.rs` 3 项）：API 结构有效；**可靠检测控制器无法跟随的移动
  路径（长距离巡航 → stable=false，不误报成功）**；误差记录正确。
- **发现控制器局限**：当前 PID 悬停控制器无"倾斜垂直分量补偿"（`des_thrust` 未
  `/= cos(tilt)`），移动时机体倾斜 → 垂直推力下降 → 掉高；且 EKF 动态估计误差在
  持续移动目标下使姿态环振荡。故 `run_hover`/`run_hover_wind`（固定/近固定目标）
  稳定，但长距离移动跟随失控。**任务层据此可靠检测失败**（这正是其价值）。
- 待续：MAVLink 对接；若需真路径跟随，需轨迹跟踪控制器（倾斜补偿 + 速度/加速度前馈）。

### P2-1：更多传感器（磁力计 + 气压计）✅ 部分完成
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `MagSample`/`BaroSample` 结构 + SensorConfig 磁力计（硬铁/软铁/噪声）、气压计（噪声/漂移）参数。
  - `process_attitude(q_ned, altitude)`：磁力计把 NED 地磁场（磁北+磁倾角60°）转到机体系，
    加硬铁/软铁/白噪声；气压计真值高度 + 白噪声 + 慢漂移（随机游走）+ 标准大气气压反演。
  - 不改现有 IMU/GPS 接口（独立方法，渐进接入）。
- 验证（独立 `tools/sensor_check.rs`，rustc 直跑）：磁力计水平姿态测量匹配
  B·软铁+硬铁期望、磁北为正、噪声 std≈0.05；气压计围绕真值（偏差<5m）。
- 待续：视觉里程计/VIO、RTK-GPS。
- **空速计已完成（见 P2-3）**：`AirspeedSensor` 特质 + `AirspeedSample`，EKF 标量空速融合（`airspeed_fusion_constrains_horizontal_speed` 测试通过）。

### P2-3：空速计 + EKF 空速融合 ✅ 完成
- 实现（`flyctrl-core`）：
  - `vehicle.rs`：`VehicleState` 新增 `airspeed: MeterPerSecond` 字段（与 `zero()`）；新增 `AirspeedSample { speed, timestamp_s }`。
  - `estimator/trait_def.rs`：`Estimator::step` 签名扩展为 `step(dt, imu, pos, airspeed: Option<AirspeedSample>)`。
  - `estimator/ekf.rs`：新增 `r_airspeed=0.75` 与 `airspeed_est`；标量测量更新（H=[vx/vh, vy/vh, 0...]，空速=水平速度幅值）`update_airspeed()`；`VehicleState.airspeed` 回填估计值。
  - `estimator/complementary.rs`：低通跟踪空速并回填。
  - `hal/sensor.rs`：`AirspeedSensor` 特质 + `MockAirspeed`（`set_speed`/`set_health`）+ `PitotMs4525do` STM32F407 占位。
  - `hil.rs`：`HilContext::step` 泛型化为 `A: AirspeedSensor`，读 `airspeed.read()` 透传给估计器。
  - `fly-sim-core/src/controller.rs`：新增 `SimAirspeed`（由 `plant.state_ned()` 水平速度推导真空速），接入 `HilContext::step`；`plant.state_ned()` 回填 `airspeed`。
- 验证：`cargo test -p flyctrl-core` 与 `cargo test`（fly-simulater）全绿；新增 `airspeed_fusion_constrains_horizontal_speed` / `airspeed_fusion_rejects_none_gracefully` 单测。
- 待续：真实皮托管噪声模型、空速在控制律（如 TECS）中的消费、VIO/RTK。

---
*更新：P0 完成后在本节打勾并补充实测数据。*

---

### P1-D：地面效应（近地推力增强）✅ 完成
- 目标：近地悬停/着陆时旋翼下洗气流被地面反射，推力增强（标准 zhang/Phillips 形式）。
- 实现（`fly-sim-core/src/physics.rs` `ground_effect_gain(h, d)`）：标准增益 `K_ge = 1/(1 − (R/(R+2z))²)`，水平高度 z 单调，z→0 时→∞（clamp 上限 2.0），z 很大时→1。桨半径 R 由 `prop_diam/2` 给出。
- 验证（`tests/powertrain.rs` 单调性测试）：h 递增 → 增益单调不增；z=0.1(R=0.2) 实测 ≈1.333，与标准公式吻合。
- 接入：plant 在推力计算时乘 `K_ge`（近地推力增强），已并入 P1-2 气动链路。

### P2-A：触地翻滚力矩 ✅ 完成
- 目标：机体倾斜撞地时，接触法向力相对质心产生翻滚力矩，使坠地姿态自然演化（而非"砸地即停"）。
- 实现（`fly-sim-core/src/physics.rs` `resolve_ground_contact`）：接触点相对质心 `r_c` 取机体 -Y（机腹）×`arm_length` 经四元数旋转；翻滚力矩 `τ = r_c × (n·j_n)`；新增 `rotate_vec_by_quat`；`ContactModel` 新增 `tumble_enabled`/`arm_length`（默认关，零回归）。
- 验证（`tests/physics_toy.rs` 2 项）：倾斜 30° 撞地 `|τ|>1e-3`；`tumble_enabled=false` 或水平坠落时 `|τ|=0`；45° 持续接触 `Δθ>0.1 rad` 且有限。
- 待续：凸包/机体间/障碍物多刚体碰撞（当前为纯惩罚点接触，翻滚力矩为单点近似）。

### P2-B：空间相关风场 + 阵风突风注入 ✅ 完成
- 目标：风随空间位置变化（机身不同部位风速不同）+ 确定性一次性阵风突风（可精确复现）。
- 实现（`fly-sim-core/src/wind.rs`）：`WindConfig` 新增 `spatial_scale`/`shear_exponent`/`shear_ref_height`（风切变幂律廓线）+ `gust_burst_amp`/`gust_burst_t0`/`gust_burst_hw`（1-cos 包络）。`WindField::sample` 保留旧接口；新增 `sample_at(dt,pos)` 叠加风切变缩放 + 空间相关正弦场 + 1-cos 突风。plant `step` 读取机体位置传入 `sample_at`。
- 验证（`tests/wind_spatial.rs` 5 项 + `tests/powertrain.rs` 1 项）：默认配置空间均匀/无突风（零回归）；空间相关风使不同位置风速差异>0.1；风切变使高 z 处>低 z 处 1.05×；1-cos 突风窗口内显著、窗口外为 0 且确定性可复现、峰值接近幅值；闭环带风场 plant 跑 3s 数值有限且同配置两次末位置一致。

### P2-C：热气流（thermal）上升气流模型 ✅ 完成
- 目标：热气流（上升暖气流柱）模型，补齐差距清单 #5 的"热气流"项，使滑翔/长航时/能量收集场景可仿真。
- 实现（`fly-sim-core/src/wind.rs`）：`WindConfig` 新增 `thermal_strength`（中心上升速度）/ `thermal_radius`（高斯尺度）/ `thermal_height`（封顶高度）/ `thermal_pos0:[x,z]`（初始中心）/ `thermal_drift:[vx,vz]`（水平漂移）。`WindField` 新增 `thermal_center` 状态（从 `thermal_pos0` 初始化，随 `drift` 确定性积分推进）。`sample_at` 内计算：
  - 径向高斯衰减：`w_up = strength · exp(-r²/(2·radius²))`，`r² = (x-cx)² + (z-cz)²`，3σ 截断（避免无限远微弱上升）。
  - 高度封顶：`z > thermal_height` 时 `height_factor=0`（对流泡顶无上升气流）。
  - 上升分量沿 wind.rs 既有约定写入 `wind[2]`（与风切变同用 pos[2] 为"高度"轴），保持模块内坐标系一致。
  - 全 0 退化旧行为（无热气流时不改变任何输出）。
- 验证（`tests/wind_spatial.rs` 3 项 + `tests/powertrain.rs` 1 项）：中心处上升≈strength、边缘(>3σ)≈0；封顶高度下（z<h）有流、高于封顶为 0；drift 使中心随时间平移、原点流减弱且新中心恢复强流、确定性可复现；闭环带热气流 plant 跑 3s 数值有限且同配置两次末位置一致。

---

*更新：P0/P1/P2 各阶段完成后在本节打勾并补充实测数据。*
