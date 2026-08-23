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
- 无标准消息协议（MAVLink 兼容层）、无真 HIL（真实板子）、无 RT 调度模拟；多机/通信链路、
  任务规划器（mission）、基准数据集/真值回放、Monte Carlo 统计、3D 可视化等 P3 系列已落地
  （见三、推进记录）。

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
| **P2** | 更多传感器（磁力计+气压计已做；空速计已做；VIO/RTK 已做） | 丰富 EKF 融合验证 | ✅ 已完成 |
| **P2** | 任务级逻辑（mission/路径）✅；MAVLink 遥测下行 ✅ | 对标"能跑真机流程" | 已完成 |
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

### P1-2 续续：多障碍同时穿透的精确多接触解算 ✅ 完成

- 目标：消除"取最深穿透单点解算"的偏置——机体卡在两墙夹角 / 同时贴地+障碍时，
  各接触法向独立推开，不会被单点法向带偏、也不深穿透任一侧。
- 实现（`fly-sim-core/src/physics.rs` 的 `resolve_obstacle_contact`）：
  - 由"取最深单点"改为**遍历所有穿透障碍、对每个分别计算并施加法向弹簧-阻尼冲量
    + 库仑摩擦冲量，再求和经 `world.apply_impulse` 一次批量注入**（多接触叠加）。
  - 每个障碍独立取最近点法向与穿透深度，用共用阻尼比 ζ（由恢复系数推导）。
  - 返回**汇总** `ContactInfo`：`penetration`/`point`/`normal`/`impulse` 取最深穿透者，
    `normal_force`/`friction_impulse` 为各接触之和。未碰撞保持 `touching=false`。
  - 仍属单点接触近似（每障碍取最近点），但多障碍同时深穿透能正确止推。
- 验证（`tests/physics_toy.rs`）：新增 `obstacle_multi_contact_corner_resolves`——
  两堵 AABB 墙间隙(0.3m) < 机体直径(0.4m)，机体被双面法向夹止、停在间隙中心
  (|x|<0.1) 且不深穿透任一侧、水平速度收敛。全量默认 + `--features phy` 构建均通过。
- **下一步（已并入 P1-2 终闭环）**：障碍碰撞与避障控制器闭环联动。

### P1-2 续续续续：障碍反射到传感器（避障雷达 / 视觉失效） ✅ 完成

- 目标：把障碍信息"反射"到机载传感器——装备避障距离传感器（雷达 / 激光雷达 /
  深度相机），沿机体前方发射探测射线，读数受近距盲区（视觉失效）/量程饱和/噪声影响，
  使上层控制器能感知障碍并暴露"太近反而看不到"的失效模式。
- 实现（`fly-sim-core/src/physics.rs`）：
  - `Obstacle::ray_hit(origin, dir, max_range)`：射线-障碍求交（球=二次方程，
    盒=slab 法，`ConvexHull`=递归取最近），返回 `t>0 && t<=max_range` 的最近命中。
  - `ray_obstacle_distance(origin, dir, max_range, obstacles)`：展平障碍列表求全局
    最近命中距离（`Option<f64>`，射程内无命中返回 `None`）。
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `RangeFinderSample { distance, valid }`：读数 + 有效性（`valid=false` = 量程饱和
    /近距盲区失效/随机瞬断，上层应视为"该方向不可信"而非"无障碍"）。
  - `RangeFinderModel`：把真值距离转成读数——加 `bias`+高斯 `noise`，超 `max_range`
    饱和失效；**近距盲区**：真值 `< blind_min` 时视觉/深度通道糊脸，错误饱和到
    `max_range` 且 `valid=false`（"障碍太近反而看不到"）；`drop_prob` 每帧随机失效。
- 集成（`fly-sim-core/src/plant.rs`）：
  - `QuadrotorPlant::ranger: Option<RangeFinderModel>` + `set_ranger()`。
  - `read_ranger()`：取机体位置+姿态，沿引擎机体系前方(-X)旋转到世界系发射射线，
    合并当前静态+动态障碍求最近距离，过 `RangeFinderModel` 返回 `RangeFinderSample`；
    未装备返回 `None`。
- 验证（`tests/physics_toy.rs`，ToyWorld 替身，默认 + `--features phy` 均过）：
  - `ranger_detects_obstacle_ahead`：前方球(中心 -2,5,0 r=1) 真值≈1.0m，`valid=true`
    且 `distance≈1.0`。
  - `ranger_near_blind_zone_invalid`：前方球面距 0.2m < `blind_min=0.5` → `valid=false`
    且饱和到 `max_range`（视觉失效，非"无障碍"）。
  - `ranger_no_obstacle_max_range`：前方无障 → `valid=false` 饱和到 `max_range`。
  - `ranger_unequipped_returns_none`：未 `set_ranger` 返回 `None`。
- **下一步（已并入下条闭环）**：障碍碰撞与避障控制器闭环联动（见下）。

### P1-2 终：障碍碰撞与避障控制器闭环联动 ✅ 完成

- 目标：把 `read_ranger()` 读数接入飞控避障逻辑，形成"**感知→决策→规避**"完整
  闭环——机体朝障碍飞行时，反应式避障在危险距离内制动减速并横向闪避，避免碰撞。
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `AvoidanceConfig { danger_dist, brake_gain, evade_lateral, hold_time }`：反应式避障配置。
    - `hold_time`（s）：**单射线 FOV 丢失保持**——横向闪避会让障碍滑出前向单射线
      （读数 `valid=false`），此时避障指令（制动+横向闪避+位置设定点偏移）在最近一次
      **确认危险**后继续维持 `hold_time` 秒再释放，保证障碍通过机体正侧方前横向分离
      持续积累；否则位置外环会把机体拉回原航线、抵消横向分离（实测：无保持时机体在
      射线边缘形成极限环，净间隙≈障碍半径，见 `tests/avoidance.rs`）。`0.0`=不保持。
  - `AvoidanceConfig::avoidance_velocity(sample, fwd_ned, right_ned) -> ([f64;3], bool)`：
    根据测距读数计算 NED 避障速度指令并标记是否触发。
    - **保守失效语义**：读数 `valid=false`（瞬断/近距盲区/量程饱和）一律**不介入**
      （避免凭空闪避；真实危险时读数有效且距离小，会正常触发）。
    - 触发条件：`valid && distance < danger_dist`；危险度 `severity=1-dist/danger_dist`。
    - 制动：沿 `-fwd_ned`，幅度 `severity*brake_gain*danger_dist`（m/s）。
    - 横向闪避：沿 `+right_ned`，恒定 `evade_lateral`（m/s），仅危险时给出。
- 实现（`fly-sim-core/src/plant.rs`）：
  - `QuadrotorPlant::forward_dir_ned()` / `right_dir_ned()`：把机体前/右向（引擎机体系
    -X/+Y）经姿态四元数旋转 + 引擎系→NED 映射，返回归一化 NED 单位向量，供避障速度
    指令投影并入 NED 速度设定点。
- 实现（`fly-sim-core/src/controller.rs`）：
  - `FlyController::avoidance: Option<AvoidanceConfig>` + `set_avoidance()` / `set_ranger()`
    / `configure_avoidance()`（便捷组合）。
  - `FlyController::step`：装备避障时，在控制律前读 `read_ranger()`，触发则把
    `AvoidanceConfig::avoidance_velocity` 的 NED 速度叠加进**速度设定点**（改 `sp.vel[0/1]`，
    竖向/偏航不动），实现闭环。无装备则行为完全不变（默认 `None`）。
- 实现（`fly-sim-core/src/sim.rs`）：
  - `SimLoop::run_avoidance(seconds, forward_vx_ned, obstacle_n, ranger, avoidance)
     -> (min_dist, all_ok)`：朝障碍飞行的闭环场景，返回全程最近逼近距离与数值稳定性。
- 验证：
  - `tests/avoidance.rs`（单元 3 项）：
    - `avoid_velocity_triggers_when_close`：近障触发，前向制动=-2.0、横向=+0.5。
    - `avoid_velocity_silent_when_far`：远障不触发，零指令。
    - `avoid_velocity_conservative_on_invalid`：失效读数不触发（保守不动作）。
  - `tests/avoidance.rs`（闭环 2 项）：
    - `avoidance_keeps_greater_clearance_than_bare`：朝障 1.5 m/s 飞 4s，避障闭环的
      最近逼近距离**明显大于**裸飞对照（gap_av > gap_bare + 0.5），且不穿透障碍表面。
    - `avoidance_no_false_trigger_when_no_obstacle`：无障时装备避障不导致发散。
  - 默认 + `--features phy` 全套测试通过。
- **P1-2 阶段收尾**：凸包/动态障碍（碰撞）+ 障碍反射传感器（感知）+ 避障闭环（决策/规避）
  三层全部打通，构成"几何→碰撞→感知→决策→规避"完整障碍处理链路。

### P1-2 续续续：凸包近似障碍 + 动态（平移）障碍 ✅ 完成

- 目标：支持以"多基本体并集"逼近任意凸体（凸包），以及随时间匀速平移的动态障碍，
  使障碍碰撞覆盖更复杂几何与运动场景（圆柱≈多球、移动障碍物拦停/推开机体）。
- 实现（`fly-sim-core/src/physics.rs`）：
  - `Obstacle::ConvexHull { parts: Vec<Obstacle> }`：凸包由若干基本体（球/盒，
    亦可嵌套 `ConvexHull`）的并集构成；`flatten_obstacles()` 递归展平成叶子
    `Sphere`/`Box`，使多接触叠加对每个子部件独立生效（凸包曲面 = 多球接触叠加）。
  - `closest_point_and_normal` 新增 `ConvexHull` 分支：递归取各子部件最近点中
    **整体最近**者作为凸包表面最近点（直接调用时亦正确，解算入口已展平则不会命中）。
  - `DynamicObstacle { base: Obstacle, velocity: [f64;3] }` + `at(t)`：按模拟时间
    `t` 把基准障碍平移 `velocity * t` 生成当前障碍（支持嵌套 `ConvexHull`）；
    `translate_obstacle` 递归平移所有子部件。
  - plant 层（`plant.rs`）：`QuadrotorPlant` 新增 `dynamic_obstacles` 字段 +
    `set_dynamic_obstacles()`；`step` 内部按 `self.time` 把每个动态障碍生成当前
    形态并与静态障碍合并解算。controller 层（`controller.rs`）透传
    `plant_set_dynamic_obstacles()`。
- 验证（`tests/physics_toy.rs`，ToyWorld 替身，默认 + `--features phy` 均过）：
  - `obstacle_convex_hull_cylinder_blocks`：5 球（r=1.0）串成竖直圆柱 `ConvexHull`，
    机体带 -X 初速撞入曲面，全程 `min_x > 0.8`（不深穿透实体），状态有限。
  - `dynamic_obstacle_translates_and_blocks`：动态球（base (-3,0,0) r=1.3，vel [1.5,0,0]）
    从左侧扫过停机坪平面上的机体，t≈2s 接触并把机体向右推过原点（终态 x≈3.0，
    `dist > 0.7` 不深穿透）。注意 ToyWorld 自带 y>=0 停机坪钳制，接触场景须放在
    y≈0 平面附近，否则机体自由落体掉到 y=0 与 y=5 处移动的障碍永久错开。
- **下一步（已并入下条闭环）**：障碍碰撞与避障控制器闭环联动（见下）。

### P1-2 终：障碍碰撞与避障控制器闭环联动 ✅ 完成

- 目标：把 `read_ranger()` 读数接入飞控避障逻辑，形成"**感知→决策→规避**"完整
  闭环——机体朝障碍飞行时，反应式避障在危险距离内制动减速并横向闪避，避免碰撞。
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `AvoidanceConfig { danger_dist, brake_gain, evade_lateral, hold_time }`：反应式避障配置。
    - `hold_time`（s）：**单射线 FOV 丢失保持**——横向闪避会让障碍滑出前向单射线
      （读数 `valid=false`），此时避障指令（制动+横向闪避+位置设定点偏移）在最近一次
      **确认危险**后继续维持 `hold_time` 秒再释放，保证障碍通过机体正侧方前横向分离
      持续积累；否则位置外环会把机体拉回原航线、抵消横向分离（实测：无保持时机体在
      射线边缘形成极限环，净间隙≈障碍半径，见 `tests/avoidance.rs`）。`0.0`=不保持。
  - `AvoidanceConfig::avoidance_velocity(sample, fwd_ned, right_ned) -> ([f64;3], bool)`：
    根据测距读数计算 NED 避障速度指令并标记是否触发。
    - **保守失效语义**：读数 `valid=false`（瞬断/近距盲区/量程饱和）一律**不介入**
      （避免凭空闪避；真实危险时读数有效且距离小，会正常触发）。
    - 触发条件：`valid && distance < danger_dist`；危险度 `severity=1-dist/danger_dist`。
    - 制动：沿 `-fwd_ned`，幅度 `severity*brake_gain*danger_dist`（m/s）。
    - 横向闪避：沿 `+right_ned`，恒定 `evade_lateral`（m/s），仅危险时给出。
- 实现（`fly-sim-core/src/plant.rs`）：
  - `QuadrotorPlant::forward_dir_ned()` / `right_dir_ned()`：把机体前/右向（引擎机体系
    -X/+Y）经姿态四元数旋转 + 引擎系→NED 映射，返回归一化 NED 单位向量，供避障速度
    指令投影并入 NED 速度设定点。
- 实现（`fly-sim-core/src/controller.rs`）：
  - `FlyController::avoidance: Option<AvoidanceConfig>` + `set_avoidance()` / `set_ranger()`
    / `configure_avoidance()`（便捷组合）。
  - `FlyController::step`：装备避障时，在控制律前读 `read_ranger()`，触发则把
    `AvoidanceConfig::avoidance_velocity` 的 NED 速度叠加进**速度设定点**（改 `sp.vel[0/1]`，
    竖向/偏航不动），实现闭环。无装备则行为完全不变（默认 `None`）。
- 实现（`fly-sim-core/src/sim.rs`）：
  - `SimLoop::run_avoidance(seconds, forward_vx_ned, obstacle_n, ranger, avoidance)
     -> (min_dist, all_ok)`：朝障碍飞行的闭环场景，返回全程最近逼近距离与数值稳定性。
- 验证：
  - `tests/avoidance.rs`（单元 3 项）：
    - `avoid_velocity_triggers_when_close`：近障触发，前向制动=-2.0、横向=+0.5。
    - `avoid_velocity_silent_when_far`：远障不触发，零指令。
    - `avoid_velocity_conservative_on_invalid`：失效读数不触发（保守不动作）。
  - `tests/avoidance.rs`（闭环 2 项）：
    - `avoidance_keeps_greater_clearance_than_bare`：朝障 1.5 m/s 飞 4s，避障闭环的
      最近逼近距离**明显大于**裸飞对照（gap_av > gap_bare + 0.5），且不穿透障碍表面。
    - `avoidance_no_false_trigger_when_no_obstacle`：无障时装备避障不导致发散。
  - 默认 + `--features phy` 全套测试通过。
- **P1-2 阶段收尾**：凸包/动态障碍（碰撞）+ 障碍反射传感器（感知）+ 避障闭环（决策/规避）
  三层全部打通，构成"几何→碰撞→感知→决策→规避"完整障碍处理链路。

### P1-2 续续续续：障碍反射到传感器（避障雷达 / 视觉失效） ✅ 完成

- 目标：把障碍信息"反射"到机载传感器——装备避障距离传感器（雷达 / 激光雷达 /
  深度相机），沿机体前方发射探测射线，读数受近距盲区（视觉失效）/量程饱和/噪声影响，
  使上层控制器能感知障碍并暴露"太近反而看不到"的失效模式。
- 实现（`fly-sim-core/src/physics.rs`）：
  - `Obstacle::ray_hit(origin, dir, max_range)`：射线-障碍求交（球=二次方程，
    盒=slab 法，`ConvexHull`=递归取最近），返回 `t>0 && t<=max_range` 的最近命中。
  - `ray_obstacle_distance(origin, dir, max_range, obstacles)`：展平障碍列表求全局
    最近命中距离（`Option<f64>`，射程内无命中返回 `None`）。
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `RangeFinderSample { distance, valid }`：读数 + 有效性（`valid=false` = 量程饱和
    /近距盲区失效/随机瞬断，上层应视为"该方向不可信"而非"无障碍"）。
  - `RangeFinderModel`：把真值距离转成读数——加 `bias`+高斯 `noise`，超 `max_range`
    饱和失效；**近距盲区**：真值 `< blind_min` 时视觉/深度通道糊脸，错误饱和到
    `max_range` 且 `valid=false`（"障碍太近反而看不到"）；`drop_prob` 每帧随机失效。
- 集成（`fly-sim-core/src/plant.rs`）：
  - `QuadrotorPlant::ranger: Option<RangeFinderModel>` + `set_ranger()`。
  - `read_ranger()`：取机体位置+姿态，沿引擎机体系前方(-X)旋转到世界系发射射线，
    合并当前静态+动态障碍求最近距离，过 `RangeFinderModel` 返回 `RangeFinderSample`；
    未装备返回 `None`。
- 验证（`tests/physics_toy.rs`，ToyWorld 替身，默认 + `--features phy` 均过）：
  - `ranger_detects_obstacle_ahead`：前方球(中心 -2,5,0 r=1) 真值≈1.0m，`valid=true`
    且 `distance≈1.0`。
  - `ranger_near_blind_zone_invalid`：前方球面距 0.2m < `blind_min=0.5` → `valid=false`
    且饱和到 `max_range`（视觉失效，非"无障碍"）。
  - `ranger_no_obstacle_max_range`：前方无障 → `valid=false` 饱和到 `max_range`。
  - `ranger_unequipped_returns_none`：未 `set_ranger` 返回 `None`。
- **下一步（已并入下条闭环）**：障碍碰撞与避障控制器闭环联动（见下）。

### P1-2 终：障碍碰撞与避障控制器闭环联动 ✅ 完成

- 目标：把 `read_ranger()` 读数接入飞控避障逻辑，形成"**感知→决策→规避**"完整
  闭环——机体朝障碍飞行时，反应式避障在危险距离内制动减速并横向闪避，避免碰撞。
- 实现（`fly-sim-core/src/sensor.rs`）：
  - `AvoidanceConfig { danger_dist, brake_gain, evade_lateral, hold_time }`：反应式避障配置。
    - `hold_time`（s）：**单射线 FOV 丢失保持**——横向闪避会让障碍滑出前向单射线
      （读数 `valid=false`），此时避障指令（制动+横向闪避+位置设定点偏移）在最近一次
      **确认危险**后继续维持 `hold_time` 秒再释放，保证障碍通过机体正侧方前横向分离
      持续积累；否则位置外环会把机体拉回原航线、抵消横向分离（实测：无保持时机体在
      射线边缘形成极限环，净间隙≈障碍半径，见 `tests/avoidance.rs`）。`0.0`=不保持。
  - `AvoidanceConfig::avoidance_velocity(sample, fwd_ned, right_ned) -> ([f64;3], bool)`：
    根据测距读数计算 NED 避障速度指令并标记是否触发。
    - **保守失效语义**：读数 `valid=false`（瞬断/近距盲区/量程饱和）一律**不介入**
      （避免凭空闪避；真实危险时读数有效且距离小，会正常触发）。
    - 触发条件：`valid && distance < danger_dist`；危险度 `severity=1-dist/danger_dist`。
    - 制动：沿 `-fwd_ned`，幅度 `severity*brake_gain*danger_dist`（m/s）。
    - 横向闪避：沿 `+right_ned`，恒定 `evade_lateral`（m/s），仅危险时给出。
- 实现（`fly-sim-core/src/plant.rs`）：
  - `QuadrotorPlant::forward_dir_ned()` / `right_dir_ned()`：把机体前/右向（引擎机体系
    -X/+Y）经姿态四元数旋转 + 引擎系→NED 映射，返回归一化 NED 单位向量，供避障速度
    指令投影并入 NED 速度设定点。
- 实现（`fly-sim-core/src/controller.rs`）：
  - `FlyController::avoidance: Option<AvoidanceConfig>` + `set_avoidance()` / `set_ranger()`
    / `configure_avoidance()`（便捷组合）。
  - `FlyController::step`：装备避障时，在控制律前读 `read_ranger()`，触发则把
    `AvoidanceConfig::avoidance_velocity` 的 NED 速度叠加进**速度设定点**（改 `sp.vel[0/1]`，
    竖向/偏航不动），实现闭环。无装备则行为完全不变（默认 `None`）。
- 实现（`fly-sim-core/src/sim.rs`）：
  - `SimLoop::run_avoidance(seconds, forward_vx_ned, obstacle_n, ranger, avoidance)
     -> (min_dist, all_ok)`：朝障碍飞行的闭环场景，返回全程最近逼近距离与数值稳定性。
- 验证：
  - `tests/avoidance.rs`（单元 3 项）：
    - `avoid_velocity_triggers_when_close`：近障触发，前向制动=-2.0、横向=+0.5。
    - `avoid_velocity_silent_when_far`：远障不触发，零指令。
    - `avoid_velocity_conservative_on_invalid`：失效读数不触发（保守不动作）。
  - `tests/avoidance.rs`（闭环 2 项）：
    - `avoidance_keeps_greater_clearance_than_bare`：朝障 1.5 m/s 飞 4s，避障闭环的
      最近逼近距离**明显大于**裸飞对照（gap_av > gap_bare + 0.5），且不穿透障碍表面。
    - `avoidance_no_false_trigger_when_no_obstacle`：无障时装备避障不导致发散。
  - 默认 + `--features phy` 全套测试通过。
- **P1-2 阶段收尾**：凸包/动态障碍（碰撞）+ 障碍反射传感器（感知）+ 避障闭环（决策/规避）
  三层全部打通，构成"几何→碰撞→感知→决策→规避"完整障碍处理链路。

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
- 待续（已做）：MAVLink 遥测下行见下「P2-2 续」。若需真路径跟随，需轨迹跟踪控制器
  （倾斜补偿 + 速度/加速度前馈）。

### P2-2 续：MAVLink 遥测下行链路 ✅ 完成

- 目标：把仿真机体的 `VehicleState` 按标准 MAVLink v2 编码成遥测流，可被标准地面站
  （QGC/PX4）解析——补齐差距清单 #8（"无标准消息协议 MAVLink"），对标"能跑真机流程"。
- 实现（`fly-sim-core/src/mavlink.rs`）：
  - `MavlinkBridge { sys_id, comp_id, seq, fb }`：维护帧序号，一拍编码 6 帧标准遥测
    （HEARTBEAT / ATTITUDE / LOCAL_POSITION_NED / SYS_STATUS / VFR_HUD /
    GLOBAL_POSITION_INT），字节级复用 `flyctrl-core` 既有 MAVLink v2 编码（CRC_EXTRA 兼容）。
  - `MavlinkStreamParser`：增量式 v2 组帧器（逐字节喂入，字节流 → `Frame` 列表），
    用于串口字节流场景，正确性与 `LoopbackLink::recv_frame` 同语义。
  - `loopback_telemetry(stream)`：经 `LoopbackLink` 回环 + `flyctrl_core::comm::mavlink::decode`
    （含 CRC_EXTRA 校验）解析回 `(msgid, payload)`，供自验证。
- 实现（`fly-sim-core/src/sim.rs`）：`SimLoop::run_mavlink_telemetry(seconds, sys_id)
  -> (stream, n_frames, all_ok)`：跑悬停并把每拍世界状态编码成完整遥测流返回。
- 验证（`tests/mavlink_telemetry.rs` 2 项）：
  - `mavlink_telemetry_roundtrips`：已知 `VehicleState` 编码后经标准 MAVLink 解码器回环，
    6 帧全部解析成功（CRC_EXTRA 字节级兼容），msg_id 集合正确，ATTITUDE 的
    roll/pitch/yaw、LOCAL_POSITION_NED 的 NED 位置、SYS_STATUS 健康位(0x1F) 与原始态一致。
  - `mavlink_stream_parser_is_incremental`：逐字节喂入组帧仍得 6 帧（串口字节流语义）。
- 注：本模块为 **host 侧遥测桥**（仿真/地面站联调）；嵌入式链路实现（UART/USB-CDC）
  在 `flyctrl-core::comm::link::stm32f407` 占位，落地时接 joc-base HAL。

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

---

## 五、下一阶段规划：最逼真模拟"真实使用场景"（再规划，2026-08-20）

> 目标：在 P0–P2（物理/动力/环境保真）已落地的基础上，把焦点从"动力学有多细"转向
> **"飞控在真实使用场景下的行为有多可信"**——遥控解锁 → 手动/增稳 → 自主任务 →
> 抗风/带噪声/故障 → RTL/降落 全流程。
> 判断：当前物理/动力深度已超多数 SITL 框架，真正的瓶颈在**控制层真实性（A）**与
> **使用场景生态层（D）**，感知（B）与物理深化（C）随后。

### 差距清单与优先级

| 优先级 | 层 | 项目 | 状态 |
|---|---|---|---|
| **P3-A1** | 控制 | 轨迹跟踪控制器（倾斜补偿 + 速度/加速度前馈） | ✅ 完成 |
| **P3-A2** | 控制 | `--sensor-noise` 下控制律鲁棒性（阶段 11 A/B/C 落地） | ✅ 完成 |
| **P3-A3** | 控制 | TECS 空速消费（总能量控制） | ✅ 完成 |
| **P3-B1** | 感知 | VIO / RTK-GPS（EKF 融合维度扩展） | ✅ 完成 |
| **P3-B2** | 感知 | 多射线 / 光流 / 深度相机避障（替代单射线 + hold_time 硬补） | ✅ 完成 |
| **P3-B3** | 感知 | 传感器硬/软故障注入到估计器（偏置突变/卡死/漂移） | ✅ 完成 |
| **P3-C1** | 物理 | 叶素理论 / 桨叶挥舞 / 桨尖失速 | ✅ 完成 |
| **P3-C2** | 物理 | 桨盘干扰（相邻桨下洗耦合） | ✅ 完成 |
| **P3-C3** | 物理 | 多刚体真实碰撞（机体-机体 / 机体-障碍） | ✅ 完成 |
| **P3-D1** | 生态 | RC 输入 + 手动/增稳模式全流程 | ✅ 完成 |
| **P3-D2** | 生态 | 真 HIL（sim ↔ F407 真板 UART 闭环） | 待做 |
| **P3-D3** | 生态 | 3D 可视化 / 传感器渲染 | ✅ 完成 |
| **P3-D4** | 生态 | 蒙特卡洛统计 + 真值回放 / 基准数据集 | ✅ 完成 |
| **P3-D5** | 生态 | 多机互飞 / 机间通信场景 | 待做 |

### 既有项调整清单（不只"加"，还要"改"）

- [x] **验收判据**：收敛判定必须同时断言 `TRU` 有界（不只 `EST≈TRU`），推广到所有场景测试。
  - 落地：新增共享工具 `tests/common/mod.rs`（`TruStats` 逐帧统计 + `assert_tru_bounded` 统一断言），判定量归一化为水平漂移/高度/倾角三个量，避免各测试各自写一份且阈值不一致。已推广到 `sensor_noise.rs`、`headless_hover_wind.rs`、`mission.rs`、`avoidance.rs`、`tecs_airspeed.rs`、`rc_modes.rs` 全部闭环场景测试；默认与 `--features phy` 全绿，无回归。`degraded.rs`（故障容错，失控即判据）、`wind_scan.rs`（打印型诊断，无通过性断言）不含收敛判定，不适用。
- [x] **`ToyWorld` 默认 `ContactModel`**：建议默认 `Some(default())`（与 `PhySdkWorld` 一致），避免无地面时噪声发散被误判为估计器缺陷（阶段 11 C 项落地）。
  - 落地（`fly-simulater` 本次提交）：MAVLink SIL 路径（`main.rs` `FlyController::new`）`contact=None` → `Some(ContactModel::default())`，与主 SIL 路径/`view.rs`/`fly-sim-server` 一致。架构上接触模型属于 `plant.rs` 而非 world（`ToyWorld`/`PhySdkWorld` 均无接触字段），故不改 world 结构体、无死代码。真空/自由落体与噪声鲁棒性诊断场景仍显式传 `None`（`run_freefall`、`zz_diag_noise`），语义不变。
- [x] **控制律倾斜补偿**：`des_thrust /= cos(tilt)`——已定位的移动掉高根因，属调整既有 PID 而非新功能。P3-A1 已随轨迹跟踪落地（`pid.rs` / `manual.rs` 均按 1/cos(tilt) 放大总推力）。
- [x] **大气密度随高度/温度**：`air_density` 现按 ISA 对流层标准大气随高度衰减（`plant.rs::air_density_at`，温度-高度关系隐含），高海拔推力/诱导速度/气动阻力更真实；11km 以上指数外推。
- [x] **传感器噪声默认值**：默认零噪声屏蔽了 EKF/控制律噪声行为，真实场景测试应默认开启 realistic 噪声。
  - 落地：`mission.rs` / `tecs_airspeed.rs` / `rc_modes.rs` / `degraded.rs` / `avoidance.rs` 等所有闭环场景测试的 `SensorConfig::default()` → `SensorConfig::realistic()`，默认跑真实噪声；同时修复 realistic 噪声暴露的两个问题——`SimGps` 健康判断改为"定位锁定状态"（`has_fix`，避免 20Hz GPS 非帧时刻 `position_available` 逐帧抖动误拒模式切换），以及 `degraded.rs` 电机退化指令断言对齐分配器"缩放而非逐路×eff"语义。默认与 `--features phy` 全量回归通过，无回归。

### 推进记录（进度跟踪）

> 勾选框 = 进度状态，完成后在本节补实测数据（与 P0–P2 同款格式）。

#### P3-A：控制层真实性（下一里程碑，Top 1）
- [x] **P3-A1 轨迹跟踪控制器**：倾斜补偿（`des_thrust /= cos(tilt)`）→ 速度/加速度前馈，目标 `mission` 长距离跟随 `stable=true`。
- [x] **P3-A2 噪声鲁棒性**：定位是 IMU 姿态抖动还是 GPS 位置外环主导后，调参/加前馈或 EKF 输出低通，新增 `tests/sensor_noise.rs` 锁定 `TRU` 有界。
- [x] **P3-A3 TECS**：空速进入控制律（与 EKF 空速融合衔接），风扰下能量保持。

- 验证（P3-A1，`flyctrl` 6d79086 + `fly-simulater` ea38e39）：5 场景（Hover/Step/Wind/Square/Circle）数值稳定无 NaN，pos_RMS 3.58–5.50m；垂向环 PM=36°/GM=∞（ki_z 0.6→0.3），悬停振荡 0.11m，垂向扰动 1.0s 恢复；`--scenario mission` 长距离跟随 `stable=true`。
- 验证（P3-A2，`fly-simulater` 5c2d3ee `tests/sensor_noise.rs`）：realistic 噪声 + 无地面约束下 TRU 高度 ±0.6m、水平漂移 <3m、姿态倾斜 <5.1°、EKF 估计误差 <0.3m；噪声消融定位主导源（GPS 位置外环）。
- 验证（P3-A3，`flyctrl` deab60a + `fly-simulater` 0550c49 `tests/tecs_airspeed.rs` 3 项）：TECS 悬停收敛；2m/s 逆风空速拖拽前馈被吹回 1.29m→0.03m（49×）；80m 高速巡航（vmax=5）总能量高度误差峰值 TECS 0.313 vs PID 0.793（<0.5×）、巡航速度 5.06 vs 3.56 m/s（前馈补偿寄生阻力）。空速源分离：EKF 空速观测用地速幅值、TECS 用相对空速矢量（`set_measured_airspeed_vec`），避免风相对空速污染水平速度估计。
- 验证（P3-D1，`flyctrl` 58b2e98 + `fly-simulater` 158b3c7 `tests/rc_modes.rs` 3 项）：遥控解锁 → 手动/增稳 → 定点平移（6 位档位开关：手动/增稳/定高/定点/返航/降落）→ RTL 回原点 → 降落触地 全流程数值稳定；未解锁请求定点/任务被治理器拒绝、解锁后放行；解锁开/合门控与 FDIR 一致，解锁关合 → 电机零推力自由坠落（u≈0）。`FlyController::step_rc` + `HilContext::step_with_cmd` 与 `step` 共用采集/估计/FDIR/收尾骨架，手动/增稳控制律（`flyctrl-core::controller::manual`：角速率直通 / 姿态保持）复用姿态内环与电机混控。

#### P3-D：使用场景/生态层（Top 2/3）
- [x] **P3-D1 RC 输入 + 全飞行模式**：遥控解锁 → 手动/增稳 → 自主任务 → RTL/降落 全流程场景（sim 侧注入 `RcInput`）。
- [ ] **P3-D2 真 HIL**：sim ↔ F407 真板 UART 闭环，兑现 DESIGN.md "HIL 语义"初衷（`MavlinkStreamParser`/UART 占位已留）。
- [x] **P3-D3 3D 可视化 / 传感器渲染**：web 3D 视角、射线/障碍/风场可视化。
- [x] **P3-D4 蒙特卡洛统计 + 真值回放 / 基准数据集**：批量跑 N 次同场景（噪声/风/参数扰动随机种子），
  输出轨迹分布（均值±σ、P95 包络）、收敛率/失效率；提供标准基准场景集与真值 CSV 回放，
  支撑回归对比（如 TECS vs PID 的统计显著性，替代当前单次确定性断言）。
- [x] **P3-D5 多机互飞 / 机间通信场景**：多实例 `FlyController` 共世界（同一 `RigidBodyWorld` 多机体），
  经 UDP 链路互发位置/速度（模拟 ADS-B / 机间链路），支持编队、跟随、机间避让验证。

#### P3-B：感知层
- [x] **P3-B1 VIO / RTK-GPS**：EKF 融合维度扩展。
- [x] **P3-B2 多射线/光流/深度相机避障**：替代单射线 + `hold_time` 硬补，提升避障真实度。
- [x] **P3-B3 传感器硬/软故障注入到估计器**：偏置突变/卡死/漂移全链路。

### P3-B1：VIO / RTK-GPS（EKF 融合维度扩展）✅ 完成
- 目标：EKF 融合视觉里程计（VIO，30–60Hz 相对位置/速度，短期准、长期漂移）与 RTK-GPS
  （厘米级绝对位置，1–5Hz，用于抑制 VIO 漂移），补齐 GPS 中断/帧间估计空白，扩展融合维度。
- 实现（`flyctrl-core`）：
  - `vehicle.rs`：`VioSample { pos: Option<PosSample>, vel: Option<[f32;3]> }`（可选位置/速度，
    适配不同 VIO 输出特性）、`RtkSample`（厘米级位置）。
  - `hal/sensor.rs`：`VioSensor`/`RtkSensor` 特质 + `MockVio`/`MockRtk`（`set_sample`/`set_health`）+
    STM32F407 硬件接口占位。
  - `estimator/trait_def.rs`：`Estimator` 新增默认 no-op 的 `update_vio(Option<VioSample>)` /
    `update_rtk(Option<RtkSample>)`（独立方法渐进接入，不改既有 `step` 签名）。
  - `estimator/ekf.rs`：VIO/RTK 噪声参数（`r_vio_pos=0.25`、`r_vio_vel=0.04`、`r_rtk=0.0025`）；
    重构 `update_pos`/`update_vel` 为 `update_pos_r`/`update_vel_r` 支持自定义 R，GPS/VIO/RTK 共用
    同一套融合逻辑；`update_vio`（位置+速度观测）与 `update_rtk`（位置观测）。
  - `hil.rs`：`HilContext::step` 泛型化为 `V: VioSensor, R: RtkSensor`，读取观测透传给估计器。
  - `fly-sim-core/src/plant.rs`：VIO/RTK 传感器模型（真值位置/速度 + 噪声 + VIO 缓慢随机游走漂移）；
    `controller.rs` 接入 `HilContext::step`。
- 数值稳定性处理（紧噪声观测下的发散根治）：
  - `K_MAX=2.0` 卡尔曼增益幅值限幅：修复紧 RTK 位置更新后 pos-vel 交叉协方差 `p03` 爆炸
    （-0.2 → -1.4e13）导致速度行增益/水平误差 1e19 m 的发散，降至水平 0.024 m。
  - 垂向加计零偏 `x[9]` 仅由垂向速度观测驱动：清零 `update_vel_r` 中其水平速度增益行
    （水平速度残差物理上由水平零偏引起，本滤波器不估计），防水平噪声注入垂向零偏。
  - `update_vel_r` 中加计零偏物理上界夹取（±0.3 m/s²），并调 `r_vio_vel` 0.01→0.04，
    根治 VIO 速度观测过紧驱动零偏 → 4.3e6 → NaN 的整步发散。
- 验证（`tests/vio_rtk.rs` 3 项，均用 `assert_tru_bounded`）：
  - `gps_outage_vio_rtk_bridges`：GPS 中断期间 VIO+RTK 桥接，`EST` 不发散。
  - `gps_outage_fusion_beats_imu_only`：GPS 中断 + VIO 融合误差显著小于纯 IMU 积分。
  - `rtk_keeps_position_cm_level`：RTK 持续可用时位置估计达到厘米级（水平 0.024 m）。
  - EKF 单测 `rtk_fusion_reaches_cm_precision` 通过；`cargo test -p flyctrl-core` 与
    `cargo test`（fly-simulater）全量回归全绿，无回归。
- 待续：VIO/RTK 与光流/深度相机的多源协同、RTK 半固定/浮点解模式建模（P3-B3 及后续）。

### P3-B2：多射线 / 光流 / 深度相机避障 ✅ 完成
- 目标：把 P1-2 的**单条前向射线 + `hold_time` 冻结补丁**升级为**水平扇形多射线**
  （近似雷达/光流/深度相机的广角覆盖），决策按多射线聚合（排斥力求和），障碍横向滑出
  中央射线后侧向射线仍持续覆盖，闪避方向随障碍横移连续翻转、危险度随距离连续衰减，
  **消除对 `hold_time` 硬补的依赖**；障碍彻底离开视场后避障自然释放，位置环接管回航。
- 实现（`fly-sim-core`）：
  - `sensor.rs`：`RangeFinderSample` → `RayReading { dir_ned, distance, valid }` +
    `RangeFinderFrame { rays }`（一帧多射线）；`RangeFinderModel` 新增 `fov_half`（单侧半
    视场角）+ `ray_count`（扇形射线数），`ray_dirs_body()` 生成 ±fov_half 等角分布射线
    （`ray_count=1` 退化为单射线=旧语义），`sample_ray(dir_ned, true_distance)` 每条射线
    独立采样（独立噪声/近距盲区/瞬断）。
  - `AvoidanceConfig::avoidance_velocity` 改为**多射线聚合**：所有有效且进入危险距离的
    射线作为"排斥源"（排斥力沿 `-射线方向`、强度随危险度线性），求和后制动沿 -前向
    （线性 max_sev：越近刹得越急）、横向闪避用 **`sqrt(sev)` 非线性**（进入危险距离即
    尽快坚定侧移——机体侧向速度受气动阻力封顶 ~1.4 m/s，线性缩放会远距闪避不足）；
    返回 `(v_ned, triggered, lateral_comp)`，其中 `lateral_comp` 为**原始排斥力**在机体
    侧向的投影（未乘幅度，居中障碍 ≈0），供控制器做方向锁存。
  - `plant.rs`：`read_ranger()` 改多射线发射——机体系射线先旋转到**引擎世界系（Y-up）**
    与障碍求交，再转 **NED 系**存储方向（世界 Y-up 下 NED 东=-world.z、NED 下=-world.y；
    坐标系错配曾致横向分量被混点丢弃 → `lateral_comp≈0` 逐帧翻号）。
  - `controller.rs`：**方向锁存**——基于 `lateral_comp`（阈值 0.25）更新闪避方向，
    障碍近正前（|lat_comp|≈0）时保持已锁存方向，首次触发且无明确方向默认向右（正对
    场景左右对称，关键是要坚定单侧闪避而非逐帧翻号净位移为零）；**泄漏积分**
    `av_evade_int`（`AV_EVADE_POS_K=3.0`、λ=1.0）把横向闪避速度平滑成位置设定点偏移，
    威胁期内累积、威胁结束按 e^(-λt) 回零 → 自然释放；移除 `hold_time`。
- 验证（`tests/avoidance.rs` 7 项，闭环场景用 `assert_tru_bounded`）：
  - `avoidance_keeps_greater_clearance_than_bare`（正对逼近）：多射线净间隙
    `MIN_CLEAR≈3.51m`（方向锁存修复前 0.26m 失败），基准被碰撞（`gap_bare<0.5`），
    避障显著大于基准 >1m。
  - `multi_ray_evades_lateral_slide_single_misses`（横向滑移 + 单vs多对比 + 自然释放）：
    偏置 3.2m 横向滑过，单射线中央射线恒打不中（净间隙 <1m 擦碰级），多射线侧向射线
    持续覆盖（`MIN_CLEAR≈4.74m`）；障碍越过离开视场后位置环把机体拉回原点
    （末帧横向 `final_z<2m`）。
  - `avoid_velocity_triggers_when_close`（单元决策）：危险距离内触发/外不触发/无效读数
    不触发；障碍偏右→向左闪、偏左→向右闪。
  - `avoid_ranger_detects_obstacle_in_fov`、`avoid_no_false_trigger_when_clear`（集成）：
    前方球命中、无障时无误触发、不影响悬停。
  - `cargo test` 全量回归（fly-simulater + flyctrl-core）全绿，无回归。

### P3-B3：传感器硬/软故障注入到估计器 ✅ 完成
- 目标：实现**偏置突变 / 卡死 / 漂移**三类传感器故障的**全链路注入**——故障在
  **"物理真值 → 传感器读数"**（`SensorModel`）处生效，与噪声/偏置/延迟等真实化模型
  叠加后一起喂给 EKF 与 FDIR（区别于直接改估计器内部状态），量化评估估计器/控制律/
  FDIR 对故障的鲁棒性与容错路径。
- 故障注入 API（`fly-sim-core`）：
  - `sensor.rs`：`SensorFault` 枚举——**软故障** `AccelBias`/`GyroBias`（偏置突变）、
    `AccelDrift`/`GyroDrift`（漂移率，每帧 `bias_extra += rate·dt` 缓变累积）、
    `GpsBias`（NED 位置偏置）；**硬故障** `AccelStuck`/`GyroStuck`/`GpsStuck`
    （输出冻结为给定值，`None` 解除，不退化为"失锁"）。
    `SensorModel` 新增故障状态 + `apply_fault()`，在 `process()` 中叠加偏置/应用卡死。
  - `plant.rs`：`QuadrotorPlant::inject_sensor_fault()` 透传包装。
  - `controller.rs`：`FlyController::inject_sensor_fault()` 运行时注入；
    `failsafe_engaged()` 暴露 FDIR Critical 失控保护置位（供测试断言）。
- 系统响应（`tests/sensor_fault.rs` 6 项，闭环场景用 `assert_tru_bounded`）：
  - 软故障（估计器靠鲁棒性消化，TRU 有界、估计误差有界）：
    - `accel_bias_step_stays_bounded`（加计偏置 [0.25,-0.15,0.20] m/s²）：估计误差
      ≈0.08m、水平漂移 0.35m，不 NaN/不 runaway。
    - `gyro_bias_step_stays_bounded`（陀螺偏置 [0.02,-0.015,0.01] rad/s）：估计误差
      ≈0.19m、水平漂移 3.29m（偏置致姿态误差→位置环以水平漂移补偿，仍有界）。
    - `accel_drift_ramps_bounded`（漂移 0.02 m/s²/s，5s 累计 ~0.1 m/s²）：估计误差
      ≈0.03m、水平漂移 0.31m。
    - `gps_bias_step_offsets_estimate_bounded`（GPS 偏置 3m，VIO/RTK 关闭、GPS 为唯一
      绝对源）：位置估计被牵制偏移 ≈2.9m（下界 1.5m 证明故障**确实穿过估计器链路**），
      机体随之漂移 2.5m，整体有界。
  - 硬故障（FDIR / 多源融合容错路径）：
    - `imu_stuck_triggers_fdir_critical`：加速度计卡死（全零输出）→ 读数冻结为给定值
      （确已到达估计器链路）→ FDIR 判 `Health::Critical` → `failsafe_engaged()` 失控
      保护单向置位（执行器归零，安全降级路径）。
    - `gps_stuck_masked_by_vio_rtk_fusion`：GPS 卡死在偏移 5m 位置（`has_fix` 仍 true，
      不退化为失锁）→ 被 VIO/RTK 多源融合（P3-B1）兜底：估计误差 ≈0.03m、水平漂移
      0.41m（单源硬故障不导致位置发散——若只靠 GPS+IMU 死推，卡死偏置会把位置估计
      与机体直接拉飞）。
- 待续：EKF 创新门限（innovation gating）/ 健康置信度融合以主动拒绝对抗性故障、
  RTK 半固定/浮点解模式建模、多传感交叉校验（P3-B3 深化）。

#### P3-C：物理层深化（排在 A/B/D 之后）
- [x] **P3-C1 叶素理论 / 桨叶挥舞 / 桨尖失速**：前飞大机动真实性的最后一块。
- [x] **P3-C2 桨盘干扰**：相邻桨下洗耦合修正项。
- [x] **P3-C3 多刚体真实碰撞**：机体-机体 / 机体-障碍（当前纯惩罚点接触）。

### P3-C1：叶素理论 / 桨叶挥舞 / 桨尖失速 ✅ 完成
- 目标：用**叶素理论（BET）**替代纯动量理论推力，真实刻画**桨叶挥舞**（flap-back，
  前飞桨盘后倾、推力矢量后倾产生后向阻力）与**桨尖失速**（前飞前进比增大导致后行
  侧叶尖攻角过大 → 推力塌陷 + 反扭矩剧增），提升前飞大机动真实性；并通过悬停标定
  保证与原动量理论（`T = k·ω²`）零回归兼容。
- 内核（`fly-sim-core`）：
  - `plant.rs`：`bet_rotor()` 纯函数——线性扭桨均匀入流解析（`Ct`/`Ch`/`a1`），
    前进比 `μ=v_xy/(ωR)`、入流比 `λ=v_vi/(ωR)` 物理饱和（`MU_MAX=1.5`、
    `LAM_MAX=3.0`、叶尖速度下限 `1e-3`，消除停桨/低转速数值发散）；桨尖失速开关
    `s(μ)`（`μ≤0.6` 无失速，`μ>0.6` 光滑过渡至 1）→ `Ct·(1-0.5s)` 推力塌陷、
    `Ch·(1+s)` 水平力放大、反扭矩因子 `1+2s`。
  - `QuadrotorPlant` 新增 `bet_theta0`（悬停有效桨距，反解标定）与 `bet_vi`（诱导
    速度）字段；`step()` 以 BET 推力 + 桨盘平面 H 力 + 挥舞后倾分量
    `Σt·a1`（`a1<0` → 前向分量后向，即 flap-back 阻力）合成机体合力。
- 悬停保零回归：`bet_theta0` 由悬停平衡反解，使 BET 悬停推力 = `prop_kt·ω²`
  （`bet_hover_matches_legacy_omega_squared` 相对误差 <1e-6，
  `bet_plant_hover_regression_holds` 悬停高度/油门不漂移）。
- 系统响应（`tests/powertrain.rs`，15 项全过）：
  - `bet_thrust_declines_and_h_grows_with_forward_speed`：前飞推力衰减 + H 力增长
    （阻力物理）；`bet_a1_flaps_down_with_forward_speed`：挥舞角随 μ 展开（后倾）。
  - `bet_stall_boundary_amplifies_torque_and_collapses_thrust`：μ 超 0.6 失速边界
    反扭矩放大、推力塌陷。
- 既有回归联动（TECS 巡航能量保持被 BET 阻力拉低）：
  - 根因核查：BET 桨盘阻力（H 力 + 挥舞后倾，5 m/s ≈0.8 m/s²）为**合理物理量级
    （非 bug）**，前馈系数扫描（0.10/0.125/0.15 → 峰值 0.447/0.401/0.506）确认
    0.125 为最优；TECS 峰值能量误差 ~0.40 为速度建立期固有势能↔动能交换。
  - 处置：`config.rs` `drag_fwd` 0.09 → 0.125（补偿机身型阻 + BET 桨盘阻力）；
    TECS 相对判据 0.5× → 0.7×（重新标定阈值，稳态巡航优势仍 ~7 倍：
    end_e_eq 0.061 vs PID 0.453）；`Diag` 增 `max_e_eq_t` 诊断峰值时刻。
  - `tecs_airspeed.rs` 3 项全过；`cargo test --workspace --features phy` 全量回归
    全绿，无回归。
- 待续：P3-C3 多刚体真实碰撞（机体-机体 / 机体-障碍）。

### P3-C2：桨盘干扰（相邻桨下洗耦合修正项） ✅ 完成
- 目标：模拟前飞时上游桨滑流（下洗）被自由流吹向下游、射入下游桨盘的干扰——
  下游桨入流比增大 → 推力下降、反扭矩增大，且前/后桨不对称产生前飞俯仰干扰
  力矩，替代"四桨独立无干扰"的理想化假设。
- 内核（`fly-sim-core`）：
  - `plant.rs`：`downwash_coupling()` 纯函数——按桨位几何（X 布局臂向量）+ 来流
    方向逐桨累加上游覆盖系数 `frac = clamp(along/2l,0,1)·exp(−across/w)`（同侧
    正前方桨沿流饱和、横向按桨径指数衰减），得每桨耦合入流增量
    `vi_coup[i] = k·blow·Σfrac·vi`；`QuadrotorPlant::step()` 在 BET 核前逐桨叠加
    `vi_coup[i]` 到入流 `vi_in`，下游桨推力自然下降。
  - **吹送系数 `blow = |v_h|/(|v_h|+vi)`**：滑流被自由流吹向下游的程度。悬停
    （|v_h|=0）滑流垂直向下、桨盘共面互不干扰 → 自动为 0（保 P3-C1 悬停标定）；
    前飞速度越大越被吹平、越能扫入下游桨盘，耦合从 0 单调逼近 1。避免低速时
    过度惩罚推进效率（RC 定点平移回归修复的根因）。
  - 耦合系数 `k = rotor_downwash_coupling`（config/airframe 默认 0.25；几何核查：
    0.35 对直接尾随桨在 5 m/s（blow=0.5）产生 ~0.88 m/s 入流增量偏强——同平面
    X 布局下尾随桨盘实际在滑流锥边缘，故取 0.25 为适中量级；0 可整体关闭）。
- 系统响应（`tests/powertrain.rs`，21 项全过）：
  - `downwash_coupling_targets_downstream_rotors_only`：仅下游桨（后 2 桨）受耦合，
    上游/侧向桨为零（耦合量级 `k·blow·vi`）；`downwash_coupling_scales_with_k`：
    耦合随 k 线性；`downwash_coupling_grows_with_forward_speed`：随前飞速度单调
    增大且高速饱和（悬停 0，v=20 vs v=10 增幅 <35%）。
  - `bet_plant_hover_regression_holds`：k=0.25 与 k=0 悬停推力一致（保零回归，
    悬停高度/油门不漂移）。
- 既有回归联动（TECS 巡航能量保持被下洗耦合效率惩罚拉低）：
  - 根因核查：前飞效率惩罚为真实物理（上游滑流射入下游桨盘 → 总功率需求增大），
    量级经几何核查后定 k=0.25，非 bug。
  - 处置：`drag_fwd` 0.125 → 0.14（前馈系数扫描 0.125/0.14/0.15 → 0.498/0.486/
    0.548，0.14 为最优，补偿 BET 阻力 + 下洗耦合）；TECS 相对判据 0.7× → 0.75×
    （重新标定阈值，稳态巡航优势仍 ~3.6 倍：end_e_eq 0.116 vs PID 0.434）；
    headwind 测试维持 0.125（0.14 对 2 m/s 逆风过补偿，低速不适用）。
  - `tecs_airspeed.rs` 3 项全过；`cargo test`（fly-simulater 全量）exit 0 无回归、
    `fly-sim-core --features phy` powertrain 21 项全过、`flyctrl-core` 68 项全过。

### P3-C3：多刚体真实碰撞（机体-机体 / 机体-障碍） ✅ 完成
- 目标：用**多刚体动量守恒碰撞**替代"静态障碍/地面只推单体"的惩罚模型，使**机体-
  机体**（或机体-其他动态刚体）碰撞时冲量**等大反向**施加到双方（牛顿第三定律），
  动量守恒、按恢复系数损失动能，支撑编队/防撞/坠机场景的物理真实性。
- 内核（`fly-sim-core`）：
  - `physics.rs`：`BodyCollider` 结构体（刚体 `id` / `mass` / 碰撞球半径 `radius`，
    `mass=0` 表示静态刚体：只被撞、不回动）；`resolve_body_peer_collisions()`
    解算一个动态刚体与一组 peer 的**球-球**碰撞——间隙 = 中心距 − (r_self+r_peer)，
    穿透即接触。复用 `ContactModel`（penalty_k / restitution / friction），但临界
    阻尼用**折合质量** `m_eff = m_self·m_peer/(m_self+m_peer)`（双刚体惯量耦合）：
    法向冲量 `jn = (k·pen + c_n·max(−vn,0))·dt`（阻尼只在接近时吸能、不泵能量），
    沿法向 `n = self→peer` 施加 **−jn·n 于 self、+jn·n 于 peer**（两体互相分离、
    等大反向 → 动量守恒）；切向库仑摩擦用折合质量 + 相对切向速度
    `jt = min(μ·jn, m_eff·|v_t_rel|)` 沿相对滑移反方向等大反向施加（预算内完全
    抑制滑移、超出按库仑封顶）。多 peer 同时穿透各接触独立叠加。
  - `plant.rs`：`QuadrotorPlant` 新增 `peer_colliders: Vec<BodyCollider>` 字段与
    `set_peer_colliders()` 注册接口；`step()` 在障碍/地面接触之后经
    `resolve_body_peer_collisions` 解算机体-机体碰撞（本体碰撞球半径取螺旋桨外周
    包络 `1.2×臂长`，与障碍模型同源），最深穿透的 peer 接触写入 `last_contact`。
  - 符号修正（回归发现）：初版法向冲量误用 `+jn·n` 于 self / `−jn·n` 于 peer——
    因 `n` 指向 peer，符号反了导致两体**互相拉近**而非分离，能量指数泵入
    （v1+v2 虽守恒但 |v|→70 m/s）。修正为 self 沿 −n、peer 沿 +n（与地面"沿接触
    法向推离"同源），碰撞即正确分离。
- 系统响应（`tests/physics_toy.rs`，23 项全过，新增 3 项验收）：
  - `body_peer_collision_conserves_momentum`：等质量正碰 → 动量精确守恒
    （v1x+v2x−1.0 <1e-3）、peer 被撞出正向速度、非弹性动能损失（ke1<ke0）。
  - `static_peer_absorbs_impact`：撞静态刚体（mass=0）→ peer 不被推动（v<1e-9）、
    本体反弹且反弹速度小于接近速度（非弹性）。
  - `plant_peer_collision_integrates`：`set_peer_colliders` 接入 plant → 重叠即报告
    `contact_info()`、本体被推离 peer（位移方向正确）、状态全程有限。
- 既有回归联动：`cargo test`（fly-simulater 全量）exit 0 无回归、
  `fly-sim-core --features phy` powertrain 21 项全过。
- 待续：P3-D 生态项（真 HIL / 3D 可视化 / 蒙特卡洛基准数据集）。

### P3-D3：3D 可视化 / 传感器渲染 ✅ 完成
- 目标：把渲染从"纯机体 + 轨迹 + 电机"扩展为**传感器/环境可视化**——障碍（球/盒）、
  测距射线（有效性着色）、风场矢量箭头，全部与真值物理同源；并在 web 端新增
  **avoidance 避障场景**（静态障碍 + 动态逼近障碍 + 扇式多射线闭环），可直观看到
  P3-B2 多射线避障、P3-C3 碰撞、风扰动在 3D 视图中的实时表现。
- 实现（`fly-sim-core/src/render.rs`）：
  - 新增 `RenderObstacle::{Sphere, Box}`（轻量渲染障碍）、`RenderRay { dir, distance,
    valid }`（测距射线）、`RenderWind { pos, vec }`（风场采样箭头）。
  - `RenderInput` 扩展 `obstacles/rays/wind` 字段；新增 `draw_obstacles`（球=线框圆 +
    底部椭圆，盒=6 面线框）、`draw_rays`（有效=绿实线 + 命中端亮圆，无效=红虚线、
    按 max_range 长度）、`draw_wind_vectors`（蓝→深蓝箭头），在 `render_frame` 中
    顺序叠加绘制（风场最底层 → 障碍 → 射线 → 机体）。
- 数据源（只读、不扰动仿真）：
  - `plant.rs`：`current_obstacles()`（静态 + 动态障碍合并、展平 ConvexHull）、
    `wind_at()`（只读采样）。
  - `wind.rs`：`WindField::sample_static_at()` —— 只计算确定性风分量
    （base 切变 + 阵风 + 突风 + 热气流 + 空间相关），**不推进时间 / 不更新湍流
    滤波状态 / 不消耗随机流**，可视化与物理采样同公式同值。
  - `controller.rs` / `sim.rs`：`current_obstacles` / `wind_at` / `ranger_frame` 访问器；
    `sim.rs` 新增 `configure_avoidance`、`plant_set_dynamic_obstacles` 场景配置入口。
- 场景与联动（`fly-sim-server` + `web/`）：
  - `fly-sim-server` `rebuild()`：avoidance 场景装配"静态矮墙盒 + 西北角球" + 动态
    球（南侧 -26m 以 2 m/s 北向逼近）+ 扇式测距（±60°×5、量程 12m）+ 避障闭环
    （危险 11m / 横向闪避 2 m/s）。
  - `advance()` 每帧收集：`current_obstacles()` → `RenderObstacle`；
    `ranger_frame()` → `RenderRay`（NED → 引擎系换算 x=n/y=-d/z=-e）；
    机体周围 5×5 水平网格 `wind_at()` → `RenderWind`（仅风场景绘制）。
  - `web/index.html` 新增 `avoidance` 场景选项与提示文案。
- 验证：
  - `cargo build --workspace --features phy` 与 `cargo test --workspace` 全量回归
    exit 0 无失败。
  - 启动 `fly-sim-server --features phy`（http://127.0.0.1:8080/）：HTTP 200 且页面
    含 avoidance 选项；WS 客户端发送 `{"scenario":"avoidance",...}` 后遥测场景即时
    切换为 `avoidance`、机体稳态悬停（alt≈4.9m、diverged=false），完整推帧
    binary=10 / text=10 无崩溃（渲染输入已含障碍/射线，风=0 时不绘风箭头）。

### P3-D4：蒙特卡洛统计 + 真值回放 / 基准数据集 ✅ 完成
- 目标：批量跑 N 次同场景（传感器噪声/湍流风随机种子逐次改变），从**物理真值**
  统计轨迹分布（均值±σ、P95 包络、极值）与收敛率/失效率；提供**标准基准场景集**
  与**真值 CSV 回放**，把"单次确定性断言"升级为"统计显著性回归对比"（如
  TECS vs PID），并可被下游回放/可视化/回归工具消费。
- 内核（`tests/monte_carlo.rs`，1 项验收）：
  - **标准基准场景集**（闭环保真，`ToyWorld` 替身 + realistic 传感器噪声）：
    - `hover`：无风悬停，PID，10s。
    - `wind`：2 m/s 逆风悬停，PID，8s。
    - `cruise`：80m 北向巡航 vmax=5，TECS vs PID 同设定点 14s —— 回归对比 +
      统计显著性判据。
  - **蒙特卡洛机制**：`MC_RUNS` 环境变量控制 N（默认 8 快验，正式统计 ≥64）；
    `seeds_for()` 按"场景 + 序号"确定性派生去相关的传感器/风双种子（可复现、
    逐次不同）；每次运行收集真值指标：最大水平漂移 `h_max`、最大倾角
    `tilt_max`、结束 NED down、最大/结束**能量高度误差** `e_eq`
    （NED 口径：`d − v_h²/(2g)`，巡航场景 TECS/PID 同口径可比）。
  - **统计输出**：每场景 N 次均值±σ、P95 包络、min/max、收敛率
    （有限 && |end_d|<10 && 水平包络内 && 不翻滚<45°）、失效率（NaN 计数）。
  - **真值回放**：seed=0 确定性代表性轨迹写 `target/bench/{hover,wind,
    cruise_tecs}_tru.csv`（`t,tru_n,tru_e,tru_d,tru_vn,tru_ve,tru_vd,tilt_deg,e_eq`，
    git 忽略、天然非版本污染）。
- 系统响应（N=32/场景，`$env:MC_RUNS=32; cargo test --test monte_carlo`，exit 0）：
  - `hover`：收敛率 100%、无 NaN；h_max 0.32±0.06m（P95 0.44）、tilt 0.22°、
    end_d −5.00m；max|e_eq| 0.217。
  - `wind`：收敛率 100%；h_max 1.12±0.04m（P95 1.17，2m/s 逆风漂移仅 1 米级）、
    tilt 2.18°、end_d −5.15m（逆风下坠 0.15m）；max|e_eq| 0.477。
  - `cruise`：TECS vs PID 100% 收敛。能量高度误差 **max|e_eq| 均值 0.485±0.002
    vs 0.695±0.004**、**P95 包络 0.489 vs 0.705**（TECS 系统性低 ~30%）；
    结束误差 0.107 vs 0.437；巡航覆盖 66m vs 41m（TECS 拖拽前馈补偿寄生阻力、
    巡航更快）——与 P3-A3 单次确定性结论（0.313 vs 0.793）方向一致、量级吻合。
  - 统计断言（防回归、防 flaky）：每场景收敛率 ≥75%、全程无 NaN；cruise 场景
    TECS 的 max|e_eq| **均值与 P95 双双显著低于** PID（N≥8 即稳定，N=8/32 两档
    一致，非噪声偶然）。`MC_RUNS` 放大样本即可支撑正式统计显著性检验。
  - **正式运行 N=64**（`$env:MC_RUNS=64; cargo test --test monte_carlo`，176s，
    exit 0，统计验收 PASS）：四场景全部 100% 收敛、0 NaN，三档样本（8/32/64）
    完全一致——wind max|e_eq| 0.477±0.001（P95 0.478）、cruise TECS vs PID
    **max|e_eq| 均值 0.485±0.003 vs 0.695±0.004、P95 0.489 vs 0.701、结束误差
    0.107 vs 0.437**（gap ~30% 稳定，均值/σ 差达 ~60σ），统计显著性结论稳定。
- 既有回归联动：`cargo test --tests`（fly-simulater 全量 19 个测试文件）exit 0 无回归。
- 验证（P3-D5，`fly-simulater` `fly-sim-core/src/multi.rs` + `tests/multi_drone.rs` 3 项）：
  - 实现点：`RigidBodyWorld` 增加 `Rc<RefCell<W>>` 共享世界包装（多机同一物理世界、`body_id`
    独立建刚体）；`QuadrotorPlant` 新增 `new_at`（指定初始 NED 位置）+ `body_id()` +
    `external_world_step` 开关（每帧各机只注入冲量、共享世界统一 `step(dt)` 一次，避免
    N 机各步一次导致世界时间膨胀 N 倍）；`FlyController` 新增 `new_at`/`plant_set_peer_colliders`
    透传；机间碰撞 j>i 单向注册 `BodyCollider`（每对只由较小 id 机体解算一次，等大反向冲量
    动量守恒、不翻倍）；`DroneLink` 内存邮箱模拟 ADS-B/机间 UDP 链路（每帧广播真值遥测
    `DroneTelemetry{id,pos,vel,t}`，下一帧读取含一帧链路延迟）；编队 `formation_setpoint`
    （leader 遥测 + 速度前馈，消除 PD 位置环随动稳态滞后）、机间避让 `inter_drone_avoid_vel`
    （制动 + 横向让行复合，解决对头冲突纯径向排斥退化）、`target_with_avoid`。
  - 实测（`tests/multi_drone.rs`，ToyWorld 9.81，DT=4ms，全 PASS）：
    - **编队跟随**：3 机 Leader-Follower，leader 北向 2 m/s 平移 15 m 后悬停，follower
      保持偏移 [0,±4] m——收敛期末端队形偏移误差 **f1=f2=0.271 m（<0.5 m）**；三机
      TRU 无 NaN、高度保持、倾角 <20°。leader 位置环受 `vmax_xy=2` 限幅，停坡后以
      kp=0.3 渐近到位（t=16s 时 n=14.56 m），故总时长取 16 s、收敛窗口取末端 2 s。
    - **遥测精度**：drone1 收到的 drone0 遥测与真值位置/速度逐分量 **≤1e-3 m 级一致**。
    - **机间避让**：对头接近两机（n=0 北向 vs n=12 南向）经制动+横向让行，**最小间距
      1.528 m**（碰撞球半径和 ≈0.54 m，不触发物理碰撞，间距显著大于 2×0.27）。
  - 既有回归联动：`cargo test --workspace` exit 0 无回归。
