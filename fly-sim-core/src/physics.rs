//! 物理引擎抽象层（阶段 7：依赖倒置）。
//!
//! 仿真层（`plant.rs` 的四旋翼推进模型）只依赖本模块定义的 `RigidBodyWorld` trait，
//! 而不依赖具体物理引擎（Rust `phy-sdk` 或测试替身）。这样替换物理引擎实现（或注入玩具级
//! 替身做单元测试）无需改动任何仿真逻辑。
//!
//! **接口语义约定（实现者必须遵守）**：
//! - `apply_impulse` / `apply_torque_impulse` 施加的是**瞬态**线/角冲量（N·s / N·m·s）：
//!   这一步消耗的作用量，step 后不会残留。这保证四旋翼每帧重算重注入，不会出现幽灵推力。
//! - `step(dt)` 推进动力学并返回状态码（0=OK，非 0=检测到非有限状态，世界损坏）。
//! - `get_rigid_transforms` 批量读回所有刚体的 `(pos.xyz + quat.wxyz)` 7 元组，
//!   按 body_id 顺序排列。这是真实引擎 ABI 的接口形态，替身也实现它，保持统一。

#[cfg(feature = "phy")]
use phy_math::Vec3 as V3;
#[cfg(feature = "phy")]
use phy_sdk::rigid::{RigidSubsystem, RigidWorld};
#[cfg(feature = "phy")]
use phy_sdk::{get_as, get_as_mut, PhysicsBuilder, World};

/// 刚体 7 元组：(pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z)。
/// 世界系与引擎一致（Y-up，右手）。
pub type RigidTransform = [f64; 7];

/// 物理引擎最小接口（仿真层依赖此 trait，而非具体引擎）。
///
/// 设计取舍（与"通用物理引擎接口"提案的差异，见 PLAN 阶段 7 讨论）：
/// - 不暴露 `set_state` / `at_point`：四旋翼推力沿机体过质心轴，纯力矩用 `apply_torque_impulse`，
///   `at_point` 传 None 即可，无需力臂；引擎持有状态，仿真层不写回。
/// - 保留批量 `get_rigid_transforms`：真实引擎 ABI 即此形态，替身对齐避免为单 body 改 ABI。
/// - 重力矢量由引擎内部持有（世界系 (0,-g,0)），不每次传入。
pub trait RigidBodyWorld {
    /// 当前刚体数量。
    fn body_count(&self) -> usize;

    /// 在世界中添加一个刚体，返回其 body_id（>=0）。
    /// `pos7` = (x,y,z, qw,qx,qy,qz)；`inertia3` = 主转动惯量 (Ixx,Iyy,Izz)。
    /// `mass == 0` 表示静态刚体（无限质量，不受力运动，用作地面）。
    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64;

    /// 施加瞬态线冲量（世界系 3 向量，N·s）。`mode` 透传给底层（0=默认）。
    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], mode: i32);

    /// 施加瞬态角冲量（世界系 3 向量，N·m·s）。
    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], mode: i32);

    /// 读回线速度（世界系，m/s）。
    fn get_velocity(&self, id: i64) -> [f64; 3];

    /// 读回角速度（世界系，rad/s）。
    fn get_angular_velocity(&self, id: i64) -> [f64; 3];

    /// 批量读回所有刚体 7 元组到 `buf`（长度需 >= body_count()*7），返回写入个数。
    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize;

    /// 读回单个刚体 `id` 的 7 元组 (x,y,z, qw,qx,qy,qz) 到 `buf`（长度需 >= 7）。
    /// 接触解算等需要按 body_id 精准读位姿时使用，避免 `get_rigid_transforms`
    /// 返回的首元素（body 0）未必是目标 body。
    fn get_body_transform(&self, id: i64, buf: &mut [f64; 7]);

    /// 推进一个时间步，返回状态码（0=OK）。
    fn step(&mut self, dt: f64) -> i32;

    /// 世界累计时间（s）。
    fn time(&self) -> f64;
}

// ============================================================ 接触 / 碰撞模型（P1-2）

/// 地面接触模型参数（惩罚弹簧-阻尼 + 库仑摩擦）。
///
/// 采用**惩罚法弹簧-阻尼**（而非逐步恢复系数），从根本上避免"每步施加反弹冲量
/// 反泵能量"导致机体被弹飞的问题：恢复系数 e 仅用于推导阻尼比 ζ，
/// 阻尼力只在接近（v_n < 0）时吸收能量。
#[derive(Clone, Debug)]
pub struct ContactModel {
    /// 地面顶面世界 Y 坐标（引擎系，Y-up）。默认 -5.0。
    pub ground_y: f64,
    /// 恢复系数 e ∈ [0,1]：e=0 纯非弹（无反弹），e=1 完全弹性。用于推导阻尼比。
    pub restitution: f64,
    /// 库仑摩擦系数 μ（切向力预算 = μ · 法向力）。
    pub friction: f64,
    /// 法向惩罚刚度 k_n（N/m）。越大接触越"硬"、穿透越小，但需更小 dt 稳定。
    pub penalty_k: f64,
    /// 接触体半高（沿 Y），接触判定面 = ground_y + terrain_surface_y(x,z) + contact_half_h。
    pub contact_half_h: f64,
    /// 地形高度场。`None` = 无限平面（接触面恒为 `ground_y`）；
    /// `Some(t)` = 接触面随 (x,z) 变化。用于解锁循迹/避障/斜坡着陆场景（P1 扩展）。
    pub terrain: Option<TerrainField>,
}

impl Default for ContactModel {
    fn default() -> Self {
        // 默认：贴近地面（停机坪）的软接触，低反弹、较强摩擦（落地不打滑）。
        ContactModel {
            ground_y: -5.0,
            restitution: 0.2,
            friction: 0.8,
            penalty_k: 8000.0,
            contact_half_h: 0.1,
            terrain: None,
        }
    }
}

// ============================================================ 地形高度场（P1 扩展：地形）

/// 地形表面高度场（引擎系 Y-up，世界 (x,z) 平面采样）。
///
/// 把"接触判定面"从无限平面扩展为随水平位置变化的曲面，使四旋翼能在斜坡/
/// 丘陵上着陆与滑行，支撑循迹/避障/地形跟随场景。地形只影响**接触判定面**，
/// 不改变重力/气动（与既有惩罚接触模型正交）。
#[derive(Clone, Debug)]
pub enum TerrainField {
    /// 平坦地面（高度场恒为常数），等价于 `terrain=None` 但显式表达。
    Flat(f64),
    /// 规则网格高度图：原点 `(ox,oz)`、网格间距 `dx`、行/列数 `nx`/`nz`，
    /// `heights[(iz*nx + ix)]` 为网格点高度，块内双线性插值。超出范围用边缘值钳制。
    HeightMap {
        origin_x: f64,
        origin_z: f64,
        spacing: f64,
        nx: usize,
        nz: usize,
        heights: Vec<f64>,
    },
}

impl TerrainField {
    /// 在水平位置 (x,z) 处采样地形表面高度（引擎系 Y）。
    pub fn height_at(&self, x: f64, z: f64) -> f64 {
        match self {
            TerrainField::Flat(h) => *h,
            TerrainField::HeightMap {
                origin_x,
                origin_z,
                spacing,
                nx,
                nz,
                heights,
            } => {
                if *nx == 0 || *nz == 0 || *spacing <= 0.0 {
                    return 0.0;
                }
                // 网格浮点坐标（可能为负/越界）。
                let gx = (x - origin_x) / spacing;
                let gz = (z - origin_z) / spacing;
                // 钳制到有效网格区间 [0, nx-1] × [0, nz-1]。
                let gx_c = gx.clamp(0.0, (*nx - 1) as f64);
                let gz_c = gz.clamp(0.0, (*nz - 1) as f64);
                let ix0 = gx_c.floor() as usize;
                let iz0 = gz_c.floor() as usize;
                let ix1 = (ix0 + 1).min(*nx - 1);
                let iz1 = (iz0 + 1).min(*nz - 1);
                let fx = gx_c - ix0 as f64;
                let fz = gz_c - iz0 as f64;
                let h00 = heights[iz0 * nx + ix0];
                let h10 = heights[iz0 * nx + ix1];
                let h01 = heights[iz1 * nx + ix0];
                let h11 = heights[iz1 * nx + ix1];
                // 双线性插值。
                let hx0 = h00 + (h10 - h00) * fx;
                let hx1 = h01 + (h11 - h01) * fx;
                hx0 + (hx1 - hx0) * fz
            }
        }
    }
}

/// 计算接触判定面的世界 Y 坐标：ground_y + 地形表面高度(x,z) + 半高。
fn terrain_surface_y(m: &ContactModel, x: f64, z: f64) -> f64 {
    let th = match &m.terrain {
        Some(t) => t.height_at(x, z),
        None => 0.0,
    };
    m.ground_y + th + m.contact_half_h
}

/// 最近一次接触解算结果（供日志 / 调试）。
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactInfo {
    /// 本步是否接触地面（或障碍）。
    pub touching: bool,
    /// 穿透深度（>0 表示陷入）。
    pub penetration: f64,
    /// 法向接触力（N，沿法向离开障碍为正）。
    pub normal_force: f64,
    /// 切向摩擦冲量大小（N·s）。
    pub friction_impulse: f64,
    /// 接触法向（单位向量，引擎系 Y-up，指向离开障碍）。
    pub normal: [f64; 3],
    /// 接触点（引擎系世界坐标）。
    pub point: [f64; 3],
    /// 本步注入的接触冲量（世界系 3 向量，N·s）。
    pub impulse: [f64; 3],
}

// ============================================================ 障碍物（P1-2 续：障碍碰撞）

/// 静态障碍物（碰撞体）。在引擎世界坐标系（Y-up: x=北, y=上, z=-东）中描述。
///
/// 障碍本身静态，碰撞由**惩罚弹簧-阻尼模型**解算（不向物理世界注入原生刚体），
/// 与 `ContactModel` 地面接触保持一致的"惩罚模型"设计哲学：静态体只在解算时
/// 读取位姿并施加法向/切向冲量，绝不改变世界刚体拓扑。
#[derive(Clone, Debug)]
pub enum Obstacle {
    /// 球：中心 (引擎世界系) + 半径（m）。
    Sphere { center: [f64; 3], radius: f64 },
    /// 轴对齐盒（AABB）：最小角 (x,y,z) + 最大角 (x,y,z)。
    Box { min: [f64; 3], max: [f64; 3] },
    /// 凸包（凸体）近似：由若干基本体（球/盒，亦可嵌套 ConvexHull）的**并集**构成，
    /// 递归展平后作为多个独立碰撞体解算（多接触叠加对每个子部件生效）。
    ///
    /// 用于以"多球/多盒"逼近任意凸体（如圆柱≈一串球、长方体角≈盒、胶囊≈两端球+柱）。
    /// 解算时本变体在 `resolve_obstacle_contact` 入口被 `flatten_obstacles` 递归展平成
    /// 叶子 `Sphere`/`Box`，故 `closest_point_and_normal` 不会收到 `ConvexHull`。
    ConvexHull { parts: Vec<Obstacle> },
}

/// 把障碍列表递归展平：`ConvexHull` 展开为其 `parts`（继续递归），其余原样保留。
///
/// 展平后每个元素是基本碰撞体（`Sphere`/`Box`），供 `resolve_obstacle_contact`
/// 统一做多接触叠加解算。
pub fn flatten_obstacles(obstacles: &[Obstacle]) -> Vec<Obstacle> {
    let mut out = Vec::with_capacity(obstacles.len());
    for o in obstacles {
        match o {
            Obstacle::ConvexHull { parts } => {
                out.extend(flatten_obstacles(parts));
            }
            other => out.push(other.clone()),
        }
    }
    out
}

/// 动态障碍：由一个**基准障碍**（t=0 时位置）与匀速平移速度构成。
///
/// 解算时按模拟时间 `t` 把基准障碍平移 `vel * t` 得到当前障碍（支持 `ConvexHull`，
/// 递归平移其所有 `parts`）。旋转/非匀速运动暂不支持（首版聚焦平移动态障碍）。
///
/// 用法：在 `QuadrotorPlant` 上通过 `set_dynamic_obstacles` 注册，`step` 内部按
/// `self.time` 重新生成当前障碍并交给 `resolve_obstacle_contact`。
#[derive(Clone, Debug)]
pub struct DynamicObstacle {
    /// 基准障碍（t=0 时所在位置）。
    pub base: Obstacle,
    /// 匀速平移速度 [vx,vy,vz]（m/s，引擎世界系）。
    pub velocity: [f64; 3],
}

impl DynamicObstacle {
    /// 生成 `t` 时刻的障碍（基准障碍平移 `velocity * t`）。
    pub fn at(&self, t: f64) -> Obstacle {
        translate_obstacle(&self.base, &self.velocity, t)
    }
}

/// 把障碍按平移 `vel * t` 生成新障碍（支持嵌套 `ConvexHull`）。
fn translate_obstacle(o: &Obstacle, vel: &[f64; 3], t: f64) -> Obstacle {
    match o {
        Obstacle::Sphere { center, radius } => Obstacle::Sphere {
            center: [center[0] + vel[0] * t, center[1] + vel[1] * t, center[2] + vel[2] * t],
            radius: *radius,
        },
        Obstacle::Box { min, max } => Obstacle::Box {
            min: [min[0] + vel[0] * t, min[1] + vel[1] * t, min[2] + vel[2] * t],
            max: [max[0] + vel[0] * t, max[1] + vel[1] * t, max[2] + vel[2] * t],
        },
        Obstacle::ConvexHull { parts } => Obstacle::ConvexHull {
            parts: parts.iter().map(|p| translate_obstacle(p, vel, t)).collect(),
        },
    }
}

impl Obstacle {
    /// 返回机体中心 `p` 到障碍的**最近接触点**（引擎世界系）与碰撞法向
    /// （指向机体、即离开障碍的方向，单位向量）。
    ///
    /// - 球：法向即径向 `(p-center)/|p-center|`；接触点取 `p` 本身（球用中心作接触点）。
    /// - 盒：最近点夹取至盒表面/体内；体外法向指向最近面，体内法向取"最近面"方向以推开。
    fn closest_point_and_normal(&self, p: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        match self {
            Obstacle::Sphere { center, radius: _ } => {
                let mut d = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
                let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                let n = if dist > 1e-9 {
                    [d[0] / dist, d[1] / dist, d[2] / dist]
                } else {
                    [0.0, 1.0, 0.0] // 退化：从球心向上推
                };
                (p, n)
            }
            Obstacle::Box { min, max } => {
                let cx = p[0].clamp(min[0], max[0]);
                let cy = p[1].clamp(min[1], max[1]);
                let cz = p[2].clamp(min[2], max[2]);
                let cp = [cx, cy, cz];
                let inside = p[0] >= min[0] && p[0] <= max[0]
                    && p[1] >= min[1] && p[1] <= max[1]
                    && p[2] >= min[2] && p[2] <= max[2];
                let mut n = [0.0f64; 3];
                if inside {
                    // 朝最近面方向推：取各面距离最小值
                    let dpx0 = p[0] - min[0];
                    let dpx1 = max[0] - p[0];
                    let dpy0 = p[1] - min[1];
                    let dpy1 = max[1] - p[1];
                    let dpz0 = p[2] - min[2];
                    let dpz1 = max[2] - p[2];
                    let mut best = (dpx0, 0usize);
                    if dpx1 < best.0 { best = (dpx1, 1); }
                    if dpy0 < best.0 { best = (dpy0, 2); }
                    if dpy1 < best.0 { best = (dpy1, 3); }
                    if dpz0 < best.0 { best = (dpz0, 4); }
                    if dpz1 < best.0 { best = (dpz1, 5); }
                    match best.1 {
                        0 => n[0] = -1.0,
                        1 => n[0] = 1.0,
                        2 => n[1] = -1.0,
                        3 => n[1] = 1.0,
                        4 => n[2] = -1.0,
                        _ => n[2] = 1.0,
                    }
                    (cp, n)
                } else {
                    let d = [p[0] - cx, p[1] - cy, p[2] - cz];
                    let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    if dist > 1e-9 {
                        n = [d[0] / dist, d[1] / dist, d[2] / dist];
                    } else {
                        n = [0.0, 1.0, 0.0];
                    }
                    (cp, n)
                }
            }
            Obstacle::ConvexHull { parts } => {
                // 递归取各子部件最近点中**整体最近**者，作为凸包表面最近点。
                // 注：`resolve_obstacle_contact` 入口已把 ConvexHull 展平，本分支主要在
                // 直接调用 `closest_point_and_normal` 时出现。
                let mut best_cp = p;
                let mut best_n = [0.0f64, 1.0, 0.0];
                let mut best_dist = f64::INFINITY;
                for part in parts {
                    let (cp, n) = part.closest_point_and_normal(p);
                    let d = [p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]];
                    let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    if dist < best_dist {
                        best_dist = dist;
                        best_cp = cp;
                        best_n = n;
                    }
                }
                (best_cp, best_n)
            }
        }
    }

    /// 从 `origin` 沿单位方向 `dir` 发射一条射线，返回射中本障碍的最近正向距离（m）。
    ///
    /// 只返回 `t>0` 且 `t<=max_range` 的最近命中；未命中返回 `None`。
    /// - 球：解析射线-球求交（二次方程）。
    /// - 盒：slab 法（对各轴求进入/退出 t，取最大进入 t）。
    /// - 凸包：递归取各子部件的最近命中。
    fn ray_hit(&self, origin: [f64; 3], dir: [f64; 3], max_range: f64) -> Option<f64> {
        let hit = match self {
            Obstacle::Sphere { center, radius } => {
                // |origin + t*dir - center|^2 = r^2
                let oc = [origin[0] - center[0], origin[1] - center[1], origin[2] - center[2]];
                let b = oc[0] * dir[0] + oc[1] * dir[1] + oc[2] * dir[2];
                let c = oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - radius * radius;
                let disc = b * b - c; // a = dir·dir = 1
                if disc < 0.0 {
                    return None;
                }
                let sq = disc.sqrt();
                let t1 = -b - sq;
                let t2 = -b + sq;
                if t1 > 1e-6 {
                    Some(t1)
                } else if t2 > 1e-6 {
                    Some(t2) // 起点在球内：从近端出
                } else {
                    None
                }
            }
            Obstacle::Box { min, max } => {
                // slab 法：tmin/tmax 各轴夹取
                let mut tmin = -f64::INFINITY;
                let mut tmax = f64::INFINITY;
                for i in 0..3 {
                    if dir[i].abs() < 1e-9 {
                        if origin[i] < min[i] || origin[i] > max[i] {
                            return None; // 平行且在外侧
                        }
                    } else {
                        let inv = 1.0 / dir[i];
                        let t1 = (min[i] - origin[i]) * inv;
                        let t2 = (max[i] - origin[i]) * inv;
                        let (t_enter, t_exit) = if t1 < t2 { (t1, t2) } else { (t2, t1) };
                        if t_enter > tmin {
                            tmin = t_enter;
                        }
                        if t_exit < tmax {
                            tmax = t_exit;
                        }
                        if tmin > tmax {
                            return None;
                        }
                    }
                }
                if tmax < 0.0 {
                    return None; // 盒在身后
                }
                let t = if tmin > 1e-6 { tmin } else { tmax };
                if t > 1e-6 {
                    Some(t)
                } else {
                    None
                }
            }
            Obstacle::ConvexHull { parts } => {
                let mut best: Option<f64> = None;
                for part in parts {
                    if let Some(t) = part.ray_hit(origin, dir, max_range) {
                        best = Some(best.map_or(t, |b| b.min(t)));
                    }
                }
                best
            }
        };
        hit.filter(|&t| t > 1e-6 && t <= max_range)
    }
}

/// 从 `origin` 沿单位方向 `dir` 发射射线，返回一组障碍中最近命中距离（m）。
///
/// 障碍列表会被 `flatten_obstacles` 递归展平（凸包 → 子部件并集）。返回 `None`
/// 表示射程内无命中（量程外或全空）。`max_range` 为传感器最大量程。
pub fn ray_obstacle_distance(
    origin: [f64; 3],
    dir: [f64; 3],
    max_range: f64,
    obstacles: &[Obstacle],
) -> Option<f64> {
    let flat = flatten_obstacles(obstacles);
    let mut best: Option<f64> = None;
    for o in &flat {
        if let Some(t) = o.ray_hit(origin, dir, max_range) {
            best = Some(best.map_or(t, |b| b.min(t)));
        }
    }
    best
}

/// 解算刚体 `id` 与一组静态障碍的碰撞，把冲量经 `world.apply_impulse` 注入。
///
/// 采用与地面接触相同的**惩罚弹簧-阻尼 + 库仑摩擦**模型（复用 `ContactModel` 的
/// `penalty_k` / `restitution` / `friction` 参数）。`body_radius` 为机体碰撞球半径
/// （螺旋桨外周包络），用于把"机体中心 vs 障碍"的间隙转换为"表面 vs 表面"接触。
///
/// **多接触叠加**：当多个障碍同时穿透时，对每一个穿透障碍分别计算并施加"法向弹簧-阻尼
/// 冲量 + 库仑摩擦冲量"，再求和注入（`world.apply_impulse` 一次批量施加）。这比"取最深
/// 穿透单点解算"更真实——机体卡在两面墙夹角 / 同时贴地+障碍时，各接触法向独立推开，
/// 不会被单点法向带偏。仍属单点接触近似（每个障碍只取最近点），但多障碍同时深穿透时
/// 能正确止推、不穿入。
///
/// 返回**汇总**接触信息：最深穿透、`point`/`normal`/`impulse` 取穿透最深的那个障碍、
/// `normal_force`/`friction_impulse` 为各接触之和。未碰撞时 `touching=false`。
pub fn resolve_obstacle_contact<W: RigidBodyWorld>(
    world: &mut W,
    id: i64,
    mass: f64,
    obstacles: &[Obstacle],
    body_radius: f64,
    cm: &ContactModel,
    dt: f64,
) -> ContactInfo {
    let mut tf = [0.0f64; 7];
    world.get_body_transform(id, &mut tf);
    let p = [tf[0], tf[1], tf[2]];
    let vel = world.get_velocity(id);

    // 递归展平 ConvexHull，使多接触叠加对每个基本碰撞体（球/盒）独立生效。
    let obstacles = flatten_obstacles(obstacles);

    // 阻尼比来自恢复系数（与地面接触同推导），所有障碍共用。
    let zeta = if cm.restitution >= 1.0 {
        0.0
    } else {
        (-cm.restitution.ln()) / (2.0 * std::f64::consts::PI)
    }
    .clamp(0.0, 1.0);
    let c_crit = 2.0 * (cm.penalty_k * mass).sqrt();
    let c_n = zeta * c_crit;

    // 累计每个穿透障碍的接触（支撑多接触叠加）。
    let mut total_impulse = [0.0f64; 3];
    let mut total_normal_force = 0.0;
    let mut total_friction_impulse = 0.0;
    // 汇总信息取最深穿透者。
    let mut deepest: Option<([f64; 3], [f64; 3], f64, f64)> = None; // (点, 法向, 穿透, 法向力)

    for obs in obstacles {
        let (cp, n) = obs.closest_point_and_normal(p);
        // 表面-表面间隙：机体碰撞球中心到障碍"表面点"的距离，减去机体半径。
        let gap = match obs {
            Obstacle::Sphere { center, radius } => {
                let d = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
                let g = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() - (radius + body_radius);
                g
            }
            Obstacle::Box { .. } => {
                let d = [p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]];
                let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                dist - body_radius
            }
            // 入口已 `flatten_obstacles`，循环内不会遇到 ConvexHull。
            Obstacle::ConvexHull { .. } => unreachable!("ConvexHull already flattened at entry"),
        };
        let penetration = -gap; // >0 = 穿透
        if penetration <= 0.0 {
            continue;
        }

        // 法向速度（沿 n 正=远离障碍）
        let vn = vel[0] * n[0] + vel[1] * n[1] + vel[2] * n[2];
        // 法向冲量（弹簧 + 阻尼，只在接近时吸收），只能推不能拉
        let f_spring = cm.penalty_k * penetration;
        let f_damp = -c_n * vn.min(0.0);
        let jn = (f_spring + f_damp) * dt;
        let jn = jn.max(0.0);

        let mut impulse = [0.0f64; 3];
        for i in 0..3 {
            impulse[i] = n[i] * jn;
        }
        // 切向摩擦（库仑）：抵消切向速度，预算 = μ·法向力冲量
        let vn_vec = [n[0] * vn, n[1] * vn, n[2] * vn];
        let vt = [vel[0] - vn_vec[0], vel[1] - vn_vec[1], vel[2] - vn_vec[2]];
        let vt_mag = (vt[0] * vt[0] + vt[1] * vt[1] + vt[2] * vt[2]).sqrt();
        if vt_mag > 1e-6 {
            let budget = cm.friction * jn;
            let scale = (budget / (mass * vt_mag)).min(1.0);
            for i in 0..3 {
                impulse[i] -= scale * mass * vt[i];
            }
        }

        // 累计到总冲量（多接触叠加）。
        for i in 0..3 {
            total_impulse[i] += impulse[i];
        }
        let fric_partial =
            (impulse[0] * impulse[0] + impulse[1] * impulse[1] + impulse[2] * impulse[2])
                .sqrt()
                - jn.max(0.0);
        total_normal_force += f_spring + f_damp;
        total_friction_impulse += fric_partial.max(0.0);

        // 记录最深穿透者作为汇总信息来源。
        match deepest {
            Some((_, _, best_pen, _)) if best_pen >= penetration => {}
            _ => deepest = Some((cp, n, penetration, f_spring + f_damp)),
        }
    }

    if let Some((cp, n, penetration, _)) = deepest {
        world.apply_impulse(id, &total_impulse, 0);
        ContactInfo {
            touching: true,
            penetration,
            normal_force: total_normal_force,
            friction_impulse: total_friction_impulse,
            normal: n,
            point: cp,
            impulse: total_impulse,
        }
    } else {
        ContactInfo {
            touching: false,
            penetration: 0.0,
            normal_force: 0.0,
            friction_impulse: 0.0,
            normal: [0.0; 3],
            point: [0.0; 3],
            impulse: [0.0; 3],
        }
    }
}

// ============================================================ 机体-机体碰撞（P3-C3）

/// 参与机体-机体碰撞的动态刚体碰撞体描述。
///
/// 每个碰撞体 = 一个**动态刚体**（有自己的质量/速度/位置，受碰撞冲量作用），
/// 用**球**近似其碰撞体积（四旋翼取螺旋桨外周包络，见 plant 的 `1.2×臂长`）。
/// 与静态 `Obstacle` 的惩罚模型互补：机体-机体是**两个动态刚体**之间的碰撞，
/// 冲量必须等大反向施加（牛顿第三定律），保证**动量守恒**——这是 P3-C3 与
/// "只推单体"的静态障碍/地面接触的本质区别。
#[derive(Clone, Copy, Debug)]
pub struct BodyCollider {
    /// 刚体 id（`RigidBodyWorld::add_body` 返回值）。
    pub id: i64,
    /// 刚体质量（kg）。`0` 表示静态刚体（无限质量，只被撞、不回动）。
    pub mass: f64,
    /// 碰撞球半径（m）。表面-表面间隙 = 中心距 - (r_self + r_peer)。
    pub radius: f64,
}

/// 解算一个动态刚体 `self_` 与一组**其他动态刚体** `peers` 的碰撞。
///
/// 对每个穿透的 peer 计算"弹簧-阻尼法向冲量 + 库仑摩擦冲量"，并**等大反向**
/// 施加到 `self_` 与 peer（牛顿第三定律）——与静态障碍/地面"只推单体"不同，
/// 这是**双刚体动量守恒**的碰撞。几何：球-球，间隙 = 中心距 - (r_self+r_peer)。
///
/// 复用 `ContactModel` 参数（`penalty_k` / `restitution` / `friction`），但临界阻尼用
/// **折合质量** `m_eff = m_self·m_peer/(m_self+m_peer)`（双质量弹簧-阻尼的标准量，
/// 与静态障碍用 `m_self` 的区别：peer 也是动态刚体，惯性须计耦合）；
/// 法向冲量 `jn = (k·pen + c_n·max(-vn,0))·dt`（阻尼只在接近时吸能、不泵能量），
/// 施加 `-jn·n` 于 self、`+jn·n` 于 peer（`n` = self→peer，两体沿 -n/+n **互相分离**、
/// 等大反向动量守恒）。切向库仑摩擦同样用折合质量 + **相对切向
/// 速度**：`jt = min(μ·jn, m_eff·|v_t_rel|)` 沿相对滑移反方向等大反向施加（预算内
/// 完全抑制相对滑移、超出按库仑封顶），动量依然守恒。
///
/// 返回每个发生接触的 peer 的 `ContactInfo`（`normal` 指向 peer、`impulse` 为本体
/// 受到的冲量）；无接触返回空 `Vec`。多 peer 同时穿透时各接触独立叠加（对本体的
/// 多个冲量求和注入，各 peer 也各自收到反冲）。
///
/// `peer.mass == 0`（静态刚体）：仅本体受冲量、peer 不回动（`m_eff` 退化为本体质量，
/// 与静态障碍模型一致）。
pub fn resolve_body_peer_collisions<W: RigidBodyWorld>(
    world: &mut W,
    self_: BodyCollider,
    peers: &[BodyCollider],
    cm: &ContactModel,
    dt: f64,
) -> Vec<ContactInfo> {
    let mut out = Vec::new();

    let mut tf_self = [0.0f64; 7];
    world.get_body_transform(self_.id, &mut tf_self);
    let p_self = [tf_self[0], tf_self[1], tf_self[2]];
    let v_self = world.get_velocity(self_.id);

    // 阻尼比由恢复系数推导（与地面/障碍同源）：ζ = -ln(e)/(2π)，clamp [0,1]。
    let zeta = if cm.restitution >= 1.0 {
        0.0
    } else {
        (-cm.restitution.ln()) / (2.0 * std::f64::consts::PI)
    }
    .clamp(0.0, 1.0);

    for peer in peers {
        if peer.id == self_.id {
            continue;
        }
        let mut tf_peer = [0.0f64; 7];
        world.get_body_transform(peer.id, &mut tf_peer);
        let p_peer = [tf_peer[0], tf_peer[1], tf_peer[2]];
        let v_peer = world.get_velocity(peer.id);

        // 球-球间隙：中心距 - (r_self + r_peer)；<0 即穿透。
        let d = [p_peer[0] - p_self[0], p_peer[1] - p_self[1], p_peer[2] - p_self[2]];
        let dist = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let gap = dist - (self_.radius + peer.radius);
        if gap >= 0.0 || dist < 1e-9 {
            continue;
        }
        let penetration = -gap;
        // 法向：self -> peer（单位）。
        let n = [d[0] / dist, d[1] / dist, d[2] / dist];

        // 折合质量；静态 peer（无限质量）退化为本体质量。
        let m_eff = if peer.mass <= 0.0 {
            self_.mass
        } else {
            self_.mass * peer.mass / (self_.mass + peer.mass)
        };
        // 相对速度（peer 相对 self），法向分量 vn>0 = 分离。
        let v_rel = [v_peer[0] - v_self[0], v_peer[1] - v_self[1], v_peer[2] - v_self[2]];
        let vn = v_rel[0] * n[0] + v_rel[1] * n[1] + v_rel[2] * n[2];

        let c_crit = 2.0 * (cm.penalty_k * m_eff).sqrt();
        let c_n = zeta * c_crit;

        // 法向冲量（弹簧 + 阻尼，只在接近时吸能），只推不拉。
        let f_spring = cm.penalty_k * penetration;
        let f_damp = -c_n * vn.min(0.0);
        let jn = ((f_spring + f_damp) * dt).max(0.0);

        // 等大反向法向冲量（n = self→peer）：self 沿 **-n** 推离 peer、peer 沿 **+n** 推离 self，
        // 两体互相分离（与地面模型"沿接触法向推离"同源），且等大反向 → 动量守恒。
        let mut imp_self = [-n[0] * jn, -n[1] * jn, -n[2] * jn];
        let mut imp_peer = [n[0] * jn, n[1] * jn, n[2] * jn];

        // 切向库仑摩擦（折合质量 + 相对切向速度）：抑制相对滑移，预算 = μ·jn。
        let vt_rel = [v_rel[0] - n[0] * vn, v_rel[1] - n[1] * vn, v_rel[2] - n[2] * vn];
        let vt_mag = (vt_rel[0] * vt_rel[0] + vt_rel[1] * vt_rel[1] + vt_rel[2] * vt_rel[2]).sqrt();
        let mut friction_impulse = 0.0;
        if vt_mag > 1e-9 && jn > 0.0 {
            let budget = cm.friction * jn;
            let jt = budget.min(m_eff * vt_mag);
            // dir = -v_t_hat：self 沿反滑移方向受力、peer 等大反向。
            let dir = [-vt_rel[0] / vt_mag, -vt_rel[1] / vt_mag, -vt_rel[2] / vt_mag];
            for k in 0..3 {
                imp_self[k] += jt * dir[k];
                imp_peer[k] -= jt * dir[k];
            }
            friction_impulse = jt;
        }

        world.apply_impulse(self_.id, &imp_self, 0);
        if peer.mass > 0.0 {
            world.apply_impulse(peer.id, &imp_peer, 0);
        }

        out.push(ContactInfo {
            touching: true,
            penetration,
            normal_force: f_spring + f_damp,
            friction_impulse,
            normal: n,
            point: [
                0.5 * (p_self[0] + p_peer[0]),
                0.5 * (p_self[1] + p_peer[1]),
                0.5 * (p_self[2] + p_peer[2]),
            ],
            impulse: imp_self,
        });
    }
    out
}

/// 解算刚体 `id` 与（可能带地形的）地面的接触，并把冲量经 `world.apply_impulse` 注入。
///
/// 返回接触信息。引擎系 Y-up；接触判定面 `contact_y = terrain_surface_y(m, x, z)`。
/// 当 body 低于 contact_y 时认为穿透，施加弹簧-阻尼法向力 + 库仑摩擦。
///
/// **地形法向**：平地形（无 `terrain` 或 `Flat`）法向为竖直 (0,1,0)；HeightMap 地形
/// 用法向梯度构造局部法向 `n = normalize(-∂h/∂x, 1, -∂h/∂z)`，使斜坡上重力切向分量
/// 能驱动机体沿坡下滑、接触法向冲量方向正确。
///
/// 阻尼比由恢复系数推导：ζ = -ln(e) / (2π)，clamp 到 [0,1]；临界阻尼 c_crit = 2√(k_n·m)，
/// 法向阻尼 c_n = ζ·c_crit。阻尼力只在接近时（v_n < 0）吸收能量，绝不泵能量。
pub fn resolve_ground_contact<W: RigidBodyWorld>(
    world: &mut W,
    id: i64,
    mass: f64,
    m: &ContactModel,
    dt: f64,
) -> ContactInfo {
    let mut tf = [0.0f64; 7];
    world.get_body_transform(id, &mut tf);
    let pos = [tf[0], tf[1], tf[2]];

    let contact_y = terrain_surface_y(m, pos[0], pos[2]);
    let pen = contact_y - pos[1]; // >0 即穿透
    if pen <= 0.0 {
        return ContactInfo {
            touching: false,
            ..Default::default()
        };
    }

    // 局部法向：平地 = (0,1,0)；地形 = 梯度法向。
    let n = terrain_normal(m, pos[0], pos[2]);

    let vel = world.get_velocity(id);
    // 法向速度（沿局部法向投影）。
    let vn = vel[0] * n[0] + vel[1] * n[1] + vel[2] * n[2];

    // 阻尼比来自恢复系数；e>=1 视为无阻尼（纯弹性，理论上不应泵能量因为只吸接近能量）。
    let zeta = if m.restitution >= 1.0 {
        0.0
    } else {
        (-m.restitution.ln()) / (2.0 * std::f64::consts::PI)
    }
    .clamp(0.0, 1.0);
    let c_crit = 2.0 * (m.penalty_k * mass).sqrt();
    let c_n = zeta * c_crit;

    // 法向冲量（弹簧 + 阻尼）沿 n。阻尼只在接近时（vn<0）吸收；离开时(vn>0)不额外推。
    let f_spring = m.penalty_k * pen;
    let f_damp = -c_n * vn.min(0.0);
    let jn = (f_spring + f_damp) * dt;
    let jn = jn.max(0.0); // 接触只能推，不能拉。

    let mut impulse = [0.0f64; 3];
    impulse[0] = n[0] * jn;
    impulse[1] = n[1] * jn;
    impulse[2] = n[2] * jn;

    // 切向（坡面）速度 = 总速度 - 法向分量。
    let vt_x = vel[0] - vn * n[0];
    let vt_y = vel[1] - vn * n[1];
    let vt_z = vel[2] - vn * n[2];
    let speed_t = (vt_x * vt_x + vt_y * vt_y + vt_z * vt_z).sqrt();
    if speed_t > 1e-9 {
        // 库仑摩擦：限定切向冲量预算 = μ·法向力冲量，且不超过 m·|v_t|（停下即止）。
        let budget = m.friction * jn;
        let scale = (budget / (mass * speed_t)).min(1.0);
        impulse[0] -= scale * mass * vt_x;
        impulse[1] -= scale * mass * vt_y;
        impulse[2] -= scale * mass * vt_z;
    }

    world.apply_impulse(id, &impulse, 0);

    ContactInfo {
        touching: true,
        penetration: pen,
        normal_force: f_spring + f_damp,
        friction_impulse: (impulse[0] * impulse[0] + impulse[2] * impulse[2]).sqrt(),
        normal: n,
        point: [pos[0], contact_y, pos[2]],
        impulse,
    }
}

/// 局部接触法向（单位向量，引擎系 Y-up）。
/// - 平地形：`(0,1,0)`。
/// - HeightMap：用中心差分梯度 `(-∂h/∂x, 1, -∂h/∂z)` 归一化。
fn terrain_normal(m: &ContactModel, x: f64, z: f64) -> [f64; 3] {
    match &m.terrain {
        Some(TerrainField::HeightMap { spacing, .. }) if *spacing > 0.0 => {
            let e = *spacing * 0.5; // 差分步长
            let hx1 = m
                .terrain
                .as_ref()
                .map(|t| t.height_at(x + e, z))
                .unwrap_or(0.0);
            let hx0 = m
                .terrain
                .as_ref()
                .map(|t| t.height_at(x - e, z))
                .unwrap_or(0.0);
            let hz1 = m
                .terrain
                .as_ref()
                .map(|t| t.height_at(x, z + e))
                .unwrap_or(0.0);
            let hz0 = m
                .terrain
                .as_ref()
                .map(|t| t.height_at(x, z - e))
                .unwrap_or(0.0);
            let nx = -(hx1 - hx0) / (2.0 * e);
            let nz = -(hz1 - hz0) / (2.0 * e);
            let ny = 1.0;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-12 {
                [nx / len, ny / len, nz / len]
            } else {
                [0.0, 1.0, 0.0]
            }
        }
        _ => [0.0, 1.0, 0.0],
    }
}

// ============================================================ phy-sdk adapter

/// 真实物理引擎的 Rust 适配器：包装 `phy-sdk`（`phy_sdk::World<f64>`）。
///
/// 这是 `RigidBodyWorld` 的生产实现，直接用 safe Rust 调 `phy-sdk` 的强类型 API，
/// 无需 `unsafe`、无 C-ABI 绑定、无运行时 DLL 依赖。物理引擎以 `rlib` 形态被
/// cargo 静态链入仿真二进制（`phy-sdk` 的 cdylib 仍保留给非 Rust 宿主，与此无关）。
#[cfg(feature = "phy")]
pub struct PhySdkWorld {
    world: World<f64>,
    rigid_idx: usize,
    t: f64,
}

#[cfg(feature = "phy")]
impl PhySdkWorld {
    /// 创建空刚体世界（仅启用刚体子系统，无 demo 物体），由仿真层自行添加机体与地面。
    pub fn create_empty() -> Self {
        let world: World<f64> = PhysicsBuilder::new().rigid().build();
        // rigid 是 build 时第一个（也是唯一）声明的子系统，索引 0。
        let mut s = Self { world, rigid_idx: 0, t: 0.0 };
        // 四旋翼仿真必须关闭引擎的"休眠"机制（B1 Sleeping）：
        // 悬停稳定后机体速度趋于 0，引擎会误判为"近静止"并将 body 置 sleeping、
        // 清零速度并冻结位置——这会让本应维持悬停/可坠落的机体被锁死，且 disarm
        // 后无法靠重力坠回。飞控需要世界始终积分。
        //
        // world.step 的休眠判定是"near_rest（速度低于阈值）且 sleep_time>=st 即休眠"。
        // - 不能把 sleep_time 设为 0：初始静止体 sleep_time 本为 0，0>=0 在**第一步**
        //   立即休眠（自由落体零推力场景正是如此，机体起步即冻结）。
        // - 也不能把速度阈值设为无穷大：那会让 near_rest 恒为真，sleep_time 照样累积、
        //   到默认 0.5s 后仍休眠（本 bug 第一次修复就踩中）。
        // 正确做法是把 sleep_time（休眠时长阈值 st）设为无穷大：sleep_time >= ∞ 永不
        // 成立 → 永不休眠。速度阈值保持默认即可。
        if let Some(rw) = get_as_mut::<RigidSubsystem<f64>>(&mut s.world, s.rigid_idx) {
            rw.world.params.sleep_time = f64::INFINITY;
        }
        s
    }

    fn rigid(&self) -> &RigidWorld<f64> {
        get_as(&self.world, self.rigid_idx)
            .map(|s: &RigidSubsystem<f64>| &s.world)
            .expect("rigid 子系统缺失")
    }

    fn rigid_mut(&mut self) -> &mut RigidWorld<f64> {
        get_as_mut(&mut self.world, self.rigid_idx)
            .map(|s: &mut RigidSubsystem<f64>| &mut s.world)
            .expect("rigid 子系统缺失")
    }
}

/// `Body` 的局部逆惯量对角阵（`inv_inertia_local` 为 `Mat3`，这里从主惯量构造）。
#[cfg(feature = "phy")]
fn inv_inertia_mat3(ix: f64, iy: f64, iz: f64) -> phy_math::na::Matrix3<f64> {
    let sx = 1.0 / ix.max(1e-9);
    let sy = 1.0 / iy.max(1e-9);
    let sz = 1.0 / iz.max(1e-9);
    phy_math::na::Matrix3::from_diagonal(&V3::new(sx, sy, sz))
}

#[cfg(feature = "phy")]
impl RigidBodyWorld for PhySdkWorld {
    fn body_count(&self) -> usize {
        self.rigid().bodies.len()
    }

    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64 {
        use phy_math::na::UnitQuaternion;
        let (px, py, pz) = (pos7[0], pos7[1], pos7[2]);
        let (qw, qx, qy, qz) = (pos7[3], pos7[4], pos7[5], pos7[6]);
        // 几何：用 Box 占位（半长 = 机臂长），贴近四旋翼体积；惯量由 inertia3 显式覆盖。
        let arm = (inertia3[0] + inertia3[1] + inertia3[2]).sqrt().max(0.1);
        let shape = phy_rigid::shape::Shape::Box {
            half: V3::new(arm, arm, arm),
        };
        let inv_mass = if mass <= 0.0 { 0.0 } else { 1.0 / mass };
        let mut b = phy_rigid::shape::Body::new(shape, V3::new(px, py, pz), inv_mass);
        b.rot = UnitQuaternion::new_normalize(phy_math::na::Quaternion::new(qw, qx, qy, qz));
        b.inv_inertia_local = inv_inertia_mat3(inertia3[0], inertia3[1], inertia3[2]);
        self.rigid_mut().add_body(b) as i64
    }

    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], _mode: i32) {
        let b = &mut self.rigid_mut().bodies[id as usize];
        b.apply_impulse(V3::new(j3[0], j3[1], j3[2]));
    }

    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], _mode: i32) {
        // 真引擎无"纯力矩"接口：角冲量 = I_world⁻¹ · k，直接注入 ang_vel。
        let b = &mut self.rigid_mut().bodies[id as usize];
        let dw = b.inv_inertia_world() * V3::new(k3[0], k3[1], k3[2]);
        b.ang_vel += dw;
    }

    fn get_velocity(&self, id: i64) -> [f64; 3] {
        let v = self.rigid().bodies[id as usize].vel;
        [v.x, v.y, v.z]
    }

    fn get_angular_velocity(&self, id: i64) -> [f64; 3] {
        let w = self.rigid().bodies[id as usize].ang_vel;
        [w.x, w.y, w.z]
    }

    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize {
        let bodies = &self.rigid().bodies;
        let n = bodies.len();
        let need = n * 7;
        let len = buf.len().min(need);
        for i in 0..len / 7 {
            let b = &bodies[i];
            let q = b.rot.quaternion(); // (w, i, j, k)
            let off = i * 7;
            buf[off] = b.pos.x;
            buf[off + 1] = b.pos.y;
            buf[off + 2] = b.pos.z;
            buf[off + 3] = q.w;
            buf[off + 4] = q.i;
            buf[off + 5] = q.j;
            buf[off + 6] = q.k;
        }
        len
    }

    fn get_body_transform(&self, id: i64, buf: &mut [f64; 7]) {
        let b = &self.rigid().bodies[id as usize];
        let q = b.rot.quaternion(); // (w, i, j, k)
        buf[0] = b.pos.x;
        buf[1] = b.pos.y;
        buf[2] = b.pos.z;
        buf[3] = q.w;
        buf[4] = q.i;
        buf[5] = q.j;
        buf[6] = q.k;
    }

    fn step(&mut self, dt: f64) -> i32 {
        match self.world.step_checked(dt) {
            Ok(()) => {
                self.t += dt;
                0
            }
            Err(_) => -1,
        }
    }

    fn time(&self) -> f64 {
        self.t
    }
}

// ============================================================ 玩具级替身（测试用）

/// 玩具级物理世界：半隐式欧拉刚体积分，固定重力 (0,-g,0)，简单地面碰撞。
///
/// 目的（见阶段 7 设计讨论）：
/// 1. 作为"测试替身"验证 `plant.rs` 的冲量施加逻辑正确，**无需启动真实物理引擎**；
/// 2. 锁住 phase 2/3/4 真实度回归（推力→上升、力矩→角速度、地面效应→近地增益）；
/// 3. 验证 `RigidBodyWorld` 接口的充分性——若替身无法满足仿真需求，说明接口有缺口。
///
/// 简化假设（够用即可，非高保真）：
/// - 刚体无耦合惯量（对角惯量），角速度直接 `ω += I^{-1}·k`。
/// - 姿态四元数用 `q += 0.5·(0,ω)⊗q·dt` 后归一化（足够慢速姿态积分）。
/// - 地面：y <= 0 时位置钳制 y=0、线速度 y 分量置 0（无反弹，模拟停机坪）。
/// - 瞬态冲量：一步消耗，不跨帧残留（遵守 trait 语义约定）。
pub struct ToyWorld {
    mass: Vec<f64>,
    inertia: Vec<[f64; 3]>,
    pos: Vec<[f64; 3]>,
    quat: Vec<[f64; 4]>, // (w,x,y,z)
    vel: Vec<[f64; 3]>,
    ang: Vec<[f64; 3]>,
    gravity: f64,
    time: f64,
}

impl ToyWorld {
    pub fn new(gravity: f64) -> Self {
        Self {
            mass: Vec::new(),
            inertia: Vec::new(),
            pos: Vec::new(),
            quat: Vec::new(),
            vel: Vec::new(),
            ang: Vec::new(),
            gravity,
            time: 0.0,
        }
    }

    fn quat_mul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
        // (w,x,y,z) Hamilton 积。
        let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
        let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    }

    fn quat_norm(q: &mut [f64; 4]) {
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        if n > 1e-12 {
            for i in 0..4 {
                q[i] /= n;
            }
        }
    }

    /// 用单位四元数 `q`（w,x,y,z）把**机体**向量旋到**世界**系（与 `plant.rs::rotate_by_quat` 同义）。
    fn quat_rot_vec(q: &[f64; 4], v: &[f64; 3]) -> [f64; 3] {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let r00 = 1.0 - 2.0 * (y * y + z * z);
        let r01 = 2.0 * (x * y - w * z);
        let r02 = 2.0 * (x * z + w * y);
        let r10 = 2.0 * (x * y + w * z);
        let r11 = 1.0 - 2.0 * (x * x + z * z);
        let r12 = 2.0 * (y * z - w * x);
        let r20 = 2.0 * (x * z - w * y);
        let r21 = 2.0 * (y * z + w * x);
        let r22 = 1.0 - 2.0 * (x * x + y * y);
        [
            r00 * v[0] + r01 * v[1] + r02 * v[2],
            r10 * v[0] + r11 * v[1] + r12 * v[2],
            r20 * v[0] + r21 * v[1] + r22 * v[2],
        ]
    }

    /// 用单位四元数 `q`（w,x,y,z）把**世界**向量旋到**机体**系（与 `plant.rs::rotate_by_quat_conj` 同义）。
    fn quat_rot_vec_conj(q: &[f64; 4], v: &[f64; 3]) -> [f64; 3] {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let r00 = 1.0 - 2.0 * (y * y + z * z);
        let r01 = 2.0 * (x * y + w * z);
        let r02 = 2.0 * (x * z - w * y);
        let r10 = 2.0 * (x * y - w * z);
        let r11 = 1.0 - 2.0 * (x * x + z * z);
        let r12 = 2.0 * (y * z + w * x);
        let r20 = 2.0 * (x * z + w * y);
        let r21 = 2.0 * (y * z - w * x);
        let r22 = 1.0 - 2.0 * (x * x + y * y);
        [
            r00 * v[0] + r01 * v[1] + r02 * v[2],
            r10 * v[0] + r11 * v[1] + r12 * v[2],
            r20 * v[0] + r21 * v[1] + r22 * v[2],
        ]
    }
}

impl RigidBodyWorld for ToyWorld {
    fn body_count(&self) -> usize {
        self.mass.len()
    }

    fn add_body(&mut self, mass: f64, pos7: &RigidTransform, inertia3: &[f64; 3]) -> i64 {
        let id = self.mass.len() as i64;
        self.mass.push(mass);
        self.inertia.push(*inertia3);
        self.pos.push([pos7[0], pos7[1], pos7[2]]);
        self.quat.push([pos7[3], pos7[4], pos7[5], pos7[6]]);
        self.vel.push([0.0; 3]);
        self.ang.push([0.0; 3]);
        id
    }

    fn apply_impulse(&mut self, id: i64, j3: &[f64; 3], _mode: i32) {
        let i = id as usize;
        let m = self.mass[i];
        if m <= 0.0 {
            return; // 静态刚体不受冲量。
        }
        // 线冲量直接是速度增量：Δv = j / m。
        for k in 0..3 {
            self.vel[i][k] += j3[k] / m;
        }
    }

    fn apply_torque_impulse(&mut self, id: i64, k3: &[f64; 3], _mode: i32) {
        let i = id as usize;
        let m = self.mass[i];
        if m <= 0.0 {
            return;
        }
        // 世界系刚体动力学约定（与 PhySdkWorld 一致）：`ang` 存储【世界系】角速度，
        // 角增量 = I_world⁻¹ · k_world。对对角机体惯量 I_body，世界系逆惯量作用为：
        //   先把世界力矩 k_world 旋到机体系 -> 乘以机体逆惯量 -> 再旋回世界系。
        let inv_i = [
            1.0 / self.inertia[i][0].max(1e-9),
            1.0 / self.inertia[i][1].max(1e-9),
            1.0 / self.inertia[i][2].max(1e-9),
        ];
        let k_body = Self::quat_rot_vec_conj(&self.quat[i], k3);
        let dw_body = [k_body[0] * inv_i[0], k_body[1] * inv_i[1], k_body[2] * inv_i[2]];
        let dw_world = Self::quat_rot_vec(&self.quat[i], &dw_body);
        for k in 0..3 {
            self.ang[i][k] += dw_world[k];
        }
    }

    fn get_velocity(&self, id: i64) -> [f64; 3] {
        self.vel[id as usize]
    }

    fn get_angular_velocity(&self, id: i64) -> [f64; 3] {
        self.ang[id as usize]
    }

    fn get_rigid_transforms(&self, buf: &mut [f64]) -> usize {
        let n = self.mass.len();
        let need = n * 7;
        let len = buf.len().min(need);
        for i in 0..len / 7 {
            let off = i * 7;
            buf[off..off + 3].copy_from_slice(&self.pos[i]);
            buf[off + 3..off + 7].copy_from_slice(&self.quat[i]);
        }
        len
    }

    fn get_body_transform(&self, id: i64, buf: &mut [f64; 7]) {
        let i = id as usize;
        buf[0..3].copy_from_slice(&self.pos[i]);
        buf[3..7].copy_from_slice(&self.quat[i]);
    }

    fn step(&mut self, dt: f64) -> i32 {
        for i in 0..self.mass.len() {
            let m = self.mass[i];
            if m <= 0.0 {
                // 静态刚体：不动。
                continue;
            }
            // ---- 线运动：半隐式欧拉（重力 + 已注入的速度）----
            self.vel[i][1] -= self.gravity * dt; // 重力（世界 -Y）
            for k in 0..3 {
                self.pos[i][k] += self.vel[i][k] * dt;
            }

            // 注：地面接触统一由 `plant.rs::resolve_ground_contact`（contact=Some 时）处理，
            // 此处不再硬编码 y>=0 地板。这样与 PhySdkWorld（仅依赖接触模型、contact=None
            // 即真空无地板）行为一致；否则 contact=None 场景下 ToyWorld 多一块地板会导致
            // 两引擎在坠落/触地表现上不一致。

            // ---- 姿态积分（世界系约定，与 PhySdkWorld 一致）----
            // `ang` 为【世界系】角速度 ω。世界系四元数导数：q̇ = 0.5 · (0,ω) ⊗ q。
            // 稳态悬停时控制器力矩 ≈ 0，ω ≈ 0，姿态保持初始 90° 翻转（推力竖直向上）。
            let omega = self.ang[i];
            let dq = Self::quat_mul(&[0.0, omega[0], omega[1], omega[2]], &self.quat[i]);
            for k in 0..4 {
                self.quat[i][k] += 0.5 * dq[k] * dt;
            }
            Self::quat_norm(&mut self.quat[i]);
        }
        self.time += dt;
        0
    }

    fn time(&self) -> f64 {
        self.time
    }
}
