//! P3-D5 多机互飞 / 机间通信场景。
//!
//! 多实例 `FlyController` 共享同一个物理世界（`Rc<RefCell<W>>`，见 `physics.rs` 的
//! 共享世界包装），各自以 `body_id` 独立操作刚体。`MultiDroneSim` 负责：
//! - **世界统一推进**：每帧各机只注入冲量（`external_world_step=true`），再对共享世界
//!   只 `step(dt)` 一次——避免 N 架机各步一次导致世界时间膨胀 N 倍；
//! - **机间碰撞**：用各机 `body_id` 组装 `BodyCollider`，j>i 单向注册 peer（同一对
//!   碰撞只由较小 id 的机体解算一次，解算对双方等大反向施力 → 动量守恒、不翻倍）；
//! - **机间通信**：`DroneLink` 内存邮箱模拟 ADS-B / 机间 UDP 链路，每帧广播各机真值
//!   遥测（位置/速度），下一帧各机读取邻居遥测做编队/跟随/避让决策（含一帧链路延迟）；
//! - **场景辅助**：编队偏移设定点、机间避让速度指令等构造函数，供验收测试驱动。

use std::cell::RefCell;
use std::rc::Rc;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::vehicle::VehicleState;

use crate::controller::{ControllerKind, FlyController, hover_setpoint};
use crate::physics::{BodyCollider, ContactModel, RigidBodyWorld};
use crate::sensor::SensorConfig;

/// 机间遥测报文（模拟 ADS-B / 机间链路）：共享目标状态。
///
/// 只含机间协同所需的最少信息（ID + NED 位置 + NED 速度），代表"经机间链路广播的
/// 邻居状态"，与完整 `VehicleState` 区分：遥测是**传输形态**（可加噪声/延迟/丢包），
/// 真值状态是**物理事实**。
#[derive(Clone, Copy, Debug)]
pub struct DroneTelemetry {
    /// 机体 id（机间链路标识）。
    pub id: u8,
    /// NED 位置 [n, e, d]（m，d 向下为正）。
    pub pos: [f32; 3],
    /// NED 速度 [vn, ve, vd]（m/s）。
    pub vel: [f32; 3],
    /// 链路时间戳（s，本实现为仿真时间）。
    pub t: f64,
}

impl DroneTelemetry {
    pub fn zero() -> Self {
        Self {
            id: 0,
            pos: [0.0; 3],
            vel: [0.0; 3],
            t: 0.0,
        }
    }
}

/// 把 NED 真值状态转成机间遥测报文（`id` 由调用方指定）。
pub fn telemetry_from_state(id: u8, s: &VehicleState) -> DroneTelemetry {
    DroneTelemetry {
        id,
        pos: [s.pos[0].0, s.pos[1].0, s.pos[2].0],
        vel: [s.vel[0].0, s.vel[1].0, s.vel[2].0],
        t: 0.0,
    }
}

/// 机间通信链路（内存实现）：每帧由 `MultiDroneSim` 广播各机真值遥测，各机下一帧
/// 读取邻居遥测（模拟 ADS-B 周期广播 + 本地接收缓存，带一帧延迟）。
#[derive(Clone, Debug)]
pub struct DroneLink {
    /// 每节点接收到的邻居遥测（按节点 id 索引；不包含自身）。
    rx: Vec<Vec<DroneTelemetry>>,
}

impl DroneLink {
    pub fn new(n: usize) -> Self {
        Self { rx: vec![Vec::new(); n] }
    }

    /// 广播：把 `telem` 写入每个节点（除自身）的接收缓存，供下一帧读取。
    fn broadcast(&mut self, telem: &[DroneTelemetry]) {
        for i in 0..self.rx.len() {
            self.rx[i] = telem
                .iter()
                .filter(|t| (t.id as usize) != i)
                .copied()
                .collect();
        }
    }

    /// 节点 `id` 当前可读取的邻居遥测（上一帧广播缓存）。
    pub fn neighbors(&self, id: usize) -> &[DroneTelemetry] {
        &self.rx[id]
    }
}

/// 多机场景管理器：N 架 `FlyController` 共世界 + 机间通信链路。
pub struct MultiDroneSim<W> {
    world: Rc<RefCell<W>>,
    drones: Vec<FlyController<Rc<RefCell<W>>>>,
    link: DroneLink,
    dt: f64,
    time: f64,
}

impl<W: RigidBodyWorld> MultiDroneSim<W> {
    /// 创建多机场景。
    ///
    /// - `world`：初始物理世界（多机将共享它）。
    /// - `cfgs`：每架机的机型配置（长度 = 机数）。
    /// - `init_pos`：每架机初始 NED 位置 [n,e,d]（m），长度与 `cfgs` 一致。
    /// - `dt`：控制/仿真周期（s）。
    ///
    /// 内部完成：共享世界包装 → 逐机 `new_at` 建刚体 → 机间碰撞 j>i 单向注册 →
    /// 切外部世界步进 → 初始遥测广播（帧 0 即有邻居数据）。
    pub fn new(world: W, cfgs: &[VehicleConfig], init_pos: &[[f32; 3]], dt: f64) -> Self {
        assert_eq!(cfgs.len(), init_pos.len(), "cfgs 与 init_pos 长度必须一致");
        assert!(!cfgs.is_empty(), "至少需要一架机");
        let world = Rc::new(RefCell::new(world));
        let n = cfgs.len();

        let mut drones = Vec::with_capacity(n);
        let mut ids = Vec::with_capacity(n);
        for i in 0..n {
            let ctrl = FlyController::new_at(
                Rc::clone(&world),
                &cfgs[i],
                dt,
                None, // 多机场景默认无风（避让/编队判定不受风扰）
                SensorConfig::default(),
                ControllerKind::Pid,
                Some(ContactModel::default()),
                Vec::new(),
                init_pos[i],
            );
            ids.push(ctrl.body_id());
            drones.push(ctrl);
        }

        // 机间碰撞体（质量/碰撞球半径与 plant 的 peer 解算口径一致：1.2×臂长）。
        let colliders: Vec<BodyCollider> = (0..n)
            .map(|i| BodyCollider {
                id: ids[i],
                mass: cfgs[i].mass as f64,
                radius: 1.2 * cfgs[i].arm_length as f64,
            })
            .collect();
        // j>i 单向注册：每对 (i,j) 只由较小 id 的机体解算一次。`resolve_body_peer_collisions`
        // 对 self 与 peer 都等大反向施力，故单向注册即覆盖整对，且不会同对碰撞被解算两次
        // （双机各算一次 → 冲量翻倍）。最后一架机无 peer（被动接收碰撞冲量）。
        for i in 0..n {
            let peers: Vec<BodyCollider> = colliders[i + 1..].to_vec();
            drones[i].plant_set_peer_colliders(peers);
            drones[i].set_external_world_step(true);
        }

        let mut sim = Self {
            world,
            drones,
            link: DroneLink::new(n),
            dt,
            time: 0.0,
        };
        // 帧 0 即有初始邻居遥测（初始位置 + 零速度），供第一帧编队/避让决策。
        sim.broadcast_telemetry(&init_pos.iter().copied().collect::<Vec<_>>());
        sim
    }

    /// 用给定初始 NED 位置列表广播一次遥测（初始帧用；`vel` 视为 0）。
    fn broadcast_telemetry(&mut self, init_pos: &[[f32; 3]]) {
        let telem: Vec<DroneTelemetry> = init_pos
            .iter()
            .enumerate()
            .map(|(i, p)| DroneTelemetry {
                id: i as u8,
                pos: *p,
                vel: [0.0; 3],
                t: 0.0,
            })
            .collect();
        self.link.broadcast(&telem);
    }

    /// 步进一帧：各机按 `setpoints[i]` 闭环（外部世界步进模式，只注入冲量），
    /// 然后共享世界统一推进一次，最后广播本帧真值遥测（供下一帧机间决策）。
    ///
    /// 返回各机控制律输出的估计状态（与 `world_state` 真值略有差异）。
    pub fn step(&mut self, setpoints: &[Setpoint]) -> Vec<VehicleState> {
        assert_eq!(setpoints.len(), self.drones.len(), "setpoints 数量必须与机数一致");
        let mut states = Vec::with_capacity(self.drones.len());
        for (d, sp) in self.drones.iter_mut().zip(setpoints.iter()) {
            states.push(d.step(sp));
        }
        // 共享世界统一步进一次（外部世界步进模式下各 plant 已跳过 world.step）。
        let rc = self.world.borrow_mut().step(self.dt);
        assert_eq!(rc, 0, "共享世界 step 检测到 NaN/Inf，世界已损坏");
        self.time += self.dt;

        // 广播本帧真值遥测（下一帧机间决策使用）。
        let mut telem = Vec::with_capacity(self.drones.len());
        for (i, d) in self.drones.iter().enumerate() {
            let s = d.world_state();
            telem.push(DroneTelemetry {
                id: i as u8,
                pos: [s.pos[0].0, s.pos[1].0, s.pos[2].0],
                vel: [s.vel[0].0, s.vel[1].0, s.vel[2].0],
                t: self.time,
            });
        }
        self.link.broadcast(&telem);
        states
    }

    /// 机数。
    pub fn len(&self) -> usize {
        self.drones.len()
    }

    pub fn is_empty(&self) -> bool {
        self.drones.is_empty()
    }

    /// 仿真已推进时间（s）。
    pub fn time(&self) -> f64 {
        self.time
    }

    /// 节点 `id` 经链路收到的邻居遥测（上一帧广播缓存，本帧机间决策依据）。
    pub fn neighbors(&self, id: usize) -> &[DroneTelemetry] {
        self.link.neighbors(id)
    }

    /// 节点 `id` 的 NED 真值状态（引擎世界系真值 → NED）。
    pub fn world_state(&self, id: usize) -> VehicleState {
        self.drones[id].world_state()
    }

    /// 节点 `id` 的引擎世界系真值位姿 `(pos, quat)`（绕开 NED 映射，供坐标诊断）。
    pub fn debug_up(&self, id: usize) -> ([f64; 3], [f64; 4]) {
        self.drones[id].debug_up()
    }

    /// 节点 `id` 的直接控制访问（动态改装避障/传感器/风场等）。
    pub fn drone(&mut self, id: usize) -> &mut FlyController<Rc<RefCell<W>>> {
        &mut self.drones[id]
    }
}

// ============================================================ 编队 / 跟随 / 避让辅助

/// 编队设定点：`follower` 相对 `leader` 保持水平偏移 `offset_ne`（[n,e] 偏移，m），
/// 高度由 `alt_d` 指定。
///
/// 用 leader 的机间遥测位置 + 偏移构造 follower 期望位置——队形随 leader 真值平移，
/// 是最简"跟随"形式（Leader-Follower 编队）。`leader` 遥测经 `MultiDroneSim::neighbors`
/// 获取（带一帧链路延迟，真实反映机间通信时序）。
///
/// **速度前馈**：水平位置环是 PD（无积分，`des_v = kp_xy·e + sp.vel`），跟随匀速目标时
/// 稳态滞后 = v/kp_xy（kp_xy=0.3 时 2 m/s → ~6.7 m）。故把 leader 速度叠加到设定点速度，
/// 消除随动滞后：leader 匀速时 follower 零误差跟踪；leader 减速/停止时靠同一前馈同步减速。
pub fn formation_setpoint(leader: &DroneTelemetry, offset_ne: [f32; 2], alt_d: f32) -> Setpoint {
    let mut sp = hover_setpoint(
        leader.pos[0] + offset_ne[0],
        leader.pos[1] + offset_ne[1],
        alt_d,
    );
    sp.vel[0].0 += leader.vel[0];
    sp.vel[1].0 += leader.vel[1];
    sp
}

/// 机间避让速度指令（NED 水平，m/s）：对每个进入 `danger` 半径的邻居，叠加
/// **制动 + 横向避让** 两个分量（P3-D5 机间避让 / 模拟 ADS-B 冲突解脱）。
///
/// 纯径向排斥在对头（相向）冲突上是退化的——排斥方向与飞行方向同轴，反而加剧
/// 接近。故这里对每个在 `danger` 内的邻居：
/// 1. **制动**：抵消沿连线方向的接近相对速度（`closing<0` 时反向推回）；
/// 2. **横向**：沿连线垂向施加固定手性的偏航分量（双方错开不同侧，实现真正侧向让行）。
///
/// 强度因子 `sev = danger/dist - 1`（`dist=danger` 时 0，`dist→0` 时发散，越近越强）。
/// - `brake_k`：制动增益（对接近速度）；`lat_k`：横向增益。返回值叠加到设定点速度上。
pub fn inter_drone_avoid_vel(
    me: &DroneTelemetry,
    neighbors: &[DroneTelemetry],
    danger: f32,
    brake_k: f32,
    lat_k: f32,
) -> [f32; 3] {
    let mut out = [0.0f32; 3];
    for n in neighbors {
        let r = [n.pos[0] - me.pos[0], n.pos[1] - me.pos[1]];
        let dist = (r[0] * r[0] + r[1] * r[1]).sqrt();
        if dist >= danger || dist < 1e-6 {
            continue;
        }
        let rhat = [r[0] / dist, r[1] / dist];
        let sev = danger / dist - 1.0; // 0 at danger，越近越强
        // 1) 制动：抵消接近相对速度（closing<0 表示接近，反向推回）。
        let dv = [n.vel[0] - me.vel[0], n.vel[1] - me.vel[1]];
        let closing = dv[0] * rhat[0] + dv[1] * rhat[1];
        if closing < 0.0 {
            let mag = -closing * brake_k * sev;
            out[0] -= rhat[0] * mag;
            out[1] -= rhat[1] * mag;
        }
        // 2) 横向：沿连线垂向（固定手性），双方错开不同侧。
        let perp = [-rhat[1], rhat[0]];
        let sw = sev * lat_k;
        out[0] += perp[0] * sw;
        out[1] += perp[1] * sw;
    }
    out
}

/// 构造"位置目标 + 机间避让速度"设定点：以 `target` 为位置目标（高度保持），
/// 叠加 `inter_drone_avoid_vel` 的避让速度，供对头/追尾冲突场景使用。
pub fn target_with_avoid(
    target: [f32; 3],
    me: &DroneTelemetry,
    neighbors: &[DroneTelemetry],
    danger: f32,
    brake_k: f32,
    lat_k: f32,
) -> Setpoint {
    let av = inter_drone_avoid_vel(me, neighbors, danger, brake_k, lat_k);
    let mut sp = hover_setpoint(target[0], target[1], target[2]);
    sp.vel[0].0 += av[0];
    sp.vel[1].0 += av[1];
    sp.vel[2].0 += av[2];
    sp
}
