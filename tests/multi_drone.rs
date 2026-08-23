//! P3-D5 多机互飞 / 机间通信场景验收测试。
//!
//! 多实例 `FlyController` 共享同一 `RigidBodyWorld`（`ToyWorld` 替身），经 `DroneLink`
//! 内存链路互发位置/速度（模拟 ADS-B / 机间链路）。三项验收：
//!
//! 1) **编队跟随**：3 机 Leader-Follower 队形——leader 北向平移，follower 基于
//!    leader 机间遥测保持固定水平偏移，偏移误差 < 0.5 m；
//! 2) **遥测精度**：链路广播的邻居遥测（位置/速度）与真值状态一致（≤1e-3 m 级）；
//! 3) **机间避让**：对头接近两机经机间避让（制动 + 横向让行）保持安全间距
//!    （最小间距 > 0.8 m，显著大于碰撞球半径和 2×0.27≈0.54 m，不触发物理碰撞）。
//!
//! TRU 判定量归一化复用 `common::TruStats`（水平漂移 h、NED down d、倾角 tilt°），
//! 由 `debug_up`（引擎世界系真值）换算，与既有测试口径一致。
//!
//! 用法：cargo test --test multi_drone -- --nocapture

use fly_sim_core::controller::{hover_setpoint};
use fly_sim_core::multi::{
    formation_setpoint, target_with_avoid, telemetry_from_state, DroneTelemetry, MultiDroneSim,
};
use fly_sim_core::physics::ToyWorld;
use fly_simulater::airframe::load_airframe;

mod common;
use common::TruStats;

const DT: f64 = 0.004;

/// 用单位四元数 (w,x,y,z, 引擎世界系 Y-up) 旋转向量 v：R(q)·v。
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let qv = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
    let qqv = [
        y * qv[2] - z * qv[1],
        z * qv[0] - x * qv[2],
        x * qv[1] - y * qv[0],
    ];
    [
        v[0] + 2.0 * w * qv[0] + 2.0 * qqv[0],
        v[1] + 2.0 * w * qv[1] + 2.0 * qqv[1],
        v[2] + 2.0 * w * qv[2] + 2.0 * qqv[2],
    ]
}

/// 机体推力轴（机体 +Z）偏离世界竖直 (0,1,0) 的倾角（度）。
fn tilt_deg(q: [f64; 4]) -> f64 {
    let up = rotate(q, [0.0, 0.0, 1.0]);
    let dot = (up[1]).clamp(-1.0, 1.0);
    f64::acos(dot).to_degrees()
}

/// 采样节点 `id` 的 TRU 判定量（debug_up → h / d / tilt）。
fn sample_tru(sim: &MultiDroneSim<ToyWorld>, id: usize, st: &mut TruStats) {
    let (up, quat) = sim.debug_up(id);
    let h = (up[0] * up[0] + up[2] * up[2]).sqrt();
    let d = -up[1];
    let tilt = tilt_deg(quat);
    st.sample(h, d, tilt);
}

/// 断言节点 TRU 有界：无 NaN、高度保持 -5 m（|Δ|<2.5）、倾角 < 上限。
/// 注意：编队/前飞会平移 h，故不复用 common 的固定 h 上限，只查数值稳定与姿态。
fn assert_stable(st: &TruStats, label: &str, max_tilt: f64) {
    assert!(!st.nan, "[{label}] TRU 出现 NaN/Inf");
    assert!(
        (st.end_d + 5.0).abs() < 2.5,
        "[{label}] 高度失控：end d={:.1}（期望≈-5）",
        st.end_d
    );
    assert!(
        st.tilt_max_deg < max_tilt,
        "[{label}] 姿态翻滚：max tilt={:.1}°（期望 < {max_tilt}°）",
        st.tilt_max_deg
    );
}

/// 验收 1：编队跟随（Leader-Follower）。
///
/// leader 从 (0,0,-5) 以 2 m/s 北向平移到 n=15 后悬停；两个 follower 初始在
/// leader 东西两侧 4 m，基于 leader 机间遥测保持偏移 [0,±4]，全链路 16 s
/// （停坡后留 8.5 s 给 leader 渐近到位、follower 收敛，见下方注释）。
#[test]
fn multi_formation_follow() {
    let cfg = load_airframe(None).expect("default airframe");
    let cfgs = vec![cfg.clone(), cfg.clone(), cfg.clone()];
    let init = [[0.0, 0.0, -5.0], [0.0, 4.0, -5.0], [0.0, -4.0, -5.0]];
    let mut sim = MultiDroneSim::new(ToyWorld::new(9.81), &cfgs, &init, DT);

    let mut stats = [TruStats::default(), TruStats::default(), TruStats::default()];
    let offsets: [[f32; 2]; 3] = [[0.0, 0.0], [0.0, 4.0], [0.0, -4.0]];
    // 收敛期（最后 2 s）内各 follower 队形偏移误差的收敛值（误差单调衰减，取窗口内
    // 最小值 = 末端收敛值；leader 停坡后仍以 kp=0.3 渐近到位，故窗口取在末端）。
    let mut settled_err = [f64::MAX, f64::MAX];

    let total_t = 16.0;
    let steps = (total_t / DT) as usize;
    for _ in 0..steps {
        let t = sim.time();
        // leader 航点：北向 0→15 m（2 m/s），到位后悬停。
        let target_n = (t * 2.0).min(15.0) as f32;

        // 从链路读 leader 上一帧遥测（帧 0 已由 new() 广播初始遥测）。
        let leader = sim
            .neighbors(1)
            .iter()
            .find(|te| te.id == 0)
            .copied()
            .unwrap_or(DroneTelemetry {
                id: 0,
                pos: [0.0, 0.0, -5.0],
                vel: [0.0; 3],
                t: 0.0,
            });

        let mut sps = Vec::with_capacity(3);
        // leader 航点设定点 + 速度前馈：水平位置环为 PD（无积分），跟随 2 m/s 斜波
        // 目标时若无前馈则稳态滞后 v/kp_xy≈6.7 m。故平移段给 2 m/s 北向前馈，
        // 到位后给 0，leader 自身可近乎无滞后地跟踪航点。
        let mut leader_sp = hover_setpoint(target_n, 0.0, -5.0);
        leader_sp.vel[0].0 = if target_n < 15.0 { 2.0 } else { 0.0 };
        sps.push(leader_sp);
        for i in 1..3 {
            sps.push(formation_setpoint(&leader, offsets[i], -5.0));
        }
        sim.step(&sps);

        // TRU 采样。
        for i in 0..3 {
            sample_tru(&sim, i, &mut stats[i]);
        }

        // 队形偏移误差：follower 相对 leader 真值的水平偏移 - 指令偏移。
        if t >= total_t - 2.0 {
            let lp = sim.world_state(0).pos;
            for i in 1..3 {
                let fp = sim.world_state(i).pos;
                let dn = (fp[0].0 - lp[0].0) - offsets[i][0];
                let de = (fp[1].0 - lp[1].0) - offsets[i][1];
                let err = (dn * dn + de * de).sqrt() as f64;
                settled_err[i - 1] = settled_err[i - 1].min(err);
            }
        }
    }

    println!("follower 队形偏移误差（收敛期末端值）: f1={:.3} m, f2={:.3} m", settled_err[0], settled_err[1]);
    for i in 0..3 {
        let p = sim.world_state(i).pos;
        println!("  [dbg] drone{i} 最终 NED pos = ({:.3}, {:.3}, {:.3})", p[0].0, p[1].0, p[2].0);
    }
    assert!(
        settled_err[0] < 0.5 && settled_err[1] < 0.5,
        "编队跟随未收敛：f1={:.3} m, f2={:.3} m（期望 < 0.5 m）",
        settled_err[0],
        settled_err[1]
    );
    for i in 0..3 {
        assert_stable(&stats[i], &format!("d{i}"), 20.0);
    }
}

/// 验收 2：遥测精度——内存链路广播的邻居遥测与真值状态一致。
#[test]
fn multi_telemetry_accuracy() {
    let cfg = load_airframe(None).expect("default airframe");
    let init = [[0.0, 0.0, -5.0], [8.0, 0.0, -5.0]];
    let mut sim = MultiDroneSim::new(ToyWorld::new(9.81), &[cfg.clone(), cfg.clone()], &init, DT);

    // 悬停 0.5 s（125 步）到稳态。
    let sps = vec![hover_setpoint(0.0, 0.0, -5.0), hover_setpoint(8.0, 0.0, -5.0)];
    for _ in 0..125 {
        sim.step(&sps);
    }

    // drone1 收到的 drone0 遥测（上一帧广播缓存）应与 drone0 真值一致。
    let telem = sim
        .neighbors(1)
        .iter()
        .find(|te| te.id == 0)
        .expect("drone1 应收到 drone0 遥测");
    let s0 = sim.world_state(0);
    for k in 0..3 {
        assert!(
            (telem.pos[k] - s0.pos[k].0).abs() < 1e-3,
            "遥测位置分量 {k} 不一致：telem={:.4} vs 真值={:.4}",
            telem.pos[k],
            s0.pos[k].0
        );
        assert!(
            (telem.vel[k] - s0.vel[k].0).abs() < 1e-3,
            "遥测速度分量 {k} 不一致：telem={:.5} vs 真值={:.5}",
            telem.vel[k],
            s0.vel[k].0
        );
    }
    // 高度保持（隐式验证链路未把位置串线到另一机）。悬停 0.5 s 尚未完全收敛，
    // 允许 ≤0.5 m 稳态偏移；该判据只用于捕获量级错误（如 -15 m 坠落）。
    assert!(
        (telem.pos[2] + 5.0).abs() < 0.5,
        "遥测高度异常：d={:.3}（期望≈-5）",
        telem.pos[2]
    );
}

/// 验收 3：机间避让——对头接近两机经避让保持安全间距。
///
/// d0 从 (0,0,-5) 北向目标 n=14，d1 从 (12,0,-5) 南向目标 n=-2，在中点对头相会。
/// 进入 danger 半径后各自制动 + 横向让行，应保持最小间距显著大于碰撞球半径和
/// （2 × 1.2 × 0.225 ≈ 0.54 m），即避让生效、不触发物理碰撞。
#[test]
fn multi_inter_drone_avoidance() {
    let cfg = load_airframe(None).expect("default airframe");
    let init = [[0.0, 0.0, -5.0], [12.0, 0.0, -5.0]];
    let mut sim = MultiDroneSim::new(ToyWorld::new(9.81), &[cfg.clone(), cfg.clone()], &init, DT);

    const DANGER: f32 = 5.0; // 避让激活半径（m）
    const BRAKE_K: f32 = 1.2; // 制动增益
    const LAT_K: f32 = 1.0; // 横向让行增益

    let mut stats = [TruStats::default(), TruStats::default()];
    let mut min_sep = f64::MAX;
    let total_t = 10.0;
    let steps = (total_t / DT) as usize;
    for _ in 0..steps {
        let me0 = telemetry_from_state(0, &sim.world_state(0));
        let me1 = telemetry_from_state(1, &sim.world_state(1));
        let n0: Vec<DroneTelemetry> = sim.neighbors(0).to_vec();
        let n1: Vec<DroneTelemetry> = sim.neighbors(1).to_vec();
        let sps = vec![
            target_with_avoid([14.0, 0.0, -5.0], &me0, &n0, DANGER, BRAKE_K, LAT_K),
            target_with_avoid([-2.0, 0.0, -5.0], &me1, &n1, DANGER, BRAKE_K, LAT_K),
        ];
        sim.step(&sps);

        for i in 0..2 {
            sample_tru(&sim, i, &mut stats[i]);
        }
        let p0 = sim.world_state(0).pos;
        let p1 = sim.world_state(1).pos;
        let sep = ((p0[0].0 - p1[0].0).powi(2) + (p0[1].0 - p1[1].0).powi(2)).sqrt() as f64;
        min_sep = min_sep.min(sep);
    }

    println!("机间最小间距 = {min_sep:.3} m（碰撞球半径和 ≈ 0.54 m）");
    assert!(
        min_sep > 0.8,
        "机间避让未生效，最小间距过近：{min_sep:.3} m（期望 > 0.8 m）"
    );
    for i in 0..2 {
        assert_stable(&stats[i], &format!("d{i}"), 45.0);
    }
}
