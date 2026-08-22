//! P3-D4：蒙特卡洛基准数据集（统计 + 真值回放）。
//!
//! 批量跑 N 次同场景（传感器噪声/湍流风随机种子逐次改变），从**物理真值**统计
//! 轨迹分布（均值±σ、P95 包络、极值）与收敛率/失效率，并把每个标准场景的代表性
//! 真值轨迹（seed=0 确定性复现）写为 CSV 回放文件（`target/bench/`，git 忽略）。
//!
//! 标准基准场景集（闭环保真，`ToyWorld` 替身 + realistic 传感器噪声）：
//!   - `hover`       无风悬停（PID，10s）
//!   - `wind`        2 m/s 逆风悬停（PID，8s）
//!   - `cruise`      80m 北向巡航 vmax=5（TECS vs PID，14s）—— 回归对比 + 统计显著性
//!
//! 用法：
//!   cargo test --test monte_carlo -- --nocapture            # 默认 N=8 次/场景（快）
//!   $env:MC_RUNS=64; cargo test --test monte_carlo -- --nocapture   # 真实统计 N=64
//!
//! 定位：打印型诊断 + 统计验收（替代单次确定性断言，见 FIDELITY_ROADMAP P3-D4）。
//! 统计判据（防回归）：每场景收敛率 ≥ 75%、全程无 NaN、TECS 巡航能量误差均值/P95
//! 显著优于 PID。用 `MC_RUNS` 放大样本即可支撑正式统计显著性分析。

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::wind::{WindConfig, WindField};
use fly_simulater::airframe::load_airframe;
use flyctrl_core::controller::Setpoint;

const DT: f64 = 0.004;

/// 每次场景默认跑 N 次（可被 `MC_RUNS` 环境变量覆盖，正式统计建议 ≥64）。
fn mc_runs() -> usize {
    std::env::var("MC_RUNS")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(8)
}

/// 机体推力轴（机体 +Z）偏离世界竖直 (0,1,0) 的倾角（度）。
fn tilt_deg(q: [f64; 4]) -> f64 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let v = [0.0f64, 0.0, 1.0];
    let qv = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
    let qqv = [
        y * qv[2] - z * qv[1],
        z * qv[0] - x * qv[2],
        x * qv[1] - y * qv[0],
    ];
    let up = [
        v[0] + 2.0 * w * qv[0] + 2.0 * qqv[0],
        v[1] + 2.0 * w * qv[1] + 2.0 * qqv[1],
        v[2] + 2.0 * w * qv[2] + 2.0 * qqv[2],
    ];
    let dot = up[1].clamp(-1.0, 1.0);
    f64::acos(dot).to_degrees()
}

/// 单次运行的 TRU 真值汇总（与 `common::TruStats` 同语义，另加能量高度误差）。
#[derive(Clone, Copy, Debug)]
struct RunMetric {
    finite: bool,
    /// 相对原点最大水平漂移（m）。
    h_max: f64,
    /// 结束 NED down（m）。
    end_d: f64,
    /// 姿态偏离水平最大值（°）。
    tilt_max_deg: f64,
    /// 最大 |能量高度误差|（巡航：TECS/PID 同口径）。
    max_e_eq: f64,
    /// 结束能量高度误差。
    end_e_eq: f64,
    /// 收敛 = 有限 && |end_d|<10 && 水平漂移 <15 && 不翻滚（<45°）。
    converged: bool,
}

/// 单帧真值（CSV 回放用）。
struct TruthFrame {
    t: f64,
    pos: [f64; 3], // NED
    vel: [f64; 3], // NED
    tilt_deg: f64,
    e_eq: f64,
}

/// 简单的均值/标准差/P95 汇总。
struct Stats {
    mean: f64,
    std: f64,
    p95: f64,
    min: f64,
    max: f64,
}

impl Stats {
    fn from(vals: &[f64]) -> Stats {
        let n = vals.len();
        let mean = vals.iter().sum::<f64>() / n as f64;
        let var = vals.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / n as f64;
        let mut s = vals.to_vec();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p95 = s[((n as f64 * 0.95).ceil() as usize - 1).min(n - 1)];
        Stats {
            mean,
            std: var.sqrt(),
            p95,
            min: s[0],
            max: s[n - 1],
        }
    }
}

/// 一次场景的蒙特卡洛汇总。
struct Agg {
    n: usize,
    finite: usize,
    converged: usize,
    h_max: Stats,
    tilt_max_deg: Stats,
    end_d: Stats,
    max_e_eq: Stats,
    end_e_eq: Stats,
}

/// 场景 + 种子 → 传感器/风种子（去相关、可复现）。
fn seeds_for(label: &str, i: usize) -> (u64, u64) {
    let h = (label.len() as u64)
        .wrapping_mul(0x9E37_79B9)
        .wrapping_mul(0x9E37_79B9)
        .wrapping_add(i as u64);
    let sensor = 0x5EED_1357u64.wrapping_add(h).wrapping_mul(0x100000001B3);
    let wind = 0x1234_5678u64.wrapping_add(h ^ 0xDEAD_BEEF).wrapping_mul(0x2545_F491_4F6C_DD1D);
    (sensor, wind)
}

/// 跑单次仿真：真值逐帧统计 + （可选）真值轨迹记录。
#[allow(clippy::too_many_arguments)]
fn run_one(
    kind: ControllerKind,
    drag_fwd: f32,
    vmax_xy: f32,
    wind_base: Option<[f64; 3]>,
    sp: Setpoint,
    seconds: f64,
    sensor_seed: u64,
    wind_seed: u64,
    max_h: f64, // 水平包络判据（hover/wind=漂移限；cruise=前飞距离包络）
    record: bool,
) -> (RunMetric, Vec<TruthFrame>) {
    let mut cfg = load_airframe(None).expect("default airframe");
    cfg.drag_fwd = drag_fwd;
    cfg.vmax_xy = vmax_xy;
    let mut sc = SensorConfig::realistic();
    sc.seed = sensor_seed;
    let wind = wind_base.map(|b| {
        WindField::new(WindConfig {
            base: b,
            seed: wind_seed,
            ..Default::default()
        })
    });
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        wind,
        sc,
        kind,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let total = (seconds / DT) as u64;
    let g = cfg.gravity as f32;
    // 设定点能量高度（NED）：d_eq_sp = d_sp - v_sp²/(2g)
    let vsp = (sp.vel[0].0 * sp.vel[0].0 + sp.vel[1].0 * sp.vel[1].0).sqrt();
    let sp_d_eq = sp.pos[2].0 - vsp * vsp / (2.0 * g);

    let mut m = RunMetric {
        finite: true,
        h_max: 0.0,
        end_d: 0.0,
        tilt_max_deg: 0.0,
        max_e_eq: 0.0,
        end_e_eq: 0.0,
        converged: false,
    };
    let mut frames = Vec::new();
    for i in 0..total {
        ctrl.step(&sp);
        let st = ctrl.world_state();
        let (_, quat) = ctrl.debug_up();
        if !(quat[0].is_finite() && quat[1].is_finite() && quat[2].is_finite() && quat[3].is_finite()) {
            m.finite = false;
            break;
        }
        let tilt = tilt_deg(quat);
        let h = (st.pos[0].0 as f64).hypot(st.pos[1].0 as f64);
        let d = st.pos[2].0 as f64;
        m.h_max = m.h_max.max(h);
        m.tilt_max_deg = m.tilt_max_deg.max(tilt);
        m.end_d = d;
        // 能量高度（真值，NED）：h_eq = d - v_h²/(2g)；误差 = 设定 - 实际。
        let vh = (st.vel[0].0 as f64).hypot(st.vel[1].0 as f64);
        let h_eq = d - vh * vh / (2.0 * g as f64);
        let e_eq = sp_d_eq as f64 - h_eq;
        m.max_e_eq = m.max_e_eq.max(e_eq.abs());
        m.end_e_eq = e_eq;
        if record {
            frames.push(TruthFrame {
                t: i as f64 * DT,
                pos: [st.pos[0].0 as f64, st.pos[1].0 as f64, st.pos[2].0 as f64],
                vel: [st.vel[0].0 as f64, st.vel[1].0 as f64, st.vel[2].0 as f64],
                tilt_deg: tilt,
                e_eq,
            });
        }
    }
    m.converged = m.finite && m.end_d.abs() < 10.0 && m.h_max < max_h && m.tilt_max_deg < 45.0;
    (m, frames)
}

/// 跑一次场景的 N 次蒙特卡洛：聚合统计 + 保留首个（seed=0）代表性轨迹用于 CSV 回放。
fn mc_run(
    label: &str,
    kind: ControllerKind,
    drag_fwd: f32,
    vmax_xy: f32,
    wind_base: Option<[f64; 3]>,
    sp: Setpoint,
    seconds: f64,
    max_h: f64,
    n: usize,
) -> (Agg, Vec<TruthFrame>) {
    let mut ms = Vec::with_capacity(n);
    let mut rep = Vec::new();
    for i in 0..n {
        let (sensor_seed, wind_seed) = seeds_for(label, i);
        let (m, frames) = run_one(
            kind,
            drag_fwd,
            vmax_xy,
            wind_base,
            sp,
            seconds,
            sensor_seed,
            wind_seed,
            max_h,
            i == 0, // 只保留首个确定性（seed 基）运行做回放基准
        );
        if i == 0 {
            rep = frames;
        }
        ms.push(m);
    }
    let agg = aggregate(&ms);
    (agg, rep)
}

fn aggregate(ms: &[RunMetric]) -> Agg {
    let n = ms.len();
    let finite = ms.iter().filter(|m| m.finite).count();
    let converged = ms.iter().filter(|m| m.converged).count();
    let col = |f: fn(&RunMetric) -> f64| Stats::from(&ms.iter().map(f).collect::<Vec<_>>());
    Agg {
        n,
        finite,
        converged,
        h_max: col(|m| m.h_max),
        tilt_max_deg: col(|m| m.tilt_max_deg),
        end_d: col(|m| m.end_d),
        max_e_eq: col(|m| m.max_e_eq),
        end_e_eq: col(|m| m.end_e_eq),
    }
}

fn report(label: &str, agg: &Agg) {
    let n = agg.n as f64;
    println!("[{label}] n={} 收敛率={:.0}% ({}/{}) 失效(NaN)={}",
        agg.n, 100.0 * agg.converged as f64 / n, agg.converged, agg.n, agg.n - agg.finite);
    println!("   h_max[m]      : mean {:.2} ± {:.2}   p95 {:.2}   (min {:.2}, max {:.2})",
        agg.h_max.mean, agg.h_max.std, agg.h_max.p95, agg.h_max.min, agg.h_max.max);
    println!("   tilt_max[deg] : mean {:.2} ± {:.2}   p95 {:.2}   (min {:.2}, max {:.2})",
        agg.tilt_max_deg.mean, agg.tilt_max_deg.std, agg.tilt_max_deg.p95,
        agg.tilt_max_deg.min, agg.tilt_max_deg.max);
    println!("   end_d[m]      : mean {:.2} ± {:.2}   p95 {:.2}   (min {:.2}, max {:.2})",
        agg.end_d.mean, agg.end_d.std, agg.end_d.p95, agg.end_d.min, agg.end_d.max);
    println!("   max|e_eq|     : mean {:.3} ± {:.3}   p95 {:.3}   (min {:.3}, max {:.3})",
        agg.max_e_eq.mean, agg.max_e_eq.std, agg.max_e_eq.p95,
        agg.max_e_eq.min, agg.max_e_eq.max);
    println!("   end_e_eq      : mean {:.3} ± {:.3}   p95 {:.3}",
        agg.end_e_eq.mean, agg.end_e_eq.std, agg.end_e_eq.p95);
}

/// 真值 CSV 回放：`target/bench/<name>`（git 忽略，确定性 seed=0 代表性轨迹）。
fn dump_csv(name: &str, frames: &[TruthFrame]) {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("bench");
    std::fs::create_dir_all(&dir).expect("创建 bench 目录");
    let path = dir.join(name);
    let mut s = String::from("t,tru_n,tru_e,tru_d,tru_vn,tru_ve,tru_vd,tilt_deg,e_eq\n");
    for f in frames {
        s.push_str(&format!(
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.3},{:.6}\n",
            f.t, f.pos[0], f.pos[1], f.pos[2], f.vel[0], f.vel[1], f.vel[2], f.tilt_deg, f.e_eq
        ));
    }
    std::fs::write(&path, s).expect("写 CSV");
    println!("   真值回放 -> {}", path.display());
}

#[test]
fn monte_carlo_benchmark() {
    let n = mc_runs();
    println!("\n==== P3-D4 蒙特卡洛基准数据集（N={}/场景，MC_RUNS 可覆盖） ====", n);

    // 1) hover：无风悬停（PID，10s）
    let sp_h = hover_setpoint(0.0, 0.0, -5.0);
    let (a_h, f_h) = mc_run("hover", ControllerKind::Pid, 0.09, 2.0, None, sp_h, 10.0, 15.0, n);
    report("hover   PID 无风 10s", &a_h);
    dump_csv("hover_tru.csv", &f_h);

    // 2) wind：2 m/s 逆风悬停（PID，8s）
    let sp_w = hover_setpoint(0.0, 0.0, -5.0);
    let (a_w, f_w) = mc_run(
        "wind",
        ControllerKind::Pid,
        0.125,
        2.0,
        Some([2.0, 0.0, 0.0]),
        sp_w,
        8.0,
        15.0,
        n,
    );
    report("wind    PID 逆风2m/s 8s", &a_w);
    dump_csv("wind_tru.csv", &f_w);

    // 3) cruise：80m 北向巡航 vmax=5（TECS vs PID，14s）—— 统计显著性回归对比
    let sp_c = hover_setpoint(80.0, 0.0, -5.0);
    let (a_t, f_t) = mc_run("cruise_tecs", ControllerKind::Tecs, 0.14, 5.0, None, sp_c, 14.0, 100.0, n);
    let (a_p, _f_p) = mc_run("cruise_pid", ControllerKind::Pid, 0.125, 5.0, None, sp_c, 14.0, 100.0, n);
    report("cruise  TECS 80m@5 14s", &a_t);
    report("cruise  PID  80m@5 14s", &a_p);
    dump_csv("cruise_tecs_tru.csv", &f_t);

    println!("\n==== 统计验收（回归判据） ====");
    let mut ok = true;
    for (label, agg) in [("hover", &a_h), ("wind", &a_w), ("cruise_tecs", &a_t), ("cruise_pid", &a_p)] {
        let rate = agg.converged as f64 / agg.n as f64;
        let no_nan = agg.finite == agg.n;
        println!("[{label}] 收敛率 {:.0}% (>=75%: {})，无 NaN: {}", 100.0 * rate, rate >= 0.75, no_nan);
        assert!(no_nan, "[{label}] 蒙特卡洛运行出现 NaN");
        assert!(rate >= 0.75, "[{label}] 收敛率过低 {:.0}%", 100.0 * rate);
    }
    // TECS vs PID：能量高度误差均值 + P95 包络均显著更小（拖拽前馈的系统性收益，非噪声偶然）。
    println!(
        "[cruise TECS vs PID] max|e_eq| mean {:.3} vs {:.3}，p95 {:.3} vs {:.3}",
        a_t.max_e_eq.mean, a_p.max_e_eq.mean, a_t.max_e_eq.p95, a_p.max_e_eq.p95,
    );
    assert!(
        a_t.max_e_eq.mean < a_p.max_e_eq.mean,
        "TECS 巡航能量误差均值未优于 PID: {:.3} vs {:.3}",
        a_t.max_e_eq.mean, a_p.max_e_eq.mean,
    );
    assert!(
        a_t.max_e_eq.p95 < a_p.max_e_eq.p95,
        "TECS 巡航能量误差 P95 包络未优于 PID: {:.3} vs {:.3}",
        a_t.max_e_eq.p95, a_p.max_e_eq.p95,
    );
    ok &= a_t.max_e_eq.mean < a_p.max_e_eq.mean && a_t.max_e_eq.p95 < a_p.max_e_eq.p95;
    println!("统计验收 {}（真值回放 CSV 在 target/bench/）", if ok { "PASS" } else { "FAIL" });
}
