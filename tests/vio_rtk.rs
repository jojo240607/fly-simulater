//! P3-B1 "VIO/RTK-GPS 多源融合"验收测试（无头模式，ToyWorld 替身）。
//!
//! 验证三类行为：
//!  1) `gps_outage_vio_rtk_bridges`      —— GPS 中断时 VIO+RTK 兜底位置：
//!      估计误差有界、TRU 不翻机（不 NaN / 不 runaway / 不翻滚）。
//!  2) `gps_outage_fusion_beats_imu_only`—— 同中断窗口下，有 VIO/RTK 融合的
//!      估计误差 < 纯 IMU 死推（VIO 桥接 GPS 空白，而非听任积分漂移）。
//!  3) `rtk_keeps_position_cm_level`     —— RTK 厘米级绝对位置注入后，位置估计
//!      误差显著小于 VIO 单独（VIO 长期漂移被 RTK 持续纠正）。
//!
//! 用法：cargo test --test vio_rtk -- --nocapture
//!
//! 坐标系：NED。`est` = `FlyController::step` 返回的 EKF 估计（VehicleState），
//! `tru` = `world_state` 真值；估计误差 = est - tru。TRU 有界判据复用
//! `common::assert_tru_bounded`（水平漂移/高度/倾角三个归一化量）。

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;
use flyctrl_core::controller::Setpoint;

mod common;
use common::{assert_tru_bounded, TruStats};

const DT: f64 = 0.004;

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

/// GPS 中断期间注入的观测源（VIO/RTK 开关）。
#[derive(Clone, Copy)]
struct Fusion {
    vio: bool,
    rtk: bool,
}

/// 一次 GPS 中断仿真的汇总诊断。
struct Diag {
    finite: bool,
    max_tilt_deg: f64,
    max_h_err: f32,   // 中断窗口内最大水平估计误差（NED 平面范数）
    max_d_err: f32,   // 中断窗口内最大垂直（NED down）估计误差
    max_tot_err: f32, // 中断窗口内最大 3D 估计误差
    tru: TruStats,    // TRU 真值有界统计
}

impl Diag {
    fn default() -> Self {
        Self {
            finite: true,
            max_tilt_deg: 0.0,
            max_h_err: 0.0,
            max_d_err: 0.0,
            max_tot_err: 0.0,
            tru: TruStats::default(),
        }
    }
}

/// 悬停 -5m → 预热收敛 → GPS 失锁并保持 → 记录中断窗口内的估计误差与真值有界统计。
fn run_gps_outage(fusion: Fusion, warm_s: f64, outage_s: f64) -> Diag {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        // 场景测试默认真实噪声（屏蔽噪声会掩盖 EKF/多源融合行为）。
        SensorConfig::realistic(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp: Setpoint = hover_setpoint(0.0, 0.0, -5.0);

    // 预热收敛（GPS 正常，估计进入稳态）。
    let warm = (warm_s / DT) as u64;
    for _ in 0..warm {
        ctrl.step(&sp);
    }

    // 注入融合配置（先按需关闭 VIO/RTK），再统一注入 GPS 失锁。
    ctrl.set_vio_available(fusion.vio);
    ctrl.set_rtk_available(fusion.rtk);
    ctrl.set_gps_available(false);

    let mut d = Diag::default();
    let n = (outage_s / DT) as u64;
    for _ in 0..n {
        let est = ctrl.step(&sp);
        let tru = ctrl.world_state();
        let (_, quat) = ctrl.debug_up();
        if !(est.pos[0].0.is_finite()
            && est.pos[1].0.is_finite()
            && est.pos[2].0.is_finite()
            && quat[0].is_finite())
        {
            d.finite = false;
            break;
        }
        d.max_tilt_deg = d.max_tilt_deg.max(tilt_deg(quat));
        let he = (est.pos[0].0 - tru.pos[0].0).hypot(est.pos[1].0 - tru.pos[1].0);
        let de = (est.pos[2].0 - tru.pos[2].0).abs();
        d.max_h_err = d.max_h_err.max(he);
        d.max_d_err = d.max_d_err.max(de);
        d.max_tot_err = d.max_tot_err.max(he.hypot(de));
        // TRU 真值归一化（NED）：h=物理水平漂移、d=物理 NED down、tilt。
        d.tru.sample(
            (tru.pos[0].0 as f64).hypot(tru.pos[1].0 as f64),
            tru.pos[2].0 as f64,
            tilt_deg(quat),
        );
    }
    d
}

#[test]
fn gps_outage_vio_rtk_bridges() {
    let fusion = Fusion { vio: true, rtk: true };
    let d = run_gps_outage(fusion, 4.0, 8.0);
    println!(
        "[gps_outage_vio_rtk_bridges] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "GPS 中断 + VIO/RTK 融合出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "gps_outage_vio_rtk_bridges", -5.0, 3.0, 45.0);
    // VIO 高频位置 + RTK 厘米级绝对位置兜底：中断窗口内估计误差必须有界（非发散）。
    assert!(
        d.max_h_err < 1.0,
        "GPS 中断期间 VIO/RTK 水平估计误差过大：{:.2}m（期望 < 1.0m）",
        d.max_h_err
    );
    assert!(
        d.max_d_err < 2.0,
        "GPS 中断期间 VIO/RTK 垂直估计误差过大：{:.2}m（期望 < 2.0m）",
        d.max_d_err
    );
}

#[test]
fn gps_outage_fusion_beats_imu_only() {
    let with = run_gps_outage(Fusion { vio: true, rtk: true }, 4.0, 8.0);
    let wout = run_gps_outage(Fusion { vio: false, rtk: false }, 4.0, 8.0);
    println!(
        "[gps_outage_fusion_beats_imu_only] 有融合 tot={:.2}m | 纯 IMU tot={:.2}m",
        with.max_tot_err, wout.max_tot_err
    );
    assert!(with.finite && wout.finite, "对比仿真出现 NaN/Inf");
    // 验收判据：有 VIO/RTK 兜底的估计误差应显著优于纯 IMU 死推。
    assert!(
        with.max_tot_err < wout.max_tot_err,
        "有融合估计误差应 < 纯 IMU：{:.2}m vs {:.2}m",
        with.max_tot_err, wout.max_tot_err
    );
    // 融合兜底须真正"桥接"（误差足够小），而非两者都大。
    assert!(
        with.max_tot_err < 2.0,
        "有融合最大 3D 估计误差应 < 2m，got {:.2}m",
        with.max_tot_err
    );
    // 纯 IMU 死推在 8s 窗口内应出现明显漂移（无位置/速度观测，积分发散）。
    assert!(
        wout.max_tot_err > 1.0,
        "纯 IMU 死推应漂移 > 1m，got {:.2}m（可能 IMU 模型太理想）",
        wout.max_tot_err
    );
}

#[test]
fn rtk_keeps_position_cm_level() {
    let rtk_on = run_gps_outage(Fusion { vio: true, rtk: true }, 4.0, 8.0);
    let rtk_off = run_gps_outage(Fusion { vio: true, rtk: false }, 4.0, 8.0);
    println!(
        "[rtk_keeps_position_cm_level] VIO+RTK h_err={:.3}m tot={:.2}m | VIO 单独 h_err={:.3}m tot={:.2}m",
        rtk_on.max_h_err, rtk_on.max_tot_err, rtk_off.max_h_err, rtk_off.max_tot_err
    );
    assert!(rtk_on.finite && rtk_off.finite, "对比仿真出现 NaN/Inf");
    // 验收判据：RTK 注入后估计误差应小于 VIO 单独（抑制 VIO 长期漂移）。
    assert!(
        rtk_on.max_tot_err < rtk_off.max_tot_err,
        "VIO+RTK 估计误差应 < VIO 单独：{:.2}m vs {:.2}m",
        rtk_on.max_tot_err, rtk_off.max_tot_err
    );
    // RTK 厘米级：水平估计误差应远小于 VIO 单独漂移（数 dm 内）。
    assert!(
        rtk_on.max_h_err < 0.5,
        "RTK 融合水平估计误差应达 dm 级，got {:.3}m（期望 < 0.5m）",
        rtk_on.max_h_err
    );
}
