//! P3-B3 "传感器硬/软故障注入到估计器"验收测试（无头模式，ToyWorld 替身）。
//!
//! 对应 FIDELITY_ROADMAP P3-B3 目标"偏置突变 / 卡死 / 漂移全链路"，验证故障在
//! **物理真值 → 传感器读数** 处注入（[`SensorFault`] → [`SensorModel`]）后，
//! 经 `read_sensors` 喂给 EKF/FDIR 的系统行为：
//!  1) 软故障（偏置突变 / 漂移）：
//!     - `accel_bias_step_stays_bounded` / `gyro_bias_step_stays_bounded`：
//!       IMU 读数被叠加偏置，估计器靠自身鲁棒性消化（EKF 零偏估计 / 多源融合 /
//!       控制器反馈），TRU 真值有界、估计误差有界（不 NaN / 不 runaway）。
//!     - `gps_bias_step_offsets_estimate_bounded`：GPS 位置偏置（VIO/RTK 关闭，
//!       GPS 为唯一绝对源）直接注入位置估计——估计被牵制偏移 ≈ 偏置量（有界），
//!       证明故障**确实**穿过估计器链路（非被忽略）。
//!  2) 硬故障（卡死）：
//!     - `imu_stuck_triggers_fdir_critical`：加速度计卡死被 FDIR 检测为
//!       `Health::Critical` → 失控保护单向置位（执行器归零）。
//!     - `gps_stuck_masked_by_vio_rtk_fusion`：GPS 卡死不退化为失锁（`has_fix` 仍
//!       true），但被 VIO/RTK 多源融合兜底（P3-B1 容错收益：单源硬故障不导致位置
//!       估计发散——若只靠 GPS+IMU 死推，卡死偏置会把位置估计/机体直接拉飞）。
//!
//! 用法：cargo test --test sensor_fault -- --nocapture
//!
//! 坐标系：NED。`est` = `FlyController::step` 返回的 EKF 估计（`VehicleState`），
//! `tru` = `world_state` 真值；估计误差 = est - tru。TRU 有界判据复用
//! `common::assert_tru_bounded`（水平漂移/高度/倾角三个归一化量）。

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::{SensorConfig, SensorFault};
use fly_simulater::airframe::load_airframe;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::fdir::Health;

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

/// 一次故障注入仿真的汇总诊断。
struct FaultDiag {
    finite: bool,
    max_tilt_deg: f64,
    max_h_err: f32,   // 故障窗口内最大水平估计误差（NED 平面范数）
    max_d_err: f32,   // 故障窗口内最大垂直（NED down）估计误差
    max_tot_err: f32, // 故障窗口内最大 3D 估计误差
    tru: TruStats,    // TRU 真值有界统计
}

impl FaultDiag {
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

/// 悬停 -5m → 预热收敛 → 按需关闭 VIO/RTK → 注入故障 → 记录故障窗口内的估计误差与
/// 真值有界统计。
fn run_fault(fault: SensorFault, warm_s: f64, window_s: f64, rtk_on: bool, vio_on: bool) -> FaultDiag {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        // 场景测试默认真实噪声（屏蔽噪声会掩盖 EKF/多源融合/故障注入行为）。
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

    // 按需关闭 VIO/RTK：隔离 GPS 故障对位置估计的影响（GPS 作唯一绝对源时偏置
    // 直接注入；保留时验证多源融合兜底）。
    ctrl.set_vio_available(vio_on);
    ctrl.set_rtk_available(rtk_on);
    ctrl.inject_sensor_fault(fault);

    let mut d = FaultDiag::default();
    let n = (window_s / DT) as u64;
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

/// P3-B3 验收 1：加速度计偏置突变（软故障）——估计器靠鲁棒性消化，TRU 有界、
/// 估计误差有界。
#[test]
fn accel_bias_step_stays_bounded() {
    let d = run_fault(SensorFault::AccelBias([0.25, -0.15, 0.20]), 4.0, 6.0, true, true);
    println!(
        "[accel_bias_step] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "加速度计偏置突变出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "accel_bias_step", -5.0, 4.0, 45.0);
    assert!(
        d.max_tot_err < 2.0,
        "加速度计偏置突变下估计误差过大：{:.2}m（期望 < 2.0m）",
        d.max_tot_err
    );
}

/// P3-B3 验收 2：陀螺偏置突变（软故障）——姿态估计靠陀螺积分 + 速度/位置间接约束，
/// 小偏置下仍保持有界。
#[test]
fn gyro_bias_step_stays_bounded() {
    let d = run_fault(SensorFault::GyroBias([0.02, -0.015, 0.01]), 4.0, 6.0, true, true);
    println!(
        "[gyro_bias_step] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "陀螺偏置突变出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "gyro_bias_step", -5.0, 4.0, 45.0);
    assert!(
        d.max_tot_err < 2.0,
        "陀螺偏置突变下估计误差过大：{:.2}m（期望 < 2.0m）",
        d.max_tot_err
    );
}

/// P3-B3 验收 3：加速度计偏置漂移（软故障）——缓变漂移（5s 累计 ~0.1 m/s²），
/// 估计器在线跟踪，TRU 有界。
#[test]
fn accel_drift_ramps_bounded() {
    let d = run_fault(SensorFault::AccelDrift([0.02, 0.0, 0.015]), 4.0, 5.0, true, true);
    println!(
        "[accel_drift] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "加速度计漂移出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "accel_drift", -5.0, 4.0, 45.0);
    assert!(
        d.max_tot_err < 2.0,
        "加速度计漂移下估计误差过大：{:.2}m（期望 < 2.0m）",
        d.max_tot_err
    );
}

/// P3-B3 验收 4：IMU 硬故障（加速度计卡死）——FDIR 检测为 Critical → 失控保护
/// 归零执行器（安全降级路径）。
#[test]
fn imu_stuck_triggers_fdir_critical() {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp: Setpoint = hover_setpoint(0.0, 0.0, -5.0);
    // 预热收敛 4s，健康应为 Nominal、未触发失控保护。
    for _ in 0..1000 {
        ctrl.step(&sp);
    }
    assert_eq!(ctrl.health(), Health::Nominal, "预热后健康应为 Nominal");
    assert!(!ctrl.failsafe_engaged(), "预热后不应触发失控保护");

    // 注入加速度计卡死（全零输出：范数 0 < 6 m/s² 且逐帧恒定 → 判冻结）。
    ctrl.inject_sensor_fault(SensorFault::AccelStuck(Some([0.0, 0.0, 0.0])));
    // 跑足 FDIR imu_stale_timeout（20 帧=100ms）+ 裕量（含恢复延迟）。
    for _ in 0..80 {
        ctrl.step(&sp);
    }

    // 故障确已到达估计器链路：IMU 读数冻结为给定值。
    let imu = ctrl.last_imu();
    assert_eq!(
        [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0],
        [0.0, 0.0, 0.0],
        "加速度计卡死应把读数冻结为给定值"
    );
    assert_eq!(ctrl.health(), Health::Critical, "IMU 卡死应被 FDIR 判 Critical");
    assert!(ctrl.failsafe_engaged(), "IMU 卡死应触发失控保护（执行器归零）");
}

/// P3-B3 验收 5：GPS 位置偏置突变（软故障）——VIO/RTK 关闭、GPS 为唯一绝对位置源，
/// 偏置直接注入位置估计：估计被牵制偏移 ≈ 偏置量（证明故障穿过估计器链路），
/// 且整体有界（不 NaN / 不 runaway）。
#[test]
fn gps_bias_step_offsets_estimate_bounded() {
    let d = run_fault(SensorFault::GpsBias([3.0, 0.0, 0.0]), 4.0, 8.0, false, false);
    println!(
        "[gps_bias_step] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "GPS 偏置突变出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "gps_bias_step", -5.0, 6.0, 45.0);
    // 偏置 3m：位置估计被牵制偏移 ≈3m（有界）。下界 1.5m 证明故障确实注入。
    assert!(
        d.max_h_err > 1.5,
        "GPS 偏置 3m 应把位置估计牵制偏移 ≈3m（全链路注入），got {:.2}m",
        d.max_h_err
    );
    assert!(
        d.max_h_err < 5.0,
        "GPS 偏置 3m 下水平估计误差过大：{:.2}m（期望 < 5.0m）",
        d.max_h_err
    );
}

/// P3-B3 验收 6：GPS 硬故障（卡死）——GPS 输出冻结在偏移 5m 位置且不退化为失锁
/// （`has_fix` 仍 true），但 VIO/RTK 多源融合（P3-B1）兜底：位置估计不被卡死值
/// 牵制，TRU/估计均保持有界（单源硬故障不导致位置发散）。
#[test]
fn gps_stuck_masked_by_vio_rtk_fusion() {
    let d = run_fault(
        SensorFault::GpsStuck(Some([5.0, 0.0, -5.0, 0.0, 0.0, 0.0])),
        4.0,
        8.0,
        true,
        true,
    );
    println!(
        "[gps_stuck_masked] finite={} h_err={:.3}m d_err={:.3}m tot={:.3}m tilt={:.1}° tru_h={:.2}m tru_d={:.2}m",
        d.finite, d.max_h_err, d.max_d_err, d.max_tot_err, d.max_tilt_deg, d.tru.h_max, d.tru.end_d
    );
    assert!(d.finite, "GPS 卡死 + 多源融合出现 NaN/Inf");
    assert_tru_bounded(&d.tru, "gps_stuck_masked", -5.0, 3.0, 45.0);
    // 卡死偏置 5m 被 RTK/VIO 兜底：水平估计误差应远小于卡死偏置量。
    assert!(
        d.max_h_err < 1.5,
        "GPS 卡死应被 VIO/RTK 融合兜底，水平估计误差过大：{:.2}m（期望 < 1.5m）",
        d.max_h_err
    );
}
