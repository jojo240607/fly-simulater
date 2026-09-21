//! SIL 磁力计闭环回归：真实磁力计建模（decl + 硬/软铁 + 噪声）下悬停稳定。
//!
//! 用 PhySdkWorld（真实物理引擎）验证：
//! 1) 磁力计非零场（不再零场关闭）不破坏悬停闭环（位置/姿态收敛）；
//! 2) EKF 磁航向锚定生效：yaw 被锚定到地理北（0），decl 注入与 plant 世界
//!    地磁一致 → 磁航向转地理航向正确；
//! 3) 硬铁/软铁/噪声存在时锚定仍稳定（无正反馈发散）。

use fly_sim_core::controller::{hover_setpoint, ControllerKind, FlyController};
use fly_sim_core::physics::{ContactModel, PhySdkWorld};
use fly_sim_core::sensor::SensorConfig;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::invariants;

const DT: f64 = 0.004;

/// 磁航向锚定回归（**带非零陀螺零偏**——否则 yaw 本就不漂，测试会空过）。
///
/// 历史教训：本测试原本用 `SensorConfig::default()`（`gyro_bias=[0,0,0]`），
/// 即使 `EkfEstimator` 缺 `Estimator::update_mag` 的 trait 委托、磁锚定静默失效（F5），
/// yaw 也不漂 → 断言空过、bug 逃逸。加非零陀螺零偏后，锚定一死 yaw 必然漂。
#[test]
fn hover_stable_with_mag_heading_active() {
    let cfg = VehicleConfig::default_quad();
    let mut scfg = SensorConfig::default();
    scfg.mag_decl_deg = 7.0; // 磁北偏东 7°
    scfg.mag_hard_iron = [0.3, -0.2, 0.4]; // uT 硬铁
    scfg.mag_soft_iron = [0.98, 1.03, 0.99]; // 软铁缩放
    scfg.mag_noise = 0.05; // uT 白噪声
    // **非零陀螺零偏**：z 轴 0.006 rad/s（0.34°/s）——锚定失效时 60s 漂 0.36 rad=20.6°，
    // 刚好越过阈值；roll/pitch 零偏取小值，避免带出位置漂移（那不是本测试的范围）。
    scfg.gyro_bias = [0.001, -0.0005, 0.006];
    let world = PhySdkWorld::create_empty();
    let mut fc = FlyController::new_at(
        world,
        &cfg,
        DT,
        None,
        scfg,
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
        [0.0, 0.0, -5.0],
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let total = (60.0 / DT) as u64; // 60s（零偏 0.006 rad/s 下若锚定失效，yaw 会漂 >20°）
    let mut max_yaw_abs = 0.0f32;
    let mut last_yaw = 0.0f32;
    let mut max_att_err = 0.0f32;
    for _ in 0..total {
        let st = fc.step(&sp);
        assert!(invariants::state_finite(&st), "EKF 状态发散（NaN/Inf）");
        let yaw = st.att.yaw();
        max_yaw_abs = max_yaw_abs.max(yaw.abs());
        last_yaw = yaw;
        let att_err = st.att.roll().abs().max(st.att.pitch().abs());
        max_att_err = max_att_err.max(att_err);
    }
    let end = fc.world_state();
    let dz = (end.pos[2].0 - (-5.0)).abs();
    let horiz = (end.pos[0].0).hypot(end.pos[1].0);
    eprintln!(
        "[mag_hover] last_yaw={:.2}° max|yaw|={:.2}° dz={dz:.2} horiz={horiz:.2}",
        last_yaw.to_degrees(),
        max_yaw_abs.to_degrees()
    );
    // 位置：**有界**即可，不要求高精度。
    //
    // 实测发现：未补偿的硬铁 `[0.3,−0.2,0.4]` 使航向偏 ~13° → 位置保持从
    // <0.5m 劣化到 ~2.7m/60s（有界、不发散）。根因是 EKF **无磁硬铁状态**，
    // 属阶段 6（鲁棒性）课题。本测试守的是"硬铁下磁锚定仍稳定不发散"。
    assert!(
        dz < 1.0 && horiz < 4.0,
        "悬停位置应有界（硬铁下允许 ~3m 漂移）：dz={dz:.2} horiz={horiz:.2}"
    );
    assert!(
        end.att.roll().abs() < 0.2 && end.att.pitch().abs() < 0.2,
        "姿态发散：roll={} pitch={}",
        end.att.roll(),
        end.att.pitch()
    );
    // **磁锚定必须生效**：陀螺 z 零偏 0.006 rad/s → 若无锚定，60s 漂 0.36 rad = 20.6°。
    // 阈值 0.30 rad（17°）区分"有锚定（有界）"与"无锚定（线性漂移）"。
    // 注：不再要求 |yaw|<0.1——硬铁 `[0.3,-0.2,0.4]` 使磁北方向偏移，
    // 锚定会忠实地稳定在一个**偏置航向**（实测 ~13°）。硬铁补偿（EKF 无磁硬铁状态）
    // 属阶段 6 课题；本测试只守"锚定生效且稳定不发散"。
    assert!(
        max_yaw_abs < 0.30,
        "磁锚定应生效（yaw 有界 <17°）——若漂到 {}° 则锚定失效（检查 EkfEstimator \
         是否漏了 `Estimator::update_mag` 的 trait 委托）",
        max_yaw_abs.to_degrees()
    );
    assert!(
        max_att_err < 0.3,
        "悬停姿态误差过大：max roll/pitch {:.2}°",
        max_att_err.to_degrees()
    );
}

/// 干净磁力计 + 非零陀螺零偏：yaw 应被锚定到地理北（decl 由 EKF 修正）。
///
/// 与上一个测试互补：那个守"硬铁下锚定仍稳定"，本测试守"无误差时锚定精度"。
#[test]
fn hover_yaw_anchored_exactly_with_clean_mag() {
    let cfg = VehicleConfig::default_quad();
    let mut scfg = SensorConfig::default();
    scfg.mag_decl_deg = 7.0; // 磁北偏东 7°（EKF 用 set_mag_declination 修正）
    scfg.gyro_bias = [0.01, -0.005, 0.02];
    let world = PhySdkWorld::create_empty();
    let mut fc = FlyController::new_at(
        world,
        &cfg,
        DT,
        None,
        scfg,
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
        [0.0, 0.0, -5.0],
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(20.0 / DT) as u64 {
        let st = fc.step(&sp);
        assert!(invariants::state_finite(&st), "EKF 状态发散");
    }
    let yaw = fc.step(&sp).att.yaw();
    eprintln!("[mag_hover_clean] yaw={:.3}°", yaw.to_degrees());
    assert!(
        yaw.abs() < 0.1,
        "干净磁力计下 yaw 应被锚定到地理北（decl=7° 已修正），末态 {:.2}°",
        yaw.to_degrees()
    );
}
