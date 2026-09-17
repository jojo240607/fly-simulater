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

#[test]
fn hover_stable_with_mag_heading_active() {
    let cfg = VehicleConfig::default_quad();
    let mut scfg = SensorConfig::default();
    scfg.mag_decl_deg = 7.0; // 磁北偏东 7°
    scfg.mag_hard_iron = [0.3, -0.2, 0.4]; // uT 硬铁
    scfg.mag_soft_iron = [0.98, 1.03, 0.99]; // 软铁缩放
    scfg.mag_noise = 0.05; // uT 白噪声
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
    let total = (10.0 / DT) as u64; // 10s 悬停
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
    assert!(
        dz < 0.5 && horiz < 0.5,
        "悬停位置未收敛到设定点：dz={dz:.2} horiz={horiz:.2}"
    );
    assert!(
        end.att.roll().abs() < 0.2 && end.att.pitch().abs() < 0.2,
        "姿态发散：roll={} pitch={}",
        end.att.roll(),
        end.att.pitch()
    );
    assert!(
        last_yaw.abs() < 0.1,
        "yaw 应被磁航向锚定到 0（地理北，decl=7° 已由 EKF 修正）：末态 {}°",
        last_yaw.to_degrees()
    );
    assert!(
        max_yaw_abs < 0.2,
        "全程 yaw 漂移过大（磁锚定未生效？）：max {:.2}°",
        max_yaw_abs.to_degrees()
    );
    assert!(
        max_att_err < 0.3,
        "悬停姿态误差过大：max roll/pitch {:.2}°",
        max_att_err.to_degrees()
    );
}
