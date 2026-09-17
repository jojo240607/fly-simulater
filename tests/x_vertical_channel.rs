//! [垂向通道] SIL 悬停垂向通道回归：EKF 垂直位置/速度估计精度 + 定高稳定性。
//!
//! 用途（与 `mcu_simulater/tests/x_sensor_rate.rs` 互补）：
//! - 本测试锁住 **SIL 侧**（`FlyController`，物理 dt 与控制器 dt 严格一致）的
//!   垂向估计精度与悬停稳态；若此处退化，说明 EKF 垂直通道或定高环本身出了问题；
//! - 若此处正常而 MCU 闭环侧（虚拟外设直通）垂向慢漂，则问题在**时钟口径/采样
//!   链路**而非控制律——这正是 `mcu_simulater` 闭环测试固件时钟失配 1.23× 的
//!   排查依据（详见 `mcu_simulater::sim::timing::RETIRED_BYTES_PER_MS`）。
//!
//! 判据（12s 无风悬停，(0,0,-5) 设定点，清洁传感器）：
//!   1) 末段（75%~100%）EKF 估计与真值的垂向偏差：|Δd| < 0.05m、|Δvd| < 0.05m/s；
//!   2) 末段真实高度均值 |d − (−5)| < 0.2m（含起飞瞬态后的稳态）。
//!
//! 实测基线（参考值，用于判断回归幅度）：|Δd| ≈ 0.0000m、|Δvd| ≈ 0.0000m/s、
//! 末段 d 均值 ≈ −4.99m。

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ContactModel, PhySdkWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;
const HOVER_D: f64 = -5.0;

#[test]
fn sil_vertical_channel_estimate_tracks_truth() {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, HOVER_D as f32);
    let total = (12.0 / DT) as u64;

    println!("# t  tru_d  tru_vd   est_d  est_vd   ab_z    thr     ez      iz    des_vz  acc_d  filt_vd");
    let (mut d_sum, mut d_absmax, mut err_d_max, mut err_v_max, mut n) = (0.0f64, 0.0f64, 0.0f32, 0.0f32, 0u64);

    for i in 0..total {
        let st = ctrl.step(&sp);
        let est = ctrl.debug_estimate_ned();
        let ab = ctrl.debug_accel_bias();
        let (_dr, _vr, _fd, fvd, ez, iz, des_vz, acc_d, _thr) = ctrl.debug_pid_internal();

        if i % 125 == 0 {
            println!(
                "{:5.2} {:7.3} {:7.3}  {:7.3} {:7.3}  {:7.4} {:6.3} {:7.3} {:7.3} {:7.3} {:7.3} {:7.3}",
                i as f64 * DT,
                st.pos[2].0,
                st.vel[2].0,
                est.pos[2].0,
                est.vel[2].0,
                ab[2],
                ctrl.debug_thrust_sum() / 4.0,
                ez,
                iz,
                des_vz,
                acc_d,
                fvd,
            );
        }

        // 末段（75%~100%）统计：估计-真值垂向偏差 + 真实高度偏离设定点
        if i > total * 3 / 4 {
            let (td, tvd) = (st.pos[2].0, st.vel[2].0);
            d_sum += td as f64;
            d_absmax = d_absmax.max((td as f64 - HOVER_D).abs());
            err_d_max = err_d_max.max((est.pos[2].0 - td).abs());
            err_v_max = err_v_max.max((est.vel[2].0 - tvd).abs());
            n += 1;
        }
    }

    let d_mean = d_sum / n as f64;
    println!(
        "# 末段(75%~100%, n={n})：真实 d 均值={d_mean:.4}m（设定点 {HOVER_D}）| 最大偏离={d_absmax:.4}m \
         | EKF 估计-真值 最大 |Δd|={err_d_max:.4}m、|Δvd|={err_v_max:.4}m/s"
    );

    assert!(
        err_d_max < 0.05,
        "SIL 垂向位置估计偏差过大：max |Δd|={err_d_max:.4}m（期望 <0.05m）——EKF 垂直通道退化？"
    );
    assert!(
        err_v_max < 0.05,
        "SIL 垂向速度估计偏差过大：max |Δvd|={err_v_max:.4}m/s（期望 <0.05m/s）——EKF 垂直通道退化？"
    );
    assert!(
        d_absmax < 0.2,
        "SIL 定高稳态偏离过大：max |d-{HOVER_D}|={d_absmax:.4}m（期望 <0.2m）——定高环退化？"
    );
}
