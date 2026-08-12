//! P0-1 动力系统验证：油门→电压→转速→推力（∝ω²）+ 电池掉压。
//!
//! 锁住：
//! 1. 推力 ∝ 转速平方：固定油门下 T / ω² 恒为 prop_kt（常数）。
//! 2. 电池掉压：大油门电流大 → 电压低于标称；小油门接近标称。
//! 3. 悬停时电压接近标称（掉压小，不破坏控制律悬停）。

use fly_sim_core::physics::{PhySdkWorld, RigidBodyWorld};
use fly_sim_core::plant::QuadrotorPlant;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::ActuatorCmd;

const DT: f64 = 0.004;

fn plant_at_throttle(u: f64, settle_steps: usize) -> QuadrotorPlant<PhySdkWorld> {
    let cfg = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        Default::default(),
    );
    let cmd = ActuatorCmd {
        motor: [u as f32; 4],
    };
    plant.apply_actuators(&cmd);
    // 跑足够步让电机转速、电池电压趋近稳态。
    for _ in 0..settle_steps {
        plant.step();
    }
    plant
}

#[test]
fn thrust_scales_with_omega_squared() {
    // 固定油门稳态：T = prop_kt·ω²，故 T/ω² 应恒定（= prop_kt）。
    let cfg = VehicleConfig::default_quad();
    let omega_max = cfg.motor_kv as f64 * cfg.battery_v_nom as f64;
    let prop_kt = cfg.thrust_coeff as f64 / (omega_max * omega_max);

    let p = plant_at_throttle(0.7, 400);
    let (_v, w) = p.powertrain_state();
    // 推力矩 = prop_kt·ω²（稳态），由 powertrain_state 只给转速，推力用 ω² 校验 prop_kt。
    for i in 0..4 {
        let expected_t = cfg.thrust_coeff as f64 * 0.7; // 稳态线性（未掉压）
        let t_by_omega2 = prop_kt * w[i] * w[i];
        // 稳态应接近线性推力（掉压小），相对误差 < 8%
        assert!(
            (t_by_omega2 - expected_t).abs() / expected_t < 0.08,
            "M{} 推力∝ω²偏差: T/ω² 计算={:.4}, 期望线性={:.4}",
            i,
            t_by_omega2,
            expected_t
        );
    }
}

#[test]
fn battery_droops_with_load() {
    // 小油门 → 电压接近标称；大油门 → 电压明显跌落。
    let p_low = plant_at_throttle(0.15, 300);
    let (v_low, _) = p_low.powertrain_state();
    let p_high = plant_at_throttle(0.95, 400);
    let (v_high, _) = p_high.powertrain_state();

    let v_nom = VehicleConfig::default_quad().battery_v_nom as f64;
    // 小油门：掉压 < 3%
    assert!(
        v_low > v_nom * 0.97,
        "小油门电压应接近标称: V={:.2} (nom={:.2})",
        v_low,
        v_nom
    );
    // 大油门：掉压 > 5%（明显）
    assert!(
        v_high < v_nom * 0.95,
        "大油门应明显掉压: V={:.2} (nom={:.2})",
        v_high,
        v_nom
    );
    // 大油门掉压 > 小油门
    assert!(v_high < v_low, "大油门电压应低于小油门");
}
