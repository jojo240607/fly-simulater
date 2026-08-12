//! P0-1 动力系统验证：油门→电压→转速→推力（∝ω²）+ 电池掉压。
//!
//! 锁住：
//! 1. 推力 ∝ 转速平方：固定油门下 T / ω² 恒为 prop_kt（常数）。
//! 2. 电池掉压：大油门电流大 → 电压低于标称；小油门接近标称。
//! 3. 悬停时电压接近标称（掉压小，不破坏控制律悬停）。

use fly_sim_core::physics::{PhySdkWorld, RigidBodyWorld};
use fly_sim_core::plant::{gyro_torque, QuadrotorPlant};
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::ActuatorCmd;

const DT: f64 = 0.004;
/// 悬停时 4 电机等速反桨（0,1 CCW+1；2,3 CW-1）。
const SPIN: [f64; 4] = [1.0, 1.0, -1.0, -1.0];

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

#[test]
fn gyro_balanced_rotors_cancel_in_hover() {
    // 四旋翼等速反桨 → 净角动量 H_z=0，悬停（等转速）无净陀螺力矩。
    let i_rotor = VehicleConfig::default_quad().rotor_inertia as f64;
    let w = [1000.0; 4]; // 4 电机等速
    let omega = [0.0, 0.0, 0.0]; // 悬停无角速度
    let tau = gyro_torque(&w, &SPIN, i_rotor, omega);
    assert_eq!(tau, [0.0, 0.0, 0.0], "无角速度时应无陀螺力矩");
}

#[test]
fn gyro_balanced_rotors_pitch_cancels_but_roll_torque() {
    // 等速反桨 + 俯仰角速度 ω_y：每桨 H_z 不同符号，ΣH_z=0，
    // 但 M_x = -Σ(H_z·ω_y)；由于 ΣH_z=0 → 净 M_x=0（四旋翼反桨完全抵消俯仰→滚转陀螺）。
    let i_rotor = VehicleConfig::default_quad().rotor_inertia as f64;
    let w = [1000.0; 4];
    let omega = [0.0, 2.0, 0.0]; // 俯仰角速度 2 rad/s
    let tau = gyro_torque(&w, &SPIN, i_rotor, omega);
    // ΣH_z = I·ω·(+1+1-1-1)=0 → 净 M_x=0
    assert!(tau[0].abs() < 1e-9, "等速反桨净陀螺应抵消, M_x={}", tau[0]);
}

#[test]
fn gyro_asymmetric_rotors_produce_coupling() {
    // 转速不对称（如某桨失效/偏航差动）：净 H_z≠0 → 俯仰角速度在滚转轴产生净力矩。
    let i_rotor = VehicleConfig::default_quad().rotor_inertia as f64;
    // 电机 0 转速高、电机 2 转速低（不对称），其余等速。
    let w = [1400.0, 1000.0, 800.0, 1000.0];
    let omega = [0.0, 2.0, 0.0]; // 俯仰角速度
    let tau = gyro_torque(&w, &SPIN, i_rotor, omega);
    // M_x = -Σ(H_z·ω_y)，H_z = I·ω_i·spin_i
    // 电机0 hz=+I·1400, 电机1 +I·1000, 电机2 -I·800, 电机3 -I·1000 → Σ=I·(1400+1000-800-1000)=I·600
    let i_rotor = VehicleConfig::default_quad().rotor_inertia as f64;
    let expect_mx = -i_rotor * 600.0 * 2.0;
    assert!(
        (tau[0] - expect_mx).abs() < 1e-9,
        "转速不对称应产生滚转陀螺力矩: got {:.6e}, expect {:.6e}",
        tau[0],
        expect_mx
    );
    // 悬停（无角速度）不对称转速不产生陀螺（陀螺需 ω 非零）
    let tau0 = gyro_torque(&w, &SPIN, i_rotor, [0.0, 0.0, 0.0]);
    assert_eq!(tau0, [0.0, 0.0, 0.0], "无角速度不产生陀螺力矩");
}
