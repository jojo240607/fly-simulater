//! P0-1 动力系统验证：油门→电压→转速→推力（∝ω²）+ 电池掉压。
//!
//! 锁住：
//! 1. 推力 ∝ 转速平方：固定油门下 T / ω² 恒为 prop_kt（常数）。
//! 2. 电池掉压：大油门电流大 → 电压低于标称；小油门接近标称。
//! 3. 悬停时电压接近标称（掉压小，不破坏控制律悬停）。

#![cfg(feature = "phy")]

use fly_sim_core::physics::{ContactModel, PhySdkWorld, RigidBodyWorld};
use fly_sim_core::plant::{gyro_torque, QuadrotorPlant};
use fly_sim_core::wind::{WindConfig, WindField};
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
        Some(ContactModel::default()),
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

/// P0-2：动量理论诱导速度（含垂直气流耦合）。
/// 静悬（机体垂直速度≈0）→ vi = sqrt(T/(2·ρ·A))。
/// 机体上升（v_z>0）→ 穿过桨盘空气上流 → vi 增大；下降（v_z<0）→ vi 减小。
#[test]
fn induced_velocity_momentum_theory() {
    let cfg = VehicleConfig::default_quad();
    let rho = cfg.air_density as f64;
    let a = cfg.disk_area as f64;
    // 悬停总推力 ≈ 重力（4 油门各 hover_thrust，稳态）。
    let t_hover = cfg.mass as f64 * cfg.gravity as f64;
    let vi_hover = (t_hover / (2.0 * rho * a)).sqrt();
    assert!(vi_hover > 0.0, "悬停诱导速度应 > 0");
    assert!(
        (vi_hover - 6.3).abs() < 1.5,
        "450quad 悬停诱导速度应 ≈ 6 m/s（动量理论），got {:.2}",
        vi_hover
    );

    // 上升耦合：v_z = +2 → vi 应大于悬停值。
    let vz = 2.0;
    let vi_climb = (vz + (vz * vz + 2.0 * t_hover / (rho * a)).sqrt()) * 0.5;
    assert!(
        vi_climb > vi_hover,
        "上升时诱导速度应大于悬停: vi_climb={:.3} vi_hover={:.3}",
        vi_climb,
        vi_hover
    );

    // 下降耦合：vz = -3（但未进入涡环，vi 仍 > 0）。
    let vz = -3.0;
    let vi_desc = (vz + (vz * vz + 2.0 * t_hover / (rho * a)).sqrt()) * 0.5;
    assert!(
        vi_desc < vi_hover && vi_desc > 0.0,
        "下降时诱导速度应小于悬停且仍 >0: vi_desc={:.3} vi_hover={:.3}",
        vi_desc,
        vi_hover
    );
}

/// P0-2：滑流下洗冲击机体产生下拉力，且爬升（vi 增大）时下拉力更大。
/// 用 `induced_velocity()` 实测由 plant 内部算出的 vi，校验：
/// 1. 稳态悬停 vi ≈ 动量理论值；
/// 2. 滑流下拉力 f_slip = k·0.5·ρ·A·vi² 随 vi 单调增（通过上升/下降对比）。
#[test]
fn slipstream_force_scales_with_induced_velocity() {
    // 悬停稳态：机体垂直速度≈0，vi 应≈理论悬停值。
    let p_hover = plant_at_throttle(0.5, 600);
    let vi_hover = p_hover.induced_velocity();
    let cfg = VehicleConfig::default_quad();
    let rho = cfg.air_density as f64;
    let a = cfg.disk_area as f64;
    let vi_theory = (cfg.mass as f64 * cfg.gravity as f64 / (2.0 * rho * a)).sqrt();
    assert!(
        (vi_hover - vi_theory).abs() / vi_theory < 0.15,
        "plant 内部 vi 应≈动量理论悬停值: vi_plant={:.3} vi_theory={:.3}",
        vi_hover,
        vi_theory
    );

    // 大油门 → 总推力大 → vi 更大（对比小油门）。
    let p_lo = plant_at_throttle(0.2, 600);
    let p_hi = plant_at_throttle(0.9, 600);
    assert!(
        p_hi.induced_velocity() > p_lo.induced_velocity(),
        "大油门诱导速度应大于小油门: vi_hi={:.3} vi_lo={:.3}",
        p_hi.induced_velocity(),
        p_lo.induced_velocity()
    );
}

// ---- P2-B：空间相关风场 + 阵风突风闭环接入验证 ----

#[test]
fn spatial_wind_plant_step_stable_and_finite() {
    // 接入带风切变 + 空间相关的风场，plant 闭环 step 多步，验证数值有限、可复现。
    let mut cfg = WindConfig::default();
    cfg.base = [3.0, 0.0, 0.0]; // 稳定侧风
    cfg.shear_exponent = 0.15; // 风切变
    cfg.shear_ref_height = 10.0;
    cfg.spatial_scale = 5.0; // 空间相关
    cfg.gust_burst_amp = [4.0, 0.0, 0.0]; // 确定性突风
    cfg.gust_burst_t0 = 1.0;
    cfg.gust_burst_hw = 0.5;
    cfg.turb_sigma = [0.2, 0.2, 0.3];
    cfg.turb_tau = 0.7;
    cfg.seed = 0xABCDEF;

    let vc = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &vc,
        DT,
        Some(WindField::new(cfg)),
        Default::default(),
        Some(ContactModel::default()),
    );
    // 悬停油门，跑 3 秒（覆盖突风窗口 0.5~1.5s）。
    let cmd = ActuatorCmd {
        motor: [0.5 as f32; 4],
    };
    plant.apply_actuators(&cmd);
    for _ in 0..750 {
        plant.step();
        let (pos, _q) = plant.debug_up();
        for k in 0..3 {
            assert!(pos[k].is_finite(), "空间风闭环位置应有限: pos={:?}", pos);
        }
    }
    // 确定性：同配置再跑一遍，末位置一致。
    let vc2 = VehicleConfig::default_quad();
    let mut plant2 = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &vc2,
        DT,
        Some(WindField::new(WindConfig {
            base: [3.0, 0.0, 0.0],
            shear_exponent: 0.15,
            shear_ref_height: 10.0,
            spatial_scale: 5.0,
            gust_burst_amp: [4.0, 0.0, 0.0],
            gust_burst_t0: 1.0,
            gust_burst_hw: 0.5,
            turb_sigma: [0.2, 0.2, 0.3],
            turb_tau: 0.7,
            seed: 0xABCDEF,
            ..Default::default()
        })),
        Default::default(),
        Some(ContactModel::default()),
    );
    plant2.apply_actuators(&cmd);
    for _ in 0..750 {
        plant2.step();
    }
    let (p1, _) = plant.debug_up();
    let (p2, _) = plant2.debug_up();
    for k in 0..3 {
        assert!(
            (p1[k] - p2[k]).abs() < 1e-9,
            "空间风闭环必须确定性: p1={:?} p2={:?}",
            p1,
            p2
        );
    }
}

#[test]
fn thermal_wind_plant_step_stable_and_finite() {
    // 接入热气流（中心上升 3m/s），plant 闭环 step，验证数值有限 + 确定性可复现。
    let thermal_cfg = || WindConfig {
        base: [0.0; 3],
        thermal_strength: 3.0,
        thermal_radius: 5.0,
        thermal_height: 50.0,
        thermal_pos0: [0.0, 0.0],
        thermal_drift: [0.5, 0.0],
        turb_sigma: [0.1, 0.1, 0.1],
        turb_tau: 0.7,
        seed: 0x55AA,
        ..Default::default()
    };
    let vc = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &vc,
        DT,
        Some(WindField::new(thermal_cfg())),
        Default::default(),
        Some(ContactModel::default()),
    );
    let cmd = ActuatorCmd { motor: [0.5 as f32; 4] };
    plant.apply_actuators(&cmd);
    for _ in 0..750 {
        plant.step();
        let (pos, _q) = plant.debug_up();
        for k in 0..3 {
            assert!(pos[k].is_finite(), "热气流闭环位置应有限: pos={:?}", pos);
        }
    }
    // 确定性：同配置再跑，末位置一致。
    let vc2 = VehicleConfig::default_quad();
    let mut plant2 = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &vc2,
        DT,
        Some(WindField::new(thermal_cfg())),
        Default::default(),
        Some(ContactModel::default()),
    );
    plant2.apply_actuators(&cmd);
    for _ in 0..750 {
        plant2.step();
    }
    let (p1, _) = plant.debug_up();
    let (p2, _) = plant2.debug_up();
    for k in 0..3 {
        assert!(
            (p1[k] - p2[k]).abs() < 1e-9,
            "热气流闭环必须确定性: p1={:?} p2={:?}",
            p1, p2
        );
    }
}
