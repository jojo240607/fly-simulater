//! P0-1 动力系统验证：油门→电压→转速→推力（∝ω²）+ 电池掉压。
//!
//! 锁住：
//! 1. 推力 ∝ 转速平方：固定油门下 T / ω² 恒为 prop_kt（常数）。
//! 2. 电池掉压：大油门电流大 → 电压低于标称；小油门接近标称。
//! 3. 悬停时电压接近标称（掉压小，不破坏控制律悬停）。

#![cfg(feature = "phy")]

use fly_sim_core::physics::{ContactModel, Obstacle, PhySdkWorld};
use fly_sim_core::plant::{bet_rotor, downwash_coupling, gyro_torque, QuadrotorPlant};
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
        Vec::new(),
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
        Vec::new(),
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
        Vec::new(),
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
        Vec::new(),
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
        Vec::new(),
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

/// P1-2 续：障碍碰撞 plant 集成（经 set_obstacles 注入）。
/// 机体从高处自由落体撞向球障碍，应被偏离而非穿透；状态全程有限。
#[test]
fn obstacle_collision_plant_integration() {
    let cfg = VehicleConfig::default_quad();
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        None,
        Default::default(),
        Some(ContactModel::default()),
        Vec::new(),
    );
    // 球障碍：中心 (0,0,-3) 半径 1，机体起始于球正上方 (0,0,-1.5) 带向下速度。
    plant.set_obstacles(vec![Obstacle::Sphere {
        center: [0.0, 0.0, -3.0],
        radius: 1.0,
    }]);
    let cmd = ActuatorCmd { motor: [0.0 as f32; 4] };
    plant.apply_actuators(&cmd);
    for _ in 0..1000 {
        plant.step();
        let (pos, _) = plant.debug_up();
        assert!(pos.iter().all(|v| v.is_finite()), "障碍碰撞中位置应有限: {:?}", pos);
    }
    let (pos, _) = plant.debug_up();
    // 机体碰撞球半径 = 1.2*arm_length；球表面距中心 1.0；应停在表面附近而非穿透。
    // 取北-东-下三维距离球心：
    let d = (pos[0].powi(2) + pos[1].powi(2) + (pos[2] + 3.0).powi(2)).sqrt();
    let body_r = 1.2 * cfg.arm_length as f64;
    // 不允许深穿透（> 0.5m）
    assert!(d > 1.0 - 0.5, "不应深穿透障碍球, d={} (球半径+机体半径≈{})", d, 1.0 + body_r);
}

// ============================================================ P3-C1 叶素理论（BET）升级验收
// 验收点：
// 1. 悬停保零回归：BET 悬停推力必须精确等于旧 ∝ω² 模型（thrust = prop_kt·ω²）；
// 2. 前飞推力-速度曲线：随前进比 μ 增大，推力单调衰减（失速）、桨盘平面 H 力增大；
// 3. 失速边界：μ 进入失速区后反扭矩放大（torque_factor>2）、推力塌陷（<70%）；
// 4. 挥舞展开：前飞纵向挥舞角 a1 随 μ 单调增大（低头，负值更负）；
// 5. plant 级悬停回归：BET 接入后稳态 Σ(prop_kt·ω²) ≈ 重力。

/// P3-C1：BET 内核级参数（派生自 default_quad，与 plant 内部悬停标定一致）。
/// 返回 (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, prop_kt, omega_max)。
fn bet_derived() -> (
    f64, f64, f64, f64, f64, f64, f64, f64, f64, f64,
) {
    let cfg = VehicleConfig::default_quad();
    let rho = cfg.air_density as f64;
    let a_rotor = (cfg.disk_area as f64).max(1e-4) / 4.0;
    let r_rotor = (a_rotor / std::f64::consts::PI).sqrt();
    let sigma = cfg.rotor_solidity as f64;
    let cla = cfg.rotor_cl_alpha as f64;
    let cd0 = cfg.rotor_cd0 as f64;
    let stall = cfg.rotor_stall_alpha as f64;
    let omega_max = cfg.motor_kv as f64 * cfg.battery_v_nom as f64;
    let prop_kt = cfg.thrust_coeff as f64 / (omega_max * omega_max);
    let ct_hover = prop_kt / (rho * a_rotor * r_rotor * r_rotor);
    let lam_i_hover = (prop_kt / (2.0 * rho * a_rotor)).sqrt() / r_rotor;
    let theta0 = 3.0 * (4.0 * ct_hover / (sigma * cla) + 0.5 * lam_i_hover);
    (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, prop_kt, omega_max)
}

/// 悬停状态（油门 u=0.5）下的 BET 输入：转速 ω、诱导速度 vi、叶尖速度 vt。
fn bet_hover_inputs() -> (f64, f64, f64) {
    let (rho, _a, r_rotor, _s, _c, _cd, _st, _t0, prop_kt, omega_max) = bet_derived();
    let u = 0.5;
    let t_req = VehicleConfig::default_quad().thrust_coeff as f64 * u;
    let omega = (t_req / prop_kt).sqrt();
    assert!(omega < omega_max, "悬停转速应低于最大转速（无掉压）");
    let a_disk = VehicleConfig::default_quad().disk_area as f64;
    let sum_t = 4.0 * prop_kt * omega * omega;
    let vi = (2.0 * sum_t / (rho * a_disk)).max(0.0).sqrt() * 0.5;
    let vt = omega * r_rotor;
    (omega, vi, vt)
}

/// P3-C1 验收 1：悬停保零回归。
#[test]
fn bet_hover_matches_legacy_omega_squared() {
    let (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, prop_kt, _mx) = bet_derived();
    let (omega, vi, _vt) = bet_hover_inputs();

    let bet = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, 0.0, vi, theta0, sigma, cla, cd0, stall);
    let expected = prop_kt * omega * omega;
    assert!(
        (bet.thrust - expected).abs() / expected < 1e-6,
        "BET 悬停推力应=prop_kt·ω²: bet={:.6} expected={:.6}",
        bet.thrust,
        expected
    );
    // 悬停 μ=0：无失速放大 / 无桨盘平面 H 力 / 无挥舞。
    assert!(
        (bet.torque_factor - 1.0).abs() < 0.05,
        "悬停反扭矩因子应≈1: {}",
        bet.torque_factor
    );
    assert!(bet.h_force.abs() < 1e-9, "悬停无桨盘平面 H 力: {}", bet.h_force);
    assert!(bet.a1.abs() < 1e-9, "悬停无挥舞: {}", bet.a1);
}

/// P3-C1 验收 2：前飞推力-速度曲线——推力单调衰减，H 力单调增大。
#[test]
fn bet_thrust_declines_and_h_grows_with_forward_speed() {
    let (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, _kt, _mx) = bet_derived();
    let (omega, vi, vt) = bet_hover_inputs();

    let mut prev_t = f64::MAX;
    let mut prev_h = f64::MIN;
    for mu10 in 0..=8 {
        let mu = mu10 as f64 / 10.0; // 0.0 .. 0.8
        let bet = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, mu * vt, vi, theta0, sigma, cla, cd0, stall);
        assert!(
            bet.thrust <= prev_t + 1e-9,
            "推力应随前飞单调衰减: mu={} thrust={:.6} prev={:.6}",
            mu, bet.thrust, prev_t
        );
        prev_t = bet.thrust;
        if mu10 > 0 {
            assert!(
                bet.h_force > prev_h,
                "桨盘平面 H 力应随前飞增大: mu={} h={:.6} prev={:.6}",
                mu, bet.h_force, prev_h
            );
        }
        prev_h = bet.h_force;
    }
}

/// P3-C1 验收 3：失速边界——反扭矩放大、推力塌陷。
#[test]
fn bet_stall_boundary_amplifies_torque_and_collapses_thrust() {
    let (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, _kt, _mx) = bet_derived();
    let (omega, vi, vt) = bet_hover_inputs();

    let hover = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, 0.0, vi, theta0, sigma, cla, cd0, stall);
    let deep = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, 0.85 * vt, vi, theta0, sigma, cla, cd0, stall);
    // 深失速（μ=0.85）：反扭矩放大 > 2，推力塌陷到悬停 70% 以下；悬停不放大。
    assert!(
        deep.torque_factor > 2.0,
        "深失速反扭矩应放大>2: {}",
        deep.torque_factor
    );
    assert!(
        deep.thrust < hover.thrust * 0.70,
        "深失速推力应塌陷<70%: {} vs {}",
        deep.thrust,
        hover.thrust
    );
    assert!(
        hover.torque_factor < 1.1,
        "悬停不应失速放大: {}",
        hover.torque_factor
    );
}

/// P3-C1 验收 4：挥舞展开——前飞纵向挥舞角 a1 随 μ 单调增大（低头，负值更负）。
#[test]
fn bet_a1_flaps_down_with_forward_speed() {
    let (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, _kt, _mx) = bet_derived();
    let (omega, vi, vt) = bet_hover_inputs();

    let mut prev_abs = 0.0f64;
    for mu10 in 0..=8 {
        let mu = mu10 as f64 / 10.0;
        let bet = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, mu * vt, vi, theta0, sigma, cla, cd0, stall);
        assert!(
            bet.a1.abs() >= prev_abs - 1e-12,
            "|a1| 应随前飞单调增: mu={} a1={:.6}",
            mu,
            bet.a1
        );
        assert!(bet.a1 <= 0.0, "前飞挥舞应为低头（负）: mu={} a1={:.6}", mu, bet.a1);
        prev_abs = bet.a1.abs();
    }
}

/// P3-C1 验收 5：plant 级悬停回归——BET 接入后稳态 Σ(prop_kt·ω²) ≈ 重力。
/// 初始高度 5m >> 地面效应高度上限（0.135m），无地面效应干扰。
#[test]
fn bet_plant_hover_regression_holds() {
    let cfg = VehicleConfig::default_quad();
    let omega_max = cfg.motor_kv as f64 * cfg.battery_v_nom as f64;
    let prop_kt = cfg.thrust_coeff as f64 / (omega_max * omega_max);
    let p = plant_at_throttle(0.5, 600);
    let (_v, w) = p.powertrain_state();
    let total_t: f64 = w.iter().map(|w| prop_kt * w * w).sum();
    let mg = cfg.mass as f64 * cfg.gravity as f64;
    assert!(
        (total_t - mg).abs() / mg < 0.05,
        "BET 悬停总推力应≈mg: total_t={:.3} mg={:.3}",
        total_t,
        mg
    );
}

// ============================================================
// P3-C2：桨盘干扰（相邻桨下洗耦合修正项）
//
// 前飞时上游桨的滑流（下洗 vi）被自由流吹向下游，部分射入下游桨盘 → 下游桨入流比
// 增大、推力下降，前后桨不对称 → 前飞俯仰干扰力矩。悬停（v_xy=0）滑流垂直向下、桨盘
// 共面互不干扰，耦合为 0（保 P3-C1 悬停标定）；k=0 关闭。
//
// 验收 1：悬停 / 关闭 / 无下洗 → 全 0（保零回归）。
// 验收 2：前飞只命中下游桨（上游桨零耦合），且左右对称。
// 验收 3：耦合量随 k、vi 单调增大（且∝k·vi 线性）。
// 验收 4：下游桨入流增大 → BET 推力低于上游桨（前后桨不对称的机理源头）。
// 验收 5：plant 级悬停回归——k=0.35 与 k=0 悬停推力一致（保零回归）。
// 验收 6：吹送系数——耦合随前飞速度单调增大（悬停 0，高速饱和）。

/// X 布局臂向量（与 `plant::step` 内力矩约定一致）：m0=前右, m1=后左, m2=前左, m3=后右。
fn arms_x() -> [[f64; 2]; 4] {
    let l = VehicleConfig::default_quad().arm_length as f64;
    [[l, l], [-l, -l], [l, -l], [-l, l]]
}

/// P3-C2 验收 1：悬停 / 关闭 / 无下洗 → 全 0。
#[test]
fn downwash_coupling_zero_when_hover_disabled_or_no_downwash() {
    let arms = arms_x();
    let vi = 5.0;
    // 悬停（v_xy=0）：上游方向未定义 → 全 0。
    assert_eq!(downwash_coupling([0.0, 0.0], arms, vi, 0.35), [0.0; 4]);
    // 速度低于阈值（<1e-3）视为悬停。
    assert_eq!(downwash_coupling([0.0005, 0.0], arms, vi, 0.35), [0.0; 4]);
    // 关闭（k=0）。
    assert_eq!(downwash_coupling([8.0, 0.0], arms, vi, 0.0), [0.0; 4]);
    // 无下洗（vi=0）。
    assert_eq!(downwash_coupling([8.0, 0.0], arms, 0.0, 0.35), [0.0; 4]);
}

/// P3-C2 验收 2：前飞只命中下游桨，且左右对称。
#[test]
fn downwash_coupling_targets_downstream_rotors_only() {
    let arms = arms_x();
    let vi = 5.0;
    let k = 0.35;
    // 正前飞（+X 机体前向）：前桨 m0/m2 无上游，后桨 m1/m3 被前桨滑流覆盖。
    let fwd = downwash_coupling([10.0, 0.0], arms, vi, k);
    assert_eq!(fwd[0], 0.0, "前右桨不应有耦合");
    assert_eq!(fwd[2], 0.0, "前左桨不应有耦合");
    assert!(fwd[1] > 0.0, "后左桨应受下洗耦合");
    assert!(fwd[3] > 0.0, "后右桨应受下洗耦合");
    assert!(
        (fwd[1] - fwd[3]).abs() < 1e-12,
        "正前飞左右对称: {} vs {}",
        fwd[1],
        fwd[3]
    );
    // 正右侧飞（+Y）：右桨 m0/m3 无上游，左桨 m1/m2 被覆盖。
    let side = downwash_coupling([0.0, 10.0], arms, vi, k);
    assert_eq!(side[0], 0.0, "右前桨不应有耦合");
    assert_eq!(side[3], 0.0, "右后桨不应有耦合");
    assert!(side[1] > 0.0 && side[2] > 0.0, "左桨应受下洗耦合: {:?}", side);
    // 耦合量级：同侧满覆盖 frac≈1、对角指数衰减，总 frac≈1+ε；再乘吹送系数
    // blow=|v|/(|v|+vi)（v=10, vi=5 → blow=2/3）。量级≈k·blow·vi。
    let blow = 10.0 / (10.0 + vi);
    assert!(
        fwd[1] > k * blow * vi * 0.9 && fwd[1] <= k * blow * vi * 2.0,
        "后桨耦合量级应≈k·blow·vi: {} (k·blow·vi={})",
        fwd[1],
        k * blow * vi
    );
}

/// P3-C2 验收 3：耦合量随 k、vi 单调增大（且∝k·vi 线性）。
#[test]
fn downwash_coupling_scales_with_k_and_vi() {
    let arms = arms_x();
    let rear = 1; // 后左桨
    let a = downwash_coupling([8.0, 0.0], arms, 5.0, 0.2)[rear];
    let b = downwash_coupling([8.0, 0.0], arms, 5.0, 0.35)[rear];
    let c = downwash_coupling([8.0, 0.0], arms, 5.0, 0.5)[rear];
    assert!(b > a && c > b, "耦合应随 k 单调增大: {a} < {b} < {c}");
    let d = downwash_coupling([8.0, 0.0], arms, 3.0, 0.35)[rear];
    let e = downwash_coupling([8.0, 0.0], arms, 5.0, 0.35)[rear];
    assert!(e > d, "耦合应随 vi 单调增大: {d} < {e}");
    // 线性：frac 与 k、vi 无关 → coupling ∝ k·vi。
    let ratio = b / a;
    assert!(
        (ratio - 0.35 / 0.2).abs() < 1e-9,
        "耦合应∝k 线性: {ratio} vs {}",
        0.35 / 0.2
    );
}

/// P3-C2 验收 4：下游桨入流增大 → BET 推力低于上游桨（前后桨不对称的机理源头）。
/// BEMT：C_T = (σa/4)·(θ0/3 − λ/2)，λ 增大 → C_T 下降。
#[test]
fn downwash_coupling_inflow_reduces_downstream_thrust() {
    let (rho, a_rotor, r_rotor, sigma, cla, cd0, stall, theta0, _kt, _mx) = bet_derived();
    let (omega, vi, _vt) = bet_hover_inputs();
    let vi_coup = downwash_coupling([10.0, 0.0], arms_x(), vi, 0.35);
    // 悬停：前桨（无耦合）vs 后桨（入流增大 vi+vi_coup[1]），同 ω 同桨距。
    let front =
        bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, 0.0, vi, theta0, sigma, cla, cd0, stall);
    let rear = bet_rotor(
        rho, omega, r_rotor, a_rotor, 0.0, 0.0, vi + vi_coup[1], theta0, sigma, cla, cd0, stall,
    );
    assert!(
        rear.thrust < front.thrust * 0.9,
        "下游桨入流增大应明显降推: front={:.4} rear={:.4}",
        front.thrust,
        rear.thrust
    );
    assert!(rear.thrust > 0.0, "下游桨推力应仍为正: {}", rear.thrust);
    // 前飞 μ 扫掠：全程下游桨推力都低于上游桨。
    let vt = omega * r_rotor;
    for mu10 in 1..=8 {
        let mu = mu10 as f64 / 10.0;
        let f = bet_rotor(rho, omega, r_rotor, a_rotor, 0.0, mu * vt, vi, theta0, sigma, cla, cd0, stall)
            .thrust;
        let r = bet_rotor(
            rho,
            omega,
            r_rotor,
            a_rotor,
            0.0,
            mu * vt,
            vi + vi_coup[1],
            theta0,
            sigma,
            cla,
            cd0,
            stall,
        )
        .thrust;
        assert!(
            r < f,
            "前飞 μ={mu} 下游推力应低于上游: front={f:.4} rear={r:.4}"
        );
    }
}

/// 以自定义 config 构造悬停稳态 plant（P3-C2 用于对比 k=默认(0.25) 与 k=0）。
fn plant_at_throttle_cfg(
    cfg: &VehicleConfig,
    u: f64,
    settle_steps: usize,
) -> QuadrotorPlant<PhySdkWorld> {
    let mut plant = QuadrotorPlant::new(
        PhySdkWorld::create_empty(),
        cfg,
        DT,
        None,
        Default::default(),
        Some(ContactModel::default()),
        Vec::new(),
    );
    let cmd = ActuatorCmd {
        motor: [u as f32; 4],
    };
    plant.apply_actuators(&cmd);
    for _ in 0..settle_steps {
        plant.step();
    }
    plant
}

/// P3-C2 验收 5：plant 级悬停回归——k=默认(0.25) 与 k=0 悬停推力一致（保零回归）。
#[test]
fn downwash_coupling_plant_hover_regression_unchanged() {
    let cfg_default = VehicleConfig::default_quad();
    let mut cfg_off = VehicleConfig::default_quad();
    cfg_off.rotor_downwash_coupling = 0.0;

    let omega_max = cfg_default.motor_kv as f64 * cfg_default.battery_v_nom as f64;
    let prop_kt = cfg_default.thrust_coeff as f64 / (omega_max * omega_max);

    let p_on = plant_at_throttle_cfg(&cfg_default, 0.5, 600);
    let p_off = plant_at_throttle_cfg(&cfg_off, 0.5, 600);
    let t_on: f64 = p_on
        .powertrain_state()
        .1
        .iter()
        .map(|w| prop_kt * w * w)
        .sum();
    let t_off: f64 = p_off
        .powertrain_state()
        .1
        .iter()
        .map(|w| prop_kt * w * w)
        .sum();
    let mg = cfg_default.mass as f64 * cfg_default.gravity as f64;
    // 悬停 v_xy≈0 → 耦合=0 → 两者回归一致（且都≈mg）。
    assert!(
        (t_on - mg).abs() / mg < 0.05 && (t_off - mg).abs() / mg < 0.05,
        "悬停回归应≈mg: k=default→{t_on:.3}, k=0→{t_off:.3}, mg={mg:.3}"
    );
    assert!(
        (t_on - t_off).abs() / mg < 1e-3,
        "悬停开启/关闭桨盘干扰应一致（保零回归）: {t_on:.3} vs {t_off:.3}"
    );
}

/// P3-C2 验收 6：吹送系数——耦合随前飞速度单调增大（悬停 0，高速饱和）。
/// 悬停滑流垂直向下、桨盘共面互不干扰；前飞速度越大滑流越被吹向下游扫入下游桨盘。
#[test]
fn downwash_coupling_grows_with_forward_speed() {
    let arms = arms_x();
    let vi = 5.0;
    let rear = 1; // 后左桨
    let mut prev = 0.0f64;
    for sp in [0.5, 1.0, 2.0, 4.0, 6.0, 10.0, 20.0] {
        let c = downwash_coupling([sp, 0.0], arms, vi, 0.35)[rear];
        assert!(
            c >= prev - 1e-12,
            "耦合应随前飞速度单调增大: v={sp} c={c:.4} prev={prev:.4}"
        );
        prev = c;
    }
    // 高速饱和渐近：v=20 vs v=10 增幅小（吹送系数趋近 1）。
    let c10 = downwash_coupling([10.0, 0.0], arms, vi, 0.35)[rear];
    let c20 = downwash_coupling([20.0, 0.0], arms, vi, 0.35)[rear];
    assert!(
        (c20 - c10) / c10 < 0.35,
        "高速耦合应趋近饱和: v=10→{c10:.4} v=20→{c20:.4}"
    );
    // 低速（悬停附近）耦合远小于高速饱和值：低速不显著惩罚推进效率。
    let c1 = downwash_coupling([1.0, 0.0], arms, vi, 0.35)[rear];
    assert!(
        c1 < c20 * 0.30,
        "低速耦合应远小于高速饱和值（避免低速过度惩罚）: {c1:.4} vs {c20:.4}"
    );
}
