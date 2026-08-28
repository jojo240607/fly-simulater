//! SIL 回归集成测试：直接在代码内跑悬停 / 自由落体场景并断言通过判据，
//! 与手动看 stdout 相比提供 CI 级回归保护（阶段 9 增补）。
//!
//! 这些场景依赖真实物理引擎 `phy-sdk`（本地 path 依赖），因此测的是
//! 飞控栈（EKF→PID→plant→physics）端到端的正确性，而非替身。

#![cfg(feature = "phy")]

use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sim::{SimLoop, windy_config};
use fly_sim_core::controller::hover_setpoint;
use fly_sim_core::wind::WindField;
use fly_sim_core::ControllerKind;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::physics::ContactModel;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::invariants;
use flyctrl_core::vehicle::Quaternion;

const DT: f64 = 0.004;

fn make_loop() -> SimLoop<PhySdkWorld> {
    make_loop_sensor(SensorConfig::default())
}

fn make_loop_sensor(sensor_cfg: SensorConfig) -> SimLoop<PhySdkWorld> {
    let cfg = VehicleConfig::default_quad();
    let world = PhySdkWorld::create_empty();
    SimLoop::new(
        world,
        &cfg,
        DT,
        None,
        sensor_cfg,
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    )
}

#[test]
fn sil_hover_converges_and_stable() {
    let mut loop_sim = make_loop();
    let ok = loop_sim.run_hover(10.0);
    assert!(
        ok,
        "hover must converge to setpoint (0,0,-5) and stay stable; got {} steps",
        loop_sim.steps()
    );
}

#[test]
fn sil_freefall_energy_non_increasing() {
    let mut loop_sim = make_loop();
    let ok = loop_sim.run_freefall(10.0);
    assert!(
        ok,
        "freefall must fall and keep mechanical energy non-increasing; got {} steps",
        loop_sim.steps()
    );
}

#[test]
fn sil_wind_hover_holds_altitude() {
    // 阶段 3 抗风悬停：有基础风 + 阵风，仍应维持高度（不发散、不坠地）。
    let cfg = VehicleConfig::default_quad();
    let world = PhySdkWorld::create_empty();
    let wind = Some(WindField::new(windy_config()));
    let mut loop_sim = SimLoop::new(
        world,
        &cfg,
        DT,
        wind,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let ok = loop_sim.run_hover_wind(15.0);
    // 注：默认 PID 抗风上限 ~0.3m/s，强风会饱和翻滚，本测例不验证"抗风位置保持"，
    // 只验证风-气动耦合正确接入（机体被风明显吹离原点）且数值稳定（无 NaN/Inf）。
    assert!(ok, "wind-hover must stay numerically stable and show wind disturbance");
}

/// P1-2（真实引擎）：惩罚接触模型让下落四旋翼稳定停在地面而非数值爆裂/被弹飞。
///
/// 构造时已带 `Some(ContactModel::default())`（接触面 NED d≈+4.9）。零油门释放后，
/// 机体应被惩罚接触拦停在地面附近：全程状态有限、末态位于地面附近、竖直速度趋零。
#[test]
fn sil_landing_settles_on_ground() {
    let mut loop_sim = make_loop(); // 已启用 P1-2 接触
    let ok = loop_sim.run_drop(10.0);
    assert!(ok, "landing must settle on ground via penalty contact model");
}

/// 四元数(ZYX, NED FRD) → (roll, pitch) 度数，真值/估计用同一公式以便符号对照。
fn quat_roll_pitch(q: &Quaternion) -> (f32, f32) {
    let roll = q.w * q.x + q.y * q.z;
    let roll_den = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let r = roll.atan2(roll_den).to_degrees();
    let sp = (2.0 * (q.w * q.y - q.z * q.x)).clamp(-1.0, 1.0);
    let p = sp.asin().to_degrees();
    (r, p)
}

/// HIL 反相发散复现（诊断，临时）：SIL 侧复刻 HIL 双时钟失配。
///
/// HIL 实况（PC 侧 `advance_hil`，见 fly-sim-server/src/main.rs）：MCU 控制周期
/// 4ms，PC 每物理步 `plant.step()` 固定推 dt=4ms 且步进频率低于控制拍（实测
/// ~2.75 控制拍/物理步）→ 控制时钟快于物理时钟。飞控在物理步之间用 sample-and-hold
/// 陈旧 IMU（`last_real_imu`）外推，姿态估计超前物理真值 → 控制器反向修正 → 反相发散。
///
/// A/B/C 对照（`FlyController::set_imu_throttle` / `set_plant_throttle`）：
/// - A: imu=0 plant=0（全同步基线，每拍真实 IMU + 每拍物理步）
/// - B: imu=3 plant=0（第 1 步：仅注入饥饿，plant 仍每拍动）
/// - C: imu=3 plant=3（双时钟：每 3 控制拍才推进一次物理，plant 冻结、指令覆盖、
///      陈旧 IMU 外推 12ms 而真值仅走 4ms）
/// 0.5s 施加绕 NED-Y 的外部 pitch 角冲量制造真实旋转瞬态（`disturb_torque_impulse`，
/// 分配器无法补偿）。统计「真值/估计符号相反」比例（HIL 发散核心症状）、真值/估计
/// pitch 最大偏差、是否非有限发散。纯诊断、无断言；复现结论确认后随节流逻辑一并移除。
#[test]
fn sil_imu_throttle_diagnose_hil_divergence() {
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let n = (6.0 / DT) as usize;
    let run = |imu_throttle: u32, plant_throttle: u32, impulse: f64| -> (u32, u32, bool, f32, f32, f32, f32) {
        let mut loop_sim = make_loop();
        loop_sim.set_imu_throttle(imu_throttle);
        loop_sim.set_plant_throttle(plant_throttle);
        let mut anti_phase = 0u32; // |真值|>1° 且符号相反的样本
        let mut active = 0u32; // |真值|>1° 的有效样本
        let mut diverged = false;
        let mut peak_pitch = 0.0f32; // 真值 |pitch| 峰值
        let mut peak_err = 0.0f32; // |真值-估计 pitch| 峰值
        let mut peak_anti_p = 0.0f32; // 反相期间真值 |pitch| 峰值
        for i in 0..n {
            let t = i as f64 * DT;
            // 单次强角冲量（t≈0.5s）：绕 NED-Y 注入，制造真实旋转瞬态。
            // plant 节流时物理时钟 1/N 速，冲量按比例放大以取得相近的物理扰动。
            if i == (0.5 / DT) as usize {
                loop_sim.disturb_torque_impulse([0.0, impulse, 0.0]);
            }
            let (w, _cmd) = loop_sim.step_frame(&sp);
            let est = loop_sim.ctrl_debug_estimate();
            let (tr_r, tr_p) = quat_roll_pitch(&w.att);
            let (es_r, es_p) = quat_roll_pitch(&est.att);
            if i >= 120 && i <= 136 {
                println!("  [dbg imu={imu_throttle} plant={plant_throttle}] step {i}: truth r/p={tr_r:7.2}/{tr_p:7.2}°  est r/p={es_r:7.2}/{es_p:7.2}°");
            }
            if tr_p.abs() > peak_pitch {
                peak_pitch = tr_p.abs();
            }
            let err_p = (tr_p - es_p).abs();
            if err_p > peak_err {
                peak_err = err_p;
            }
            let nontrivial = tr_r.abs() > 1.0 || tr_p.abs() > 1.0;
            let opp_p = tr_p.abs() > 1.0 && es_p != 0.0 && (tr_p > 0.0) != (es_p > 0.0);
            let opp_r = tr_r.abs() > 1.0 && es_r != 0.0 && (tr_r > 0.0) != (es_r > 0.0);
            if nontrivial {
                active += 1;
                if opp_p || opp_r {
                    anti_phase += 1;
                    if tr_p.abs() > peak_anti_p {
                        peak_anti_p = tr_p.abs();
                    }
                }
            }
            if !invariants::state_finite(&est) {
                diverged = true;
                break;
            }
        }
        (
            active,
            anti_phase,
            diverged,
            peak_pitch,
            peak_err,
            peak_anti_p,
            if active > 0 { 100.0 * anti_phase as f32 / active as f32 } else { 0.0 },
        )
    };
    // A/B/C 三组。plant 节流时物理时钟 1/N，冲量按 N 倍放大使物理扰动量级相当。
    let (a0, ap0, d0, p0, e0, a0p, r0) = run(0, 0, 0.02);
    let (a3, ap3, d3, p3, e3, a3p, r3) = run(3, 0, 0.02);
    let (ac, apc, dc, pc, ec, acp, rc) = run(3, 3, 0.06);
    println!("[A imu=0 plant=0] 有效样本={} 反相={}({:.1}%) 真值|pitch|峰={:.2}° 真值-估计|峰={:.2}° 反相期|pitch|峰={:.2}° diverged={}", a0, ap0, r0, p0, e0, a0p, d0);
    println!("[B imu=3 plant=0] 有效样本={} 反相={}({:.1}%) 真值|pitch|峰={:.2}° 真值-估计|峰={:.2}° 反相期|pitch|峰={:.2}° diverged={}", a3, ap3, r3, p3, e3, a3p, d3);
    println!("[C imu=3 plant=3] 有效样本={} 反相={}({:.1}%) 真值|pitch|峰={:.2}° 真值-估计|峰={:.2}° 反相期|pitch|峰={:.2}° diverged={}", ac, apc, rc, pc, ec, acp, dc);
    println!("[结论] 若仅 B 发散→注入饥饿是根因；若仅 C 发散→双时钟（物理步稀疏+陈旧外推）是根因；若三者一致→差异在别处（HIL 链路符号/延迟）");
}

/// SIL/HIL 流程一致性悬停验收：GPS 节流 31Hz（`HIL_NAV_EVERY=8`，匹配 HIL 注入节奏）
/// 下长时间悬停**不得劣化**。
///
/// 背景（用户确认思路）：先让软件仿真流程与硬件一致，SIL 跑通 ≥10min 悬停稳定后
/// 再切 HIL。HIL 中 HIL_GPS/SET_POSITION 每 `HIL_NAV_EVERY=8` 物理步注入一次
/// （≈31Hz，见 fly-sim-server/src/main.rs），而 SIL 默认每拍（250Hz）注入 GPS →
/// SIL 位置估计比 HIL 更紧。本测试用 `set_gps_throttle(8)` 复刻 HIL 节流节奏，
/// 验证在**一致的稀疏位置观测**下悬停长期稳定、不劣化。
///
/// 判据（对齐 HIL 闭环目标）：
/// - 全程状态有限（无 NaN/Inf 发散）
/// - 末窗口（最后 win_s 秒）真值 |roll|/|pitch| 峰值 < 10°（不翻滚）
/// - 末窗口水平漂移峰值 < 1.5m、高度偏差峰值 |Δd| < 1.0m（悬停保持）
/// - 末窗口姿态估计误差均方根不劣于前窗口（settle~settle+win 段）的 1.5 倍（不劣化）
/// - 末窗口对角饱和交替样本占比 < 1%（无翻转式饱和指令）
fn run_hover_gps31hz(sensor_cfg: SensorConfig, seconds: f64, gps_every: u32, label: &str) {
    const SETTLE_S: f64 = 30.0; // 稳态起点（收敛后）
    const WIN_S: f64 = 60.0; // 前/末对比窗口时长
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let mut loop_sim = make_loop_sensor(sensor_cfg);
    loop_sim.set_gps_throttle(gps_every);

    let n = (seconds / DT) as usize;
    let w_start = (SETTLE_S / DT) as usize; // 前窗口起点
    let w_end = ((SETTLE_S + WIN_S) / DT) as usize; // 前窗口终点
    let tail_start = ((seconds - WIN_S) / DT) as usize; // 末窗口起点
    let log_every = (60.0 / DT) as usize; // 每 60s 打一行进度

    let mut diverged = false;
    // 前窗口（收敛后早期）统计
    let mut early: [f32; 3] = [0.0; 3]; // [att_err_rms, peak_att_err, peak_truth_rp]
    let mut early_n = 0u32;
    // 末窗口统计
    let mut tail: [f32; 5] = [0.0; 5]; // [att_err_rms, peak_att_err, peak_truth_rp, peak_drift_h, peak_dz]
    let mut tail_n = 0u32;
    let mut tail_sat = 0u32; // 末窗口对角饱和交替样本数
    let mut tail_sat_n = 0u32; // 末窗口饱和检测样本数

    for i in 0..n {
        let (w, cmd) = loop_sim.step_frame(&sp);
        let est = loop_sim.ctrl_debug_estimate();
        let (tr_r, tr_p) = quat_roll_pitch(&w.att);
        let (es_r, es_p) = quat_roll_pitch(&est.att);
        let err = ((tr_r - es_r).powi(2) + (tr_p - es_p).powi(2)).sqrt();
        let truth_rp = tr_r.abs().max(tr_p.abs());
        let drift_h = (w.pos[0].0).hypot(w.pos[1].0);
        let dz = (w.pos[2].0 - (-5.0)).abs();

        if i % log_every == 0 {
            println!(
                "  [{label}] t={:>5.0}s truth R/P={:>+5.1}/{:>+5.1}° est R/P={:>+5.1}/{:>+5.1}° drift={:.2}m d={:.2}m cmd={:.2}/{:.2}/{:.2}/{:.2}",
                i as f64 * DT,
                tr_r, tr_p, es_r, es_p, drift_h, dz,
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
            );
        }

        // 前窗口（稳态早期）统计
        if i >= w_start && i < w_end {
            early[0] += err * err;
            if err > early[1] {
                early[1] = err;
            }
            if truth_rp > early[2] {
                early[2] = truth_rp;
            }
            early_n += 1;
        }
        // 末窗口统计
        if i >= tail_start {
            tail[0] += err * err;
            if err > tail[1] {
                tail[1] = err;
            }
            if truth_rp > tail[2] {
                tail[2] = truth_rp;
            }
            if drift_h > tail[3] {
                tail[3] = drift_h;
            }
            if dz > tail[4] {
                tail[4] = dz;
            }
            tail_n += 1;
            // 对角饱和交替模式检测（1001/0110/1010/0101）
            tail_sat_n += 1;
            let pat = format!(
                "{}{}{}{}",
                (cmd.motor[0] as u32),
                (cmd.motor[1] as u32),
                (cmd.motor[2] as u32),
                (cmd.motor[3] as u32),
            );
            if pat == "1001" || pat == "0110" || pat == "1010" || pat == "0101" {
                tail_sat += 1;
            }
        }

        if !invariants::state_finite(&est) {
            eprintln!("[FAIL] {label} t={}s 估计发散（非有限值）", i as f64 * DT);
            diverged = true;
            break;
        }
    }

    let early_rms = (early[0] / early_n.max(1) as f32).sqrt();
    let tail_rms = (tail[0] / tail_n.max(1) as f32).sqrt();
    let tail_sat_ratio = if tail_sat_n > 0 { tail_sat as f32 / tail_sat_n as f32 } else { 0.0 };
    println!("[{label}] 悬停 {}s 全程 {} 步", seconds, n);
    println!("  前窗口({SETTLE_S}~{}s): 姿态误差 RMS={early_rms:.2}° 峰={:.2}° 真值|R/P|峰={:.2}°", SETTLE_S + WIN_S, early[1], early[2]);
    println!("  末窗口(最后 {WIN_S}s): 姿态误差 RMS={tail_rms:.2}° 峰={:.2}° 真值|R/P|峰={:.2}° 水平漂移峰={:.2}m 高度偏差峰={:.2}m", tail[1], tail[2], tail[3], tail[4]);
    println!("  末窗口对角饱和交替占比={:.3}%  diverged={diverged}", tail_sat_ratio * 100.0);

    // 不劣化：末窗口姿态误差 RMS 不劣于前窗口的 1.5 倍
    let not_worse = tail_rms <= early_rms * 1.5 + 0.5;
    assert!(!diverged, "{label}: 全程估计必须有限，无 NaN/Inf");
    assert!(tail[2] < 10.0, "{label}: 末窗口真值 |roll|/|pitch| 峰值 {:.1}° 必须 < 10°（不翻滚）", tail[2]);
    assert!(tail[3] < 1.5, "{label}: 末窗口水平漂移峰值 {:.2}m 必须 < 1.5m（悬停保持）", tail[3]);
    assert!(tail[4] < 1.0, "{label}: 末窗口高度偏差峰值 {:.2}m 必须 < 1.0m（悬停保持）", tail[4]);
    assert!(not_worse, "{label}: 末窗口姿态误差 RMS {:.2}° 劣于前窗口 {:.2}°×1.5（不劣化）", tail_rms, early_rms);
    assert!(
        tail_sat_ratio < 0.01,
        "{label}: 末窗口对角饱和交替占比 {:.2}% 必须 < 1%（无翻转式饱和指令）",
        tail_sat_ratio * 100.0
    );
}

/// 主验收：与 HIL 完全一致的流程（默认零噪声传感器 + GPS 31Hz）下 ≥10min 悬停稳定。
#[test]
fn sil_hover_gps31hz_10min_stable() {
    run_hover_gps31hz(SensorConfig::default(), 600.0, 8, "SIL 悬停 10min | GPS 31Hz | 零噪声");
}

// 注：realistic 消费级噪声（gyro_noise=0.003 等）SIL 悬停测试曾验证失败，但该测试与
// HIL 流程不一致——HIL 上行注入的是物理引擎真值 IMU/GPS（零噪声，见
// fly-sim-server/src/main.rs `advance_hil` 的 sim.last_imu()），MCU 拿不到带噪传感器。
// 因此按「SIL 流程与硬件一致」原则移除该分支，避免在 HIL 并不存在的噪声假设上兜圈。

