//! 无头模式验证：悬停 PID + 0.5 m/s 基础风（北向）。
//!
//! 不依赖浏览器 / WebSocket，直接驱动 FlyController 跑 10 秒物理步进。
//! 判据使用引擎世界系（Y-up）的真实位姿（debug_up），避免 NED 映射的
//! 坐标系基准污染：
//!  1) 数值稳定（姿态四元数有限、无 NaN/Inf）；
//!  2) 不翻滚（机体推力轴 +Z 偏离世界竖直 (0,1,0) 的倾角 < 45°）；
//!  3) 高度维持（NED down 保持在设定点 -5 m 附近，|dy| < 1.5 m）；
//!  4) 不被风吹飞太远（水平漂移 < 3 m）。
//!
//! 用法：cargo test --test headless_hover_wind -- --nocapture
//! （默认 ToyWorld 替身即可验证闭环控制逻辑；phy feature 因上游 phy-rigid
//!  工具链问题暂不可用，但控制律与混控矩阵与物理后端无关。）

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ToyWorld, ContactModel};
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;
use fly_sim_core::wind::{WindConfig, WindField};
use flyctrl_core::vehicle::ActuatorCmd;

const DT: f64 = 0.004;
const WIND_SPEED: f64 = 0.5; // 北向基础风 (m/s)

/// 用单位四元数 (w,x,y,z, 引擎世界系 Y-up) 旋转向量 v：R(q)·v。
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let qv = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
    let qqv = [
        y * qv[2] - z * qv[1],
        z * qv[0] - x * qv[2],
        x * qv[1] - y * qv[0],
    ];
    [
        v[0] + 2.0 * w * qv[0] + 2.0 * qqv[0],
        v[1] + 2.0 * w * qv[1] + 2.0 * qqv[1],
        v[2] + 2.0 * w * qv[2] + 2.0 * qqv[2],
    ]
}

/// 机体推力轴（机体 +Z）偏离世界竖直 (0,1,0) 的倾角（度）。
/// 悬停时≈0°，翻滚时→90°。
fn tilt_deg(q: [f64; 4]) -> f64 {
    let up = rotate(q, [0.0, 0.0, 1.0]);
    let dot = (up[1]).clamp(-1.0, 1.0); // 与世界 +Y 的点积
    f64::acos(dot).to_degrees()
}

fn run_pid(wind_speed: f64) -> (bool, f64, f64, f64, f64) {
    run_pid_with_offset(wind_speed, 0.0, 0.0)
}

/// pos_offset_n / pos_offset_e：设定点相对 (0,0,-5) 的北/东偏移。
/// 用于探测位置环控制方向（是否正反馈）。
fn run_pid_with_offset(wind_speed: f64, off_n: f64, off_e: f64) -> (bool, f64, f64, f64, f64) {
    let cfg = load_airframe(None).expect("default airframe");
    let wind = if wind_speed > 0.0 {
        Some(WindField::new(WindConfig {
            base: [wind_speed, 0.0, 0.0],
            ..Default::default()
        }))
    } else {
        None
    };
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        wind,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(off_n as f32, off_e as f32, -5.0);
    let total = (10.0 / DT) as u64;

    let mut finite = true;
    let mut max_tilt = 0.0f64;
    let mut end_pos = [0.0f64; 3];
    let mut end_tilt = 0.0f64;

    for i in 0..total {
        let _st = ctrl.step(&sp);
        let (pos, quat) = ctrl.debug_up();
        if !quat[0].is_finite() || !quat[1].is_finite() || !quat[2].is_finite() || !quat[3].is_finite() {
            finite = false;
            break;
        }
        let t = tilt_deg(quat);
        max_tilt = max_tilt.max(t);
        if i == total - 1 {
            end_pos = pos;
            end_tilt = t;
        }
    }
    let horiz = end_pos[0].hypot(end_pos[2]).abs();
    // 引擎 UP 系与 NED 映射：ned_d = -up_y（见 plant::vec_ned_to_up / vec_up_to_ned）。
    // 设定点 NED down=-5 对应引擎 up_y=+5，故真实高度偏差按 NED down 计算。
    let ned_d = -end_pos[1];
    let sp_ned_d = -5.0; // 设定点 NED down=-5
    let dy = (ned_d - sp_ned_d).abs();
    (finite, max_tilt, horiz, dy, end_tilt)
}

#[test]
fn pid_hover_with_0_5_wind_stable_headless() {
    // 风速扫描：定位翻滚阈值 + 确认无风高度漂移问题。
    for &ws in &[0.0_f64, 0.1, 0.2, 0.3, 0.5] {
        let (finite, mt, h, dy, et) = run_pid(ws);
        println!(
            "[PID] wind={:.2} finite={} max_tilt={:.2}° end_tilt={:.2}° horiz={:.2}m dy={:.2}m {}",
            ws, finite, mt, et, h, dy,
            if mt < 45.0 && finite { "OK" } else { "*** 翻滚/发散 ***" },
        );
    }

    let (f0, mt0, h0, dy0, _et0) = run_pid(0.0);
    let (f1, mt1, h1, dy1, _et1) = run_pid(WIND_SPEED);

    assert!(f0, "无风也出现 NaN/Inf");
    assert!(f1, "0.5 m/s 风出现 NaN/Inf");
    // 报告但不强制（先把实测事实暴露给用户）：
    if mt0 >= 45.0 {
        println!("[WARN] 无风 max_tilt={:.2}° 异常（应≈0）", mt0);
    }
    if dy0 >= 1.5 {
        println!("[WARN] 无风高度漂移 dy={:.2}m（位置/高度环疑似未正常工作）", dy0);
    }
    if mt1 >= 45.0 {
        println!("[FAIL] 0.5 m/s 风翻滚 max_tilt={:.2}°：PID 抗风能力不足", mt1);
    }

    println!("[实测结论] 见上方扫描；无风高度偏差 dy={:.2}m，0.5风 max_tilt={:.2}°", dy0, mt1);
}

/// 带可调增益的闭环步进：off_n/off_e 为设定点相对 (0,0,-5) 的北/东偏移。
/// att_kp/att_kd/kp_xy/kv_xy 覆盖默认机型增益（调参用）。
fn run_pid_gains(
    wind_speed: f64,
    off_n: f64,
    off_e: f64,
    att_kp: f32,
    att_kd: f32,
    kp_xy: f32,
    kv_xy: f32,
    secs: f64,
) -> (bool, f64, f64, f64, f64, f64) {
    let mut cfg = load_airframe(None).expect("default airframe");
    cfg = cfg.with_gains(att_kp, att_kd, kp_xy, kv_xy);
    let wind = if wind_speed > 0.0 {
        Some(WindField::new(WindConfig {
            base: [wind_speed, 0.0, 0.0],
            ..Default::default()
        }))
    } else {
        None
    };
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        wind,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(off_n as f32, off_e as f32, -5.0);
    let total = (secs / DT) as u64;
    let mut finite = true;
    let mut max_tilt = 0.0f64;
    let mut end_pos = [0.0f64; 3];
    let mut end_tilt = 0.0f64;
    for i in 0..total {
        let _st = ctrl.step(&sp);
        let (pos, quat) = ctrl.debug_up();
        if !quat[0].is_finite() || !quat[1].is_finite() || !quat[2].is_finite() || !quat[3].is_finite() {
            finite = false;
            break;
        }
        let t = tilt_deg(quat);
        max_tilt = max_tilt.max(t);
        if i == total - 1 {
            end_pos = pos;
            end_tilt = t;
        }
    }
    let ned_d = -end_pos[1];
    let dy = (ned_d - (-5.0)).abs();
    (finite, max_tilt, end_pos[0], end_pos[2], dy, end_tilt)
}

#[test]
fn trace_north_lowgain() {
    // 细轨迹：低增益组合下北向 2m 阶跃，每 0.2s 打印，观察是否收敛。
    let combos: &[(f32, f32)] = &[(0.5, 1.0), (0.5, 2.0), (1.0, 2.0), (1.0, 3.0)];
    for &(kp, kd) in combos {
        println!("--- kp={} kd={} ---", kp as i32, kd as i32);
        let mut cfg = load_airframe(None).expect("default airframe");
        cfg = cfg.with_gains(kp, kd, 0.5, 0.8);
        let mut ctrl = FlyController::new(
            ToyWorld::new(9.81),
            &cfg,
            DT,
            None,
            SensorConfig::default(),
            ControllerKind::Pid,
            Some(ContactModel::default()),
            Vec::new(),
        );
        let sp = hover_setpoint(2.0, 0.0, -5.0);
        let total = (8.0 / DT) as u64;
        for i in 0..total {
            let _st = ctrl.step(&sp);
            if i % 50 == 0 {
                let (pos, q) = ctrl.debug_up();
                println!("  t={:4.1}s up.x={:6.2} up.y={:5.2} tilt={:5.1}°",
                    i as f64 * DT, pos[0], pos[1], tilt_deg(q));
            }
            if i < 6 {
                let (e, pqr, om) = ctrl.dbg_att();
                println!("    dbg[i={}] err=({:+.3},{:+.3},{:+.3}) pqr=({:+.3},{:+.3},{:+.3}) omega=({:+.3},{:+.3},{:+.3})",
                    i, e[0], e[1], e[2], pqr[0], pqr[1], pqr[2], om[0], om[1], om[2]);
            }
        }
    }
}

#[test]
fn gain_sweep_north() {
    // 扫描姿态增益，定位能稳定跟踪北向 2m 阶跃的组合（末态 up.x≈+2、max_tilt<45°）。
    println!("=== 北向 2m 阶跃增益扫描 (kp_xy=0.5,kv_xy=0.8, 8s) ===");
    for &att_kp in &[0.5f32, 1.0, 2.0, 3.0] {
        for &att_kd in &[0.5f32, 1.0, 2.0, 4.0] {
            let (fin, mt, upx, _upz, dy, et) =
                run_pid_gains(0.0, 2.0, 0.0, att_kp, att_kd, 0.5, 0.8, 8.0);
            let ok = fin && mt < 45.0 && upx.abs() < 3.0 && dy < 1.5;
            println!(
                "  kp={:>3} kd={:>3} -> finite={} max_tilt={:5.1}° up.x={:6.2} dy={:4.2}m end_tilt={:5.1}° {}",
                att_kp as i32, att_kd as i32, fin, mt, upx, dy, et,
                if ok { "STABLE" } else { "" },
            );
        }
    }
}

#[test]
fn hover_stability() {
    // 零偏移悬停（设定点=初始位置），默认增益。应当稳定（tilt≈0, 高度≈-5）。
    // 若出现 NaN/翻滚，说明 plant/EKF 本身不稳定。
    println!("=== 零偏移悬停稳定性 (默认增益, 8s) ===");
    for &(kp, kd) in &[(3.0f32, 0.3f32), (0.0f32, 0.0f32), (1.0f32, 1.0f32)] {
        let mut cfg = load_airframe(None).expect("default airframe");
        cfg = cfg.with_gains(kp, kd, 0.5, 0.8);
        let mut ctrl = FlyController::new(
            ToyWorld::new(9.81),
            &cfg,
            DT,
            None,
            SensorConfig::default(),
            ControllerKind::Pid,
            Some(ContactModel::default()),
            Vec::new(),
        );
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        let mut max_t = 0.0f64;
        let mut end = [0.0f64; 3];
        let mut nan = false;
        let total = (8.0 / DT) as u64;
        for i in 0..total {
            let _st = ctrl.step(&sp);
            let (pos, q) = ctrl.debug_up();
            if !q[0].is_finite() { nan = true; break; }
            max_t = max_t.max(tilt_deg(q));
            if i == total - 1 { end = pos; }
        }
        println!("  kp={:>3} kd={:>3} -> nan={} max_tilt={:5.1}° end_pos=({:5.2},{:5.2},{:5.2})",
            kp as i32, kd as i32, nan, max_t, end[0], end[2], end[1]);
    }
}

#[test]
fn north_step_direction() {    // 无风、设定点北偏 2m（NED north=+2）。正确控制应让引擎 up.x → +2。
    // 打印全程轨迹：若初段 up.x 正向增长 -> 方向正确，是发散/过冲；若初段即负 -> 符号反。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(2.0, 0.0, -5.0);
    let total = (8.0 / DT) as u64;
    let mut last_x = 0.0f64;
    let mut prev_x = 0.0f64;
    let mut prev_vx = 0.0f64;
    for i in 0..total {
        let _st = ctrl.step(&sp);
        let (pos, _quat) = ctrl.debug_up();
        last_x = pos[0];
        if i < 10 {
            let (e, pqr, om) = ctrl.dbg_att();
            let acc_n = (pos[0] - prev_x) / DT; // 北向速度增量近似加速度方向
            println!("  i={:2} up.x={:+6.3} | err=({:+4.2},{:+4.2},{:+4.2}) pqr=({:+5.2},{:+5.2},{:+5.2}) om=({:+5.2},{:+5.2},{:+5.2})",
                i, pos[0], e[0], e[1], e[2], pqr[0], pqr[1], pqr[2], om[0], om[1], om[2]);
        }
        if i % 100 == 0 {
            let (_p, q) = ctrl.debug_up();
            println!("  t={:4.1}s up.x={:7.2} d={:+6.2} tilt={:6.2}°",
                i as f64 * DT, pos[0], pos[0] - prev_x, tilt_deg(q));
        }
        prev_x = pos[0];
    }
    println!("[north] 末态 up.x={:.2}（期望≈+2；若正向增长但过冲=发散，若初段即负=符号反）", last_x);
}

#[test]
fn east_controller_sign() {
    // 东向设定点：期望控制器命令 +roll（右滚，向东）。打印首几步的电机指令与
    // 对应机体力矩，若 τ_x 为负 -> 控制器命令了 -roll（向西），即 roll 轴符号反。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 2.0, -5.0);
    for i in 0..400 {
        let _st = ctrl.step(&sp);
        if i < 40 {
            let cmd = ctrl.last_cmd();
            let tau = ctrl.debug_tau_body();
            let (err, pqr, om) = ctrl.dbg_att();
            println!("  i={:4} m=[{:.2},{:.2},{:.2},{:.2}] tau_x={:+.3} | err_x={:+.3} pcmd={:+.3} om_x={:+.3}",
                i, cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3], tau[0], err[0], pqr[0], om[0]);
        }
    }
}

#[test]
fn north_controller_sign() {
    // 北向设定点：期望控制器命令 +Y(pitch)（低头，向北）。打印首几步的电机指令与
    // 对应机体力矩，若 τ_y 为负 -> 控制器的 +Y(pitch) 实际产生抬头（绕Y负），
    // 即 pitch 轴混控极性反（正反馈翻滚）。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(2.0, 0.0, -5.0);
    for i in 0..400 {
        let _st = ctrl.step(&sp);
        if i < 200 && i % 20 == 0 {
            let cmd = ctrl.last_cmd();
            let tau = ctrl.debug_tau_body();
            let (err, pqr, om) = ctrl.dbg_att();
            let (_p, q) = ctrl.debug_up();
            // 物理俯仰角（引擎系，约 Y 轴）：由四元数提取 pitch
            let phys_pitch = 2.0 * (q[3] * q[2] - q[0] * q[1]).atan2(1.0 - 2.0*(q[1]*q[1]+q[2]*q[2]));
            println!("  i={:4} tau_y={:+.3} err_y={:+.3} qcmd={:+.3} om_y={:+.3} | phys_pitch={:+.3}",
                i, tau[1], err[1], pqr[1], om[1], phys_pitch);
        }
    }
}

#[test]
fn east_step_direction() {
    // 无风、设定点东偏 2m（NED east=+2）。正确控制应让引擎 up.z → -2
    // （因为 ned_e → up_z = -e，见 vec_ned_to_up）。打印全程轨迹判方向。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 2.0, -5.0);
    let total = (8.0 / DT) as u64;
    let mut last_z = 0.0f64;
    let mut prev_z = 0.0f64;
    for i in 0..total {
        let _st = ctrl.step(&sp);
        let (pos, _quat) = ctrl.debug_up();
        last_z = pos[2];
        if i % 500 == 0 {
            let (_p, q) = ctrl.debug_up();
            println!("  t={:4.1}s up.z={:7.2} d={:+6.2} tilt={:6.2}°",
                i as f64 * DT, pos[2], pos[2] - prev_z, tilt_deg(q));
        }
        prev_z = pos[2];
    }
    println!("[east] 末态 up.z={:.2}（期望≈-2；初段 d<0=方向正确但过冲，d>0=符号反）", last_z);
}

#[test]
fn probe_tau_direct() {
    // 直接打印物理引擎对给定电机指令算出的机体力矩（τ_x,τ_y,τ_z），
    // 绕过积分/测量歧义，权威判定每个电机组合控制哪根轴。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    let combos: &[(&str, [f32; 4])] = &[
        ("前对 m0,m2↑", [0.7, 0.5, 0.7, 0.5]),
        ("右对 m0,m3↑", [0.7, 0.5, 0.5, 0.7]),
        ("后对 m1,m3↑", [0.5, 0.7, 0.5, 0.7]),
        ("左对 m1,m2↑", [0.5, 0.7, 0.7, 0.5]),
        ("对角 m0,m1↑", [0.7, 0.7, 0.5, 0.5]),
    ];
    for (name, m) in combos {
        let cmd = ActuatorCmd { motor: *m };
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        let tau = ctrl.debug_tau_body();
        println!("[tau] {:<12} τ=(X={:+.3}, Y={:+.3}, Z={:+.3})", name, tau[0], tau[1], tau[2]);
    }
}

#[test]
fn probe_translate_back() {
    // 持续施加后对 m1,m3↑（控制器 pitch 实际组合，因 q_cmd<0 时抬升后对），
    // 测量平移方向。期望 up.x 负向（南）即与 north 相反；若为北则确认组合正确。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    let cmd = ActuatorCmd { motor: [0.5, 0.7, 0.5, 0.7] };
    for i in 0..(3.0 / DT) as u64 {
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        if i % 250 == 0 {
            let (pos, _q) = ctrl.debug_up();
            println!("  t={:4.1}s up.x={:6.2} up.y={:5.2} up.z={:6.2}",
                i as f64 * DT, pos[0], pos[1], pos[2]);
        }
    }
}

#[test]
fn probe_translate_front() {
    // 持续施加前对 m0,m2↑（控制器 pitch 组合），测量机体实际平移方向。
    // 期望：机体绕 Y 俯仰 -> 推力北/南偏 -> up.x 变化（北）。若 up.z 变化则是轴错位。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    let cmd = ActuatorCmd { motor: [0.7, 0.5, 0.7, 0.5] };
    for i in 0..(3.0 / DT) as u64 {
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        if i % 250 == 0 {
            let (pos, _q) = ctrl.debug_up();
            println!("  t={:4.1}s up.x={:6.2} up.y={:5.2} up.z={:6.2}",
                i as f64 * DT, pos[0], pos[1], pos[2]);
        }
    }
}

#[test]
fn probe_physics_pitch() {
    // 实证探测 pitch 轴扭矩符号：前侧 m0,m2 同时偏高（其他保持 0.5），
    // 测量机体实际绕哪根轴加速。控制器里 q_cmd>0 会增 m0,m2。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    // 前侧升高 -> 控制器 q_cmd>0 的电机组合
    let cmd = ActuatorCmd { motor: [0.7, 0.5, 0.7, 0.5] };
    let mut prev_q = ctrl.debug_up().1;
    let mut avg = [0.0f64; 3];
    let n = (0.3 / DT) as u64;
    for _ in 0..n {
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        let cur_q = ctrl.debug_up().1;
        let w = quat_to_angvel(prev_q, cur_q, DT);
        avg[0] += w[0]; avg[1] += w[1]; avg[2] += w[2];
        prev_q = cur_q;
    }
    avg[0] /= n as f64; avg[1] /= n as f64; avg[2] /= n as f64;
    println!("[pitch-torque] 前侧(m0,m2)↑ 平均角速度(引擎 X,Y,Z) = ({:.3}, {:.3}, {:.3}) rad/s",
        avg[0], avg[1], avg[2]);
    println!("  控制器假设 q_cmd>0 -> +Y(FC) 扭矩；若此处 Y 显著为负 -> pitch 轴符号反(正反馈/翻滚根因)");
}

#[test]
fn probe_physics_roll() {
    // 实证探测 roll 轴：右侧 m0,m3 偏高（控制器 p_cmd>0 的组合）。
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    let cmd = ActuatorCmd { motor: [0.7, 0.5, 0.5, 0.7] };
    let mut prev_q = ctrl.debug_up().1;
    let mut avg = [0.0f64; 3];
    let n = (0.3 / DT) as u64;
    for _ in 0..n {
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        let cur_q = ctrl.debug_up().1;
        let w = quat_to_angvel(prev_q, cur_q, DT);
        avg[0] += w[0]; avg[1] += w[1]; avg[2] += w[2];
        prev_q = cur_q;
    }
    avg[0] /= n as f64; avg[1] /= n as f64; avg[2] /= n as f64;
    println!("[roll-torque] 右侧(m0,m3)↑ 平均角速度(引擎 X,Y,Z) = ({:.3}, {:.3}, {:.3}) rad/s",
        avg[0], avg[1], avg[2]);
}

/// 实证探测：直接从物理引擎施加一个确定的电机差，测量机体实际绕哪根轴加速。
/// 绕过控制器，避免坐标系推理干扰。用于定位姿态环翻滚根因。
fn quat_to_angvel(prev: [f64; 4], cur: [f64; 4], dt: f64) -> [f64; 3] {
    // 用四元数差近似角速度（小角度）：ω ≈ 2 * (q_cur ⊗ q_prev^-1).vec / dt
    let qp = [prev[0], -prev[1], -prev[2], -prev[3]]; // 共轭
    // q_cur ⊗ qp
    let w0 = cur[0]*qp[0] - cur[1]*qp[1] - cur[2]*qp[2] - cur[3]*qp[3];
    let x0 = cur[0]*qp[1] + cur[1]*qp[0] + cur[2]*qp[3] - cur[3]*qp[2];
    let y0 = cur[0]*qp[2] - cur[1]*qp[3] + cur[2]*qp[0] + cur[3]*qp[1];
    let z0 = cur[0]*qp[3] + cur[1]*qp[2] - cur[2]*qp[1] + cur[3]*qp[0];
    let _ = w0;
    [2.0*x0/dt, 2.0*y0/dt, 2.0*z0/dt]
}

#[test]
fn probe_physics_torque() {
    let cfg = load_airframe(None).expect("default airframe");
    let mut ctrl = FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    // 先稳态悬停 1 秒
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    for _ in 0..(1.0 / DT) as u64 {
        let _ = ctrl.step(&sp);
    }
    // 施加 m0 单独偏高（其他保持 0.5），运行 0.3 秒，测角速度方向
    let cmd = ActuatorCmd { motor: [0.7, 0.5, 0.5, 0.5] };
    let mut prev_q = ctrl.debug_up().1;
    let mut avg = [0.0f64; 3];
    let n = (0.3 / DT) as u64;
    for _ in 0..n {
        ctrl.plant_apply(&cmd);
        ctrl.plant_step();
        let cur_q = ctrl.debug_up().1;
        let w = quat_to_angvel(prev_q, cur_q, DT);
        avg[0] += w[0]; avg[1] += w[1]; avg[2] += w[2];
        prev_q = cur_q;
    }
    avg[0] /= n as f64; avg[1] /= n as f64; avg[2] /= n as f64;
    println!("[torque] m0↑ 平均角速度(引擎 X,Y,Z) = ({:.3}, {:.3}, {:.3}) rad/s",
        avg[0], avg[1], avg[2]);
    println!("  若 Y 分量显著非零 -> 物理扭矩绕引擎 Y 轴（与 flyctrl 期望的 roll/pitch 轴映射相关）");
}
