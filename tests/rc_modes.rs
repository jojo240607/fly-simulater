//! P3-D1 "RC 输入 + 手动/增稳模式"验收测试（无头模式，ToyWorld 替身）。
//!
//! 验证遥控完整流程与模式治理：
//!  1) `rc_flow_unlock_manual_stabilize_position_rtl_land`
//!     —— 解锁 → 手动（角速率直通）→ 增稳（姿态保持）→ 定点 → 返航 → 降落，
//!        全程无 NaN、各阶段模式正确生效、增稳/定点保持稳定。
//!  2) `rc_governor_blocks_unsafe_modes`
//!     —— 模式治理：未解锁禁入 Position；解锁后可用。
//!  3) `rc_disarm_zeroes_thrust_and_descends`
//!     —— 遥控上锁 → 电机归零、机体自由下落。
//!
//! 用法：cargo test --test rc_modes -- --nocapture
//!
//! 坐标系：引擎内部 UP（Y-up）；判据用 NED 语义
//! （ned_x=up_x, ned_y=up_z, ned_d=-up_y），与 headless_hover_wind 一致。

use fly_sim_core::controller::{ControllerKind, FlyController};
use fly_sim_core::physics::{ContactModel, ToyWorld};
use fly_sim_core::sensor::SensorConfig;
use fly_simulater::airframe::load_airframe;
use flyctrl_core::flightmode::FlightMode;
use flyctrl_core::vehicle::RcInput;

const DT: f64 = 0.004;

/// 构造一帧遥控输入（模式槽 0/1/2 = 手动/增稳/定点）。
fn rc(throttle: f32, roll: f32, pitch: f32, armed: bool, mode: u8) -> RcInput {
    RcInput {
        roll,
        pitch,
        yaw: 0.0,
        throttle,
        armed,
        mode,
        fresh: true,
    }
}

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

/// 新造一个 Pid 控制律的仿真控制器（RC 直通档不依赖控制律种类）。
fn new_ctrl() -> FlyController<ToyWorld> {
    let cfg = load_airframe(None).expect("default airframe");
    FlyController::new(
        ToyWorld::new(9.81),
        &cfg,
        DT,
        None,
        SensorConfig::default(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    )
}

/// 以固定摇杆连续步进 `seconds` 秒，返回期间最坏倾角与末态 NED。
fn run_rc(ctrl: &mut FlyController<ToyWorld>, r: &RcInput, seconds: f64) -> (f64, [f32; 3], [f32; 3]) {
    let total = (seconds / DT) as u64;
    let mut max_tilt = 0.0f64;
    let mut end_pos = [0.0f32; 3];
    let mut end_vel = [0.0f32; 3];
    for _ in 0..total {
        let st = ctrl.step_rc(r);
        let (_, quat) = ctrl.debug_up();
        assert!(
            quat.iter().all(|v| v.is_finite()) && st.pos.iter().all(|v| v.0.is_finite()),
            "RC 闭环出现 NaN"
        );
        max_tilt = max_tilt.max(tilt_deg(quat));
        end_pos = [st.pos[0].0, st.pos[1].0, st.pos[2].0];
        end_vel = [st.vel[0].0, st.vel[1].0, st.vel[2].0];
    }
    (max_tilt, end_pos, end_vel)
}

#[test]
fn rc_flow_unlock_manual_stabilize_position_rtl_land() {
    let mut ctrl = new_ctrl();

    // 初始：手动 + 已解锁（遥控拨杆解锁），油门中位悬停（角速率直通，姿态环仅阻尼）。
    assert_eq!(ctrl.flight_mode(), FlightMode::Manual);
    let (mt1, p1, _v1) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 0), 1.0);
    println!("[manual] mode={:?} tilt={:.1}° pos=({:.2},{:.2},{:.2})",
        ctrl.flight_mode(), mt1, p1[0], p1[1], p1[2]);
    assert!(ctrl.flight_mode() == FlightMode::Manual, "遥控槽0应保持手动");
    assert!(mt1 < 30.0, "手动悬停翻滚过大: {:.1}°", mt1);
    // 手动档油门直通：中位应产生非零推力（四路总和 > 0）。
    let sum: f32 = ctrl.last_cmd().motor.iter().sum();
    assert!(sum > 0.0, "手动中位油门应输出非零推力");

    // 切增稳（槽1）：松杆回中 + 油门中位 → 姿态保持水平、高度保持。
    let (mt2, p2, _v2) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 1), 3.0);
    println!("[stabilize] mode={:?} tilt={:.1}° pos=({:.2},{:.2},{:.2})",
        ctrl.flight_mode(), mt2, p2[0], p2[1], p2[2]);
    assert!(ctrl.flight_mode() == FlightMode::Stabilize, "遥控槽1应切增稳");
    assert!(mt2 < 10.0, "增稳松杆应保持水平: {:.1}°", mt2);
    let dy2 = (p2[2] - (-5.0)).abs();
    assert!(dy2 < 3.0, "增稳高度漂移过大: {:.2}m", dy2);

    // 切定点（槽3）：锚定保持点，先平移脱离原点，为返航制造实际行程。
    let (mt3, _p3, _v3) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 3), 1.0);
    ctrl.set_hold_pos(5.0, 0.0, -5.0); // 平移到北 5m
    let (mt3b, p3b, _v3b) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 3), 6.0);
    println!("[position] mode={:?} tilt={:.1}° pos=({:.2},{:.2},{:.2})",
        ctrl.flight_mode(), mt3.max(mt3b), p3b[0], p3b[1], p3b[2]);
    assert!(ctrl.flight_mode() == FlightMode::Position, "遥控槽3应切定点");
    let horiz3 = (p3b[0] - 5.0).abs();
    assert!(horiz3 < 1.5, "定点平移到 (5,0) 保持不佳: Δx={:.2}m", horiz3);

    // 返航（槽4）：回到起飞点 home=(0,0,-5)，应有可测的北向回程。
    let (_mt4, p4, _v4) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 4), 8.0);
    println!("[rtl] mode={:?} pos=({:.2},{:.2},{:.2})",
        ctrl.flight_mode(), p4[0], p4[1], p4[2]);
    assert!(ctrl.flight_mode() == FlightMode::Rtl, "遥控槽4应切返航");
    assert!(p4[0].abs() < 1.5, "返航未回到原点: x={:.2}m", p4[0]);

    // 降落（槽5）：目标降至地面（NED d=0），应显著下降（NED d 增大）。
    let d_rtl = p4[2];
    let (_mt5, p5, v5) = run_rc(&mut ctrl, &rc(0.3, 0.0, 0.0, true, 5), 3.0);
    println!("[land] mode={:?} d={:.2}→{:.2} vd={:.2}",
        ctrl.flight_mode(), d_rtl, p5[2], v5[2]);
    assert!(ctrl.flight_mode() == FlightMode::Land, "遥控槽5应切降落");
    assert!(p5[2] > d_rtl + 0.5, "降落未下降: d {:.2} → {:.2}", d_rtl, p5[2]);
}

#[test]
fn rc_governor_blocks_unsafe_modes() {
    let mut ctrl = new_ctrl();
    // 先跑若干步，让 GPS 观测生效（position_available=true），再测模式治理。
    let _ = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 0), 0.2);
    // 未解锁：Position 需解锁（requires_armed）→ 拒绝，保持手动。
    ctrl.disarm();
    assert!(!ctrl.request_mode(FlightMode::Position), "未解锁不应能进定点");
    assert!(ctrl.flight_mode() == FlightMode::Manual, "非法请求应保持当前模式");
    // 未解锁：Mission 同样拒绝。
    assert!(!ctrl.request_mode(FlightMode::Mission), "未解锁不应能进任务");
    // 解锁后：Position 可用（仿真 GPS 可用）。
    ctrl.arm();
    assert!(ctrl.request_mode(FlightMode::Position), "解锁后应能进定点");
    assert!(ctrl.flight_mode() == FlightMode::Position);
}

#[test]
fn rc_disarm_zeroes_thrust_and_descends() {
    let mut ctrl = new_ctrl();
    // 增稳悬停一段，稳定在 -5m。
    let (_m, p0, _v) = run_rc(&mut ctrl, &rc(0.5, 0.0, 0.0, true, 1), 2.0);
    assert!(ctrl.is_armed(), "遥控解锁后应 armed");

    // 遥控上锁（armed=false）：本拍起电机归零（free-fall），机体下落（NED d 增大）。
    let d_start = p0[2];
    let (_m, p1, v1) = run_rc(&mut ctrl, &rc(0.0, 0.0, 0.0, false, 1), 1.5);
    let actual = ctrl.debug_thrust_actual_u();
    println!("[disarm] d {:.2} → {:.2} vd={:.2} u={:?}",
        d_start, p1[2], v1[2], actual);
    assert!(!ctrl.is_armed(), "遥控上锁后应 disarm");
    assert!(actual.iter().all(|&u| u < 0.05), "上锁后电机应归零: {:?}", actual);
    assert!(p1[2] > d_start + 0.5, "上锁后应下落: d {:.2} → {:.2}", d_start, p1[2]);
}
