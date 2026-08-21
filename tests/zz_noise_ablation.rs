//! 临时消融诊断（P3-A2）：从 zero 噪声出发，逐一叠加单一噪声分量，
//! 定位"最小触发集"——哪个传感器分量单独就能把悬停打爆。
//! 分析完即删，结论写入 PLAN 阶段 11-A。

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::ToyWorld;
use fly_sim_core::sensor::SensorConfig;
use fly_sim_core::sim::SimLoop;
use fly_sim_core::controller::FlyController;
use fly_sim_core::plant::QuadrotorPlant;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian};
use flyctrl_core::vehicle::ActuatorCmd;

fn run_cfg(name: &str, cfg: &SensorConfig, secs: f64) -> (f64, f64, f64, f64) {
    // 返回 (d_span, mean_d, hmax, end_h)
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut sim = SimLoop::new(world, &vcfg, dt, None, cfg.clone(), ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (secs / dt) as u64;
    let mut dmin = f64::MAX;
    let mut dmax = f64::MIN;
    let mut hmax = 0.0f64;
    let mut sum = 0.0;
    let mut n = 0u64;
    let mut end_h = 0.0f64;
    for i in 0..steps {
        let (w, _cmd) = sim.step_frame(&sp);
        let d = w.pos[2].0 as f64;
        let h = (w.pos[0].0 as f64).hypot(w.pos[1].0 as f64);
        if i % 50 == 0 {
            dmin = dmin.min(d);
            dmax = dmax.max(d);
            hmax = hmax.max(h);
            sum += d;
            n += 1;
        }
        end_h = h;
        if !d.is_finite() || !w.pos[0].0.is_finite() {
            return (f64::INFINITY, f64::INFINITY, h, h);
        }
    }
    (dmax - dmin, sum / n as f64, hmax, end_h)
}

fn base() -> SensorConfig {
    SensorConfig::default()
}

/// 追踪单配置 40s：打印 EST vs TRU 位置/速度、估计姿态角、电机指令，
/// 判断发散机制是"姿态环饱和"还是"位置/速度环"还是"EST 偏离 TRU"。
fn trace_cfg(name: &str, cfg: &SensorConfig) {
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut ctrl = FlyController::new(world, &vcfg, dt, None, cfg.clone(), ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (40.0 / dt) as u64;
    let mut vmax = 0.0f64;
    let mut amax = 0.0f64;
    for i in 0..steps {
        let est = ctrl.step(&sp); // EST (EKF 估计)
        let w = ctrl.world_state(); // TRU (物理真值)
        let vh = (w.vel[0].0 as f64).hypot(w.vel[1].0 as f64);
        vmax = vmax.max(vh);
        let eatt = (est.att.w as f64).acos().min(1.0).abs(); // 姿态偏离角 (rad)
        amax = amax.max(eatt);
        if i % 2500 == 0 || !w.pos[0].0.is_finite() {
            let cmd = ctrl.last_cmd();
            println!(
                "  t={:5.1}s TRU=({:7.1},{:7.1},{:7.2}) EST=({:7.1},{:7.1},{:7.2}) vh={:6.2} eatt={:5.2}deg cmd=({:.2},{:.2},{:.2},{:.2})",
                i as f64 * dt,
                w.pos[0].0, w.pos[1].0, w.pos[2].0,
                est.pos[0].0, est.pos[1].0, est.pos[2].0,
                vh, eatt.to_degrees(),
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
            );
            if !w.pos[0].0.is_finite() { break; }
        }
    }
    println!("  [{name}] vh_max={:.1}m/s eatt_max={:.1}deg", vmax, amax.to_degrees());
}

#[test]
fn trace_early_phase() {
    let b = base();
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_rate = 20.0;
    c.gps_delay = 0.15;
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut ctrl = FlyController::new(world, &vcfg, dt, None, c.clone(), ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    println!("== gps-pos+delay early phase (t=0..4s, 0.05s 采样) ==");
    println!("   列：TRUpos/ESTpos(m)  TRUvel/ESTvel(m/s)  dvel(EST-TRU)  TRUrpy/ESTrpy(deg)  cmd");
    for i in 0..(4.0 / dt) as u64 {
        let est = ctrl.step(&sp);
        let w = ctrl.world_state();
        if i % 13 == 0 {
            let cmd = ctrl.last_cmd();
            let q = &est.att;
            let qt = &w.att;
            let eul = |q: &flyctrl_core::vehicle::Quaternion| -> (f64, f64, f64) {
                (
                    (2.0 * (q.w as f64 * q.x as f64 + q.y as f64 * q.z as f64))
                        .atan2(1.0 - 2.0 * (q.x as f64 * q.x as f64 + q.y as f64 * q.y as f64)),
                    (2.0 * (q.w as f64 * q.y as f64 - q.z as f64 * q.x as f64)).asin(),
                    (2.0 * (q.w as f64 * q.z as f64 + q.x as f64 * q.y as f64))
                        .atan2(1.0 - 2.0 * (q.y as f64 * q.y as f64 + q.z as f64 * q.z as f64)),
                )
            };
            let (r, p, _) = eul(q);
            let (tr, tp, _) = eul(qt);
            let de = |a: f32, b: f32| (a as f64 - b as f64); // EST - TRU
            println!(
                "  t={:4.2}s TRUpos=({:6.1},{:6.1}) ESTpos=({:6.1},{:6.1}) | TRUvel=({:6.2},{:6.2}) ESTvel=({:6.2},{:6.2}) dvel=({:+5.2},{:+5.2}) | TRUrpy=({:5.1},{:5.1}) ESTrpy=({:5.1},{:5.1}) | cmd=({:.2},{:.2},{:.2},{:.2})",
                i as f64 * dt,
                w.pos[0].0, w.pos[1].0,
                est.pos[0].0, est.pos[1].0,
                w.vel[0].0, w.vel[1].0,
                est.vel[0].0, est.vel[1].0,
                de(est.vel[0].0, w.vel[0].0), de(est.vel[1].0, w.vel[1].0),
                tr.to_degrees(), tp.to_degrees(),
                r.to_degrees(), p.to_degrees(),
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
            );
        }
    }
}

#[test]
fn trace_divergence_mechanism() {
    let b = base();
    // gps-pos 单独
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_rate = 20.0;
    c.gps_delay = 0.15;
    println!("== trace gps-pos+delay ==");
    trace_cfg("gps-pos+delay", &c);
    // gyro 单独
    let mut c = b.clone();
    c.gyro_noise = 0.003;
    c.gyro_bias = [0.001, -0.0005, 0.002];
    println!("== trace gyro ==");
    trace_cfg("gyro", &c);
    // accel 单独
    let mut c = b.clone();
    c.accel_noise = 0.05;
    c.accel_bias = [0.02, -0.01, 0.05];
    println!("== trace accel ==");
    trace_cfg("accel", &c);
}

#[test]
fn ablation_minimal_trigger() {
    let b = base();
    let mut cfgs: Vec<(&str, SensorConfig)> = Vec::new();
    cfgs.push(("zero", b.clone()));

    // baro 单独
    let mut c = b.clone();
    c.baro_noise = 0.3;
    c.baro_drift = 0.05;
    cfgs.push(("baro", c.clone()));

    // GPS 位置噪声单独
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    cfgs.push(("gps-pos", c.clone()));

    // GPS 延迟单独
    let mut c = b.clone();
    c.gps_delay = 0.15;
    c.gps_rate = 20.0;
    cfgs.push(("gps-delay", c.clone()));

    // GPS 位置噪声 + 延迟 + 降频
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_delay = 0.15;
    c.gps_rate = 20.0;
    cfgs.push(("gps-pos+delay", c.clone()));

    // accel 噪声/偏置/振动单独
    let mut c = b.clone();
    c.accel_noise = 0.05;
    c.accel_bias = [0.02, -0.01, 0.05];
    cfgs.push(("accel", c.clone()));
    let mut c = b.clone();
    c.vib_amp = 0.1;
    cfgs.push(("vib", c.clone()));

    // gyro 单独
    let mut c = b.clone();
    c.gyro_noise = 0.003;
    c.gyro_bias = [0.001, -0.0005, 0.002];
    cfgs.push(("gyro", c.clone()));

    // mag 单独
    let mut c = b.clone();
    c.mag_hard_iron = [0.3, -0.2, 0.4];
    c.mag_soft_iron = [0.98, 1.03, 0.99];
    c.mag_noise = 0.05;
    cfgs.push(("mag", c.clone()));

    println!("== 40s no-contact steady hover (PID), d 设定 -5 ==");
    println!("  配置            d-span(m)   mean_d(m)  hmax(m)   end_h(m)  状态");
    for (name, c) in &cfgs {
        let (span, mean, hmax, endh) = run_cfg(name, c, 40.0);
        let ok = span.is_finite() && span < 2.0 && hmax < 10.0;
        println!(
            "  {name:16} {span:8.2}   {mean:8.2}   {hmax:8.2}   {endh:8.2}  {}",
            if ok { "OK" } else { "DIVERGE" }
        );
    }
}

/// yaw 阻尼消融：固定 gps-pos+delay 触发配置，逐步增大 att_kd（yaw 环阻尼），
/// 观察最大真实偏航角速率 |gyro_z| 是否从指数发散变为有界。
/// 机理（trace_early_phase 实测）：gyro_z 0.012→8.4 rad/s 指数增长，
/// EST/TRU 一致性良好（est_omg_z=0），即物理偏航被 yaw 环驱动打转，
/// 根因是 yaw 环 ζ≈0.09 严重欠阻尼（att_kp=3.0/att_kd=0.3, J_z=0.04, tc=0.02）。
#[test]
fn yaw_damping_ablation() {
    let b = base();
    let dt = 0.004;
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    println!("== yaw damping ablation (gps-pos=0.5 rate=20Hz delay=0.15s, 20s) ==");
    println!("  att_kd   max|gyro_z|(rad/s)  max|est_yaw_deg|  end_cmd  status");
    for kd in [0.3f32, 0.6, 1.0, 1.5, 2.0, 3.0] {
        let mut c = b.clone();
        c.gps_pos_noise = 0.5;
        c.gps_rate = 20.0;
        c.gps_delay = 0.15;
        let world = ToyWorld::new(9.81);
        let vcfg = VehicleConfig::default_quad().with_gains(3.0, kd, 0.3, 0.8);
        let mut ctrl = FlyController::new(world, &vcfg, dt, None, c, ControllerKind::Pid, None, Vec::new());
        let steps = (20.0 / dt) as u64;
        let mut max_gz = 0.0f64;
        let mut max_est_yaw = 0.0f64;
        let mut end_cmd = [0.0f32; 4];
        for i in 0..steps {
            let est = ctrl.step(&sp);
            let imu = ctrl.last_imu();
            max_gz = max_gz.max((imu.gyro[2].0 as f64).abs());
            let y = (2.0 * (est.att.w as f64 * est.att.z as f64 + est.att.x as f64 * est.att.y as f64))
                .atan2(1.0 - 2.0 * (est.att.y as f64 * est.att.y as f64 + est.att.z as f64 * est.att.z as f64));
            max_est_yaw = max_est_yaw.max(y.to_degrees().abs());
            if i == steps - 1 {
                end_cmd = ctrl.last_cmd().motor;
            }
            if !est.pos[0].0.is_finite() { break; }
        }
        let ok = max_gz < 2.0;
        println!(
            "  {kd:6.1}  {max_gz:14.3}  {max_est_yaw:16.1}  ({:.2},{:.2},{:.2},{:.2})  {}",
            end_cmd[0], end_cmd[1], end_cmd[2], end_cmd[3],
            if ok { "OK" } else { "DIVERGE" }
        );
    }
}

/// 开环符号探针：直接驱动 plant 施加纯偏航差速，读引擎系 tau_body_z 与 FC 系 omega_z 符号，
/// 判定 r_cmd→ω_fc_z 的传递方向是否与 roll/pitch 一致。
/// 若 tau_body_z>0 -> omega_fc_z<0，则当前混控 +r_cmd 让 yaw 阻尼项成为正反馈。
#[test]
fn yaw_torque_sign_probe() {
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut plant = QuadrotorPlant::new(world, &vcfg, dt, None, SensorConfig::default(), None, Vec::new());
    // 从悬停油门 0.5 出发，纯偏航差速：(m0+m1) > (m2+m3)
    plant.apply_actuators(&ActuatorCmd { motor: [0.6, 0.6, 0.4, 0.4] });
    for _ in 0..20 { plant.step(); }
    let tau = plant.debug_tau_body();
    let s = plant.state_ned();
    println!(
        "m=[0.6,0.6,0.4,0.4] (m0+m1)>(m2+m3): tau_body_z={:+.4}  omega_fc_z={:+.4}",
        tau[2], s.omega[2].0
    );
    // 反向差速
    plant.apply_actuators(&ActuatorCmd { motor: [0.4, 0.4, 0.6, 0.6] });
    for _ in 0..20 { plant.step(); }
    let tau = plant.debug_tau_body();
    let s = plant.state_ned();
    println!(
        "m=[0.4,0.4,0.6,0.6] (m0+m1)<(m2+m3): tau_body_z={:+.4}  omega_fc_z={:+.4}",
        tau[2], s.omega[2].0
    );
    // 纯 roll 差速对照：(m0+m3) > (m1+m2)
    plant.apply_actuators(&ActuatorCmd { motor: [0.6, 0.4, 0.4, 0.6] });
    for _ in 0..20 { plant.step(); }
    let tau = plant.debug_tau_body();
    let s = plant.state_ned();
    println!(
        "m=[0.6,0.4,0.4,0.6] (m0+m3)>(m1+m2): tau_body_x={:+.4}  omega_fc_x={:+.4}",
        tau[0], s.omega[0].0
    );
}

/// 高密度姿态发散诊断：gps-pos 噪声下，机体如何从水平翻滚到 90°（TRU roll），
/// 而 EKF 仍认为水平（EST roll≈0）。每 0.1s 打印 TRU/EST 欧拉角、角速率、指令。
/// 判定漂移机制：姿态环饱和 / EST 姿态未收敛 / 位置外环引入的水平耦合。
#[test]
fn trace_attitude_divergence() {
    let b = base();
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_rate = 20.0;
    c.gps_delay = 0.15;
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut ctrl = FlyController::new(world, &vcfg, dt, None, c, ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let eul = |q: &flyctrl_core::vehicle::Quaternion| -> (f64, f64, f64) {
        (
            (2.0 * (q.w as f64 * q.x as f64 + q.y as f64 * q.z as f64))
                .atan2(1.0 - 2.0 * (q.x as f64 * q.x as f64 + q.y as f64 * q.y as f64)),
            (2.0 * (q.w as f64 * q.y as f64 - q.z as f64 * q.x as f64)).asin(),
            (2.0 * (q.w as f64 * q.z as f64 + q.x as f64 * q.y as f64))
                .atan2(1.0 - 2.0 * (q.y as f64 * q.y as f64 + q.z as f64 * q.z as f64)),
        )
    };
    println!("== gps-pos+delay 姿态发散早段 (t=0..10s, 0.1s 采样) ==");
    println!("   列：t  TRUrpy(yaw)  ESTrpy(yaw)  TRUomg  ESTomg  vh  cmd");
    println!("   补充列：TRUpos/TRUvel  ESTpos/ESTvel  att_err(x,y,z)  pqr_cmd");
    for i in 0..(10.0 / dt) as u64 {
        let est = ctrl.step(&sp);
        let w = ctrl.world_state();
        if i % 25 == 0 || (i < 500 && i % 5 == 0) {
            let cmd = ctrl.last_cmd();
            let (tr, tp, ty) = eul(&w.att);
            let (r, p, y) = eul(&est.att);
            let vh = (w.vel[0].0 as f64).hypot(w.vel[1].0 as f64);
            let (err, pqr, _omg) = ctrl.debug_pid_pqr();
            let fw = ctrl.debug_f_world();
            println!(
                "  t={:4.2}s TRUrpy/yaw=({:6.1},{:6.1},{:6.1}) ESTrpy/yaw=({:6.1},{:6.1},{:6.1}) | TRUomg=({:+5.2},{:+5.2},{:+5.2}) ESTomg=({:+5.2},{:+5.2},{:+5.2}) | vh={:5.2} cmd=({:.2},{:.2},{:.2},{:.2}) | TRUpos=({:6.1},{:6.1}) TRUvel=({:+5.2},{:+5.2}) | ESTpos=({:6.1},{:6.1}) ESTvel=({:+5.2},{:+5.2}) dvel=({:+5.2},{:+5.2}) | err=({:+5.3},{:+5.3},{:+5.3}) pqr=({:+5.2},{:+5.2},{:+5.2}) | Fworld=({:+5.2},{:+5.2},{:+5.2})",
                i as f64 * dt,
                tr.to_degrees(), tp.to_degrees(), ty.to_degrees(),
                r.to_degrees(), p.to_degrees(), y.to_degrees(),
                w.omega[0].0, w.omega[1].0, w.omega[2].0,
                est.omega[0].0, est.omega[1].0, est.omega[2].0,
                vh,
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                w.pos[0].0, w.pos[1].0,
                w.vel[0].0, w.vel[1].0,
                est.pos[0].0, est.pos[1].0,
                est.vel[0].0, est.vel[1].0,
                est.vel[0].0 - w.vel[0].0, est.vel[1].0 - w.vel[1].0,
                err[0], err[1], err[2],
                pqr[0], pqr[1], pqr[2],
                fw[0], fw[1], fw[2],
            );
            if !w.pos[0].0.is_finite() { break; }
        }
    }
}

/// 决定性探针：直接给 plant 施加纯 +roll 电机差速（m0,m3 增 / m1,m2 减，tau_body_x>0），
/// 观察物理上机体向哪侧翻滚、合力向哪个水平方向倾斜、NED 速度往哪个方向走。
/// 判定当前混控 +roll -> 引擎力矩 -> 物理翻滚 -> 水平推力的完整符号链是否正确：
///   若 +roll 应右倾(标准 NED) -> 推力东向 -> NED +Y 速度；
///   若实际左倾 -> 推力西向 -> NED -Y 速度，则 roll 符号链有反转。
#[test]
fn roll_force_probe() {
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut plant = QuadrotorPlant::new(world, &vcfg, dt, None, SensorConfig::default(), None, Vec::new());
    // 悬停油门 0.5 上叠加纯 roll 差速（+roll：m0,m3 增 / m1,m2 减）
    plant.apply_actuators(&ActuatorCmd { motor: [0.6, 0.4, 0.4, 0.6] });
    let eul_xyz = |q: &flyctrl_core::vehicle::Quaternion| -> (f64, f64, f64) {
        (
            (2.0 * (q.w as f64 * q.x as f64 + q.y as f64 * q.z as f64))
                .atan2(1.0 - 2.0 * (q.x as f64 * q.x as f64 + q.y as f64 * q.y as f64)),
            (2.0 * (q.w as f64 * q.y as f64 - q.z as f64 * q.x as f64)).asin(),
            (2.0 * (q.w as f64 * q.z as f64 + q.x as f64 * q.y as f64))
                .atan2(1.0 - 2.0 * (q.y as f64 * q.y as f64 + q.z as f64 * q.z as f64)),
        )
    };
    println!("== roll_force_probe: 纯 +roll 电机差速 [0.6,0.4,0.4,0.6] (tau_body_x>0) 1.6s ==");
    println!("   t     tau_x    eul_up.x(引擎)  TRU_ned_roll  Fworld_eng(x=N,y=U,z=W)  NEDvel(n,e,d)");
    for i in 0..(1.6 / dt) as u64 {
        plant.step();
        if i % 100 == 0 {
            let tau = plant.debug_tau_body();
            let (_, q_up_arr) = plant.debug_up();
            let q_up = flyctrl_core::vehicle::Quaternion { w: q_up_arr[0] as f32, x: q_up_arr[1] as f32, y: q_up_arr[2] as f32, z: q_up_arr[3] as f32 };
            let (ux, _, _) = eul_xyz(&q_up);
            let s = plant.state_ned();
            let fw = plant.debug_f_world();
            let roll = (2.0 * (s.att.w as f64 * s.att.x as f64 + s.att.y as f64 * s.att.z as f64))
                .atan2(1.0 - 2.0 * (s.att.x as f64 * s.att.x as f64 + s.att.y as f64 * s.att.y as f64));
            println!(
                "  {:4.2}s  {:+.4}   {:+.2}deg      {:+.2}deg      ({:+.2},{:+.2},{:+.2})          ({:+.2},{:+.2},{:+.2})",
                i as f64 * dt, tau[0], ux.to_degrees(), roll.to_degrees(),
                fw[0], fw[1], fw[2],
                s.vel[0].0, s.vel[1].0, s.vel[2].0,
            );
        }
    }
}

/// 坐标系探针：同一物理时刻，对比四套"真值"——
///  (1) debug_up() 原始引擎世界系四元数 q_up 的欧拉角（引擎系公式，y 为俯仰）
///  (2) state_ned()/world_state() 的 NED 四元数欧拉角（即 TRU，测试里报 90°）
///  (3) IMU 原始加计（accel_fc，期望水平悬停 = [0,0,-g]）
///  (4) Fworld（引擎世界系合力）
/// 判定 90° 差是"报告伪影"还是"物理翻滚"：若 (1) 水平但 (2)=90°，则 TRU 报告有误；
/// 若 (1) 也 90°，则物理确实侧倾，需查 EST 为何仍报 0。
#[test]
fn probe_frame_conventions() {
    let b = base();
    let mut c = b.clone();
    c.gps_pos_noise = 0.5;
    c.gps_rate = 20.0;
    c.gps_delay = 0.15;
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut ctrl = FlyController::new(world, &vcfg, dt, None, c, ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    // 引擎系欧拉（前-X 右-Y 上-Z）：约定 roll=绕X(pitch 项)、pitch=绕Y、yaw=绕Z
    let eul_xyz = |q: &flyctrl_core::vehicle::Quaternion| -> (f64, f64, f64) {
        (
            (2.0 * (q.w as f64 * q.x as f64 + q.y as f64 * q.z as f64))
                .atan2(1.0 - 2.0 * (q.x as f64 * q.x as f64 + q.y as f64 * q.y as f64)),
            (2.0 * (q.w as f64 * q.y as f64 - q.z as f64 * q.x as f64)).asin(),
            (2.0 * (q.w as f64 * q.z as f64 + q.x as f64 * q.y as f64))
                .atan2(1.0 - 2.0 * (q.y as f64 * q.y as f64 + q.z as f64 * q.z as f64)),
        )
    };
    println!("== 坐标系探针 (zero-noise gps-pos=0.5 下, 12s/0.5s 采样) ==");
    println!("   t=0s  初始悬停喷油 0.51，期望：q_up=identity(水平)、TRU 若报 90° 即报告伪影、IMU accel=[0,0,-g]");
    let steps = (12.0 / dt) as u64;
    for i in 0..steps {
        ctrl.step(&sp);
        if i % 125 == 0 {
            let imu = ctrl.last_imu();
            let (pos_up, q_up_arr) = ctrl.debug_up();
            let q_up = flyctrl_core::vehicle::Quaternion {
                w: q_up_arr[0] as f32,
                x: q_up_arr[1] as f32,
                y: q_up_arr[2] as f32,
                z: q_up_arr[3] as f32,
            };
            let w = ctrl.world_state(); // TRU (state_ned)
            let est = ctrl.debug_estimate_ned();
            let (ux, uy, uz) = eul_xyz(&q_up);
            let (rx, ry, rz) = eul_xyz(&w.att);
            let fw = ctrl.debug_f_world();
            println!(
                "  t={:4.2}s q_up=({:+.2},{:+.2},{:+.2},{:+.2}) eul_up=(x{:+.1},y{:+.1},z{:+.1}) | TRU_ned=(r{:+.1},p{:+.1},y{:+.1}) EST_ned=(r{:+.1},p{:+.1},y{:+.1}) | imu_accel=({:+.2},{:+.2},{:+.2}) imu_gyro=({:+.3},{:+.3},{:+.3}) | Fworld_up=({:+.2},{:+.2},{:+.2}) | TRUvel=({:+.2},{:+.2},{:+.2})",
                i as f64 * dt,
                q_up_arr[0], q_up_arr[1], q_up_arr[2], q_up_arr[3],
                ux.to_degrees(), uy.to_degrees(), uz.to_degrees(),
                rx.to_degrees(), ry.to_degrees(), rz.to_degrees(),
                (2.0 * (est.att.w as f64 * est.att.x as f64 + est.att.y as f64 * est.att.z as f64))
                    .atan2(1.0 - 2.0 * (est.att.x as f64 * est.att.x as f64 + est.att.y as f64 * est.att.y as f64))
                    .to_degrees(),
                (2.0 * (est.att.w as f64 * est.att.y as f64 - est.att.z as f64 * est.att.x as f64)).asin().to_degrees(),
                (2.0 * (est.att.w as f64 * est.att.z as f64 + est.att.x as f64 * est.att.y as f64))
                    .atan2(1.0 - 2.0 * (est.att.y as f64 * est.att.y as f64 + est.att.z as f64 * est.att.z as f64))
                    .to_degrees(),
                imu.accel[0].0, imu.accel[1].0, imu.accel[2].0,
                imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0,
                fw[0], fw[1], fw[2],
                w.vel[0].0, w.vel[1].0, w.vel[2].0,
            );
        }
    }
}

/// 决定性闭环符号探针：zero 噪声下给定东向(+Y)设定点 pos=[0,20,-5]，
/// 观察机体实际向哪个水平方向飞行：
///   正确符号链：东向指令 -> 东向推力 -> TRU NED +Y 增长；
///   若 +Y 指令产生西向推力 -> TRU NED -Y（游走发散），则 roll 符号链反转。
/// 同时打印 TRU/EST 欧拉角与电机指令，判定 P 项符号。
#[test]
fn closed_loop_sign_probe() {
    let world = ToyWorld::new(9.81);
    let vcfg = VehicleConfig::default_quad();
    let dt = 0.004;
    let mut ctrl = FlyController::new(world, &vcfg, dt, None, SensorConfig::default(), ControllerKind::Pid, None, Vec::new());
    let sp = Setpoint {
        pos: [Meter(0.0), Meter(20.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let eul = |q: &flyctrl_core::vehicle::Quaternion| -> (f64, f64, f64) {
        (
            (2.0 * (q.w as f64 * q.x as f64 + q.y as f64 * q.z as f64))
                .atan2(1.0 - 2.0 * (q.x as f64 * q.x as f64 + q.y as f64 * q.y as f64)),
            (2.0 * (q.w as f64 * q.y as f64 - q.z as f64 * q.x as f64)).asin(),
            (2.0 * (q.w as f64 * q.z as f64 + q.x as f64 * q.y as f64))
                .atan2(1.0 - 2.0 * (q.y as f64 * q.y as f64 + q.z as f64 * q.z as f64)),
        )
    };
    println!("== closed_loop_sign_probe: 北向(+X)设定点 20m, zero 噪声, 14s/0.5s 采样 ==");
    println!("   t      TRUpos(n,e,d)      TRUvel(n,e,d)     TRUrpy(deg)      ESTrpy(deg)     cmd");
    let sp = Setpoint {
        pos: [Meter(20.0), Meter(0.0), Meter(-5.0)],
        vel: [MeterPerSecond(0.0); 3],
        acc: [MeterPerSecondSquared::ZERO; 3],
        yaw: Radian(0.0),
    };
    let steps = (14.0 / dt) as u64;
    for i in 0..steps {
        let est = ctrl.step(&sp);
        let w = ctrl.world_state();
        if i % 125 == 0 {
            let (tr, tp, _) = eul(&w.att);
            let (er, ep, _) = eul(&est.att);
            let cmd = ctrl.last_cmd();
            println!(
                "  {:5.1}s  ({:7.2},{:7.2},{:7.2})  ({:6.2},{:6.2},{:6.2})  ({:6.1},{:6.1})  ({:6.1},{:6.1})  ({:.2},{:.2},{:.2},{:.2})",
                i as f64 * dt,
                w.pos[0].0, w.pos[1].0, w.pos[2].0,
                w.vel[0].0, w.vel[1].0, w.vel[2].0,
                tr.to_degrees(), tp.to_degrees(),
                er.to_degrees(), ep.to_degrees(),
                cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
            );
            if !w.pos[0].0.is_finite() { break; }
        }
    }
}

