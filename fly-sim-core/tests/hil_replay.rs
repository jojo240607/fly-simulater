//! HIL 输入录制 → SIL 回放对比（诊断 harness，无硬断言）。
//!
//! 读服务器 `rec_event` 落盘的 `.dbg/trae-debug-log-hil-input-replay.ndjson`，
//! 把 MCU 实际收到的输入序列（IMU/偏航/气压/GPS + 注入节奏）**原样**回放到
//! `flyctrl_core::hil::HilContext::step_hil`（SIL/MCU 共享单步），输出 SIL 的
//! 姿态估计与电机指令，与 MCU 实际下行 `att`/`act`/`lpos` 对齐对比。
//!
//! 两种消费模型（区分假设 A/B，见 debug-hil-input-replay.md）：
//! - **Model0**：每注入帧一调用（理想消费，SIL 常规路径）——隔离"输入数值本身"
//!   是否触发发散（若发散 → 输入序列的数值轨迹即根因）。
//! - **Model1**：按 MCU 4ms 控制拍重建 tick 网格（tick 周期 = 4ms/ratio，ratio 由
//!   att 遥测 20ms 间隔 / 墙钟窗口测得），帧间 `imu=None` → `step_hil` 回退
//!   sample-and-hold（复刻 HIL 注入稀疏/缺帧节奏）。若 Model0 稳而 Model1 发散 →
//!   根因在注入节奏（H2）；两者都稳而 MCU 发散 → 根因在 MCU 端处理（H3/H4）。
//!
//! 回放对齐：三者统一按"相对录制起点的墙钟秒"分桶（Model0 每帧=其到达墙钟，
//! Model1 每拍=t0+k*tick_pc，MCU 事件=自身墙钟），输出各桶估计/指令偏差。

use std::io::BufRead;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::pid::PidController;
use flyctrl_core::controller::Setpoint;
use flyctrl_core::estimator::ekf::EkfEstimator;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::{
    Meter, MeterPerSecond, MeterPerSecondSquared, Radian, RadianPerSecond, Second,
};
use flyctrl_core::vehicle::{ImuSample, PosSample};

const REC_PATH: &str = r"d:\project\game\fly-simulater\.dbg\trae-debug-log-hil-input-replay.ndjson";

// ---------------------------------------------------------------- 解析（最小 NDJSON）

fn f32v(s: &str) -> f32 {
    if let Some(h) = s.trim().strip_prefix("0x") {
        f32::from_bits(u32::from_str_radix(h, 16).unwrap())
    } else {
        s.trim().parse().unwrap_or(f32::NAN)
    }
}

/// 取 `"key":` 后的值（标量或 `[...]` 数组，原样字符串返回）。
fn val<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{key}\":");
    let start = line.find(&pat)? + pat.len();
    let rest = &line[start..];
    if let Some(a) = rest.strip_prefix('[') {
        let end = a.find(']')? + 1;
        Some(&a[..end])
    } else {
        let end = rest.find([',', '}']).unwrap_or(rest.len());
        Some(&rest[..end])
    }
}

fn arr(s: &str) -> Vec<f32> {
    s.trim()
        .trim_start_matches('[')
        .trim_end_matches(']')
        .split(',')
        .map(f32v)
        .collect()
}

struct Inj {
    ts: i64,   // 墙钟 ms
    sim: u64,  // sim_t_us（物理仿真时间）
    seq: u64,
    nav: bool,
    imu: [f32; 6],
    yaw: f32,
    baro: f32,
    pos: [f32; 3],
    vel: [f32; 3],
}

struct Att {
    ts: i64,
    r: f32,
    p: f32,
    y: f32,
}
struct Act {
    ts: i64,
    m: [f32; 4],
}
struct Lpos {
    ts: i64,
    x: f32,
    y: f32,
    z: f32,
}

fn load() -> (Vec<Inj>, Vec<Att>, Vec<Act>, Vec<Lpos>) {
    let f = std::fs::File::open(REC_PATH).expect("录制文件不存在（先跑 HIL 录制）");
    let mut injs = vec![];
    let mut atts = vec![];
    let mut acts = vec![];
    let mut lposs = vec![];
    for line in std::io::BufReader::new(f).lines() {
        let line = line.unwrap();
        if line.trim().is_empty() {
            continue;
        }
        let evt = val(&line, "evt").unwrap_or("").trim_matches('"').to_string();
        match evt.as_str() {
            "inj" => {
                let imu = arr(val(&line, "imu").unwrap_or(""));
                injs.push(Inj {
                    ts: val(&line, "ts").unwrap().parse().unwrap(),
                    sim: val(&line, "sim_t_us").unwrap_or("0").parse().unwrap(),
                    seq: val(&line, "seq").unwrap().parse().unwrap(),
                    nav: val(&line, "nav").unwrap().parse::<i32>().unwrap() != 0,
                    imu: [imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]],
                    yaw: f32v(val(&line, "yaw").unwrap()),
                    baro: f32v(val(&line, "baro").unwrap()),
                    pos: {
                        let p = arr(val(&line, "pos").unwrap_or(""));
                        [p[0], p[1], p[2]]
                    },
                    vel: {
                        let p = arr(val(&line, "vel").unwrap_or(""));
                        [p[0], p[1], p[2]]
                    },
                });
            }
            "att" => atts.push(Att {
                ts: val(&line, "ts").unwrap().parse().unwrap(),
                r: f32v(val(&line, "r").unwrap()),
                p: f32v(val(&line, "p").unwrap()),
                y: f32v(val(&line, "y").unwrap()),
            }),
            "act" => {
                let m = arr(val(&line, "m").unwrap_or(""));
                acts.push(Act {
                    ts: val(&line, "ts").unwrap().parse().unwrap(),
                    m: [m[0], m[1], m[2], m[3]],
                });
            }
            "lpos" => lposs.push(Lpos {
                ts: val(&line, "ts").unwrap().parse().unwrap(),
                x: f32v(val(&line, "x").unwrap()),
                y: f32v(val(&line, "y").unwrap()),
                z: f32v(val(&line, "z").unwrap()),
            }),
            _ => {}
        }
    }
    (injs, atts, acts, lposs)
}

// ---------------------------------------------------------------- 回放核心

struct TickOut {
    wall_s: f64,
    att: [f32; 3], // rad
    cmd: [f32; 4],
    pos: [f32; 3],
    finite: bool,
}

fn make_ctx() -> HilContext<EkfEstimator, PidController> {
    let cfg = VehicleConfig::default_quad();
    HilContext::new(
        EkfEstimator::default_quad(),
        PidController::from_config(&cfg.ctrl_params()),
        Second(0.004),
    )
}

fn hover_sp() -> Setpoint {
    Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-5.0)], Radian(0.0))
}

fn imu_of(j: &Inj) -> ImuSample {
    ImuSample {
        accel: [
            MeterPerSecondSquared(j.imu[0]),
            MeterPerSecondSquared(j.imu[1]),
            MeterPerSecondSquared(j.imu[2]),
        ],
        gyro: [
            RadianPerSecond(j.imu[3]),
            RadianPerSecond(j.imu[4]),
            RadianPerSecond(j.imu[5]),
        ],
    }
}

fn gps_of(j: &Inj) -> PosSample {
    PosSample::with_vel(
        [Meter(j.pos[0]), Meter(j.pos[1]), Meter(j.pos[2])],
        [
            MeterPerSecond(j.vel[0]),
            MeterPerSecond(j.vel[1]),
            MeterPerSecond(j.vel[2]),
        ],
    )
}

fn att_rad(q: &flyctrl_core::vehicle::Quaternion) -> [f32; 3] {
    [q.roll(), q.pitch(), q.yaw()]
}

/// Model0：每注入帧一调用（理想消费）。返回逐 tick 输出（wall = 该帧到达墙钟）。
fn replay_model0(injs: &[Inj]) -> Vec<TickOut> {
    let mut ctx = make_ctx();
    let mut sim_imu = SimImu::new();
    let sp = hover_sp();
    let mut out = vec![];
    for j in injs {
        let r = ctx.step_hil(
            Some(imu_of(j)),
            if j.nav { Some(gps_of(j)) } else { None },
            Some(j.baro),
            None,
            None,
            &sp,
            true,
            true,
            true,
            &mut sim_imu,
        );
        out.push(TickOut {
            wall_s: (j.ts - injs[0].ts) as f64 / 1000.0,
            att: att_rad(&r.est.att),
            cmd: r.cmd.motor,
            pos: [r.est.pos[0].0, r.est.pos[1].0, r.est.pos[2].0],
            finite: r.est.att.w.is_finite(),
        });
    }
    out
}

/// Model1：按 MCU 4ms 控制拍重建 tick 网格（tick 周期 = 4ms/ratio），帧间回退
/// sample-and-hold（`imu=None` → step_hil 保持最近真实帧）。返回逐 tick 输出
/// （wall = t0 + k*tick_pc）。
fn replay_model1(injs: &[Inj], atts: &[Att], acts: &[Act], lposs: &[Lpos]) -> (Vec<TickOut>, f64) {
    // ratio = MCU 时间 / 墙钟窗口（att 遥测 20ms 间隔 → 5 控制拍）。
    let a0 = atts[0].ts.min(acts[0].ts.min(lposs[0].ts));
    let a1 = atts[atts.len() - 1].ts.max(acts[acts.len() - 1].ts.max(lposs[lposs.len() - 1].ts));
    let mcu_ms = atts.len() as f64 * 20.0;
    let wall_ms = (a1 - a0) as f64;
    let ratio = mcu_ms / wall_ms;
    let tick_pc_ms = 4.0 / ratio;

    let t0 = injs[0].ts;
    let mut ctx = make_ctx();
    let mut sim_imu = SimImu::new();
    let sp = hover_sp();
    let mut out = vec![];
    let mut iptr = 0usize; // 最近已到达帧
    let mut last_seq = 0u64;
    let n_tick = ((injs[injs.len() - 1].ts - t0) as f64 / tick_pc_ms).ceil() as usize;
    for k in 0..=n_tick {
        let t = t0 + (k as f64 * tick_pc_ms) as i64;
        while iptr + 1 < injs.len() && injs[iptr + 1].ts <= t {
            iptr += 1;
        }
        let j = &injs[iptr];
        let newf = j.seq != last_seq;
        if newf {
            last_seq = j.seq;
        }
        let r = ctx.step_hil(
            if newf { Some(imu_of(j)) } else { None },
            if newf && j.nav { Some(gps_of(j)) } else { None },
            if newf { Some(j.baro) } else { None },
            None,
            None,
            &sp,
            true,
            true,
            true,
            &mut sim_imu,
        );
        out.push(TickOut {
            wall_s: (t - t0) as f64 / 1000.0,
            att: att_rad(&r.est.att),
            cmd: r.cmd.motor,
            pos: [r.est.pos[0].0, r.est.pos[1].0, r.est.pos[2].0],
            finite: r.est.att.w.is_finite(),
        });
    }
    (out, tick_pc_ms)
}

// ---------------------------------------------------------------- 汇总

/// 按墙钟秒分桶，输出各桶 SIL 与 MCU 的姿态/指令/位置偏差。
fn report(name: &str, sim: &[TickOut], atts: &[Att], acts: &[Act], lposs: &[Lpos], wall0: i64) {
    // 统计每桶 [b, b+1)s：sim 均值 / MCU 最近值均值。
    use std::collections::BTreeMap;
    let mut sm: BTreeMap<i64, Vec<&TickOut>> = BTreeMap::new();
    for s in sim {
        sm.entry(s.wall_s as i64).or_default().push(s);
    }
    let mut att_m: BTreeMap<i64, Vec<&Att>> = BTreeMap::new();
    for a in atts {
        att_m.entry((a.ts - wall0) / 1000).or_default().push(a);
    }
    let mut act_m: BTreeMap<i64, Vec<&Act>> = BTreeMap::new();
    for a in acts {
        act_m.entry((a.ts - wall0) / 1000).or_default().push(a);
    }
    let mut lp_m: BTreeMap<i64, Vec<&Lpos>> = BTreeMap::new();
    for l in lposs {
        lp_m.entry((l.ts - wall0) / 1000).or_default().push(l);
    }

    let mut div = 0usize; // SIL 发散 tick 数（|R|或|P|>45° 或非有限）
    let mut diag_sat = 0usize; // SIL 对角饱和指令数
    for s in sim {
        if !s.finite || s.att[0].abs() > 0.785 || s.att[1].abs() > 0.785 {
            div += 1;
        }
        if (s.cmd[0] > 0.9 && s.cmd[1] < 0.1 && s.cmd[2] > 0.9 && s.cmd[3] < 0.1)
            || (s.cmd[0] < 0.1 && s.cmd[1] > 0.9 && s.cmd[2] < 0.1 && s.cmd[3] > 0.9)
        {
            diag_sat += 1;
        }
    }
    println!(
        "== {name} ==  ticks={} 发散tick={} ({:.1}%) 对角饱和tick={} ({:.1}%)",
        sim.len(),
        div,
        100.0 * div as f64 / sim.len().max(1) as f64,
        diag_sat,
        100.0 * diag_sat as f64 / sim.len().max(1) as f64
    );
    println!(
        "{:>5} | {:>7} | {:>7} {:>7} {:>7} | {:>6} {:>6} {:>6} {:>6} | {:>6} | {:>7} {:>7} | {:>6} | {:>7} {:>7}",
        "wall_s", "SIL_fin", "dR", "dP", "dY", "dM0", "dM1", "dM2", "dM3", "d|pos|", "SIL_z", "MCU_z", "dX", "SIL_x", "MCU_x"
    );
    for b in 0..=sim[sim.len() - 1].wall_s as i64 {
        let ss = &sm.get(&b).cloned().unwrap_or_default();
        if ss.is_empty() {
            continue;
        }
        let n = ss.len();
        let (mut r, mut p, mut y, mut m, mut x, mut yp, mut z) =
            (0f32, 0f32, 0f32, [0f32; 4], 0f32, 0f32, 0f32);
        let mut fin = 0usize;
        for s in ss {
            fin += if s.finite { 1 } else { 0 };
            r += s.att[0].to_degrees();
            p += s.att[1].to_degrees();
            y += s.att[2].to_degrees();
            for i in 0..4 {
                m[i] += s.cmd[i];
            }
            x += s.pos[0];
            yp += s.pos[1];
            z += s.pos[2];
        }
        r /= n as f32;
        p /= n as f32;
        y /= n as f32;
        for i in 0..4 {
            m[i] /= n as f32;
        }
        x /= n as f32;
        yp /= n as f32;
        z /= n as f32;

        let ma = att_m.get(&b).cloned().unwrap_or_default();
        let (mut ar, mut ap, mut ay) = (0f32, 0f32, 0f32);
        for a in &ma {
            ar += a.r.to_degrees();
            ap += a.p.to_degrees();
            ay += a.y.to_degrees();
        }
        if !ma.is_empty() {
            ar /= ma.len() as f32;
            ap /= ma.len() as f32;
            ay /= ma.len() as f32;
        }
        let mc = act_m.get(&b).cloned().unwrap_or_default();
        let mut mm = [0f32; 4];
        for a in &mc {
            for i in 0..4 {
                mm[i] += a.m[i];
            }
        }
        if !mc.is_empty() {
            for i in 0..4 {
                mm[i] /= mc.len() as f32;
            }
        }
        let ml = lp_m.get(&b).cloned().unwrap_or_default();
        let (mut lx, mut lz) = (0f32, 0f32);
        for l in &ml {
            lx += l.x;
            lz += l.z;
        }
        if !ml.is_empty() {
            lx /= ml.len() as f32;
            lz /= ml.len() as f32;
        }

        println!(
            "{:>5} | {:>7}% | {:>7.1} {:>7.1} {:>7.1} | {:>6.2} {:>6.2} {:>6.2} {:>6.2} | {:>6.2} | {:>7.1} {:>7.1} | {:>6.1} | {:>7.1} {:>7.1}",
            b, 100.0 * fin as f64 / n as f64,
            r - ar, p - ap, y - ay,
            m[0] - mm[0], m[1] - mm[1], m[2] - mm[2], m[3] - mm[3],
            ((x - lx) * (x - lx) + (yp) .powi(2)).sqrt(),
            z, lz,
            x - lx,
            x, lx
        );
    }
    println!();
}

#[test]
fn hil_replay_recorded_input() {
    let (injs, atts, acts, lposs) = load();
    println!(
        "录制: inj={} att={} act={} lpos={} | 帧 sim 覆盖 {:.3}s | 墙钟 {:.1}s",
        injs.len(),
        atts.len(),
        acts.len(),
        lposs.len(),
        injs[injs.len() - 1].sim as f64 / 1e6,
        (injs[injs.len() - 1].ts - injs[0].ts) as f64 / 1000.0
    );
    println!("注: 录制缺失 sim 字段注入时该字段为 0（当前录制不打印）；att 为 MCU EKF 姿态(rad)。");

    let m0 = replay_model0(&injs);
    report("Model0 每帧1拍(理想消费)", &m0, &atts, &acts, &lposs, injs[0].ts);

    let (m1, tick_pc) = replay_model1(&injs, &atts, &acts, &lposs);
    println!(
        "Model1 tick_pc={:.2}ms (ratio={:.3}) ticks={} sim={:.2}s",
        tick_pc,
        4.0 / tick_pc,
        m1.len(),
        m1.len() as f64 * 0.004
    );
    report("Model1 4ms控制拍(缺帧回退)", &m1, &atts, &acts, &lposs, injs[0].ts);
}
