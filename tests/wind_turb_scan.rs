//! 阵风+湍流 × 风速 的**倾角能力曲线**——复刻 M 场 `x_hover_env` 的风场。
//!
//! # 为什么需要它（对应"解决根本问题、不掩盖"）
//!
//! M 场 `x_hover_env` 在 2.5m/s 侧风 + 阵风 + Dryden 湍流下 pitch 达 35.3°（闸 25°）。
//! 但 H 场既有的 `wind_scan` 只扫**恒定风**（实测到 2.0m/s 倾角仍 <0.5°），
//! `headless_hover_wind` 只到 **0.5m/s** 且原文写明"**本测例不验证抗风位置保持**"。
//! 于是"25° 闸在 2.5m/s+阵风+湍流下对当前控制器**到底可不可达**"此前**从未被测过**。
//!
//! 在不知道答案前就加低通/改判据，属于掩盖症状。本测例先把曲线量出来：
//! - `base=0` 行 = **纯阵风+湍流**（阵风幅度固定在 1.2m/s）→ 分离"恒风 vs 阵风湍流"；
//! - 逐档加恒风 → 看倾角是**渐进上升**（正常抗风）还是**悬崖式失稳**（裕度边界）。
//!
//! 用法：`cargo test --release --test wind_turb_scan --features phy -- --nocapture`
//!
//! 本测例只**打印量化对比**，不作通过性断言（判据归属见 `docs/test-roadmap.md` §1）。

#![cfg(feature = "phy")]

use fly_sim_core::controller::{ControllerKind, FlyController, hover_setpoint};
use fly_sim_core::physics::{ContactModel, PhySdkWorld};
use fly_sim_core::sensor::{SensorConfig, SensorFault};
use fly_sim_core::wind::{WindConfig, WindField};
use fly_simulater::airframe::load_airframe;

const DT: f64 = 0.004;

/// 四元数分量 -> (roll, pitch) 度。
fn rp_deg(w: f32, x: f32, y: f32) -> (f64, f64) {
    let (w, x, y) = (w as f64, x as f64, y as f64);
    let roll = f64::atan2(2.0 * w * x, 1.0 - 2.0 * x * x);
    let pitch = f64::asin((2.0 * w * y).clamp(-1.0, 1.0));
    (roll.to_degrees(), pitch.to_degrees())
}

/// 跑一趟：全程 max|roll|/max|pitch| + 后段（>10s）稳态 max（区分启动瞬态与持续失稳）。
fn run(speed: f64, secs: f64, rate_mode: bool) -> (f64, f64, f64, f64, bool, f64, f64) {
    run_var(speed, secs, rate_mode, 0)
}

/// `var` 风场变体：0=完整(北+东) 1=仅北(东置零) 2=仅东(北置零) 3=无恒风(留阵风+湍流)
fn run_var(speed: f64, secs: f64, rate_mode: bool, var: u8) -> (f64, f64, f64, f64, bool, f64, f64) {
    let (a, b, c, d, e, f, g, _pn, _pe) = run_var_bias(speed, secs, rate_mode, var, 0);
    (a, b, c, d, e, f, g)
}

/// `bias`：0=两者(既有) 1=都不注入 2=只陀螺漂移 3=只加计漂移。
/// 返回额外带出战：末位置 (n,e)、末估计速度 (n,e)。
fn run_var_bias(speed: f64, secs: f64, rate_mode: bool, var: u8, bias: u8) -> (f64, f64, f64, f64, bool, f64, f64, f64, f64) {
    let cfg = load_airframe(None).expect("default airframe");
    // **逐字复刻 `x_hover_env` 的风场**，只把 base 北向风速参数化（东向按 2.5:1.0 同比）。
    // **引用唯一真源** `WindConfig::beaufort3()`，只把 base 风速参数化以扫能力曲线。
    // 原先此处手抄一份风场、与 M 场 `x_hover_env` 的参数并不一致 —— 那样两场输入
    // 不可比，H 场结论无法作为 M 场验收依据。
    let mut w = WindConfig::beaufort3();
    let dir = if w.base[0].abs() > 1e-9 { w.base[2] / w.base[0] } else { 0.0 };
    w.base = [speed, 0.0, dir * speed];
    match var {
        1 => w.base[2] = 0.0,          // 仅北风
        2 => w.base[0] = 0.0,          // 仅东风
        3 => { w.base = [0.0, 0.0, 0.0]; } // 无恒风（阵风/湍流仍在）
        _ => {}
    }
    let wind = Some(WindField::new(w));
    let mut ctrl = FlyController::new(
        PhySdkWorld::create_empty(),
        &cfg,
        DT,
        wind,
        SensorConfig::realistic(),
        ControllerKind::Pid,
        Some(ContactModel::default()),
        Vec::new(),
    );
    // 温漂故障注入（可分离）：0=两者（既有行为，逐位不变）
    match bias {
        1 => {}
        2 => {
            ctrl.inject_sensor_fault(SensorFault::GyroDrift([0.0001, -0.00005, 0.0001]));
        }
        3 => {
            ctrl.inject_sensor_fault(SensorFault::AccelDrift([0.0002, 0.0, 0.0002]));
        }
        _ => {
            ctrl.inject_sensor_fault(SensorFault::GyroDrift([0.0001, -0.00005, 0.0001]));
            ctrl.inject_sensor_fault(SensorFault::AccelDrift([0.0002, 0.0, 0.0002]));
        }
    }
    // 控制律选择：固件解锁后默认 ALT_HOLD ⇒ rate_mode_xy=true（位置环旁路）；
    // `step()` 路径默认 false（位置环开启）。两者判据不可互相套用。
    ctrl.set_rate_mode_xy(rate_mode);
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let total = (secs / DT) as u64;
    let steady_from = (10.0 / DT) as u64;
    let (mut mr, mut mp, mut sr, mut sp_, mut fin) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, true);
    let mut drift = 0.0f64;
    let mut drift_end = 0.0f64;
    let mut pos_end = [0.0f64; 2];
    for i in 0..total {
        let st = ctrl.step(&sp);
        if !(st.att.w.is_finite() && st.att.x.is_finite() && st.att.y.is_finite() && st.att.z.is_finite()) {
            fin = false;
            break;
        }
        let (r, p) = rp_deg(st.att.w, st.att.x, st.att.y);
        mr = mr.max(r.abs());
        mp = mp.max(p.abs());
        if i >= steady_from {
            sr = sr.max(r.abs());
            sp_ = sp_.max(p.abs());
        }
        // 水平漂移（真值，NED 水平模长）——ALT_HOLD/rate 模式下位置环被旁路，
        // 这是该模式下真正决定验收成败的量（x_hover_env 现挂在此）。
        let d = ((st.pos[0].0 * st.pos[0].0 + st.pos[1].0 * st.pos[1].0) as f64).sqrt();
        drift = drift.max(d);
        drift_end = d;
        pos_end = [st.pos[0].0 as f64, st.pos[1].0 as f64];
    }
    (mr, mp, sr, sp_, fin, drift, drift_end, pos_end[0], pos_end[1])
}

#[test]
fn wind_turb_scan_all() {
    // 同口径旋钮：H 场与 M 场用**同一个**环境变量控制 `mag_alpha`，使"两端关磁锚"
    // 的对照实验两侧配置一致（M 场经 apply_env_calib 写固件的 G_MAG_ALPHA）。
    // 哨兵：-1 = 用编译期值（默认 0.05）；0 = 显式关闭磁锚定。
    let mut mag_alpha = -1.0f32;
    if let Ok(v) = std::env::var("ZZ_MAG_ALPHA") {
        if let Ok(t) = v.parse::<f32>() {
            mag_alpha = t;
            unsafe { flyctrl_core::estimator::ekf::G_MAG_ALPHA = t };
        }
    }
    println!("\n[knob] G_MAG_ALPHA = {mag_alpha}（-1 = 编译期默认 0.05；0 = 关闭磁锚定）");
    let speeds = [0.0f64, 2.5, 3.4, 5.4, 7.9]; // B0/旧基线/B3下限/B3上限/B4上限
    for &(label, rm) in &[
        ("位置环开启（step/自主设定点路径，H 场既有口径）", false),
        ("rate_mode_xy=true（固件 ALT_HOLD 口径，M 场 x_hover_env 实际在跑的）", true),
    ] {
        println!("\n=== {label} ===");
        println!("复刻 x_hover_env 风场（阵风 1.2m/s@0.12Hz + Dryden 湍流固定注入），只变 base 恒风");
        println!(
            "{:>7} | {:>10} | {:>10} | {:>12} | {:>12} | {:>8} | {:>6}",
            "base", "max|roll|", "max|pitch|", "稳态roll(>10s)", "稳态pitch", "水平漂移", "finite"
        );
        println!("{}", "-".repeat(88));
        for &v in speeds.iter() {
            let (mr, mp, sr, sp_, fin, dr, _de) = run(v, 60.0, rm);
            println!(
                "{:>7.1} | {:>10.2} | {:>10.2} | {:>12.2} | {:>12.2} | {:>7.2}m | {:>6}",
                v, mr, mp, sr, sp_, dr, if fin { "yes" } else { "NO" }
            );
        }
    }
    println!("\n（M 场 x_hover_env 的判据：max|roll| 与 max|pitch| 均 < 25°；其配置为 base=2.5）");
    // 只打印量化对比，不作通过性断言。
    assert!(true);
}

/// **水平位置积分增益 `ki_xy` 扫描**（B3 上限风、位置环口径）。
///
/// 背景：水平外环原为 **P-only** ⇒ 恒风下稳态偏移 `e = des_v/kp_xy`，实测
/// B3 风下 6.64m，与 `2.0/0.3 = 6.67m` 吻合。垂向早有 `iz` 抗稳态下沉，水平没有。
/// 本扫描给水平加积分后的效果量化 —— **默认值不在这里定**，只出数据。
#[test]
fn ki_xy_scan_at_beaufort3() {
    println!("\n水平位置积分 ki_xy 扫描（风=beaufort3 上限 5.4m/s + 阵风 + 湍流，位置环，60s）");
    println!(
        "{:>8} | {:>10} | {:>10} | {:>12} | {:>12}",
        "ki_xy", "max|roll|", "max|pitch|", "水平漂移", "末态漂移"
    );
    println!("{}", "-".repeat(64));
    for &ki in &[0.0f32, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50] {
        unsafe { flyctrl_core::controller::pid::G_KI_XY = ki };
        let (mr, mp, _sr, _sp, fin, dr, de) = run(5.4, 60.0, false);
        println!(
            "{:>8.3} | {:>10.2} | {:>10.2} | {:>11.2}m | {:>11.2}m{}",
            ki,
            mr,
            mp,
            dr,
            de,
            if fin { "" } else { "  NOT FINITE" }
        );
    }
    println!("{}", "-".repeat(64));
    unsafe { flyctrl_core::controller::pid::G_KI_XY = -1.0 }; // 复位到编译期默认
    assert!(true);
}

/// **水平积分上限 `I_XY_MAX` 扫描**（固定 ki_xy=0.10，B3 风，位置环，60s）。
///
/// 动机：ki_xy=0.10 时稳态残值 0.85m 反推 `des_v_need = 0.3×0.85 + 2.0 = 2.26`
/// **超过默认上限 2.0** ⇒ 积分撞夹子，残值被夹死为 `(需要值 − 上限)/kp_xy`。
/// 本扫描量化"抬高上限"的收益（注意上限越大，抗饱和保护越弱：饱和后回算可
/// 把积分推到很大的值，恢复期可能甩尾）。
#[test]
fn i_xy_max_scan_at_beaufort3() {
    println!("\n水平积分上限 I_XY_MAX 扫描（ki_xy=0.10 固定，B3 风，位置环，60s）");
    println!(
        "{:>9} | {:>10} | {:>10} | {:>12} | {:>12}",
        "I_XY_MAX", "max|roll|", "max|pitch|", "水平漂移", "末态漂移"
    );
    println!("{}", "-".repeat(66));
    unsafe { flyctrl_core::controller::pid::G_KI_XY = 0.10 };
    for &im in &[2.0f32, 2.5, 3.0, 3.5, 5.0] {
        unsafe { flyctrl_core::controller::pid::G_I_XY_MAX = im };
        let (mr, mp, _sr, _sp, fin, dr, de) = run(5.4, 60.0, false);
        println!(
            "{:>9.1} | {:>10.2} | {:>10.2} | {:>11.2}m | {:>11.2}m{}",
            im, mr, mp, dr, de,
            if fin { "" } else { "  NOT FINITE" }
        );
    }
    println!("{}", "-".repeat(66));
    unsafe { flyctrl_core::controller::pid::G_KI_XY = -1.0 };
    unsafe { flyctrl_core::controller::pid::G_I_XY_MAX = -1.0 };
    assert!(true);
}

/// **姿态是否"物理必需"** 的对照（回答"H 场 13.6° 是不是超调"）。
///
/// 判据不是拍的数，而是**阻力平衡**：被控对象线阻力为
/// `F = 0.5·ρ·Cd·v·|v|`（见 `plant.rs::aero_drag_body`；ρ≈1.225、机身
/// `drag_coeff=[0.18,0.18,0.10]`、`mass=1.2kg`）⇒ 平稳抗风所需倾角
/// `θ = atan(F / (m·g))`。B3 风（`beaufort3()`：北 5.4 / 东 2.16 m/s）：
///
/// | 轴 | 风速分量 | 阻力 | **必需倾角** | 实测峰值（位置环） |
/// |---|---|---|---|---|
/// | pitch(北) | 5.40 m/s | 3.215 N | **15.27°** | 13.62° |
/// | roll(东)  | 2.16 m/s | 0.514 N | **2.50°**  | **6.68°** |
///
/// **两条结论**：
/// 1. pitch 13.62° **低于**必需值 15.27° ⇒ 姿态**不是超调**，是物理必需；实测
///    偏低说明机体在下风方向有 ~0.3m/s 滑移（反推相对风速 5.09 < 5.4）。
///    ⇒ H 场"13.6°"这一项**不是问题**，无需治。
/// 2. roll 6.68° 是必需值 2.50° 的 **2.7 倍** ⇒ **东轴有未被风场解释的持续倾角**。
///    阵风是北向单轴（`gust_amp=[1.2,0,0]`）、湍流东向 σ 仅 0.1m/s，都解释不了。
///    注意仓库历史：P3-A1 提交信息提到"修正偏航力矩混控符号与 roll 期望姿态方向、
///    消除发散与**东向漂移**"—— 东轴曾有已知问题，**本项疑似残留**，待查。
#[test]
fn attitude_is_physically_justified_at_beaufort3() {
    const RHO: f64 = 1.225;
    const CD: f64 = 0.18;
    const M: f64 = 1.2;
    const G: f64 = 9.81;
    let need = |v: f64| (0.5 * RHO * CD * v * v / (M * G)).atan().to_degrees();
    let (pitch_need, roll_need) = (need(5.4), need(2.16));
    println!("\n物理必需倾角：pitch(北 5.4m/s)={pitch_need:.2}°  roll(东 2.16m/s)={roll_need:.2}°");
    // 位置环口径实测（同 wind_turb_scan_all 的 60s）
    let (mr, mp, _sr, _sp, fin, dr, de) = run(5.4, 60.0, false);
    println!(
        "实测（60s，位置环）：max|roll|={mr:.2}° max|pitch|={mp:.2}° 峰值漂移={dr:.2}m 末态={de:.2}m finite={fin}"
    );
    println!(
        "对照：pitch 实测/必需 = {:.2}x（<1 说明有下风滑移，正常）；roll 实测/必需 = {:.2}x（>1 即东轴多余倾角）",
        mp / pitch_need,
        mr / roll_need
    );
    assert!(true); // 只做量化对照，不作通过性断言
}

/// **东轴多余倾角的分离实验**：滚转到底是"风驱动的"还是"内部不对称"？
///
/// 背景：完整 B3 风（北 5.4 + 东 2.16 m/s）下实测 max|roll|=6.68°，而按阻力平衡
/// 东分量只该要 2.50°（2.67 倍）。本测例把风场拆开，直接判定来源：
/// - 仅北风（东置零）后滚转若**仍在** ⇒ 与东向风无关 ⇒ **内部不对称**（控制/混控/估计）；
/// - 仅北风后滚转**消失** ⇒ 确实是东向风驱动 ⇒ 说明阻力模型或我的换算低估了。
#[test]
fn east_roll_separation_at_beaufort3() {
    println!("\n东轴分离实验（位置环，60s）：拆开风场看滚转从哪来");
    println!(
        "{:>22} | {:>10} | {:>10} | {:>10}",
        "风场变体", "max|roll|", "max|pitch|", "末态漂移"
    );
    println!("{}", "-".repeat(62));
    for (tag, v) in [
        ("完整 B3（北5.4+东2.16）", 0u8),
        ("仅北风（东置零）", 1),
        ("仅东风（北置零）", 2),
        ("无恒风（留阵风湍流）", 3),
    ] {
        let (mr, mp, _sr, _sp, _fin, _dr, de, ..) = run_var(5.4, 60.0, false, v);
        println!("{:>22} | {:>10.2} | {:>10.2} | {:>9.2}m", tag, mr, mp, de);
    }
    println!("{}", "-".repeat(62));
    assert!(true);
}

/// **"扰动越少漂移越大"的反常追查**。
///
/// 假设：恒风给出**直流扰动**（积分可消除）⇒ 残差只剩波动；无恒风时扰动是
/// **纯交流**（阵风+湍流）⇒ 积分消不掉零均值扰动，反而可能**注入自身动态去放大它**。
/// 若假设成立，则：**关掉积分（ki_xy=0）应让"无恒风"一档的漂移变小**（而带恒风的
/// 那档应变大）—— 这是一条可否证的判据，不是解释性说辞。
#[test]
fn drift_anomaly_probe() {
    println!("\n反常追查：恒风扫描 + 积分开关对照（位置环，60s）");
    println!(
        "{:>7} | {:>11} | {:>11} | {:>11} | {:>11}",
        "base风", "ki=0峰值", "ki=0末态", "ki=0.1峰值", "ki=0.1末态"
    );
    println!("{}", "-".repeat(66));
    for &v in &[0.0f64, 0.5, 1.0, 2.0, 3.4, 5.4] {
        unsafe { flyctrl_core::controller::pid::G_KI_XY = 0.0 };
        let (_r0, _p0, _a, _b, _f0, d0, e0, ..) = run_var(v, 60.0, false, 0);
        unsafe { flyctrl_core::controller::pid::G_KI_XY = 0.10 };
        let (_r1, _p1, _a2, _b2, _f1, d1, e1, ..) = run_var(v, 60.0, false, 0);
        println!(
            "{:>7.1} | {:>10.2}m | {:>10.2}m | {:>10.2}m | {:>10.2}m",
            v, d0, e0, d1, e1
        );
    }
    println!("{}", "-".repeat(66));
    unsafe { flyctrl_core::controller::pid::G_KI_XY = -1.0 };
    assert!(true);
}

/// **水平速度低通（`vel_lpf_h_tau` A/B）**：能否界住"零均值扰动下的无界游走"？
///
/// 诊断依据：水平位置/速度此前**无任何低通**直驱倾角指令（垂向一直有）。
/// 签名：无恒风时漂移**峰值≡末态**（60s 仍在增长 = 无界游走）；有恒风时峰值在
/// 中途（有界）。故本扫描同时看两档，并用"峰值/末态"比判断是否仍有增长趋势。
/// 另记姿态（低通的代价不只是相位滞后，还可能改变摆幅）。
#[test]
fn vel_lpf_h_tau_scan() {
    println!("\n水平速度低通 tau 扫描（位置环，60s；峰值/末态比 -> 1 表示仍在增长）");
    println!(
        "{:>7} | {:>9} {:>9} {:>6} | {:>9} {:>9} {:>6} | {:>9} {:>9}",
        "tau", "无风峰值", "无风末态", "比", "B3峰值", "B3末态", "比", "无风|roll|", "B3|roll|"
    );
    println!("{}", "-".repeat(92));
    for &tau in &[0.0f32, 0.02, 0.05, 0.10, 0.20, 0.40] {
        unsafe { flyctrl_core::controller::pid::G_VEL_LPF_H_TAU = tau };
        let (r0, _p0, _a, _b, _f0, d0, e0, ..) = run_var(0.0, 60.0, false, 0);
        let (r1, _p1, _a2, _b2, _f1, d1, e1) = run_var(5.4, 60.0, false, 0);
        println!(
            "{:>7.2} | {:>8.2}m {:>8.2}m {:>5.2} | {:>8.2}m {:>8.2}m {:>5.2} | {:>8.2} {:>8.2}",
            tau, d0, e0, d0 / e0.max(1e-6), d1, e1, d1 / e1.max(1e-6), r0, r1
        );
    }
    println!("{}", "-".repeat(92));
    unsafe { flyctrl_core::controller::pid::G_VEL_LPF_H_TAU = -1.0 };
    assert!(true);
}

/// **IMU 零偏分离**：无风档的持续漂移（0.16 m/s 且仍在增长）来自哪个零偏？
///
/// 判据（"偏置"的签名）：漂移**方向恒定 + 速率恒定**。分别注入：
/// 0=陀螺+加计（既有） 1=都不注入 2=只陀螺 3=只加计。
/// 读数：末位置 (n,e) —— 方向即零偏轴；再除以 60s 得平均速度。
#[test]
fn imu_bias_separation_no_wind() {
    println!("\nIMU 零偏分离（无恒风、位置环、60s）：看漂移方向与速率是否恒定");
    println!(
        "{:>18} | {:>9} | {:>9} | {:>10} | {:>12}",
        "注入", "末pos_n", "末pos_e", "漂移模长", "平均速度"
    );
    println!("{}", "-".repeat(72));
    for (tag, b) in [
        ("陀螺+加计(既有)", 0u8),
        ("都不注入", 1),
        ("只陀螺漂移", 2),
        ("只加计漂移", 3),
    ] {
        let (_r, _p, _a, _c, _f, _d, de, pn, pe) = run_var_bias(0.0, 60.0, false, 0, b);
        let mag = (pn * pn + pe * pe).sqrt();
        println!(
            "{:>18} | {:>9.2} | {:>9.2} | {:>9.2}m | {:>10.3} m/s",
            tag, pn, pe, mag, mag / 60.0
        );
    }
    println!("{}", "-".repeat(72));
    assert!(true);
}

/// **陀螺零偏在线估计 A/B**：能否消掉无风档的持续漂移（9.36m）？
///
/// 依据：分离实验确认该漂移**完全**由陀螺零偏贡献（只陀螺 = 9.36m，只加计 = 1.52m）。
/// 而 EKF 的 `x[6..8]` 一直是状态向量的一部分、传播时也被扣除，却**从未被观测
/// 更新** ⇒ 恒为 0。本测例把它接上，扫收敛速率看效果与代价。
#[test]
fn gyro_bias_estimation_scan() {
    println!("\n陀螺零偏在线估计扫描（无恒风 + B3，位置环 60s；同时看姿态是否被带坏）");
    println!(
        "{:>7} | {:>9} {:>9} {:>6} | {:>9} {:>9} | {:>9} {:>9}",
        "k", "无风末态", "无风峰值", "比", "B3末态", "B3峰值", "无风|roll|", "B3|pitch|"
    );
    println!("{}", "-".repeat(84));
    for &k in &[0.0f32, 0.02, 0.05, 0.1, 0.2, 0.5] {
        unsafe { flyctrl_core::estimator::ekf::G_GYRO_BIAS_K = k };
        let (r0, p0, _a, _b, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
        let (r1, p1, _a2, _b2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
        println!(
            "{:>7.2} | {:>8.2}m {:>8.2}m {:>5.2} | {:>8.2}m {:>8.2}m | {:>8.2} {:>8.2}",
            k, e0, d0, d0 / e0.max(1e-6), e1, d1, r0, p1
        );
    }
    println!("{}", "-".repeat(84));
    unsafe { flyctrl_core::estimator::ekf::G_GYRO_BIAS_K = 0.0 };
    assert!(true);
}

/// **水平加速度门控阈值扫描**：能否同时拿到"无风不漂"与"有风不劣化"？
///
/// 背景：att_alpha 统一到 0.02 后，无风档 9.36m -> 3.12m（受益），但 B3 档
/// 3.39m -> 19.22m（劣化，且连带 4 项测试失败）。原因是锚定的前提"比力=重力"
/// 在水平加速（阵风/湍流/机动）时不成立，而**原有三个门控都看不出来**
/// （稳态抗风时幅值仍=g、方向仍竖直、|ω| 仍小）。
/// 新增第四个门控：`a_h = |R·a| 水平分量`（世界系非重力水平加速度）。
/// 本扫描找"无风也好、有风也好"的阈值。阈值 <=0 表示门控关闭（= 修复前行为）。
#[test]
fn att_acc_gate_scan() {
    println!("\n水平加速度门控阈值扫描（位置环 60s；阈值<=0 = 门控关闭，即修复前）");
    println!(
        "{:>8} | {:>9} {:>9} | {:>9} {:>9} | {:>9} {:>9}",
        "阈值", "无风末态", "无风峰值", "B3末态", "B3峰值", "无风|roll|", "B3|pitch|"
    );
    println!("{}", "-".repeat(78));
    unsafe { flyctrl_core::estimator::ekf::G_GYRO_BIAS_K = 0.0 };
    for &g in &[0.0f32, 5.0, 3.0, 2.0, 1.5, 1.0, 0.5] {
        unsafe { flyctrl_core::estimator::ekf::G_ATT_ACC_GATE = g };
        let (r0, p0, _a, _b, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
        let (r1, p1, _a2, _b2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
        println!(
            "{:>8.1} | {:>8.2}m {:>8.2}m | {:>8.2}m {:>8.2}m | {:>8.2} {:>8.2}",
            g, e0, d0, e1, d1, r0, p1
        );
    }
    println!("{}", "-".repeat(78));
    unsafe { flyctrl_core::estimator::ekf::G_ATT_ACC_GATE = -1.0 };
    assert!(true);
}

/// **交流能量门控扫描**：用 a_h 的**交流幅度**（而非瞬时值/均值）调锚定增益。
///
/// 依据：扰动是零均值交流 ⇒ 瞬时值抖（前一轮实测 2.0 阈值处无风档 88.73m）、
/// 均值≈0 抓不住。故改用 `a_h` 的 EMA 绝对偏差（一阶包络）作门控信号。
/// 目标：无风档 ≤3m（拿陀螺零偏抑制收益）**且** B3 档回到 ≤3.4m（修复前水平）。
#[test]
fn att_acc_ac_gate_scan() {
    println!("\n交流能量门控扫描（位置环 60s；阈值<=0 = 关闭；瞬时门控置 0 隔离本项）");
    println!(
        "{:>8} | {:>9} {:>9} | {:>9} {:>9} | {:>9} {:>9}",
        "阈值", "无风末态", "无风峰值", "B3末态", "B3峰值", "无风|roll|", "B3|pitch|"
    );
    println!("{}", "-".repeat(78));
    unsafe { flyctrl_core::estimator::ekf::G_GYRO_BIAS_K = 0.0 };
    unsafe { flyctrl_core::estimator::ekf::G_ATT_ACC_GATE = 0.0 }; // 瞬时门控关，隔离
    for &g in &[0.0f32, 0.1, 0.2, 0.4, 0.8, 1.5, 3.0] {
        unsafe { flyctrl_core::estimator::ekf::G_ATT_ACC_AC = g };
        let (r0, p0, _a, _b, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
        let (r1, p1, _a2, _b2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
        println!(
            "{:>8.1} | {:>8.2}m {:>8.2}m | {:>8.2}m {:>8.2}m | {:>8.2} {:>8.2}",
            g, e0, d0, e1, d1, r0, p1
        );
    }
    println!("{}", "-".repeat(78));
    unsafe { flyctrl_core::estimator::ekf::G_ATT_ACC_AC = -1.0 };
    assert!(true);
}

/// **阶段 2 风观测器验证**：估计能否逼近注入的恒风真值？
///
/// 观测器是准稳态代数解：`|v_rel| = sqrt(g·tanθ/k)`，方向 = 推力水平分量方向，
/// `v_wind = v_ground − v_rel`。注入 B3 风（北 5.4 / 东 2.16 m/s，另加阵风湍流）。
#[test]
fn wind_observer_check() {
    println!("\n阶段 2 风观测器：估计 vs 注入真值（B3 北5.4/东2.16 m/s）");
    // 用 run_var_bias 跑一遍并打印风估计需要控制器内部读数 —— 此处改用间接判据：
    // 若观测器工作，drag_k>0 与 =0 应有可测差异（否则说明没接上/没生效）。
    println!("{:>26} | {:>9} {:>9} | {:>9} {:>9}", "配置", "无风末态", "B3末态", "无风|roll|", "B3|pitch|");
    println!("{}", "-".repeat(70));
    for (tag, k_on) in [("drag_k 关闭(默认)", false), ("drag_k 开启", true)] {
        if !k_on {
            // 关闭：SIL 构造后再清零（set_drag_k(0) 语义 = 不启用）
            std::env::set_var("ZZ_DRAGK_OFF", "1");
        } else {
            std::env::remove_var("ZZ_DRAGK_OFF");
        }
        let (r0, p0, _a, _b, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
        let (r1, p1, _a2, _b2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
        println!("{:>26} | {:>8.2}m {:>8.2}m | {:>8.2} {:>8.2}", tag, e0, d0, r0, p1);
    }
    println!("{}", "-".repeat(70));
    assert!(true);
}

/// **阶段 2 风观测器收敛性验证**（先证明零件可用，再接负载）。
///
/// 注入 B3 风：北 5.4 / 东 2.16 m/s（`WindConfig::beaufort3().base` 的 NED→UP 逆映射）。
/// 观测器是准稳态代数解，**预期**：量级接近、方向正确；湍流下会有偏差（其局限）。
/// 需要 `ZZ_DRAG_K=1`（否则 drag_k=0、估计恒 0）。
#[test]
fn wind_observer_convergence() {
    let enabled = std::env::var("ZZ_DRAG_K").is_ok();
    println!("\n阶段 2 风观测器收敛性（ZZ_DRAG_K={}）", if enabled { "1" } else { "未设→估计应恒 0" });
    // 真值：WindConfig::beaufort3().base 是 UP 系 [n, -d, -e] ⇒ 逆映射回 NED
    let w = fly_sim_core::wind::WindConfig::beaufort3();
    println!("  注入真值（NED）: 北 {:.2} / 东 {:.2} m/s", w.base[0], -w.base[2]);
    for &(tag, sp) in &[("无恒风（只阵风湍流）", 0.0f64), ("B3 北5.4/东2.16", 5.4)] {
        // 直接构造控制器跑一遍并读 wind_estimate
        let cfg = fly_simulater::airframe::load_airframe(None).expect("airframe");
        let mut wc = fly_sim_core::wind::WindConfig::beaufort3();
        let dir = if wc.base[0].abs() > 1e-9 { wc.base[2] / wc.base[0] } else { 0.0 };
        wc.base = [sp, 0.0, dir * sp];
        let mut ctrl = fly_sim_core::controller::FlyController::new(
            fly_sim_core::physics::PhySdkWorld::create_empty(),
            &cfg,
            0.004,
            Some(fly_sim_core::wind::WindField::new(wc)),
            fly_sim_core::sensor::SensorConfig::realistic(),
            fly_sim_core::controller::ControllerKind::Pid,
            Some(fly_sim_core::physics::ContactModel::default()),
            Vec::new(),
        );
        let spn = fly_sim_core::controller::hover_setpoint(0.0, 0.0, -5.0);
        for _ in 0..(60.0 / 0.004) as u64 {
            ctrl.step(&spn);
        }
        let e = ctrl.wind_estimate();
        println!("  {tag:22} -> 估计: 北 {:.2} / 东 {:.2} m/s（模长 {:.2}）", e[0], e[1], (e[0]*e[0]+e[1]*e[1]).sqrt());
    }
    assert!(true);
}

/// **路线 2.2e 闭环验证**：用磁场做全姿态修正，能否同时拿到"无风≤3m 且 B3≤3.4m"？
///
/// 依据：比力锚定被水平加速度污染（A 项根源），而磁场与加速度无耦合。
/// 零件已验证（硬铁补偿前 39.54°/后 0°）。本测例量闭环效果与代价。
/// 强度旋钮 `G_MAG3D_ALPHA`：0 = 关闭（既有行为）。
#[test]
fn mag3d_full_attitude_scan() {
    println!("\n路线 2.2e 闭环扫描（磁全姿态修正，位置环 60s）");
    println!("{:>8} | {:>9} {:>9} | {:>9} {:>9} | {:>9} {:>9}", "alpha", "无风末态", "无风峰值", "B3末态", "B3峰值", "无风|roll|", "B3|pitch|");
    println!("{}", "-".repeat(78));
    for &a in &[0.0f32, 0.05, 0.1, 0.2, 0.5, 1.0] {
        unsafe { flyctrl_core::estimator::ekf::G_MAG3D_ALPHA = a };
        let (r0, p0, _x, _y, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
        let (r1, p1, _x2, _y2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
        println!("{:>8.2} | {:>8.2}m {:>8.2}m | {:>8.2}m {:>8.2}m | {:>8.2} {:>8.2}", a, e0, d0, e1, d1, r0, p1);
    }
    println!("{}", "-".repeat(78));
    unsafe { flyctrl_core::estimator::ekf::G_MAG3D_ALPHA = 0.0 };
    assert!(true);
}

/// **二维组合扫描**：重力锚定 `att_alpha` × 磁全姿态 `mag3d_alpha`。
///
/// 目标：**无风 ≤3m 且 B3 ≤3.4m 同时成立**（= 阶段 0 的目标）。
/// 假设：2.2e 提供加速度免疫的姿态参考后，重力锚定应下调/取消 ——
/// 若成立，"0.0 vs 0.02"的两难从根上消失。
#[test]
fn att_alpha_vs_mag3d_scan() {
    println!("\n二维扫描：G_ATT_ALPHA × G_MAG3D_ALPHA（位置环 60s）");
    println!("目标：无风末态 ≤3m 且 B3末态 ≤3.4m（对照基线：改动前 3.12 / 19.22）");
    println!("{:>10} | {:>9} | {:>9} {:>9} | {:>9} {:>9} | {:>6}", "att_a", "mag3d", "无风末态", "无风峰值", "B3末态", "B3峰值", "达标");
    println!("{}", "-".repeat(84));
    let mut any_ok = false;
    for &aa in &[0.0f32, 0.005, 0.02] {
        for &m3 in &[0.0f32, 0.1, 0.3, 1.0] {
            unsafe { flyctrl_core::estimator::ekf::G_ATT_ALPHA = aa };
            unsafe { flyctrl_core::estimator::ekf::G_MAG3D_ALPHA = m3 };
            let (_r0, _p0, _x, _y, _f0, d0, e0, ..) = run_var_bias(0.0, 60.0, false, 0, 0);
            let (_r1, _p1, _x2, _y2, _f1, d1, e1, ..) = run_var_bias(5.4, 60.0, false, 0, 0);
            let ok = e0 <= 3.0 && e1 <= 3.4;
            if ok { any_ok = true; }
            println!("{:>10.3} | {:>9.2} | {:>8.2}m {:>8.2}m | {:>8.2}m {:>8.2}m | {:>6}", aa, m3, e0, d0, e1, d1, if ok { "✅" } else { "" });
        }
    }
    println!("{}", "-".repeat(84));
    println!("是否存在达标组合：{}", if any_ok { "是 ✅" } else { "否 ❌" });
    unsafe { flyctrl_core::estimator::ekf::G_ATT_ALPHA = -1.0 };
    unsafe { flyctrl_core::estimator::ekf::G_MAG3D_ALPHA = 0.0 };
    assert!(true);
}
