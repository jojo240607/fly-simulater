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
    ctrl.inject_sensor_fault(SensorFault::GyroDrift([0.0001, -0.00005, 0.0001]));
    ctrl.inject_sensor_fault(SensorFault::AccelDrift([0.0002, 0.0, 0.0002]));
    // 控制律选择：固件解锁后默认 ALT_HOLD ⇒ rate_mode_xy=true（位置环旁路）；
    // `step()` 路径默认 false（位置环开启）。两者判据不可互相套用。
    ctrl.set_rate_mode_xy(rate_mode);
    let sp = hover_setpoint(0.0, 0.0, -5.0);
    let total = (secs / DT) as u64;
    let steady_from = (10.0 / DT) as u64;
    let (mut mr, mut mp, mut sr, mut sp_, mut fin) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, true);
    let mut drift = 0.0f64;
    let mut drift_end = 0.0f64;
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
    }
    (mr, mp, sr, sp_, fin, drift, drift_end)
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
        let (mr, mp, _sr, _sp, _fin, _dr, de) = run_var(5.4, 60.0, false, v);
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
        let (_r0, _p0, _a, _b, _f0, d0, e0) = run_var(v, 60.0, false, 0);
        unsafe { flyctrl_core::controller::pid::G_KI_XY = 0.10 };
        let (_r1, _p1, _a2, _b2, _f1, d1, e1) = run_var(v, 60.0, false, 0);
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
        let (r0, _p0, _a, _b, _f0, d0, e0) = run_var(0.0, 60.0, false, 0);
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
