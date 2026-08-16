//! P2-B 空间相关风场 + 阵风突风注入验证。
//!
//! 验证项：
//! 1. 默认配置（spatial_scale=0, shear=0, burst=0）空间均匀、无突风 —— 零回归。
//! 2. 空间相关风：不同位置处基础风被确定性调制，机身不同部位风速不同。
//! 3. 风切变廓线：高 z 处基础风幅度 > 低 z 处（幂律 (z/z_ref)^α）。
//! 4. 确定性阵风突风：在 t0±hw 窗口内叠加 1-cos 包络突风，窗口外为 0，确定性可复现。

use fly_sim_core::wind::{WindConfig, WindField};

/// 在固定时间 t 处构造风场，对两个不同位置采样（绕过 dt 推进，直接验证空间场）。
fn sample_two_pos(cfg: &WindConfig, t: f64, pa: [f64; 3], pb: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    // 用 dt 逐步推进到目标时间 t，并在末尾对两个点采样。
    // 由于 sample_at 同时推进 self.time，需分别推进两个独立风场到同一时刻后采样各自位置。
    let mut wa = WindField::new(cfg.clone());
    let mut wb = WindField::new(cfg.clone());
    let n = (t / 0.01).round() as usize;
    for _ in 0..n {
        wa.sample_at(0.01, &pa);
        wb.sample_at(0.01, &pb);
    }
    (wa.sample_at(0.01, &pa), wb.sample_at(0.01, &pb))
}

#[test]
fn default_config_is_spatially_uniform_and_no_burst() {
    // 默认 WindConfig：所有空间/切变/突风字段为 0/默认 → 任意位置风速相同、无突风。
    let cfg = WindConfig::default();
    let a = [0.0, 5.0, 0.0];
    let b = [10.0, 50.0, -3.0];
    // 推进到 t=2s 后在两点采样，应完全一致。
    let (wa, wb) = sample_two_pos(&cfg, 2.0, a, b);
    for i in 0..3 {
        assert!(
            (wa[i] - wb[i]).abs() < 1e-12,
            "默认配置空间应均匀: pos_a={:?} pos_b={:?} diff[{}]={}",
            a, b, i, (wa[i] - wb[i]).abs()
        );
    }
}

#[test]
fn spatial_wind_varies_across_position() {
    // 开启 spatial_scale=5m，基础风非零 → 不同 x 位置处风速应不同。
    let mut cfg = WindConfig::default();
    cfg.base = [2.0, 0.0, 0.0];
    cfg.spatial_scale = 5.0;
    cfg.seed = 1;

    let (wa, wb) = sample_two_pos(&cfg, 1.0, [0.0, 0.0, 0.0], [1.25, 0.0, 0.0]);
    // 空间调制 sin(k*x) 在 x=0 与 x=λ/2(=2.5m) 处符号相反 → 风速差异显著。
    let diff = (wa[0] - wb[0]).abs();
    assert!(diff > 0.1, "空间相关风应使不同位置风速不同: diff={:.4}", diff);
}

#[test]
fn wind_shear_increases_with_height() {
    // 风切变：base 非零、shear_exponent=0.15、ref=10m → 高 z 处幅度 > 低 z 处。
    let mut cfg = WindConfig::default();
    cfg.base = [3.0, 0.0, 0.0];
    cfg.shear_exponent = 0.15;
    cfg.shear_ref_height = 10.0;
    cfg.spatial_scale = 0.0; // 隔离切变效应
    cfg.seed = 7;

    let (lo, hi) = sample_two_pos(&cfg, 1.0, [0.0, 0.0, 2.0], [0.0, 0.0, 20.0]);
    let lo_mag = (lo[0].abs() + lo[1].abs() + lo[2].abs());
    let hi_mag = (hi[0].abs() + hi[1].abs() + hi[2].abs());
    assert!(
        hi_mag > lo_mag * 1.05,
        "风切变应使高 z 处风速更大: lo={:.4} hi={:.4}",
        lo_mag, hi_mag
    );
}

#[test]
fn gust_burst_is_deterministic_and_windowed() {
    // 确定性阵风突风：t0=1.0, hw=0.5 → 窗口 [0.5,1.5] 内突风非零，窗口外为 0。
    let mut cfg = WindConfig::default();
    cfg.gust_burst_amp = [4.0, 0.0, 0.0];
    cfg.gust_burst_t0 = 1.0;
    cfg.gust_burst_hw = 0.5;
    cfg.base = [0.0, 0.0, 0.0]; // 隔离基础风
    cfg.turb_sigma = [0.0, 0.0, 0.0];
    cfg.gust_amp = [0.0, 0.0, 0.0];
    cfg.seed = 3;

    let mut w = WindField::new(cfg.clone());
    let mut in_window = 0.0;
    let mut out_window = 0.0;
    // 推进到 t=2s（覆盖窗口外 + 窗口内），每步 dt=0.01 → 200 步。
    // 窗口 [0.5,1.5] 对应步 50..150；窗口外从步 160 起（t>1.6）。
    for step in 0..200 {
        let wind = w.sample_at(0.01, &[0.0, 0.0, 0.0]);
        if step > 50 && step < 150 {
            in_window += wind[0].abs();
        } else if step > 160 {
            out_window = wind[0].abs().max(out_window);
        }
    }
    assert!(in_window > 1.0, "窗口内突风应显著: in_window={:.4}", in_window);
    assert!(out_window < 1e-9, "窗口外突风应为 0: out_window={:.2e}", out_window);

    // 确定性：相同配置两次推进结果一致。
    let mut w1 = WindField::new(cfg.clone());
    let mut w2 = WindField::new(cfg.clone());
    let mut s1 = [0.0f64; 3];
    let mut s2 = [0.0f64; 3];
    for _ in 0..100 {
        s1 = w1.sample_at(0.01, &[0.0, 0.0, 0.0]);
        s2 = w2.sample_at(0.01, &[0.0, 0.0, 0.0]);
    }
    for i in 0..3 {
        assert!(
            (s1[i] - s2[i]).abs() < 1e-12,
            "突风必须确定性: s1[{}]={:.6} s2[{}]={:.6}",
            i, s1[i], i, s2[i]
        );
    }
}

#[test]
fn burst_peak_approaches_amplitude_at_center() {
    // 1-cos 包络在窗口中心 (t=t0) 达到峰值 = 幅值；推进到 t0 处检查。
    let mut cfg = WindConfig::default();
    cfg.gust_burst_amp = [5.0, 2.0, 0.0];
    cfg.gust_burst_t0 = 1.0;
    cfg.gust_burst_hw = 0.5;
    cfg.base = [0.0; 3];
    cfg.turb_sigma = [0.0; 3];
    cfg.gust_amp = [0.0; 3];
    cfg.seed = 11;

    let mut w = WindField::new(cfg);
    let mut peak = [0.0f64; 3];
    // 推进到 t≈1.0（窗口中心），峰值应出现。
    for _ in 0..101 {
        let wind = w.sample_at(0.01, &[0.0, 0.0, 0.0]);
        for i in 0..3 {
            peak[i] = peak[i].max(wind[i].abs());
        }
    }
    assert!(peak[0] > 4.5, "突风峰值应接近幅值5: peak[0]={:.4}", peak[0]);
    assert!(peak[1] > 1.8, "突风峰值应接近幅值2: peak[1]={:.4}", peak[1]);
}

#[test]
fn thermal_updraft_center_strong_edge_weak() {
    // 热气流：中心处上升速度≈strength，远处（>3σ）≈0。
    let mut cfg = WindConfig::default();
    cfg.thermal_strength = 3.0;
    cfg.thermal_radius = 5.0;
    cfg.thermal_height = 50.0;
    cfg.thermal_pos0 = [0.0, 0.0];
    cfg.base = [0.0; 3];
    cfg.turb_sigma = [0.0; 3];
    cfg.gust_amp = [0.0; 3];
    cfg.seed = 5;

    let mut w = WindField::new(cfg.clone());
    // 推进到稳定时刻（dt 累积不改变静态高斯场）。
    let center = w.sample_at(0.01, &[0.0, 0.0, 0.0]);
    let edge = w.sample_at(0.01, &[20.0, 0.0, 0.0]); // 水平距 20m >> 3σ=15m
    // 中心垂直分量 wind[2] 应接近 strength（高斯峰值）。
    assert!(center[2] > 2.5, "热气流中心上升应接近 strength: w2={:.4}", center[2]);
    assert!(edge[2].abs() < 1e-9, "热气流边缘(>3σ)应≈0: w2={:.2e}", edge[2].abs());
}

#[test]
fn thermal_updraft_capped_above_height() {
    // 热气流高度封顶：z > thermal_height 处上升速度应为 0（即使水平仍在中心）。
    // 注：wind.rs 约定 pos[2] 同时作为"高度"与水平第二轴 cz；故用较大 radius，
    // 使封顶高度下水平中心处高斯仍强，封顶逻辑独立可验证。
    let mut cfg = WindConfig::default();
    cfg.thermal_strength = 3.0;
    cfg.thermal_radius = 20.0; // 较大半径：z=10 处高斯仍强
    cfg.thermal_height = 20.0;
    cfg.thermal_pos0 = [0.0, 0.0];
    cfg.base = [0.0; 3];
    cfg.turb_sigma = [0.0; 3];
    cfg.gust_amp = [0.0; 3];
    cfg.seed = 9;

    let mut w = WindField::new(cfg.clone());
    let below = w.sample_at(0.01, &[0.0, 0.0, 10.0]);
    let above = w.sample_at(0.01, &[0.0, 0.0, 30.0]);
    assert!(below[2] > 2.5, "封顶高度下应有上升气流: w2={:.4}", below[2]);
    assert!(above[2].abs() < 1e-9, "高于封顶高度应无上升气流: w2={:.2e}", above[2].abs());
}

#[test]
fn thermal_drift_moves_center_deterministically() {
    // 热气流漂移：drift=[1,0] m/s → t=10s 中心移至 x=10m；原中心(0,0)处上升流消失。
    let mut cfg = WindConfig::default();
    cfg.thermal_strength = 3.0;
    cfg.thermal_radius = 5.0;
    cfg.thermal_height = 50.0;
    cfg.thermal_pos0 = [0.0, 0.0];
    cfg.thermal_drift = [1.0, 0.0];
    cfg.base = [0.0; 3];
    cfg.turb_sigma = [0.0; 3];
    cfg.gust_amp = [0.0; 3];
    cfg.seed = 2;

    let mut w = WindField::new(cfg.clone());
    // 推进 10s（1000 步 × 0.01）。
    let mut at_origin_end = 0.0;
    for _ in 0..1000 {
        let wind = w.sample_at(0.01, &[0.0, 0.0, 0.0]);
        at_origin_end = wind[2];
    }
    // 漂移 10s 后中心在 x=10m，原点处应基本无上升气流（距中心 10m > 3σ=15m? 否，10m<15m）。
    // 用更极端：检查 t=0 时原点有流、t=10s 后原点流显著减弱（高斯衰减 exp(-100/50)=0.135）。
    assert!(
        at_origin_end < 1.5,
        "漂移后原点上升流应显著减弱(exp(-100/2*25)): w2={:.4}",
        at_origin_end
    );
    // 确定性：再构造一个独立场推进到 10s，在 x=10m 处应恢复强上升流。
    let mut w2 = WindField::new(cfg.clone());
    let mut at_new_center = 0.0;
    for _ in 0..1000 {
        let wind = w2.sample_at(0.01, &[10.0, 0.0, 0.0]);
        at_new_center = wind[2];
    }
    assert!(
        at_new_center > 2.5,
        "漂移后新中心(x=10m)应恢复强上升流: w2={:.4}",
        at_new_center
    );
}
