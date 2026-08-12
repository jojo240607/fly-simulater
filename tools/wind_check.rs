// 独立验证 fly-sim-core/src/wind.rs 的风场（Dryden 简化）统计特性，绕过 phy-rigid 编译阻塞。
// 编译：rustc --edition 2021 -O wind_check.rs -o wind_check.exe && wind_check.exe
// 这是对 wind.rs 逻辑的复刻 + 统计断言。

use std::num::Wrapping;

type WindVec = [f64; 3];

struct Lcg {
    state: Wrapping<u64>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self { state: Wrapping(seed.wrapping_add(0x9E3779B97F4A7C15)) }
    }
    fn next_f64(&mut self) -> f64 {
        self.state = self.state * Wrapping(6364136223846793005) + Wrapping(1442695040888963407);
        let mut x = self.state;
        x ^= x >> (64 - 31);
        x = x * Wrapping(0xDA942042E4DD58B5);
        x ^= x >> 32;
        (x.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

#[derive(Clone)]
struct WindConfig {
    base: WindVec,
    gust_amp: WindVec,
    gust_freq: f64,
    turb_sigma: WindVec,
    turb_tau: f64,
    seed: u64,
}
impl WindConfig {
    fn default() -> Self {
        Self { base: [0.0; 3], gust_amp: [0.0; 3], gust_freq: 0.1, turb_sigma: [0.0; 3], turb_tau: 0.5, seed: 0x1234_5678 }
    }
}

struct WindField {
    cfg: WindConfig,
    rng: Lcg,
    turb_state: WindVec,
    time: f64,
}
impl WindField {
    fn new(cfg: WindConfig) -> Self {
        Self { cfg: cfg.clone(), rng: Lcg::new(cfg.seed), turb_state: [0.0; 3], time: 0.0 }
    }
    fn sample(&mut self, dt: f64) -> WindVec {
        self.time += dt;
        let t = self.time;
        let mut gust = [0.0f64; 3];
        for i in 0..3 {
            let phase = (i as f64) * 1.7;
            let f = self.cfg.gust_freq;
            gust[i] = self.cfg.gust_amp[i]
                * (f64::sin(2.0 * std::f64::consts::PI * f * t + phase)
                    + 0.5 * f64::sin(2.0 * std::f64::consts::PI * 2.0 * f * t + 2.0 * phase)
                    + 0.25 * f64::sin(2.0 * std::f64::consts::PI * 3.0 * f * t + 3.0 * phase));
        }
        let tau_x = self.cfg.turb_tau;
        let tau_yz = self.cfg.turb_tau * 0.5;
        let mut turb = [0.0f64; 3];
        for i in 0..3 {
            let tau = if i == 0 { tau_x } else { tau_yz };
            let alpha = (dt / tau).min(1.0);
            let gain = ((2.0 - alpha) / alpha).sqrt();
            let w = self.cfg.turb_sigma[i] * gain * self.rng.next_gaussian();
            self.turb_state[i] += (w - self.turb_state[i]) * alpha;
            turb[i] = self.turb_state[i];
        }
        [
            self.cfg.base[0] + gust[0] + turb[0],
            self.cfg.base[1] + gust[1] + turb[1],
            self.cfg.base[2] + gust[2] + turb[2],
        ]
    }
}

fn main() {
    // 1. 确定性：同种子 → 同序列
    let mut a = WindField::new(WindConfig { turb_sigma: [1.0, 1.0, 1.0], ..WindConfig::default() });
    let mut b = WindField::new(WindConfig { turb_sigma: [1.0, 1.0, 1.0], ..WindConfig::default() });
    let mut det = true;
    for _ in 0..2000 {
        let wa = a.sample(0.004);
        let wb = b.sample(0.004);
        if (wa[0] - wb[0]).abs() > 1e-12 { det = false; }
    }
    check("deterministic_same_seed", det, "same seed should give same sequence");

    // 2. 纯湍流（无 base/gust）：均值≈0，标准差≈turb_sigma
    let mut w = WindField::new(WindConfig { turb_sigma: [2.0, 1.0, 0.5], ..WindConfig::default() });
    // 先预热（让低通收敛）
    for _ in 0..2000 { w.sample(0.004); }
    let n = 100_000;
    let mut sum = [0.0f64; 3];
    let mut sumsq = [0.0f64; 3];
    for _ in 0..n {
        let wv = w.sample(0.004);
        for i in 0..3 { sum[i] += wv[i]; sumsq[i] += wv[i] * wv[i]; }
    }
    for i in 0..3 {
        let mean = sum[i] / n as f64;
        let var = (sumsq[i] / n as f64 - mean * mean).max(0.0);
        let std = var.sqrt();
        let sigma = [2.0, 1.0, 0.5][i];
        // 高相关有色噪声（tau≈1s）有限样本均值估计有统计波动，阈值用 0.2σ。
        // 增益补偿后输出标准差应≈sigma。
        check(&format!("turb_mean_zero_{}", i), mean.abs() < 0.2 * sigma,
              &format!("mean={:.3}, sigma={}", mean, sigma));
        check(&format!("turb_std_in_range_{}", i), (0.6 * sigma) < std && std < (1.4 * sigma),
              &format!("std={:.3}, expect ~{}", std, sigma));
    }

    // 3. 阵风频率：gust_amp=1 时，整周期平均≈0，振幅≈1（1+0.5+0.25=1.75 峰值叠加）
    let mut w = WindField::new(WindConfig { gust_amp: [1.0, 0.0, 0.0], gust_freq: 0.1, ..WindConfig::default() });
    let period = 1.0 / 0.1;
    let steps = (period / 0.004) as usize; // 一个周期
    let mut sum = 0.0;
    let mut maxv = 0.0f64;
    for _ in 0..steps {
        let v = w.sample(0.004)[0];
        sum += v;
        maxv = maxv.max(v.abs());
    }
    let mean = sum / steps as f64;
    check("gust_mean_over_period_zero", mean.abs() < 0.01, "gust mean");
    check("gust_amplitude_present", maxv > 0.8, "gust amplitude");

    // 4. 各轴湍流相关时间差异：纵向(tau_x)比横向(tau_yz=0.5tau)更平滑
    //    → 纵向相邻采样自相关更高。用相邻样本差平方衡量。
    let mut w = WindField::new(WindConfig { turb_sigma: [1.0, 1.0, 1.0], turb_tau: 1.0, ..WindConfig::default() });
    for _ in 0..2000 { w.sample(0.004); }
    let mut diff_x = 0.0; let mut diff_y = 0.0;
    let mut prev = w.sample(0.004);
    for _ in 0..2000 {
        let v = w.sample(0.004);
        diff_x += (v[0] - prev[0]).powi(2);
        diff_y += (v[1] - prev[1]).powi(2);
        prev = v;
    }
    check("longitudinal_smoother_than_lateral", diff_x < diff_y,
          &format!("diff_x={:.4} should < diff_y={:.4}", diff_x, diff_y));

    println!("ALL OK");
}

fn check(name: &str, cond: bool, msg: &str) {
    if cond {
        println!("PASS: {}", name);
    } else {
        println!("FAIL: {} -- {}", name, msg);
        std::process::exit(1);
    }
}
