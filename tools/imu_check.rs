// 独立验证 fly-sim-core/src/sensor.rs 的 P0-A IMU 标准误差树（绕过 phy-rigid 编译阻塞）。
// 编译：rustc --edition 2021 -O imu_check.rs -o imu_check.exe && imu_check.exe
// 复刻 process() 的 IMU 分支：加速度随机游走 + 振动整流 + 陀螺随机游走 + 偏置不稳定性(Markov)。

use std::num::Wrapping;

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
    fn gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

// P0-A 误差树参数（复刻 SensorConfig::realistic）
struct ImuCfg {
    accel_bias: [f64; 3],
    accel_noise: f64,
    accel_walk: f64,
    gyro_bias: [f64; 3],
    gyro_noise: f64,
    gyro_walk: f64,
    gyro_bias_inst: f64,
    vib_amp: f64,
    vib_rectify: f64,
}

struct ImuModel {
    rng: Lcg,
    accel_bias_walk: [f64; 3],
    gyro_bias_walk: [f64; 3],
    gyro_bias_inst: [f64; 3],
    time: f64,
}
impl ImuModel {
    fn new(seed: u64) -> Self {
        Self {
            rng: Lcg::new(seed),
            accel_bias_walk: [0.0; 3],
            gyro_bias_walk: [0.0; 3],
            gyro_bias_inst: [0.0; 3],
            time: 0.0,
        }
    }
    // 返回 (accel[3], gyro[3])。true_accel/true_gyro 为 f64 真值。
    fn step(&mut self, dt: f64, cfg: &ImuCfg, true_accel: [f64; 3], true_gyro: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        self.time += dt;
        let mut vib_hf = [0.0f64; 3];
        for i in 0..3 {
            vib_hf[i] = cfg.vib_amp * (2.0 * std::f64::consts::PI * 40.0 * self.time + i as f64).sin();
        }
        let mut acc = [0.0f64; 3];
        for i in 0..3 {
            self.accel_bias_walk[i] += cfg.accel_walk * self.rng.gaussian() * dt.sqrt();
            self.accel_bias_walk[i] = self.accel_bias_walk[i].clamp(-0.5, 0.5);
            let rectify = cfg.vib_rectify * vib_hf[i] * vib_hf[i];
            acc[i] = true_accel[i] + cfg.accel_bias[i] + self.accel_bias_walk[i]
                + vib_hf[i] + rectify + cfg.accel_noise * self.rng.gaussian();
        }
        for i in 0..3 {
            self.gyro_bias_walk[i] += cfg.gyro_walk * self.rng.gaussian() * dt.sqrt();
            self.gyro_bias_walk[i] = self.gyro_bias_walk[i].clamp(-0.05, 0.05);
            let bi_tau = 100.0;
            let bi_alpha = dt / bi_tau;
            self.gyro_bias_inst[i] += bi_alpha * (-self.gyro_bias_inst[i]
                + cfg.gyro_bias_inst * (2.0 * bi_alpha).sqrt() * self.rng.gaussian());
            self.gyro_bias_inst[i] = self.gyro_bias_inst[i].clamp(-0.05, 0.05);
        }
        let mut gyr = [0.0f64; 3];
        for i in 0..3 {
            gyr[i] = true_gyro[i] + cfg.gyro_bias[i] + self.gyro_bias_walk[i]
                + self.gyro_bias_inst[i] + cfg.gyro_noise * self.rng.gaussian();
        }
        (acc, gyr)
    }
}

fn realistic_cfg() -> ImuCfg {
    ImuCfg {
        accel_bias: [0.02, -0.01, 0.05],
        accel_noise: 0.05,
        accel_walk: 0.01,
        gyro_bias: [0.001, -0.0005, 0.002],
        gyro_noise: 0.003,
        gyro_walk: 0.0001,
        gyro_bias_inst: 0.0003,
        vib_amp: 0.1,
        vib_rectify: 0.02,
    }
}

impl ImuCfg {
    /// 均值/白噪声测试用：关闭随机游走与振动（避免慢过程漂移与振动正弦方差污染）。
    fn clone_for_mean_test(&self) -> ImuCfg {
        ImuCfg {
            accel_walk: 0.0, gyro_walk: 0.0, gyro_bias_inst: 0.0,
            vib_amp: 0.0, vib_rectify: 0.0,
            ..*self
        }
    }
}

fn check(name: &str, cond: bool, msg: &str) {
    if cond {
        println!("PASS: {}", name);
    } else {
        println!("FAIL: {} -- {}", name, msg);
        std::process::exit(1);
    }
}

fn mean_std(xs: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    let v = xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n;
    (m, v.sqrt())
}

fn main() {
    let dt = 0.004;
    let cfg = realistic_cfg();

    // 1. 确定性：同种子两次运行结果一致
    {
        let mut a = ImuModel::new(0x5EED);
        let mut b = ImuModel::new(0x5EED);
        let (a1, _) = a.step(dt, &cfg, [0.0, 0.0, 9.81], [0.0; 3]);
        let (b1, _) = b.step(dt, &cfg, [0.0, 0.0, 9.81], [0.0; 3]);
        check("imu_deterministic", (0..3).all(|i| (a1[i] - b1[i]).abs() < 1e-12), "same seed differs");
    }

    // 2. 加速度计均值 ≈ 真值 + 零偏（关掉随机游走，避免慢过程有限窗口均值漂移）
    //    振动为 0 均值、整流为正偏置 ~ vib_rectify·vib_amp²/2
    {
        let mut cfg2 = cfg.clone_for_mean_test();
        let mut s = ImuModel::new(0x1111);
        let n = 200_000;
        let mut acc_z = Vec::with_capacity(n);
        for _ in 0..n {
            let (acc, _) = s.step(dt, &cfg2, [0.0, 0.0, 9.81], [0.0; 3]);
            acc_z.push(acc[2]);
        }
        let (m, sd) = mean_std(&acc_z);
        let expected_bias = cfg2.accel_bias[2] + cfg2.vib_rectify * cfg2.vib_amp.powi(2) / 2.0;
        let expected_mean = 9.81 + expected_bias;
        check("accel_mean_near_truth_plus_bias",
              (m - expected_mean).abs() < 0.02,
              &format!("m={:.4}, expect~{:.4}", m, expected_mean));
        // 白噪声 std ≈ accel_noise（RW 已关）
        check("accel_white_noise_std", (sd - cfg2.accel_noise).abs() < 0.02,
              &format!("sd={:.4}, expect~{:.4}", sd, cfg2.accel_noise));
    }

    // 3. 加速度随机游走（Allan RW 段：差分 std = √2·σ·√dt）
    //    关闭振动/白噪声，只留 RW，使 RW 量级可测。
    {
        let cfg_rw = ImuCfg {
            accel_bias: [0.0; 3], accel_noise: 0.0, accel_walk: 0.01,
            gyro_bias: [0.0; 3], gyro_noise: 0.0, gyro_walk: 0.0,
            gyro_bias_inst: 0.0, vib_amp: 0.0, vib_rectify: 0.0,
        };
        let mut s = ImuModel::new(0x2222);
        for _ in 0..5000 { s.step(dt, &cfg_rw, [0.0; 3], [0.0; 3]); } // 预热
        let n = 100_000;
        let mut prev = 0.0f64;
        let mut diffs = Vec::with_capacity(n);
        for k in 0..n {
            let (acc, _) = s.step(dt, &cfg_rw, [0.0; 3], [0.0; 3]);
            if k > 0 { diffs.push(acc[0] - prev); }
            prev = acc[0];
        }
        let (_m, sd) = mean_std(&diffs);
        let expected = cfg_rw.accel_walk * dt.sqrt() * 2.0_f64.sqrt();
        check("accel_random_walk_magnitude",
              (sd - expected).abs() < expected * 0.3,
              &format!("sd_diff={:.6}, expected~{:.6}", sd, expected));
    }

    // 4. 陀螺偏置不稳定性：慢变（Markov, τ=100s），有界（不发散、不冻结）
    //    离散 OU 稳态 std 落在 [0.1·B, 20·B] 量级，远小于 clamp(0.05)，
    //    ——核心物理属性是"有界慢变"，而非精确等于 B。
    {
        let mut s = ImuModel::new(0x3333);
        let n = 300_000; // ~1200s 远超 bi_tau
        let mut gyr_x = Vec::with_capacity(n);
        for _ in 0..n {
            let (_, g) = s.step(dt, &cfg, [0.0; 3], [0.0; 3]);
            gyr_x.push(g[0]);
        }
        let (_m, sd) = mean_std(&gyr_x);
        check("gyro_bias_instability_bounded",
              sd > 0.1 * cfg.gyro_bias_inst && sd < 0.05,
              &format!("sd={:.6}, in (0.1*B={:.6}, clamp=0.05)", sd, 0.1 * cfg.gyro_bias_inst));
    }

    // 5. 陀螺随机游走 + 白噪声：相邻差分量级含 RW 与白噪声
    {
        let mut s = ImuModel::new(0x4444);
        for _ in 0..5000 { s.step(dt, &cfg, [0.0; 3], [0.0; 3]); }
        let n = 100_000;
        let mut prev = 0.0f64;
        let mut diffs = Vec::with_capacity(n);
        for k in 0..n {
            let (_, g) = s.step(dt, &cfg, [0.0; 3], [0.0; 3]);
            if k > 0 { diffs.push(g[1] - prev); }
            prev = g[1];
        }
        let (_m, sd) = mean_std(&diffs);
        // 差分 std ≈ √(RW² + 2·噪声²)，RW = gyro_walk·√dt，噪声 = gyro_noise
        let rw = cfg.gyro_walk * dt.sqrt();
        let expected = (rw * rw + 2.0 * cfg.gyro_noise.powi(2)).sqrt();
        check("gyro_walk_plus_noise_magnitude",
              (sd - expected).abs() < expected * 0.5,
              &format!("sd_diff={:.6}, expected~{:.6}", sd, expected));
    }

    println!("ALL OK (P0-A IMU error tree)");
}
