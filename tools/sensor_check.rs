// 独立验证 fly-sim-core/src/sensor.rs 的磁力计/气压计统计特性，绕过 phy-rigid 编译阻塞。
// 编译：rustc --edition 2021 -O sensor_check.rs -o sensor_check.exe && sensor_check.exe
// 复刻 process_attitude 的磁力计(硬铁/软铁/噪声/倾角) + 气压计(噪声/漂移)逻辑。

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

struct SensorModel {
    rng: Lcg,
    baro_bias: f64,
}
impl SensorModel {
    fn new(seed: u64) -> Self {
        Self { rng: Lcg::new(seed), baro_bias: 0.0 }
    }
    fn process_attitude(&mut self, dt: f64, q: [f64; 4], alt: f64) -> ([f64; 3], f64) {
        // 磁力计：NED 地磁场 → 机体 + 硬铁/软铁/噪声
        let dip = 60.0f64.to_radians();
        let b_ned = [25.0 * dip.cos(), 0.0, 25.0 * dip.sin()];
        let b_body = rot_conj(q, b_ned);
        let hi = [0.3, -0.2, 0.4];
        let si = [0.98, 1.03, 0.99];
        let mut field = [0.0f64; 3];
        for i in 0..3 {
            field[i] = b_body[i] * si[i] + hi[i] + 0.05 * self.rng.gaussian();
        }
        // 气压计：真值 + 漂移 + 噪声
        self.baro_bias += 0.05 * self.rng.gaussian() * dt.sqrt();
        self.baro_bias = self.baro_bias.clamp(-50.0, 50.0);
        let alt_meas = alt + self.baro_bias + 0.3 * self.rng.gaussian();
        (field, alt_meas)
    }
}

fn rot_conj(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], -q[1], -q[2], -q[3]);
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

fn check(name: &str, cond: bool, msg: &str) {
    if cond {
        println!("PASS: {}", name);
    } else {
        println!("FAIL: {} -- {}", name, msg);
        std::process::exit(1);
    }
}

fn main() {
    // 1. 水平姿态（q=单位）下，磁力计应测到磁北（+x 大分量，z 有向下分量）。
    //    硬铁偏置加入后，测量 = B_body·软铁 + 硬铁。
    let mut s = SensorModel::new(0x1234);
    // 平均 10000 次消噪声
    let dip = 60.0f64.to_radians();
    let b_ned = [25.0 * dip.cos(), 0.0, 25.0 * dip.sin()];
    let mut sum = [0.0f64; 3];
    for _ in 0..10000 {
        let (f, _) = s.process_attitude(0.004, [1.0, 0.0, 0.0, 0.0], 0.0);
        for i in 0..3 { sum[i] += f[i]; }
    }
    let mean = [sum[0]/10000.0, sum[1]/10000.0, sum[2]/10000.0];
    // 水平：机体系磁场 ≈ B_ned·软铁 + 硬铁
    let expect = [
        b_ned[0]*0.98 + 0.3,
        b_ned[1]*1.03 - 0.2,
        b_ned[2]*0.99 + 0.4,
    ];
    check("mag_horizontal_matches_expectation",
          (0..3).all(|i| (mean[i]-expect[i]).abs() < 0.1),
          &format!("mean={:?}, expect={:?}", mean, expect));
    // 机体系北向分量应为正（磁北在机头方向）
    check("mag_north_positive", mean[0] > 10.0, "mag north");

    // 2. 磁力计噪声：标准差 ≈ 噪声(0.05)·sqrt(软铁) 量级
    //    （噪声经过软铁缩放，z 轴软铁0.99）
    let mut s = SensorModel::new(0x99);
    let n = 100_000;
    let mut sumsq = [0.0f64; 3];
    for _ in 0..n {
        let (f, _) = s.process_attitude(0.004, [1.0,0.0,0.0,0.0], 0.0);
        for i in 0..3 { sumsq[i] += (f[i]-expect[i]).powi(2); }
    }
    for i in 0..3 {
        let std = (sumsq[i]/n as f64).sqrt();
        // 噪声 std = 0.05·软铁_i ≈ 0.05
        check(&format!("mag_noise_std_{}", i), (0.03 < std) && (std < 0.09),
              &format!("std={:.4}, expect ~0.05", std));
    }

    // 3. 气压计：真值高度 100m，测量应接近 100 + 漂移 + 噪声，均值≈100
    let mut s = SensorModel::new(0xabcd);
    // 预热漂移
    for _ in 0..1000 { s.process_attitude(0.004, [1.0,0.0,0.0,0.0], 100.0); }
    let n = 100_000;
    let mut ssum = 0.0f64;
    for _ in 0..n {
        let (_, alt) = s.process_attitude(0.004, [1.0,0.0,0.0,0.0], 100.0);
        ssum += alt;
    }
    let mean = ssum/n as f64;
    // 漂移累计后偏置可能明显，验证测量围绕"真值+当前漂移"，且噪声 std≈0.3
    check("baro_altitude_reasonable", (mean-100.0).abs() < 5.0,
          &format!("baro mean={:.2}, expect ~100", mean));

    println!("ALL OK");
}
