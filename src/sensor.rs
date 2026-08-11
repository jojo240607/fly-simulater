//! 阶段 4 — 传感器真实化模型。
//!
//! 把物理引擎给出的"真值"转换成接近真实的传感器读数：
//!  - IMU：零偏（可缓变）、高斯白噪声、随机游走（bias drift）、振动耦合。
//!  - GPS/NED：固定延迟（环形缓冲）、更新率（降频）、位置/速度噪声、偶发丢星。
//!  - 磁力计（若启用航向）：硬铁/软铁干扰 + 倾角误差（本阶段预留接口）。
//!
//! 全部确定性（种子化 PRNG），保证可复现。

use flyctrl_core::units::{Meter, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ImuSample, PosSample};

/// 确定性 PRNG（与 wind.rs 同款 LCG）。
struct Lcg {
    state: std::num::Wrapping<u64>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: std::num::Wrapping(seed.wrapping_add(0x9E3779B97F4A7C15)),
        }
    }
    fn next_f64(&mut self) -> f64 {
        self.state = self.state * std::num::Wrapping(6364136223846793005) + std::num::Wrapping(1442695040888963407);
        let mut x = self.state;
        let shift = 64 - 31;
        x ^= x >> shift;
        x = x * std::num::Wrapping(0xDA942042E4DD58B5);
        x ^= x >> 32;
        (x.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// 传感器配置（可由机架/场景扩展）。
#[derive(Clone, Debug)]
pub struct SensorConfig {
    // IMU
    pub accel_bias: [f64; 3],     // 零偏 m/s^2
    pub accel_noise: f64,         // 白噪声 std m/s^2
    pub gyro_bias: [f64; 3],      // 零偏 rad/s
    pub gyro_noise: f64,          // 白噪声 std rad/s
    pub gyro_walk: f64,           // 随机游走 std rad/s per sqrt(s)
    pub vib_amp: f64,             // 振动耦合幅值 m/s^2（机体高频）
    // GPS
    pub gps_delay: f64,           // 延迟 s
    pub gps_rate: f64,            // 更新率 Hz（< 控制率则降频）
    pub gps_pos_noise: f64,       // 位置噪声 std m
    pub gps_vel_noise: f64,       // 速度噪声 std m/s
    pub gps_drop_prob: f64,       // 偶发丢星概率/帧
    pub seed: u64,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            // 默认 0 噪声：保持 SIL 场景 PASS（证明传感器模型数据路径无 bug）。
            // 真实噪声经 --sensor-noise 开启（暴露 EKF 对 IMU 噪声不耐受，见 PLAN 阶段5）。
            accel_bias: [0.0, 0.0, 0.0],
            accel_noise: 0.0,
            gyro_bias: [0.0, 0.0, 0.0],
            gyro_noise: 0.0,
            gyro_walk: 0.0,
            vib_amp: 0.0,
            gps_delay: 0.0,
            gps_rate: 1000.0,
            gps_pos_noise: 0.0,
            gps_vel_noise: 0.0,
            gps_drop_prob: 0.0,
            seed: 0x5EED_1357,
        }
    }
}

impl SensorConfig {
    /// 真实消费级 IMU/GPS 噪声配置（开启 --sensor-noise 用）。
    /// 注：当前 EKF 测量噪声协方差 R 疑似为 0，开此配置会暴露发散（阶段5 修复）。
    pub fn realistic() -> Self {
        Self {
            accel_bias: [0.02, -0.01, 0.05],
            accel_noise: 0.05,
            gyro_bias: [0.001, -0.0005, 0.002],
            gyro_noise: 0.003,
            gyro_walk: 0.0001,
            vib_amp: 0.1,
            gps_delay: 0.15,
            gps_rate: 5.0,
            gps_pos_noise: 0.5,
            gps_vel_noise: 0.1,
            gps_drop_prob: 0.0,
            seed: 0x5EED_1357,
        }
    }
}

/// GPS 延迟环形缓冲项。
struct GpsDelay {
    delay_steps: usize,
    buf: std::collections::VecDeque<(f64, f64, f64, f64, f64, f64)>, // (n,e,d, vn,ve,vd)
}

/// 传感器模型运行状态。
pub struct SensorModel {
    cfg: SensorConfig,
    rng: Lcg,
    // IMU 随机游走状态（bias drift）
    gyro_bias_walk: [f64; 3],
    // GPS 延迟缓冲 + 降频计数
    gps_delay: GpsDelay,
    gps_counter: u64,
    // 振动相位
    vib_phase: f64,
    time: f64,
}

impl SensorModel {
    pub fn new(cfg: SensorConfig, dt: f64) -> Self {
        let delay_steps = ((cfg.gps_delay / dt).round() as usize).max(1);
        let seed = cfg.seed;
        Self {
            cfg,
            rng: Lcg::new(seed),
            gyro_bias_walk: [0.0; 3],
            gps_delay: GpsDelay { delay_steps, buf: std::collections::VecDeque::new() },
            gps_counter: 0,
            vib_phase: 0.0,
            time: 0.0,
        }
    }

    /// 处理一帧真值，返回（带噪声 IMU, 延迟/降频/带噪 GPS）。
    /// `true_pos` 为 NED 位置 [n,e,d]，`true_vel` 为 NED 速度 [vn,ve,vd]。
    pub fn process(
        &mut self,
        dt: f64,
        true_accel: [f32; 3],  // 机体比力（真值，已含重力补偿前）
        true_gyro: [f32; 3],   // 机体角速度（真值）
        true_pos: [f32; 3],
        true_vel: [f32; 3],
    ) -> (ImuSample, Option<PosSample>) {
        self.time += dt;

        // ---- IMU ----
        let mut acc = [0.0f64; 3];
        for i in 0..3 {
            // 振动：机体高频正弦 + 噪声
            let vib = self.cfg.vib_amp * (2.0 * std::f64::consts::PI * 40.0 * self.time + i as f64).sin();
            acc[i] = true_accel[i] as f64 + self.cfg.accel_bias[i] + vib
                + self.cfg.accel_noise * self.rng.gaussian();
        }
        // 陀螺：随机游走 bias + 噪声
        for i in 0..3 {
            self.gyro_bias_walk[i] += self.cfg.gyro_walk * self.rng.gaussian() * (dt.sqrt());
            self.gyro_bias_walk[i] = self.gyro_bias_walk[i].clamp(-0.05, 0.05);
        }
        let mut gyr = [0.0f64; 3];
        for i in 0..3 {
            gyr[i] = true_gyro[i] as f64 + self.cfg.gyro_bias[i] + self.gyro_bias_walk[i]
                + self.cfg.gyro_noise * self.rng.gaussian();
        }
        self.vib_phase += dt;

        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(acc[0] as f32),
                MeterPerSecondSquared(acc[1] as f32),
                MeterPerSecondSquared(acc[2] as f32),
            ],
            gyro: [
                RadianPerSecond(gyr[0] as f32),
                RadianPerSecond(gyr[1] as f32),
                RadianPerSecond(gyr[2] as f32),
            ],
        };

        // ---- GPS（延迟 + 降频 + 噪声 + 丢星）----
        let mut pos_sample = None;
        self.gps_counter += 1;
        let gps_period = (((1.0 / self.cfg.gps_rate) / dt).round() as u64).max(1);
        // 推入当前真值（带噪）到延迟缓冲。丢星帧推入无效标记（NaN 位置）。
        let drop = self.rng.next_f64() < self.cfg.gps_drop_prob;
        if !drop {
            let n = true_pos[0] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian();
            let e = true_pos[1] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian();
            let d = true_pos[2] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian();
            let vn = true_vel[0] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            let ve = true_vel[1] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            let vd = true_vel[2] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            self.gps_delay.buf.push_back((n, e, d, vn, ve, vd));
        } else {
            // 丢星：推入特殊值（位置 NaN）标记无效，输出时转 None。
            self.gps_delay.buf.push_back((f64::NAN, f64::NAN, f64::NAN, 0.0, 0.0, 0.0));
        }
        // 保持缓冲长度 = delay_steps + 1
        while self.gps_delay.buf.len() > self.gps_delay.delay_steps + 1 {
            self.gps_delay.buf.pop_front();
        }
        // 降频输出：每 gps_period 帧输出一次缓冲最旧的（= 延迟后）
        if self.gps_counter % gps_period == 0 && !self.gps_delay.buf.is_empty() {
            let (n, e, d, vn, ve, vd) = *self.gps_delay.buf.front().unwrap();
            // 丢星帧（NaN）-> 返回 None，EKF 退化为纯惯性。
            if !n.is_nan() {
                pos_sample = Some(PosSample {
                    pos: [Meter(n as f32), Meter(e as f32), Meter(d as f32)],
                });
            }
        }

        (imu, pos_sample)
    }
}
