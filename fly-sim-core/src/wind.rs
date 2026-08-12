//! 阶段 3 — 风场与环境模型。
//!
//! 设计：风以"世界系气流速度"（引擎 Y-up 坐标，与刚体速度同系）表示。
//! 注入方式 = 让气动阻力基于**相对风速** `v_rel = v_body - wind`（而非绝对速度），
//! 这样风通过既有气动模型自然作用于机体，物理一致、无需额外外力接口。
//!
//! 组成：
//!  - 基础风 `base`：恒定世界系分量（如稳定侧风）。
//!  - 阵风 `gust`：带随机相位的正弦脉冲（可多频叠加）。
//!  - 湍流 `turb`：Dryden 简化（一阶低通白噪声），各轴独立。
//! 全部确定性（种子化），保证可复现。

use std::num::Wrapping;

/// 世界系（引擎 Y-up）风速向量 [vx, vy, vz] (m/s)。
pub type WindVec = [f64; 3];

/// 线性同余 PRNG（确定性，种子化）。
struct Lcg {
    state: Wrapping<u64>,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: Wrapping(seed.wrapping_add(0x9E3779B97F4A7C15)),
        }
    }
    /// 返回 [0,1) 浮点。
    fn next_f64(&mut self) -> f64 {
        // PCG 风格混合
        self.state = self.state * Wrapping(6364136223846793005) + Wrapping(1442695040888963407);
        let mut x = self.state;
        let shift = 64 - 31;
        x ^= x >> shift;
        x = x * Wrapping(0xDA942042E4DD58B5);
        x ^= x >> 32;
        (x.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    /// 标准正态（Box-Muller）。
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

/// 风场配置（由 sim 场景或机架扩展传入）。
#[derive(Clone, Debug)]
pub struct WindConfig {
    /// 基础风（世界系 NED 经 vec_ned_to_up 转换后的 UP 向量）。
    pub base: WindVec,
    /// 阵风幅度（各轴峰值 m/s）。
    pub gust_amp: WindVec,
    /// 阵风主频 (Hz)。
    pub gust_freq: f64,
    /// 湍流强度（白噪声标准差 m/s）。
    pub turb_sigma: WindVec,
    /// 湍流时间常数 (s)：越大越平滑。
    pub turb_tau: f64,
    /// 随机种子（复现）。
    pub seed: u64,
}

impl Default for WindConfig {
    fn default() -> Self {
        Self {
            base: [0.0, 0.0, 0.0],
            gust_amp: [0.0, 0.0, 0.0],
            gust_freq: 0.1,
            turb_sigma: [0.0, 0.0, 0.0],
            turb_tau: 0.5,
            seed: 0x1234_5678,
        }
    }
}

/// 运行中风场状态。
pub struct WindField {
    cfg: WindConfig,
    rng: Lcg,
    /// 滤波后的湍流状态（各轴）。
    turb_state: WindVec,
    time: f64,
}

impl WindField {
    pub fn new(cfg: WindConfig) -> Self {
        let seed = cfg.seed;
        Self {
            cfg,
            rng: Lcg::new(seed),
            turb_state: [0.0; 3],
            time: 0.0,
        }
    }

    /// 推进 dt 并返回当前世界系（UP）风速。
    pub fn sample(&mut self, dt: f64) -> WindVec {
        self.time += dt;
        let t = self.time;

        // 阵风：各轴多频正弦叠加（P1-3 增强，更丰富频谱）+ 轴间相位差。
        // 三个频率分量（freq, 2freq, 3freq）以递减幅度叠加，模拟多尺度阵风。
        let mut gust = [0.0f64; 3];
        for i in 0..3 {
            let phase = (i as f64) * 1.7; // 轴间相位差
            let f = self.cfg.gust_freq;
            gust[i] = self.cfg.gust_amp[i]
                * (f64::sin(2.0 * std::f64::consts::PI * f * t + phase)
                    + 0.5 * f64::sin(2.0 * std::f64::consts::PI * 2.0 * f * t + 2.0 * phase)
                    + 0.25 * f64::sin(2.0 * std::f64::consts::PI * 3.0 * f * t + 3.0 * phase));
        }

        // 湍流：一阶低通白噪声（指数相关）。Dryden 尺度差异：纵向（机体 X）尺度大、
        // 相关时间长；横向/垂直尺度小、时间常数短。各轴独立时间常数。
        // （纵向时间常数 = turb_tau；横向/垂直 = turb_tau × 0.5，模拟 Dryden 谱的
        //   纵向谱在低频更强、横向谱在高频衰减更缓。）
        let tau_x = self.cfg.turb_tau;
        let tau_yz = self.cfg.turb_tau * 0.5;
        let mut turb = [0.0f64; 3];
        for i in 0..3 {
            let tau = if i == 0 { tau_x } else { tau_yz };
            let alpha = (dt / tau).min(1.0);
            // 增益补偿：一阶低通 `x+=(w-x)·alpha` 的稳态输出方差 =
            // alpha/(2-alpha)·σw²。为使输出标准差≈turb_sigma，输入白噪声用
            // σ·sqrt((2-alpha)/alpha) 补偿，避免时间常数大时湍流被过度平滑。
            let gain = ((2.0 - alpha) / alpha).sqrt();
            let w = self.cfg.turb_sigma[i] * gain * self.rng.next_gaussian();
            self.turb_state[i] += (w - self.turb_state[i]) * alpha;
            turb[i] = self.turb_state[i];
        }

        let mut wind = [0.0f64; 3];
        for i in 0..3 {
            wind[i] = self.cfg.base[i] + gust[i] + turb[i];
        }
        wind
    }
}
