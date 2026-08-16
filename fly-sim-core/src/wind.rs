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
//!  - 空间相关风场（P2-B）：风随位置变化（风切变廓线 + 空间相关正弦场），使机身不同部位风速不同。
//!  - 确定性阵风突风（P2-B）：1-cos 包络的一次性突风，可精确复现。
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
    /// 空间相关长度尺度 (m)：决定机身不同部位风速差异的空间频率。
    /// 0 表示空间均匀（退化旧行为，保持兼容）。
    pub spatial_scale: f64,
    /// 风切变指数 α（幂律廓线 `w(z) = w_ref·(z/z_ref)^α`）。
    /// 0 表示无高度切变（退化旧行为）。
    pub shear_exponent: f64,
    /// 风切变参考高度 (m)，与 `shear_exponent` 配合。
    pub shear_ref_height: f64,
    /// 确定性阵风突风幅度（一次性 1-cos 包络，各轴峰值 m/s）。
    /// 全 0 表示无突风注入（退化旧行为）。
    pub gust_burst_amp: WindVec,
    /// 阵风突风起始时间 (s) 与持续半宽 (s)（1-cos 包络 `t0±hw`）。
    pub gust_burst_t0: f64,
    pub gust_burst_hw: f64,
    /// 热气流（thermal）中心上升速度 (m/s)，0 表示无热气流（退化旧行为）。
    pub thermal_strength: f64,
    /// 热气流影响半径 (m)，高斯衰减尺度。
    pub thermal_radius: f64,
    /// 热气流有效高度上限 (m)：高于此高度不再有上升气流（对流泡封顶）。
    pub thermal_height: f64,
    /// 热气流中心初始水平位置 [x, z]（UP 系水平面，与 pos 约定一致：z 为"高度"轴。
    /// 注：与既有 wind.rs 高度轴约定保持一致，垂直分量走 wind[2]）。
    pub thermal_pos0: [f64; 2],
    /// 热气流水平漂移速度 [vx, vz] (m/s)，模拟风载热气流平移（确定性）。
    pub thermal_drift: [f64; 2],
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
            spatial_scale: 0.0,
            shear_exponent: 0.0,
            shear_ref_height: 10.0,
            gust_burst_amp: [0.0, 0.0, 0.0],
            gust_burst_t0: 1.0,
            gust_burst_hw: 0.5,
            thermal_strength: 0.0,
            thermal_radius: 0.0,
            thermal_height: 0.0,
            thermal_pos0: [0.0, 0.0],
            thermal_drift: [0.0, 0.0],
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
    /// 热气流中心当前水平位置 [x, z]，随 drift 推进。
    thermal_center: [f64; 2],
}

impl WindField {
    pub fn new(cfg: WindConfig) -> Self {
        let seed = cfg.seed;
        let center = cfg.thermal_pos0;
        Self {
            cfg,
            rng: Lcg::new(seed),
            turb_state: [0.0; 3],
            time: 0.0,
            thermal_center: center,
        }
    }

    /// 推进 dt 并返回当前世界系（UP）风速。
    /// 兼容旧调用：以原点位置采样（空间均匀 / 无风切变）。
    pub fn sample(&mut self, dt: f64) -> WindVec {
        self.sample_at(dt, &[0.0, 0.0, 0.0])
    }

    /// 推进 dt 并返回给定世界系位置 `pos`（UP，[x,y,z]）处的风速。
    ///
    /// P2-B 新增：
    /// - 风切变廓线：基础风随高度 z 按幂律 `w(z) = w_ref·(z/z_ref)^α` 缩放（仅作用于
    ///   `base` 与空间相关项，不作用于湍流/阵风的时间调制分量，避免破坏 Dryden 时间相关）。
    /// - 空间相关扰动：以 `spatial_scale` 为空间波长的正弦场 + 位置相位，使机身不同
    ///   部位（不同 x/z）感受到不同风速，体现空间相关性。
    /// - 确定性阵风突风：在 `t0±hw` 窗口内叠加 1-cos 包络突风（确定性、可复现）。
    pub fn sample_at(&mut self, dt: f64, pos: &[f64; 3]) -> WindVec {
        self.time += dt;
        let t = self.time;
        let x = pos[0];
        let z = pos[2]; // UP 系 z = 高度

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

        // 确定性阵风突风：1-cos 包络（窗口 t0±hw 外为 0），确定性、可精确复现。
        let mut burst = [0.0f64; 3];
        if self.cfg.gust_burst_hw > 0.0 {
            let dt_b = (t - self.cfg.gust_burst_t0) / self.cfg.gust_burst_hw;
            if dt_b > -1.0 && dt_b < 1.0 {
                let env = 0.5 * (1.0 - f64::cos(std::f64::consts::PI * (dt_b + 1.0)));
                for i in 0..3 {
                    burst[i] = self.cfg.gust_burst_amp[i] * env;
                }
            }
        }

        // 热气流（thermal）：上升暖气流柱，中心最大上升速度随水平半径高斯衰减、
        // 高于 thermal_height 封顶为 0。热气流中心随 drift 水平漂移（确定性）。
        // 垂直上升分量按 wind.rs 既有约定走 wind[2]（与 shear 同用 pos[2] 为"高度"轴）。
        let mut thermal = [0.0f64; 3];
        if self.cfg.thermal_strength > 0.0 && self.cfg.thermal_radius > 0.0 {
            // 推进热气流中心（仅在第一次有效推进时基于初始位置积分）。
            self.thermal_center[0] += self.cfg.thermal_drift[0] * dt;
            self.thermal_center[1] += self.cfg.thermal_drift[1] * dt;
            let cx = self.thermal_center[0];
            let cz = self.thermal_center[1];
            let dx = x - cx;
            let dz = z - cz;
            let r2 = dx * dx + dz * dz;
            let rad = self.cfg.thermal_radius.max(1e-3);
            // 高度封顶：z 高于 thermal_height 时无上升气流（对流泡顶）。
            let height_factor = if self.cfg.thermal_height > 0.0 && z > self.cfg.thermal_height {
                0.0
            } else {
                1.0
            };
            // 高斯径向衰减（3σ 截断，避免无限远仍有微小上升）。
            let sigma3 = (3.0 * rad).powi(2);
            if r2 < sigma3 {
                let w_up = self.cfg.thermal_strength
                    * f64::exp(-r2 / (2.0 * rad * rad))
                    * height_factor;
                thermal[2] += w_up; // 上升气流（wind.rs 约定垂直=wind[2]）
            }
        }

        // 湍流：一阶低通白噪声（指数相关）。Dryden 尺度差异：纵向（机体 X）尺度大、
        // 相关时间长；横向/垂直尺度小、时间常数短。各轴独立时间常数。
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

        // 空间相关因子（P2-B）：
        // - 风切变：基础风按高度幂律缩放（仅在 z>0 时生效，避免 z 负时数值异常）。
        // - 空间相关扰动：以 spatial_scale 为波长的正弦场，叠加到 base 分量，
        //   使远离原点处基础风被调制（机身不同部位 x/z 不同 → 风速不同）。
        let shear = if self.cfg.shear_exponent > 0.0 && z > 1e-6 {
            let zr = self.cfg.shear_ref_height.max(1e-3);
            (z / zr).max(0.0).powf(self.cfg.shear_exponent)
        } else {
            1.0
        };
        let spatial = if self.cfg.spatial_scale > 0.0 {
            let k = 2.0 * std::f64::consts::PI / self.cfg.spatial_scale;
            // 轴间用不同相位，避免各轴完全同步；纯确定性空间场。
            [
                f64::sin(k * x),
                f64::sin(k * z * 0.8 + 1.3),
                f64::sin(k * (x + z) * 0.5 + 2.1),
            ]
        } else {
            [0.0, 0.0, 0.0]
        };

        let mut wind = [0.0f64; 3];
        for i in 0..3 {
            // base 经风切变缩放，并叠加空间相关调制（以 base 幅度为基准，幅度随 spatial_scale 场变化）。
            let base_spat = self.cfg.base[i] * shear * (1.0 + 0.3 * spatial[i].max(-0.9));
            wind[i] = base_spat + gust[i] + burst[i] + turb[i] + thermal[i];
        }
        wind
    }
}
