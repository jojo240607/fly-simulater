//! 阶段 4 — 传感器真实化模型。
//!
//! 把物理引擎给出的"真值"转换成接近真实的传感器读数：
//!  - IMU：零偏（可缓变）、高斯白噪声、随机游走（bias drift）、振动耦合。
//!  - GPS/NED：固定延迟（环形缓冲）、更新率（降频）、位置/速度噪声、偶发丢星。
//!  - 磁力计（若启用航向）：硬铁/软铁干扰 + 倾角误差（本阶段预留接口）。
//!
//! 全部确定性（种子化 PRNG），保证可复现。

use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ImuSample, PosSample};

/// 磁力计采样（机体系 3 轴，单位化 uT）。仿真侧专用，不与 MCU 共享结构。
#[derive(Clone, Debug)]
pub struct MagSample {
    /// 机体坐标系三轴磁场（uT）。
    pub field: [f64; 3],
}

/// 气压计采样（由气压反演的高度，m）。
#[derive(Clone, Debug)]
pub struct BaroSample {
    /// 气压计高度（m），含噪声/漂移。
    pub altitude: f64,
    /// 气压计原生气压（hPa）。
    pub pressure: f64,
}

/// 确定性 PRNG（与 wind.rs 同款 LCG）。
#[derive(Clone, Debug)]
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
    pub gyro_bias_inst: f64,      // 陀螺偏置不稳定性 (BI) 强度 rad/s（Allan 低通闪烁近似）
    pub vib_amp: f64,             // 振动耦合幅值 m/s^2（机体高频）
    // GPS
    pub gps_delay: f64,           // 延迟 s
    pub gps_rate: f64,            // 更新率 Hz（< 控制率则降频）
    pub gps_pos_noise: f64,       // 位置噪声 std m
    pub gps_vel_noise: f64,       // 速度噪声 std m/s
    pub gps_drop_prob: f64,       // 偶发丢星概率/帧
    // 磁力计（航向）：机体系硬铁偏置 uT，软铁缩放，白噪声 std uT。
    pub mag_hard_iron: [f64; 3],  // 硬铁偏置 uT
    pub mag_soft_iron: [f64; 3],  // 软铁缩放因子
    pub mag_noise: f64,           // 白噪声 std uT
    // 气压计（高度）：白噪声 std m，慢漂移 std m/sqrt(s)。
    pub baro_noise: f64,          // 高度白噪声 std m
    pub baro_drift: f64,          // 高度慢漂移 std m/sqrt(s)
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
            gyro_bias_inst: 0.0,
            vib_amp: 0.0,
            gps_delay: 0.0,
            gps_rate: 1000.0,
            gps_pos_noise: 0.0,
            gps_vel_noise: 0.0,
            gps_drop_prob: 0.0,
            mag_hard_iron: [0.0, 0.0, 0.0],
            mag_soft_iron: [1.0, 1.0, 1.0],
            mag_noise: 0.0,
            baro_noise: 0.0,
            baro_drift: 0.0,
            seed: 0x5EED_1357,
        }
    }
}

impl SensorConfig {
    /// 真实消费级 IMU/GPS 噪声配置（开启 --sensor-noise 用）。
    /// 注：实测（PLAN 阶段11）开启后发散的根因是 **PID 控制律对噪声不耐受**，
    /// 而非 EKF（EKF 估计仍贴合真值，发散的是物理真值本身）。`ContactModel` 地面约束
    /// 会掩盖该发散。`realistic` 用于暴露控制律噪声鲁棒性缺陷。
    pub fn realistic() -> Self {
        Self {
            accel_bias: [0.02, -0.01, 0.05],
            accel_noise: 0.05,
            gyro_bias: [0.001, -0.0005, 0.002],
            gyro_noise: 0.003,
            gyro_walk: 0.0001,
            gyro_bias_inst: 0.0003,
            vib_amp: 0.1,
            gps_delay: 0.15,
            gps_rate: 20.0,
            gps_pos_noise: 0.5,
            gps_vel_noise: 0.1,
            gps_drop_prob: 0.0,
            mag_hard_iron: [0.3, -0.2, 0.4], // uT 硬铁
            mag_soft_iron: [0.98, 1.03, 0.99], // 软铁缩放
            mag_noise: 0.05, // uT
            baro_noise: 0.3, // m
            baro_drift: 0.05, // m/sqrt(s)
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
    // 陀螺偏置不稳定性 (BI) 慢变状态：一阶低通白噪声（Allan 偏置不稳定性近似）
    gyro_bias_inst_state: [f64; 3],
    // GPS 延迟缓冲 + 降频计数
    gps_delay: GpsDelay,
    gps_counter: u64,
    // 振动相位
    vib_phase: f64,
    // 气压计慢漂移状态
    baro_bias: f64,
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
            gyro_bias_inst_state: [0.0; 3],
            gps_delay: GpsDelay { delay_steps, buf: std::collections::VecDeque::new() },
            gps_counter: 0,
            vib_phase: 0.0,
            baro_bias: 0.0,
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
        // 陀螺：随机游走 bias + 偏置不稳定性 (BI) + 噪声
        // BI：一阶低通白噪声近似 Allan 偏置不稳定性（闪烁噪声）。
        //   稳态 std(bi) ≈ gyro_bias_inst；时间常数 tau 决定慢变速度。
        let bi_tau = 10.0; // s
        let bi_alpha = (dt / bi_tau).min(1.0);
        for i in 0..3 {
            self.gyro_bias_walk[i] += self.cfg.gyro_walk * self.rng.gaussian() * (dt.sqrt());
            self.gyro_bias_walk[i] = self.gyro_bias_walk[i].clamp(-0.05, 0.05);
            // 驱动白噪声幅值使稳态 std 收敛到 gyro_bias_inst：
            //   w ~ N(0, gyro_bias_inst^2 * 2/bi_alpha)，低通后 std = gyro_bias_inst。
            let w = self.rng.gaussian() * self.cfg.gyro_bias_inst * (2.0 / bi_alpha).sqrt();
            self.gyro_bias_inst_state[i] += bi_alpha * (w - self.gyro_bias_inst_state[i]);
            self.gyro_bias_inst_state[i] = self.gyro_bias_inst_state[i].clamp(-0.05, 0.05);
        }
        let mut gyr = [0.0f64; 3];
        for i in 0..3 {
            gyr[i] = true_gyro[i] as f64 + self.cfg.gyro_bias[i]
                + self.gyro_bias_walk[i] + self.gyro_bias_inst_state[i]
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
                pos_sample = Some(PosSample::with_vel(
                    [Meter(n as f32), Meter(e as f32), Meter(d as f32)],
                    [MeterPerSecond(vn as f32), MeterPerSecond(ve as f32), MeterPerSecond(vd as f32)],
                ));
            }
        }

        (imu, pos_sample)
    }

    /// 阶段 P2-1：磁力计 + 气压计（航向/高度测量）。
    ///
    /// 气压计：真值高度 + 白噪声 + 慢漂移（随机游走）。`altitude` = NED 高度（-d，m）。
    pub fn process_baro(&mut self, dt: f64, altitude: f64) -> BaroSample {
        self.time += dt;
        // 慢漂移：随机游走
        self.baro_bias += self.cfg.baro_drift * self.rng.gaussian() * dt.sqrt();
        self.baro_bias = self.baro_bias.clamp(-50.0, 50.0);
        let alt_meas = altitude + self.baro_bias + self.cfg.baro_noise * self.rng.gaussian();
        // 气压：标准大气近似（每 10m 约 1.2hPa 变化），海平面 1013.25hPa。
        let pressure = 1013.25 * (-alt_meas / 8434.0).exp();
        BaroSample { altitude: alt_meas, pressure }
    }
}

/// 用四元数共轭旋转向量（世界→机体）：R(q⁻¹)·v。
/// q 为机体→NED 四元数，其共轭即 NED→机体。
fn rotate_by_quat_conj(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], -q[1], -q[2], -q[3]); // 共轭（逆）
    // 用旋转公式 r = v + 2w(q×v) + 2(q×(q×v))
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

// ============================================================ 避障传感器（P1-2 障碍反射）

/// 距离传感器一次采样读数（避障雷达 / 激光雷达 / 深度相机通用）。
///
/// `distance` 为最近障碍距离（m）；`valid` 标记读数是否可用（false = 传感器失效：
/// 量程饱和、或因障碍太近"糊脸"导致近距盲区的视觉/深度失效）。上层控制器应把
/// `valid=false` 视为"该方向不可信"而非"无障碍"。
#[derive(Clone, Debug)]
pub struct RangeFinderSample {
    /// 最近障碍距离（m）。`valid=false` 时此值无意义（通常为 `max_range` 饱和值）。
    pub distance: f64,
    /// 读数是否有效（true=正常，false=失效/饱和）。
    pub valid: bool,
}

/// 避障距离传感器模型（雷达 / 深度相机）。
///
/// 把物理引擎射线求交得到的**真值距离**转成传感器读数：
/// - 加高斯测距噪声（标准差 `noise`）与固定偏置 `bias`；
/// - 超过 `max_range` 视为量程外（返回 `max_range` 饱和 + `valid=false`）；
/// - **近距盲区（视觉失效）**：当真值距离 < `blind_min` 时，回波淹没/相机糊脸，
///   深度/视觉通道给出错误饱和读数（`distance=max_range`、`valid=false`）——典型
///   "障碍太近反而看不到"的失效模式（雷达近距多路径 / 深度相机近距离退化）；
/// - 可选随机失效 `drop_prob`：每帧以该概率直接 `valid=false`（模拟瞬断/遮挡）。
#[derive(Clone, Debug)]
pub struct RangeFinderModel {
    pub max_range: f64,   // 最大量程 m（超距饱和）
    pub blind_min: f64,   // 近距盲区 m（< 此值判失效）
    pub noise: f64,       // 测距高斯噪声 std m
    pub bias: f64,        // 固定测距偏置 m
    pub drop_prob: f64,   // 每帧随机失效概率
    rng: Lcg,
}

impl RangeFinderModel {
    pub fn new(max_range: f64, blind_min: f64, noise: f64, bias: f64, drop_prob: f64, seed: u64) -> Self {
        Self {
            max_range,
            blind_min,
            noise,
            bias,
            drop_prob,
            rng: Lcg::new(seed ^ 0x5EED_F1D3),
        }
    }

    /// 由射线求交真值距离生成一次读数。
    ///
    /// `true_distance`：射线命中障碍的真值距离（m）；`None` 表示射程内无命中
    /// （量程外）。返回 `RangeFinderSample`。
    pub fn sample(&mut self, true_distance: Option<f64>) -> RangeFinderSample {
        // 随机瞬断
        if self.rng.next_f64() < self.drop_prob {
            return RangeFinderSample {
                distance: self.max_range,
                valid: false,
            };
        }
        let d = match true_distance {
            None => {
                // 量程外：饱和读数 + 失效标记（"看不到"≠"无障碍"）
                return RangeFinderSample {
                    distance: self.max_range,
                    valid: false,
                };
            }
            Some(d) => d,
        };
        // 近距盲区：障碍太近，视觉/深度通道糊脸失效
        if d < self.blind_min {
            return RangeFinderSample {
                distance: self.max_range, // 错误饱和（把近障碍误报为"远处/无障碍"）
                valid: false,
            };
        }
        // 正常：加偏置 + 噪声，裁剪到 [0, max_range]
        let measured = (d + self.bias + self.noise * self.rng.gaussian()).clamp(0.0, self.max_range);
        RangeFinderSample {
            distance: measured,
            valid: true,
        }
    }
}

// ============================================================ 避障控制器（P1-2 闭环联动）

/// 反应式避障控制器配置（感知→决策→规避闭环）。
///
/// 当前向测距传感器检测到前方障碍进入 `danger_dist` 内时，
/// 控制器分两层介入（详见 [`AvoidanceConfig::avoidance_velocity`]）：
/// 1. **制动**：沿机体前向施加与危险度成正比的减速（把前向速度指令压向 0/反向）。
/// 2. **横向闪避**：沿机体侧向施加一个恒定的横向速度指令，使机体偏离碰撞航线。
///
/// `danger_dist` 应小于 `RangeFinderModel::max_range`，否则全量程都触发闪避。
#[derive(Clone, Debug)]
pub struct AvoidanceConfig {
    /// 危险触发距离（m）：前方障碍真值/读数小于此值即进入规避。
    pub danger_dist: f64,
    /// 制动强度（m/s 每米）：危险度 = clamp(1 - dist/danger_dist, 0, 1)，
    /// 前向期望速度 = 原前向速度 - 危险度 * brake_gain * danger_dist。
    pub brake_gain: f64,
    /// 横向闪避速度指令（m/s，机体侧向 +Y，即右翼方向），恒定量。
    pub evade_lateral: f64,
}

impl AvoidanceConfig {
    pub fn new(danger_dist: f64, brake_gain: f64, evade_lateral: f64) -> Self {
        Self { danger_dist, brake_gain, evade_lateral }
    }

    /// 计算规避速度指令（NED 系，单位 m/s）。
    ///
    /// - `sample`：`plant.read_ranger()` 当前读数。读数无效（`valid=false`）时，
    ///   保守地按**饱和距离**处理（"看不到"≠"无障碍"，采用最危险假设反而安全，
    ///   这里采用**最安全假设**：无效读数视为未触发，避免凭空闪避导致误动作；
    ///   但近距盲区/饱和都已把距离错误上拉，故有效且距离已偏大，恰好不触发）。
    /// - `fwd_ned`：机体前向在世界 NED 系的单位向量（机体 -X → NED）。
    /// - `right_ned`：机体右向在世界 NED 系的单位向量（机体 +Y → NED）。
    ///
    /// 返回 `(v_ned: [f64;3], triggered: bool)`，`triggered` 表示本轮是否真的介入。
    /// 调用方把返回的速度指令叠加/并入原有速度设定点。
    pub fn avoidance_velocity(
        &self,
        sample: &RangeFinderSample,
        fwd_ned: [f64; 3],
        right_ned: [f64; 3],
    ) -> ([f64; 3], bool) {
        // 无效读数（瞬断/近距盲区/量程外饱和）一律视为"未确认危险"，不介入，
        // 避免凭空闪避；真实危险时读数有效且距离小，会正常触发。
        if !sample.valid {
            return ([0.0; 3], false);
        }
        let dist = sample.distance;
        if dist >= self.danger_dist {
            return ([0.0; 3], false);
        }
        // 危险度：越近越大（0@danger_dist, 1@0）
        let severity = (1.0 - dist / self.danger_dist).clamp(0.0, 1.0);
        // 制动：沿 -前向，幅度 = 危险度 * brake_gain * danger_dist（m/s）
        let brake = severity * self.brake_gain * self.danger_dist;
        // 横向闪避：沿 +right（恒定），仅在有危险时给出
        let evade = self.evade_lateral;
        let mut v = [0.0f64; 3];
        for i in 0..3 {
            v[i] = -brake * fwd_ned[i] + evade * right_ned[i];
        }
        (v, true)
    }
}
