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

/// P3-B3 传感器故障注入（硬/软故障，作用于估计器观测链路）。
///
/// 故障在 [`SensorModel`] 层注入——即"物理真值 → 传感器读数"的转换处，因此会
/// 与噪声/偏置/延迟等真实化模型叠加后一起喂给估计器（EKF）与 FDIR，实现全链路
/// 故障注入（区别于直接改估计器内部状态）。
///
/// 三类故障（对应 P3-B3 目标"偏置突变 / 卡死 / 漂移"）：
/// - **软故障**（`*Bias` / `*Drift`）：读数被叠加偏置或随时间缓变偏置，传感器
///   仍"工作"，估计器/控制律需靠自身鲁棒性消化（EKF 零偏估计 / 控制器反馈）。
/// - **硬故障**（`*Stuck`）：输出冻结为固定值（卡死），FDIR 据此检测健康降级
///   （IMU 冻结 → `Health::Critical` → 失控保护归零执行器）。
#[derive(Clone, Debug)]
pub enum SensorFault {
    /// 软：加速度计偏置突变（机体系，叠加在真值上，m/s²）。
    AccelBias([f64; 3]),
    /// 软：陀螺偏置突变（机体系，rad/s）。
    GyroBias([f64; 3]),
    /// 软：加速度计偏置漂移率（m/s² per s，逐帧累积进输出偏置）。
    AccelDrift([f64; 3]),
    /// 软：陀螺偏置漂移率（rad/s per s）。
    GyroDrift([f64; 3]),
    /// 软：GPS 位置偏置（NED m）。
    GpsBias([f64; 3]),
    /// 硬：加速度计卡死（输出冻结为给定值，机体系；`None` 解除）。
    AccelStuck(Option<[f64; 3]>),
    /// 硬：陀螺卡死（输出冻结为给定值；`None` 解除）。
    GyroStuck(Option<[f64; 3]>),
    /// 硬：GPS 卡死（位置/速度输出冻结为给定 NED 值 [n,e,d,vn,ve,vd]；`None` 解除）。
    GpsStuck(Option<[f64; 6]>),
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
    // P3-B3 故障注入状态：
    // 软故障：注入偏置（叠加到读数上）与漂移率（每帧 bias_extra += drift_rate·dt）。
    accel_bias_extra: [f64; 3],
    gyro_bias_extra: [f64; 3],
    accel_drift_rate: [f64; 3],
    gyro_drift_rate: [f64; 3],
    gps_bias_extra: [f64; 3],
    // 硬故障：卡死（输出冻结为给定值）。
    accel_stuck: Option<[f64; 3]>,
    gyro_stuck: Option<[f64; 3]>,
    gps_stuck: Option<[f64; 6]>,
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
            accel_bias_extra: [0.0; 3],
            gyro_bias_extra: [0.0; 3],
            accel_drift_rate: [0.0; 3],
            gyro_drift_rate: [0.0; 3],
            gps_bias_extra: [0.0; 3],
            accel_stuck: None,
            gyro_stuck: None,
            gps_stuck: None,
        }
    }

    /// P3-B3：注入传感器故障（偏置突变 / 漂移 / 卡死）。
    ///
    /// 一次性生效：`AccelBias`/`GyroBias`/`GpsBias` 立即叠加指定偏置；
    /// `AccelDrift`/`GyroDrift` 设定漂移率，此后每帧按 `rate·dt` 累积进偏置
    /// （缓变漂移）；`AccelStuck`/`GyroStuck`/`GpsStuck` 立即冻结输出为给定值
    /// （`None` 解除卡死）。重复注入同类型故障会覆盖先前值。
    pub fn apply_fault(&mut self, fault: SensorFault) {
        match fault {
            SensorFault::AccelBias(b) => self.accel_bias_extra = b,
            SensorFault::GyroBias(b) => self.gyro_bias_extra = b,
            SensorFault::AccelDrift(r) => self.accel_drift_rate = r,
            SensorFault::GyroDrift(r) => self.gyro_drift_rate = r,
            SensorFault::GpsBias(b) => self.gps_bias_extra = b,
            SensorFault::AccelStuck(s) => self.accel_stuck = s,
            SensorFault::GyroStuck(s) => self.gyro_stuck = s,
            SensorFault::GpsStuck(s) => self.gps_stuck = s,
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

        // P3-B3 软故障：偏置漂移逐帧累积（漂移率 m/s² per s / rad/s per s）。
        for i in 0..3 {
            self.accel_bias_extra[i] += self.accel_drift_rate[i] * dt;
            self.gyro_bias_extra[i] += self.gyro_drift_rate[i] * dt;
        }

        // ---- IMU ----
        let mut acc = [0.0f64; 3];
        for i in 0..3 {
            // 振动：机体高频正弦 + 噪声
            let vib = self.cfg.vib_amp * (2.0 * std::f64::consts::PI * 40.0 * self.time + i as f64).sin();
            acc[i] = true_accel[i] as f64 + self.cfg.accel_bias[i] + self.accel_bias_extra[i] + vib
                + self.cfg.accel_noise * self.rng.gaussian();
        }
        // P3-B3 硬故障：加速度计卡死——输出冻结为给定值（覆盖一切噪声/偏置/真值）。
        if let Some(stuck) = self.accel_stuck {
            acc = stuck;
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
            gyr[i] = true_gyro[i] as f64 + self.cfg.gyro_bias[i] + self.gyro_bias_extra[i]
                + self.gyro_bias_walk[i] + self.gyro_bias_inst_state[i]
                + self.cfg.gyro_noise * self.rng.gaussian();
        }
        // P3-B3 硬故障：陀螺卡死——输出冻结为给定值。
        if let Some(stuck) = self.gyro_stuck {
            gyr = stuck;
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
        // 推入当前真值（带噪 + P3-B3 偏置/卡死）到延迟缓冲。丢星帧推入无效标记（NaN 位置）。
        let drop = self.rng.next_f64() < self.cfg.gps_drop_prob;
        if !drop {
            // P3-B3 软故障：GPS 位置偏置叠加（NED m）。
            let n = true_pos[0] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian() + self.gps_bias_extra[0];
            let e = true_pos[1] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian() + self.gps_bias_extra[1];
            let d = true_pos[2] as f64 + self.cfg.gps_pos_noise * self.rng.gaussian() + self.gps_bias_extra[2];
            let vn = true_vel[0] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            let ve = true_vel[1] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            let vd = true_vel[2] as f64 + self.cfg.gps_vel_noise * self.rng.gaussian();
            // P3-B3 硬故障：GPS 卡死——输出冻结为给定值（覆盖一切噪声/偏置/真值）。
            let pushed = match self.gps_stuck {
                Some(s) => s,
                None => [n, e, d, vn, ve, vd],
            };
            self.gps_delay.buf.push_back((pushed[0], pushed[1], pushed[2], pushed[3], pushed[4], pushed[5]));
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

/// 单条射线的读数（避障雷达 / 激光雷达 / 深度相机通用）。
///
/// `distance` 为沿 `dir_ned` 方向的最近障碍距离（m）；`valid` 标记读数是否可用
/// （false = 该方向传感器失效：量程饱和、或因障碍太近"糊脸"导致近距盲区的视觉/
/// 深度失效）。上层控制器应把 `valid=false` 视为"该方向不可信"而非"无障碍"。
#[derive(Clone, Debug)]
pub struct RayReading {
    /// 该射线在 NED 系的单位方向（从机体指向读数方向，前-右-下语义）。
    pub dir_ned: [f64; 3],
    /// 该方向最近障碍距离（m）。`valid=false` 时此值无意义（通常为 `max_range` 饱和值）。
    pub distance: f64,
    /// 读数是否有效（true=正常，false=失效/饱和）。
    pub valid: bool,
}

/// 距离传感器一帧多射线读数（扇形覆盖，P3-B2）。
///
/// 由机体沿**水平扇形**内 `ray_count` 条射线（对称分布在 `±fov_half` 内）同时
/// 探测障碍，返回全部射线读数。相比单射线：障碍横向滑出中心线后仍被侧向射线
/// 持续覆盖，避障可连续、自适应地朝"更开阔一侧"机动，无需 `hold_time` 硬补。
#[derive(Clone, Debug)]
pub struct RangeFinderFrame {
    /// 本帧全部射线读数（数量 = 扇形射线数，含失效/饱和）。
    pub rays: Vec<RayReading>,
}

impl RangeFinderFrame {
    /// 中央射线（最近正前方，即扇形中线）读数。
    pub fn center(&self) -> &RayReading {
        &self.rays[self.rays.len() / 2]
    }
}

/// 避障距离传感器模型（雷达 / 深度相机，扇形多射线）。
///
/// 把物理引擎射线求交得到的**真值距离**转成传感器读数：
/// - 加高斯测距噪声（标准差 `noise`）与固定偏置 `bias`；
/// - 超过 `max_range` 视为量程外（返回 `max_range` 饱和 + `valid=false`）；
/// - **近距盲区（视觉失效）**：当真值距离 < `blind_min` 时，回波淹没/相机糊脸，
///   深度/视觉通道给出错误饱和读数（`distance=max_range`、`valid=false`）——典型
///   "障碍太近反而看不到"的失效模式（雷达近距多路径 / 深度相机近距离退化）；
/// - 可选随机失效 `drop_prob`：每帧以该概率直接 `valid=false`（模拟瞬断/遮挡）；
/// - **扇形多射线**（P3-B2）：在机体水平面内沿 `±fov_half` 对称分布 `ray_count`
///   条射线（`fov_half` 为单侧半视场角），近似雷达/光流/深度相机的广角覆盖，
///   每条射线独立采样（独立噪声/失效），方向见 [`RangeFinderModel::ray_dirs_body`]。
#[derive(Clone, Debug)]
pub struct RangeFinderModel {
    pub max_range: f64,   // 最大量程 m（超距饱和）
    pub blind_min: f64,   // 近距盲区 m（< 此值判失效）
    pub noise: f64,       // 测距高斯噪声 std m
    pub bias: f64,        // 固定测距偏置 m
    pub drop_prob: f64,   // 每帧随机失效概率
    pub fov_half: f64,    // 单侧水平半视场角 rad（扇形半宽）
    pub ray_count: usize, // 扇形射线数量（奇数时含正前方中央射线）
    rng: Lcg,
}

impl RangeFinderModel {
    pub fn new(
        max_range: f64,
        blind_min: f64,
        noise: f64,
        bias: f64,
        drop_prob: f64,
        fov_half: f64,
        ray_count: usize,
        seed: u64,
    ) -> Self {
        Self {
            max_range,
            blind_min,
            noise,
            bias,
            drop_prob,
            fov_half,
            ray_count,
            rng: Lcg::new(seed ^ 0x5EED_F1D3),
        }
    }

    /// 扇形射线方向（机体坐标系，前-右-下语义，水平面内）。
    ///
    /// 返回 `ray_count` 条方向，在机体水平面内于 `±fov_half` 间**等角分布**：
    /// `ray_count=1` 时仅正前方（[-1,0,0]），退化为单射线；`ray_count>=2` 时首尾
    /// 分别为右/左极限角（+fov_half / -fov_half），奇数条时含正前方中央射线。
    /// 射线近似雷达/光流/深度相机的广角覆盖（P3-B2）。
    pub fn ray_dirs_body(&self) -> Vec<[f64; 3]> {
        let n = self.ray_count.max(1);
        (0..n)
            .map(|i| {
                let a = if n <= 1 {
                    0.0
                } else {
                    // i=0 → +fov_half（右侧极限），i=n-1 → -fov_half（左侧极限）
                    self.fov_half - 2.0 * self.fov_half * (i as f64) / ((n - 1) as f64)
                };
                // 机体水平面：前 -X、右 +Y；绕下轴（+Z）偏转 a。
                [-a.cos(), a.sin(), 0.0]
            })
            .collect()
    }

    /// 由射线求交真值距离生成一条射线读数（P3-B2）。
    ///
    /// `dir_ned`：该射线在 NED 系的单位方向（由机体坐标系射线方向旋转而来）；
    /// `true_distance`：射线命中障碍的真值距离（m）；`None` 表示射程内无命中
    /// （量程外）。返回带方向的 `RayReading`。
    pub fn sample_ray(&mut self, dir_ned: [f64; 3], true_distance: Option<f64>) -> RayReading {
        // 随机瞬断
        if self.rng.next_f64() < self.drop_prob {
            return RayReading {
                dir_ned,
                distance: self.max_range,
                valid: false,
            };
        }
        let d = match true_distance {
            None => {
                // 量程外：饱和读数 + 失效标记（"看不到"≠"无障碍"）
                return RayReading {
                    dir_ned,
                    distance: self.max_range,
                    valid: false,
                };
            }
            Some(d) => d,
        };
        // 近距盲区：障碍太近，视觉/深度通道糊脸失效
        if d < self.blind_min {
            return RayReading {
                dir_ned,
                distance: self.max_range, // 错误饱和（把近障碍误报为"远处/无障碍"）
                valid: false,
            };
        }
        // 正常：加偏置 + 噪声，裁剪到 [0, max_range]
        let measured = (d + self.bias + self.noise * self.rng.gaussian()).clamp(0.0, self.max_range);
        RayReading {
            dir_ned,
            distance: measured,
            valid: true,
        }
    }
}

// ============================================================ 避障控制器（P1-2 闭环联动）

/// 反应式避障控制器配置（感知→决策→规避闭环）。
///
/// 当测距传感器检测到前方障碍进入 `danger_dist` 内时，
/// 控制器分两层介入（详见 [`AvoidanceConfig::avoidance_velocity`]）：
/// 1. **制动**：沿机体前向施加与危险度成正比的减速（把前向速度指令压向 0/反向）。
/// 2. **横向闪避**：沿机体侧向施加一个横向速度指令，朝"更开阔一侧"机动。
///
/// `danger_dist` 应小于 `RangeFinderModel::max_range`，否则全量程都触发闪避。
///
/// **多射线聚合（P3-B2，替代单射线 + `hold_time` 硬补）**：输入是扇形多射线帧
/// （[`RangeFinderFrame`]）。决策把所有**有效且进入危险距离**的射线作为"排斥源"
/// 聚合：排斥力沿 `-射线方向`（远离该障碍）、强度随危险度线性增大。求和后：
/// - 制动分量沿 -前向（把前向指令压向 0）；
/// - 横向分量由排斥力在机体侧向的投影决定——障碍在右 → 向左闪，障碍在左 → 向右闪，
///   障碍居中对称（投影≈0）时退化为固定向右（与旧单射线语义一致）。
///
/// 相比单射线：障碍横向滑出中央射线后，侧向射线仍持续覆盖，闪避方向随障碍横移
/// 连续翻转、危险度随距离连续衰减，无需"FOV 丢失后冻结指令 N 秒"的 `hold_time`
/// 硬补——障碍彻底离开视场（全射线距离回升）后避障自然释放，位置环再接管回航。
#[derive(Clone, Debug)]
pub struct AvoidanceConfig {
    /// 危险触发距离（m）：射线读数小于此值即进入规避。
    pub danger_dist: f64,
    /// 制动强度（m/s 每米）：危险度 = clamp(1 - dist/danger_dist, 0, 1)，
    /// 前向期望速度 = 原前向速度 - 最大危险度 * brake_gain * danger_dist。
    pub brake_gain: f64,
    /// 横向闪避速度指令（m/s，机体侧向 +Y，即右翼方向）的上限值；
    /// 实际幅度 = 上限 × sqrt(最大危险度)（非线性：进入危险距离即尽快坚定侧移，
    /// 非等到贴脸才全力），方向由聚合结果决定。
    pub evade_lateral: f64,
}

impl AvoidanceConfig {
    pub fn new(danger_dist: f64, brake_gain: f64, evade_lateral: f64) -> Self {
        Self { danger_dist, brake_gain, evade_lateral }
    }

    /// 计算规避速度指令（NED 系，单位 m/s）。
    ///
    /// - `frame`：`plant.read_ranger()` 当前多射线帧。无效读数（瞬断/近距盲区/
    ///   量程外饱和）不参与聚合（"看不到"≠"无障碍"，但也不凭空闪避）；有效且距离
    ///   小于 `danger_dist` 的射线才作为排斥源。
    /// - `fwd_ned`：机体前向在世界 NED 系的单位向量（机体 -X → NED）。
    /// - `right_ned`：机体右向在世界 NED 系的单位向量（机体 +Y → NED）。
    ///
    /// 返回 `(v_ned: [f64;3], triggered: bool, lateral_comp: f64)`，`triggered` 表示本轮是否
    /// 真的介入；`lateral_comp` 是**原始排斥力**在机体侧向的投影（叠加危险度权重后），
    /// **未乘**闪避幅度/方向——居中障碍时 ≈0（其符号被噪声主导），是控制器做方向锁存
    /// 的可靠依据（幅度量级远小于 `v_ned` 的横向分量，两者不能混用）。调用方把返回的
    /// 速度指令叠加/并入原有速度设定点。
    pub fn avoidance_velocity(
        &self,
        frame: &RangeFinderFrame,
        fwd_ned: [f64; 3],
        right_ned: [f64; 3],
    ) -> ([f64; 3], bool, f64) {
        // 聚合所有有效且进入危险距离的射线排斥力（沿 -dir，强度随危险度）。
        let mut rep = [0.0f64; 3];
        let mut max_sev = 0.0f64;
        let mut any_danger = false;
        for ray in &frame.rays {
            if !ray.valid || ray.distance >= self.danger_dist {
                continue;
            }
            let sev = (1.0 - ray.distance / self.danger_dist).clamp(0.0, 1.0);
            max_sev = max_sev.max(sev);
            for i in 0..3 {
                rep[i] += sev * (-ray.dir_ned[i]);
            }
            any_danger = true;
        }
        if !any_danger {
            return ([0.0; 3], false, 0.0);
        }
        // 制动：沿 -前向，幅度按最大危险度（贴脸最急，远离渐缓）。
        let brake = max_sev * self.brake_gain * self.danger_dist;
        // 横向闪避：**非线性危险度**（sqrt）。障碍进入危险距离即尽快坚定侧移（而非
        // 线性等到贴脸才全力闪避）——机体侧向速度受气动阻力/姿态倾角限幅（实测封顶
        // ~1.4 m/s），若闪避幅度随距离线性萎缩，远距障碍只触发极弱侧移，逼近后期才
        // 全力，机体来不及脱离 3m+ 半径球（P3-B2 正对逼近 MIN_CLEAR=0.28 根因）。
        // 制动仍用线性 max_sev（越近刹得越急），方向由聚合结果决定（不变）。
        let evade_eff = max_sev.sqrt();
        let lateral_comp = rep[0] * right_ned[0] + rep[1] * right_ned[1] + rep[2] * right_ned[2];
        let evade_dir = if lateral_comp < 0.0 { -1.0 } else { 1.0 };
        let evade = self.evade_lateral * evade_eff * evade_dir;
        let mut v = [0.0f64; 3];
        for i in 0..3 {
            v[i] = -brake * fwd_ned[i] + evade * right_ned[i];
        }
        (v, true, lateral_comp)
    }
}
