//! 机动库：数据驱动的真值轨迹——姿态解算测试的**真值来源**。
//!
//! # 为什么是"规定轨迹"
//!
//! 评价姿态估计器要算 RMSE，真值必须**已知且不受控制器/物理闭环影响**。
//! 所以这里不使用物理闭环去"飞"出姿态，而是**直接规定** `att(t)` 与
//! `accel_world(t)`，再按 IMU 的真实定义导出比力：
//!
//! ```text
//! a_meas_body = R(q)ᵀ · (accel_world − g_world)      g_world = [0, 0, 9.81] (NED)
//! ```
//!
//! # 为什么姿态与加速度要**各自独立**给定（重要）
//!
//! 若从"推力模型"反推 `accel_world = thrust·g·R·[0,0,−1] + g`，会得到
//! `a_meas_body ≡ [0, 0, −thrust·g]` —— 比力**恒沿机体 −z**，加速度计退化成
//! 不含任何姿态信息，重力锚定测试随即失去意义。
//!
//! 真实飞行中 `accel_world` 由**推力 + 气动阻力 + 重力**共同决定，横向机动时
//! `a_meas_body` 会偏离 `−z`（这正是"比力 ≠ 重力"、方向门要拦截的东西）。
//! 因此本库把两者作为**独立输入**建模，与 `EnvScenario::write_state` 的
//! `a_body = R_wb·(a_world − g)` 完全一致。
//!
//! # 约定（与 `flyctrl_core::vehicle::Quaternion` 严格一致）
//!
//! - `quat`：**机体→世界**（ZYX：`R = Rz(yaw)·Ry(pitch)·Rx(roll)`），`[w,x,y,z]`
//! - `omega_body`：机体系角速度 `[p,q,r]`（rad/s）
//! - `pos_ned` / `vel_ned` / `accel_world`：NED（x=北, y=东, z=下）
//!
//! # 数据驱动
//!
//! 一切轨迹都收敛到同一个 [`Trajectory`]（采样表 + 插值），所以
//! **合成机动**与**以后录制的实飞轨迹**走同一条通路（[`Trajectory::to_csv`] /
//! [`Trajectory::from_csv`]）。注意：实飞日志若无姿态真值，只能做"合理性回放"，
//! 不能用于 RMSE——RMSE 必须来自规定的合成轨迹。

use flyctrl_core::units::Radian;
use flyctrl_core::vehicle::{rotate_vec_by_quat_inverse, Quaternion};

/// 重力（NED，z 向下为正）。NED 下重力方向为 +z。
pub const G_NED: [f32; 3] = [0.0, 0.0, 9.81];

/// 真值轨迹的一个采样点。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrajSample {
    /// 时间（s，相对轨迹起点）
    pub t: f32,
    /// 位置 NED (m)
    pub pos_ned: [f32; 3],
    /// 速度 NED (m/s)
    pub vel_ned: [f32; 3],
    /// 世界系加速度 NED (m/s²)，**不含重力**
    pub accel_world: [f32; 3],
    /// 姿态四元数 机体→世界 `[w,x,y,z]`
    pub quat: [f32; 4],
    /// 机体角速度 `[p,q,r]` (rad/s)
    pub omega_body: [f32; 3],
}

impl TrajSample {
    /// 加速度计应当测到的比力（机体系 FRD，m/s²）。
    ///
    /// `a_body = Rᵀ·(accel_world − g)`。悬停（`accel_world=0`、水平姿态）时为
    /// `[0,0,−9.81]`；自由落体（`accel_world=g`）时为 `0`。
    pub fn specific_force_body(&self) -> [f32; 3] {
        let aw = [
            self.accel_world[0] - G_NED[0],
            self.accel_world[1] - G_NED[1],
            self.accel_world[2] - G_NED[2],
        ];
        rotate_vec_by_quat_inverse(self.quat_obj(), aw)
    }

    /// 比力幅值与重力之比 `|a|/g`——EKF 幅值门控的输入量。
    pub fn specific_force_ratio(&self) -> f32 {
        let a = self.specific_force_body();
        (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt() / 9.81
    }

    pub fn quat_obj(&self) -> Quaternion {
        Quaternion {
            w: self.quat[0],
            x: self.quat[1],
            y: self.quat[2],
            z: self.quat[3],
        }
    }

    /// ZYX 欧拉角 `[roll, pitch, yaw]`（rad）。
    pub fn euler(&self) -> [f32; 3] {
        let q = self.quat_obj();
        [q.roll(), q.pitch(), q.yaw()]
    }
}

/// 一条真值轨迹：等间隔采样表 + 线性/球面插值。
#[derive(Clone, Debug)]
pub struct Trajectory {
    name: String,
    rate_hz: f32,
    samples: Vec<TrajSample>,
}

/// 机动参数化：`eval(t) -> (ZYX 欧拉角, 世界系加速度 NED)`。
///
/// 姿态与加速度**独立给定**（见模块文档"为什么各自独立"）。
/// 机体角速度由欧拉角序列数值微分导出，保证与姿态自洽。
#[derive(Clone, Copy, Debug)]
pub enum Maneuver {
    /// A1 悬停微扰：小幅多频摆动，`|ω|` 远低于陀螺门（34°/s）。
    HoverMicro { amp_deg: f32 },
    /// A2 自稳巡航：倾角斜坡到 `tilt_deg` 后**保持 `hold_s` 秒**（准静态，`accel_world≈0`）。
    /// 倾角小于方向门阈值（25.8°）时锚定应保持开启。
    Cruise {
        tilt_deg: f32,
        ramp_s: f32,
        hold_s: f32,
    },
    /// A3 快速前飞 + 急刹：倾角反向 + 大 `|a_world|`。
    /// **最可能打穿方向门**——`a_world` 的平移分量污染重力方向。
    BrakeReversal { accel_g: f32, tilt_deg: f32, hold_s: f32 },
    /// A4 协调转弯：滚转 `bank_deg` + 航向速率 `rate_dps`，向心加速度 `g·tanφ`。
    /// `|ω|=ψ̇` 通常 > 34°/s → 陀螺门关闭，走纯陀螺积分。
    CoordinatedTurn { bank_deg: f32, rate_dps: f32 },
    /// A5 甩尾急转：大偏航速率，水平姿态。
    YawWhip { rate_dps: f32 },
    /// A6 360° 横滚：连续过倒飞。`thrust_ratio` 决定掉高（`<1` 时 `a_world` 向下）。
    Roll360 { rate_dps: f32, thrust_ratio: f32 },
    /// A7 连续自旋 + 平移：多圈偏航叠加水平加速度。
    SpinTranslate { yaw_dps: f32, accel_g: f32 },
    /// A8 强湍流：带限随机姿态摆动（确定性种子，可复现）。
    Turbulence { rms_deg: f32, band_hz: f32, seed: u64 },
    /// A9 自由落体 / 抛飞：`accel_world = g` → 比力 **0**，幅值门必须关闭。
    FreeFall { jitter_deg: f32 },
    /// A10 下降桨流 / 涡环：匀速下降 + 30Hz 级姿态抖动（测陀螺 40Hz 陷波）。
    PropwashDescent { descent_mps: f32, jitter_deg: f32 },
    /// A11 降落冲击：向上 2.5g 量级减速尖峰，比力幅值远超 2.5g → 幅值门必须关闭。
    LandingImpact { peak_g: f32 },
    /// A12 磁干扰下的机动：偏航扫掠（磁故障由 `SensorModel::apply_fault` 另加）。
    MagDisturbSweep { rate_dps: f32, bias_gauss: f32 },
    /// A13 陀螺饱和边界：逼近量程的恒定角速率。
    GyroSatBoundary { rate_dps: f32 },
    /// 单轴正弦（相位/幅频特性测量用）：`axis` 0=roll 1=pitch 2=yaw。
    /// 水平摆动、`accel_world=0`，所以比力是**无污染的重力参考**。
    AttitudeSine {
        axis: usize,
        amp_deg: f32,
        freq_hz: f32,
        duration_s: f32,
    },
}

/// 度 → 弧度。
#[inline]
fn d2r(d: f32) -> f32 {
    d * core::f32::consts::PI / 180.0
}

/// 平滑阶跃 0→1（三次 smoothstep）。
#[inline]
fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// 相位 `u` 在 `[a,b]` 内的平滑 0→1。
#[inline]
fn ramp_between(t: f32, a: f32, b: f32) -> f32 {
    if b <= a {
        return if t >= a { 1.0 } else { 0.0 };
    }
    smoothstep((t - a) / (b - a))
}

/// 确定性 xorshift64（湍流/抖动用，保证同 seed 同轨迹）。
struct Xorshift64(u64);

impl Xorshift64 {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    /// 均匀 `[-1, 1]`
    fn next_sym(&mut self) -> f32 {
        let v = (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32; // [0,1)
        v * 2.0 - 1.0
    }
}

impl Maneuver {
    /// 轨迹时长（s）。
    pub fn duration(&self) -> f32 {
        match *self {
            Maneuver::HoverMicro { .. } => 10.0,
            Maneuver::Cruise { ramp_s, hold_s, .. } => ramp_s * 2.0 + hold_s,
            Maneuver::BrakeReversal { hold_s, .. } => hold_s,
            Maneuver::CoordinatedTurn { rate_dps, .. } => (720.0 / rate_dps.abs().max(1.0)) * 2.0,
            Maneuver::YawWhip { rate_dps } => (720.0 / rate_dps.abs().max(1.0)) * 2.0,
            Maneuver::Roll360 { rate_dps, .. } => (720.0 / rate_dps.abs().max(1.0)) * 2.0,
            Maneuver::SpinTranslate { yaw_dps, .. } => (720.0 / yaw_dps.abs().max(1.0)) * 2.0,
            Maneuver::Turbulence { .. } => 15.0,
            Maneuver::FreeFall { .. } => 6.0,
            Maneuver::PropwashDescent { .. } => 8.0,
            Maneuver::LandingImpact { .. } => 5.0,
            Maneuver::MagDisturbSweep { rate_dps, .. } => 360.0 / rate_dps.abs().max(1.0) + 4.0,
            Maneuver::GyroSatBoundary { .. } => 6.0,
            Maneuver::AttitudeSine { duration_s, .. } => duration_s,
        }
    }

    /// 名称（用于报告/落盘）。
    pub fn title(&self) -> &'static str {
        match self {
            Maneuver::HoverMicro { .. } => "A1_hover_micro",
            Maneuver::Cruise { .. } => "A2_cruise",
            Maneuver::BrakeReversal { .. } => "A3_brake_reversal",
            Maneuver::CoordinatedTurn { .. } => "A4_coordinated_turn",
            Maneuver::YawWhip { .. } => "A5_yaw_whip",
            Maneuver::Roll360 { .. } => "A6_roll360",
            Maneuver::SpinTranslate { .. } => "A7_spin_translate",
            Maneuver::Turbulence { .. } => "A8_turbulence",
            Maneuver::FreeFall { .. } => "A9_free_fall",
            Maneuver::PropwashDescent { .. } => "A10_propwash_descent",
            Maneuver::LandingImpact { .. } => "A11_landing_impact",
            Maneuver::MagDisturbSweep { .. } => "A12_mag_disturb_sweep",
            Maneuver::GyroSatBoundary { .. } => "A13_gyro_sat_boundary",
            Maneuver::AttitudeSine { .. } => "sine_single_axis",
        }
    }

    /// 稳态初速度（NED，m/s）——积分位置/速度的起点。
    ///
    /// 只对"以恒定速度进行"的机动非零（如匀速下降）。注意：对
    /// "倾角保持但 `accel_world=0`"（阻力平衡）的机动，真实稳态速度由气动
    /// 阻力决定，本库不含阻力模型，故记为 0 —— **位置/速度真值在阶段 1
    /// （姿态）不使用**；阶段 3（位置）需要另行接入带阻力的平动模型。
    pub fn initial_vel(&self) -> [f32; 3] {
        match *self {
            Maneuver::PropwashDescent { descent_mps, .. } => [0.0, 0.0, descent_mps],
            _ => [0.0; 3],
        }
    }

    /// 采样：返回 `(ZYX 欧拉角 [roll,pitch,yaw], 世界系加速度 NED)`。
    ///
    /// `st`：机动内部状态（长度 6；三轴×两级）。目前只有 `Turbulence` 用。
    ///
    /// 注意：随机机动内部状态依赖**按时间顺序**调用，
    /// 因此 [`Trajectory::from_maneuver`] 保证从 `t=0` 起顺序采样。
    fn eval(&self, t: f32, rng: &mut Xorshift64, st: &mut [f32; 6]) -> ([f32; 3], [f32; 3]) {
        match *self {
            // A1 悬停微扰：多频小幅，|ω| ≈ amp·2πf ≈ 3°·3.14 ≈ 9°/s < 34°/s（门开）
            Maneuver::HoverMicro { amp_deg } => {
                let a = d2r(amp_deg);
                let e = [
                    a * (2.0 * core::f32::consts::PI * 0.5 * t).sin(),
                    a * (2.0 * core::f32::consts::PI * 0.7 * t + 1.0).sin(),
                    a * 0.5 * (2.0 * core::f32::consts::PI * 0.3 * t).sin(),
                ];
                (e, [0.0; 3])
            }
            // A2 自稳巡航：倾角斜坡 → 长保持 → 回平。accel_world≈0（阻力平衡）。
            Maneuver::Cruise {
                tilt_deg,
                ramp_s,
                hold_s,
            } => {
                let up = ramp_between(t, 0.0, ramp_s);
                let down = ramp_between(t, ramp_s + hold_s, ramp_s * 2.0 + hold_s);
                let pitch = d2r(tilt_deg) * (up - down);
                ([0.0, pitch, 0.0], [0.0; 3])
            }
            // A3 快速前飞 + 急刹：倾角反向 + 大 |a_world|。
            // 减速段 accel_world = [−accel_g·g, 0, 0]（NED x=北）。
            Maneuver::BrakeReversal {
                accel_g,
                tilt_deg,
                hold_s,
            } => {
                let third = hold_s / 3.0;
                // 加速 → 减速 → 回平
                let acc = ramp_between(t, 0.0, third);
                let dec = ramp_between(t, third, third * 2.0);
                let a_g = accel_g * 9.81;
                // 阶段 1：前倾加速（NED +x）；阶段 2：后仰减速（NED −x）
                let tilt = d2r(tilt_deg);
                let pitch = tilt * acc - tilt * dec;
                let an = a_g * acc - a_g * dec;
                ([0.0, pitch, 0.0], [an, 0.0, 0.0])
            }
            // A4 协调转弯：φ 保持，ψ = ψ̇·t。向心加速度 a = g·tanφ（东向 +y）。
            // 机体角速度（ZYX 微分）：[0, ψ̇·sinφ, ψ̇·cosφ]，|ω| = ψ̇。
            Maneuver::CoordinatedTurn { bank_deg, rate_dps } => {
                let phi = d2r(bank_deg);
                let w = d2r(rate_dps);
                let t_roll = 1.0;
                let bank = phi * smoothstep(t / t_roll);
                let bank_out = phi * smoothstep((t - (self.duration() - t_roll)) / t_roll);
                let phi_t = bank - bank_out;
                let yaw = w * t;
                let ac = 9.81 * phi_t.tan();
                ([phi_t, 0.0, yaw], [0.0, ac, 0.0])
            }
            // A5 甩尾急转：水平姿态，大偏航速率。
            Maneuver::YawWhip { rate_dps } => {
                let w = d2r(rate_dps);
                ([0.0, 0.0, w * t], [0.0; 3])
            }
            // A6 360° 横滚：roll = p·t 连续过倒飞。thrust<1 → a_world 向下不足。
            // a_world_z = g·(1 − thrust_ratio)（推力不足的自由落体分量）。
            Maneuver::Roll360 {
                rate_dps,
                thrust_ratio,
            } => {
                let p = d2r(rate_dps);
                let roll = p * t;
                let az = 9.81 * (1.0 - thrust_ratio.clamp(0.0, 2.0));
                ([roll, 0.0, 0.0], [0.0, 0.0, az])
            }
            // A7 连续自旋 + 平移：偏航连续转 + 水平加速度。
            Maneuver::SpinTranslate { yaw_dps, accel_g } => {
                let w = d2r(yaw_dps);
                let a = accel_g * 9.81;
                let g_on = ramp_between(t, 0.5, 1.5);
                ([0.0, 0.0, w * t], [a * g_on, 0.0, 0.0])
            }
            // A8 强湍流：**两级低通**白噪声（确定性种子）。
            //
            // 为什举要两级（重要，曾是真 bug）：`from_maneuver` 用**欧拉角差分**得到
            // 机体角速率 ω。一级低通驱动的白噪声，其 _导数_ 的频谱会因微分而把低通的
            // −20dB/dec 抵消掉，一直延伸到 Nyquist ⇒ 得到的 ω 与光滑的 q **不自洽**，
            // EKF 按 4ms 采样那个抖动 ω 积分就会走出不同姿态。
            // 实测（2026-09-21，`a8_turbulence_bounded_and_reproducible`）：
            //   rms=10°/band=3Hz → max|ω|=773°/s、RMSE **130.9°**（发散）；
            //   而同 max|ω| 的**光滑正弦**（1513°/s）只有 **1.9°**。
            // 两级低通后 ė 自身的频谱在 Nyquist 前已显著衰减 ⇒ 不再混叠。
            Maneuver::Turbulence {
                rms_deg,
                band_hz,
                ..
            } => {
                let a = d2r(rms_deg);
                // 一级低通决定带宽
                let alpha = (2.0 * core::f32::consts::PI * band_hz * 0.001).min(1.0);
                for i in 0..3 {
                    st[i] += alpha * (rng.next_sym() * a * 6.0 - st[i]);
                }
                // 二级低通 → 姿态（使 ė 也有界）
                for i in 0..3 {
                    st[3 + i] += alpha * (st[i] - st[3 + i]);
                }
                ([st[3], st[4], st[5] * 0.3], [0.0; 3])
            }
            // A9 自由落体：accel_world = g → 比力 = 0。
            Maneuver::FreeFall { jitter_deg } => {
                let a = d2r(jitter_deg);
                let e = [
                    a * (2.0 * core::f32::consts::PI * 1.0 * t).sin(),
                    a * (2.0 * core::f32::consts::PI * 1.7 * t).sin(),
                    0.0,
                ];
                (e, G_NED)
            }
            // A10 下降桨流：匀速下降（accel_world≈0）+ 30Hz 姿态抖动。
            // 30Hz 落在固件陀螺 40Hz 陷波带附近，测陷波有效性。
            Maneuver::PropwashDescent {
                descent_mps,
                jitter_deg,
            } => {
                let a = d2r(jitter_deg);
                let f = 30.0;
                let e = [
                    a * (2.0 * core::f32::consts::PI * f * t).sin(),
                    a * (2.0 * core::f32::consts::PI * f * t + 1.0).sin(),
                    0.0,
                ];
                let _ = descent_mps;
                (e, [0.0; 3])
            }
            // A11 降落冲击：向上减速尖峰 accel_world = [0,0,−peak_g·g]（NED 向上为负）。
            Maneuver::LandingImpact { peak_g } => {
                // 1.0s 处 0.15s 宽的尖峰
                let w = 0.15;
                let c = 1.0;
                let spike = if (t - c).abs() < w {
                    0.5 * (1.0 + ((t - c) / w * core::f32::consts::PI).cos())
                } else {
                    0.0
                };
                ([0.0, 0.0, 0.0], [0.0, 0.0, -peak_g * 9.81 * spike])
            }
            // A12 磁干扰下的机动：偏航扫掠（磁故障由 SensorModel 另加）。
            Maneuver::MagDisturbSweep { rate_dps, .. } => {
                let w = d2r(rate_dps);
                ([0.0, 0.0, w * t], [0.0; 3])
            }
            // A13 陀螺饱和边界：恒定角速率（绕机体 x），水平姿态。
            Maneuver::GyroSatBoundary { rate_dps } => {
                let p = d2r(rate_dps);
                ([p * t, 0.0, 0.0], [0.0; 3])
            }
            // 单轴正弦（相位/幅频特性测量）。
            Maneuver::AttitudeSine {
                axis,
                amp_deg,
                freq_hz,
                ..
            } => {
                let a = d2r(amp_deg);
                let w = 2.0 * core::f32::consts::PI * freq_hz;
                let mut e = [0.0f32; 3];
                e[axis.min(2)] = a * (w * t).sin();
                (e, [0.0; 3])
            }
        }
    }
}

impl Trajectory {
    /// 采样率（Hz）——真值表的分辨率（与仿真步长无关，插值后使用）。
    pub const DEFAULT_RATE_HZ: f32 = 1000.0;

    /// 由机动生成轨迹：先按 1kHz 采样 `(欧拉, accel_world)` 序列，
    /// 再数值微分欧拉角得到机体角速度（保证与姿态自洽），
    /// 最后积分得到位置/速度。
    pub fn from_maneuver(m: &Maneuver, rate_hz: f32) -> Self {
        Self::from_maneuver_dur(m, rate_hz, m.duration())
    }

    /// 同 [`from_maneuver`]，但**指定总时长**（可远长于机动的自然周期）。
    ///
    /// 语义：`t` **不做 wrap**，直接从 0 连续采样到 `total_s`，即直接继续调用
    /// `m.eval(t)`。因此：
    /// - **周期类**（`HoverMicro`/`AttitudeSine` 的正弦）：自然继续振荡 ✓
    /// - **连续旋转类**（`Roll360`/`YawWhip`/`SpinTranslate`/`GyroSatBoundary`）：
    ///   转更多圈（`roll = p·t` 本身无界）✓
    /// - **斜坡类**（`Cruise`/`BrakeReversal`/`CoordinatedTurn`）：走完自己的
    ///   建立/退出过程后**保持末端状态** ✓
    ///
    /// 用途：**持续性测试**。单次 1~2s 的机动看不出累积误差、收敛过程、陀螺漂移、
    /// 门控反复开关的长期后果——这些恰恰是姿态解算真正会翻车的地方。
    ///
    /// ⚠️ 不要用“循环播放”（把 `t` 对 `duration()` 取模）来延长：对
    /// `Roll360`/`GyroSatBoundary` 这类会在接缝处**造出虚假的真值跳变**。
    pub fn from_maneuver_dur(m: &Maneuver, rate_hz: f32, total_s: f32) -> Self {
        let rate_hz = if rate_hz > 0.0 {
            rate_hz
        } else {
            Self::DEFAULT_RATE_HZ
        };
        let dt = 1.0 / rate_hz;
        let dur = if total_s.is_finite() && total_s > 0.0 {
            total_s
        } else {
            m.duration()
        };
        let n = (dur / dt).round() as usize + 1;

        // 1) 顺序采样欧拉角 + 加速度（湍流等随机机动依赖顺序）
        let mut eulers = Vec::with_capacity(n);
        let mut accels = Vec::with_capacity(n);
        let mut rng = Xorshift64::new(match *m {
            Maneuver::Turbulence { seed, .. } => seed,
            _ => 0x1234_5678_9ABC_DEF0,
        });
        let mut lp = [0.0f32; 6];
        for i in 0..n {
            let t = i as f32 * dt;
            let (e, a) = m.eval(t, &mut rng, &mut lp);
            eulers.push(e);
            accels.push(a);
        }

        // 2) 欧拉角数值微分 → 机体角速度（ZYX 公式）
        //    p = φ̇ − ψ̇·sinθ
        //    q = θ̇·cosφ + ψ̇·cosθ·sinφ
        //    r = −θ̇·sinφ + ψ̇·cosθ·cosφ
        let mut samples = Vec::with_capacity(n);
        let mut vel = m.initial_vel();
        let mut pos = [0.0f32; 3];
        for i in 0..n {
            let t = i as f32 * dt;
            let e = eulers[i];
            let e_prev = eulers[i.saturating_sub(1)];
            let e_next = eulers[(i + 1).min(n - 1)];
            let h = if i == 0 || i == n - 1 {
                dt
            } else {
                2.0 * dt
            };
            let d = [
                (e_next[0] - e_prev[0]) / h,
                (e_next[1] - e_prev[1]) / h,
                (e_next[2] - e_prev[2]) / h,
            ];
            let (sr, cr) = e[0].sin_cos();
            let (sp, cp) = e[1].sin_cos();
            let omega = [
                d[0] - d[2] * sp,
                d[1] * cr + d[2] * cp * sr,
                -d[1] * sr + d[2] * cp * cr,
            ];
            let quat = Quaternion::from_euler(Radian(e[0]), Radian(e[1]), Radian(e[2]));
            let aw = accels[i];
            if i > 0 {
                let aw_prev = accels[i - 1];
                for k in 0..3 {
                    vel[k] += 0.5 * (aw[k] + aw_prev[k]) * dt;
                    pos[k] += vel[k] * dt;
                }
            }
            samples.push(TrajSample {
                t,
                pos_ned: pos,
                vel_ned: vel,
                accel_world: aw,
                quat: [quat.w, quat.x, quat.y, quat.z],
                omega_body: omega,
            });
        }

        Self {
            name: m.title().to_string(),
            rate_hz,
            samples,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn rate_hz(&self) -> f32 {
        self.rate_hz
    }
    pub fn samples(&self) -> &[TrajSample] {
        &self.samples
    }
    pub fn len(&self) -> usize {
        self.samples.len()
    }
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
    pub fn duration(&self) -> f32 {
        self.samples.last().map(|s| s.t).unwrap_or(0.0)
    }

    /// 在时刻 `t` 插值求真值。
    ///
    /// - 向量量（pos/vel/accel/omega）线性插值；
    /// - 姿态四元数**球面插值**（slerp，含半球对齐，避免 q/−q 跳变）。
    ///
    /// `t` 超出范围时钳到端点。
    pub fn at(&self, t: f32) -> TrajSample {
        if self.samples.is_empty() {
            return TrajSample {
                t,
                pos_ned: [0.0; 3],
                vel_ned: [0.0; 3],
                accel_world: [0.0; 3],
                quat: [1.0, 0.0, 0.0, 0.0],
                omega_body: [0.0; 3],
            };
        }
        if t <= 0.0 {
            return self.samples[0];
        }
        let dur = self.duration();
        if t >= dur {
            return *self.samples.last().unwrap();
        }
        let x = t * self.rate_hz;
        let i = x.floor() as usize;
        let i = i.min(self.samples.len().saturating_sub(2));
        let f = (x - i as f32).clamp(0.0, 1.0);
        let a = self.samples[i];
        let b = self.samples[i + 1];
        let lerp3 = |p: [f32; 3], q: [f32; 3]| {
            [
                p[0] + (q[0] - p[0]) * f,
                p[1] + (q[1] - p[1]) * f,
                p[2] + (q[2] - p[2]) * f,
            ]
        };
        TrajSample {
            t,
            pos_ned: lerp3(a.pos_ned, b.pos_ned),
            vel_ned: lerp3(a.vel_ned, b.vel_ned),
            accel_world: lerp3(a.accel_world, b.accel_world),
            quat: slerp(a.quat, b.quat, f),
            omega_body: lerp3(a.omega_body, b.omega_body),
        }
    }

    /// 导出 CSV（数据驱动：可存档、可用真实日志替换）。
    pub fn to_csv(&self) -> String {
        let mut s = String::from(
            "t,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,acc_n,acc_e,acc_d,qw,qx,qy,qz,p,q,r\n",
        );
        for k in self.samples.iter() {
            s.push_str(&format!(
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}\n",
                k.t,
                k.pos_ned[0], k.pos_ned[1], k.pos_ned[2],
                k.vel_ned[0], k.vel_ned[1], k.vel_ned[2],
                k.accel_world[0], k.accel_world[1], k.accel_world[2],
                k.quat[0], k.quat[1], k.quat[2], k.quat[3],
                k.omega_body[0], k.omega_body[1], k.omega_body[2],
            ));
        }
        s
    }

    /// 从 CSV 读入（列序与 [`Trajectory::to_csv`] 一致）。
    pub fn from_csv(name: &str, s: &str) -> Result<Self, String> {
        let mut samples = Vec::new();
        let mut rate_hz = Self::DEFAULT_RATE_HZ;
        let mut prev_t: Option<f32> = None;
        for (lineno, line) in s.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with("t,") {
                continue;
            }
            let vals: Vec<f32> = line
                .split(',')
                .map(|v| {
                    v.trim()
                        .parse::<f32>()
                        .map_err(|e| format!("第 {} 行解析失败: {e}", lineno + 1))
                })
                .collect::<Result<Vec<_>, _>>()?;
            if vals.len() < 17 {
                return Err(format!("第 {} 行字段不足（需 17 列）", lineno + 1));
            }
            let t = vals[0];
            let pos_ned = [vals[1], vals[2], vals[3]];
            let vel_ned = [vals[4], vals[5], vals[6]];
            let accel_world = [vals[7], vals[8], vals[9]];
            let quat = [vals[10], vals[11], vals[12], vals[13]];
            let omega_body = [vals[14], vals[15], vals[16]];
            if let Some(pt) = prev_t {
                let dt = t - pt;
                if dt > 1e-9 {
                    rate_hz = 1.0 / dt;
                }
            }
            prev_t = Some(t);
            samples.push(TrajSample {
                t,
                pos_ned,
                vel_ned,
                accel_world,
                quat,
                omega_body,
            });
        }
        if samples.len() < 2 {
            return Err("CSV 至少需要 2 个采样点".into());
        }
        Ok(Self {
            name: name.to_string(),
            rate_hz,
            samples,
        })
    }
}

/// 四元数球面插值（含半球对齐）。
fn slerp(a: [f32; 4], b: [f32; 4], f: f32) -> [f32; 4] {
    let mut b = b;
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if dot < 0.0 {
        for v in b.iter_mut() {
            *v = -*v;
        }
    }
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).clamp(-1.0, 1.0);
    let theta = dot.acos();
    let s = theta.sin();
    let (wa, wb) = if s.abs() < 1e-6 {
        (1.0 - f, f)
    } else {
        (((1.0 - f) * theta).sin() / s, (f * theta).sin() / s)
    };
    let q = [
        a[0] * wa + b[0] * wb,
        a[1] * wa + b[1] * wb,
        a[2] * wa + b[2] * wb,
        a[3] * wa + b[3] * wb,
    ];
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-8 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 悬停微扰：水平姿态、accel_world=0 → 比力应为 [0,0,−9.81]。
    #[test]
    fn hover_specific_force_is_minus_g() {
        let tr = Trajectory::from_maneuver(&Maneuver::HoverMicro { amp_deg: 3.0 }, 1000.0);
        let s = tr.at(2.0);
        let a = s.specific_force_body();
        assert!(
            (a[0]).abs() < 0.5 && (a[1]).abs() < 0.5 && (a[2] + 9.81).abs() < 0.5,
            "悬停比力应≈[0,0,-9.81]，实际 {a:?}"
        );
        assert!(s.specific_force_ratio() > 0.9 && s.specific_force_ratio() < 1.1);
    }

    /// 自由落体：accel_world=g → 比力应为 0（幅值门必须关闭）。
    #[test]
    fn free_fall_specific_force_is_zero() {
        let tr = Trajectory::from_maneuver(&Maneuver::FreeFall { jitter_deg: 0.5 }, 1000.0);
        let s = tr.at(2.0);
        let a = s.specific_force_body();
        let mag = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt();
        assert!(mag < 0.5, "自由落体比力应≈0，实际 |a|={mag:.3} {a:?}");
    }

    /// 协调转弯：|ω| 应≈航向速率，且滚转≈bank；向心加速度≈g·tanφ。
    #[test]
    fn coordinated_turn_rates_and_centripetal() {
        let tr = Trajectory::from_maneuver(
            &Maneuver::CoordinatedTurn {
                bank_deg: 35.0,
                rate_dps: 60.0,
            },
            1000.0,
        );
        let s = tr.at(3.0);
        let e = s.euler();
        assert!(
            (e[0] - d2r(35.0)).abs() < d2r(2.0),
            "roll 应≈35°，实际 {:.1}°",
            e[0].to_degrees()
        );
        let om = (s.omega_body[0].powi(2) + s.omega_body[1].powi(2) + s.omega_body[2].powi(2))
            .sqrt();
        assert!(
            (om - d2r(60.0)).abs() < d2r(3.0),
            "|ω| 应≈60°/s，实际 {:.1}°/s",
            om.to_degrees()
        );
        let expect_a = 9.81 * d2r(35.0).tan();
        assert!(
            (s.accel_world[1] - expect_a).abs() < 0.5,
            "向心加速度应≈{expect_a:.2}，实际 {:.2}",
            s.accel_world[1]
        );
    }

    /// 360° 横滚：应真的转过 360°（姿态连续、无万向锁截断）。
    #[test]
    fn roll360_completes_full_revolution() {
        let tr = Trajectory::from_maneuver(
            &Maneuver::Roll360 {
                rate_dps: 360.0,
                thrust_ratio: 0.5,
            },
            1000.0,
        );
        let mid = tr.at(0.5).euler()[0].to_degrees();
        let end = tr.at(1.0).euler()[0].to_degrees();
        assert!(
            (mid - 180.0).abs() < 5.0 || (mid + 180.0).abs() < 5.0,
            "0.5s 应为倒飞(±180°)，实际 {mid:.1}°"
        );
        assert!(
            end.abs() < 5.0 || (end.abs() - 360.0).abs() < 5.0,
            "1.0s 应转满一圈，实际 {end:.1}°"
        );
    }

    /// CSV 往返：导出再读回，真值应一致。
    #[test]
    fn csv_roundtrip_preserves_truth() {
        let tr = Trajectory::from_maneuver(&Maneuver::HoverMicro { amp_deg: 5.0 }, 200.0);
        let csv = tr.to_csv();
        let back = Trajectory::from_csv("rt", &csv).expect("CSV 应可解析");
        assert_eq!(back.len(), tr.len());
        for (a, b) in tr.samples().iter().zip(back.samples().iter()) {
            assert!((a.t - b.t).abs() < 1e-4);
            for k in 0..4 {
                assert!((a.quat[k] - b.quat[k]).abs() < 1e-5, "四元数往返失真");
            }
            for k in 0..3 {
                assert!((a.omega_body[k] - b.omega_body[k]).abs() < 1e-4);
            }
        }
    }
}
