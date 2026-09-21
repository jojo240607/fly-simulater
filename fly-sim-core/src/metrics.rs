//! 真值对比指标：`est` vs `truth` 的统一口径。
//!
//! 现状背景：此前各测试手写阈值（`max_roll > 0.06`、`worst_alt_err < 1.0` 等），
//! **没有一处 est-vs-truth 的误差计算**。本模块把口径固定下来，供 H 场（主机）
//! 与 M 场（MCU 指令级）共用，避免"同一个量在两处定义不同"。
//!
//! 阶段 1（姿态解算）用到的两组：
//! - [`AttMetrics`]：姿态角误差（RMSE / max / 末值）
//! - [`RateMetrics`]：角速率跟踪误差（大角速率段的主要判据）
//!
//! 另附 [`EulerErr`] 做分轴诊断（yaw 需 wrap 处理，否则 ±π 跨界会误判）。

use flyctrl_core::vehicle::Quaternion;

/// 四元数夹角误差（度）——**双覆盖**：`2·acos(|dot|)`。
///
/// `q` 与 `−q` 表示同一姿态，若不做 `|dot|` 取绝对值，会在符号翻转处报出
/// ~360° 的假误差（历史 `turn_yaw` 测试就被 Euler ±π wrap 坑过）。
pub fn quat_angle_error_deg(q_hat: [f32; 4], q_true: [f32; 4]) -> f32 {
    let mut dot = q_hat[0] * q_true[0]
        + q_hat[1] * q_true[1]
        + q_hat[2] * q_true[2]
        + q_hat[3] * q_true[3];
    if !dot.is_finite() {
        return f32::NAN;
    }
    if dot < 0.0 {
        dot = -dot;
    }
    let dot = dot.min(1.0);
    2.0 * dot.acos().to_degrees()
}

/// 单轴角度差（度），结果 wrap 到 `[-180, 180]`。
pub fn angle_diff_deg(a_deg: f32, b_deg: f32) -> f32 {
    let mut d = a_deg - b_deg;
    while d > 180.0 {
        d -= 360.0;
    }
    while d < -180.0 {
        d += 360.0;
    }
    d
}

/// 分轴欧拉误差（度）：`[roll, pitch, yaw]`，yaw 已 wrap。
///
/// 仅用于**诊断**（定位是哪一轴出问题）；判据用 [`AttMetrics`] 的夹角误差，
/// 因为它不受万向锁影响。
pub fn euler_err_deg(q_hat: [f32; 4], q_true: [f32; 4]) -> [f32; 3] {
    let e = |q: [f32; 4]| {
        let q = Quaternion {
            w: q[0],
            x: q[1],
            y: q[2],
            z: q[3],
        };
        [
            q.roll().to_degrees(),
            q.pitch().to_degrees(),
            q.yaw().to_degrees(),
        ]
    };
    let a = e(q_hat);
    let b = e(q_true);
    [
        angle_diff_deg(a[0], b[0]),
        angle_diff_deg(a[1], b[1]),
        angle_diff_deg(a[2], b[2]),
    ]
}

/// 姿态误差统计累加器。
#[derive(Clone, Copy, Debug, Default)]
pub struct AttMetrics {
    n: u32,
    sum_sq: f64,
    max_deg: f32,
    last_deg: f32,
    first_nan_step: Option<u32>,
    /// 分轴误差累加（仅诊断）
    sum_sq_axis: [f64; 3],
    max_axis: [f32; 3],
}

impl AttMetrics {
    pub fn push(&mut self, step: u32, q_hat: [f32; 4], q_true: [f32; 4]) {
        let err = quat_angle_error_deg(q_hat, q_true);
        if !err.is_finite() {
            if self.first_nan_step.is_none() {
                self.first_nan_step = Some(step);
            }
            return;
        }
        self.n += 1;
        self.sum_sq += (err as f64) * (err as f64);
        if err > self.max_deg {
            self.max_deg = err;
        }
        self.last_deg = err;
        let ax = euler_err_deg(q_hat, q_true);
        for k in 0..3 {
            self.sum_sq_axis[k] += (ax[k] as f64) * (ax[k] as f64);
            if ax[k].abs() > self.max_axis[k] {
                self.max_axis[k] = ax[k].abs();
            }
        }
    }

    /// 角度 RMSE（度）。
    pub fn rmse_deg(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.sum_sq / self.n as f64).sqrt() as f32
    }

    pub fn max_deg(&self) -> f32 {
        self.max_deg
    }

    pub fn last_deg(&self) -> f32 {
        self.last_deg
    }

    pub fn count(&self) -> u32 {
        self.n
    }

    /// 分轴 RMSE（度）`[roll, pitch, yaw]`——诊断用。
    pub fn axis_rmse_deg(&self) -> [f32; 3] {
        if self.n == 0 {
            return [f32::NAN; 3];
        }
        let mut o = [0.0f32; 3];
        for k in 0..3 {
            o[k] = (self.sum_sq_axis[k] / self.n as f64).sqrt() as f32;
        }
        o
    }

    /// 分轴最大误差（度）。
    pub fn axis_max_deg(&self) -> [f32; 3] {
        self.max_axis
    }

    /// 是否出现非有限值（数值发散的最早步）。
    pub fn first_nan_step(&self) -> Option<u32> {
        self.first_nan_step
    }

    pub fn diverged(&self) -> bool {
        self.first_nan_step.is_some()
    }

    /// 单行汇总（落盘/打印）。
    pub fn summary(&self, tag: &str) -> String {
        let a = self.axis_rmse_deg();
        format!(
            "{tag}: n={} angle_rmse={:.3}° max={:.3}° last={:.3}° | rpy_rmse=({:.2},{:.2},{:.2})° rpy_max=({:.2},{:.2},{:.2})°{}",
            self.n,
            self.rmse_deg(),
            self.max_deg(),
            self.last_deg(),
            a[0], a[1], a[2],
            self.max_axis[0], self.max_axis[1], self.max_axis[2],
            match self.first_nan_step {
                Some(s) => format!(" NONFINITE@{s}"),
                None => String::new(),
            }
        )
    }
}

/// 角速率跟踪误差统计（大角速率段的主要判据：角度误差可以大，但速率必须跟得上）。
#[derive(Clone, Copy, Debug, Default)]
pub struct RateMetrics {
    n: u32,
    sum_sq: f64,
    max_err: f32,
    sum_sq_truth: f64,
    max_truth: f32,
}

impl RateMetrics {
    pub fn push(&mut self, est: [f32; 3], truth: [f32; 3]) {
        let mut se = 0.0f32;
        let mut st = 0.0f32;
        for k in 0..3 {
            if !est[k].is_finite() || !truth[k].is_finite() {
                return;
            }
            se += (est[k] - truth[k]) * (est[k] - truth[k]);
            st += truth[k] * truth[k];
        }
        let e = se.sqrt();
        let t = st.sqrt();
        self.n += 1;
        self.sum_sq += (e as f64) * (e as f64);
        self.sum_sq_truth += (t as f64) * (t as f64);
        if e > self.max_err {
            self.max_err = e;
        }
        if t > self.max_truth {
            self.max_truth = t;
        }
    }

    /// 绝对 RMSE（rad/s）。
    pub fn rmse(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.sum_sq / self.n as f64).sqrt() as f32
    }

    /// 相对 RMSE：`rmse(ω̂−ω) / rms(ω)`——大角速率段应与绝对误差一起看。
    pub fn relative_rmse(&self) -> f32 {
        let denom = (self.sum_sq_truth / self.n.max(1) as f64).sqrt() as f32;
        if denom < 1e-6 {
            return f32::NAN;
        }
        self.rmse() / denom
    }

    pub fn max_err(&self) -> f32 {
        self.max_err
    }

    pub fn max_truth(&self) -> f32 {
        self.max_truth
    }

    pub fn summary(&self, tag: &str) -> String {
        format!(
            "{tag}: n={} rate_rmse={:.4} rad/s ({:.2}°/s) rel={:.2}% max_err={:.4} max|ω|={:.2}°/s",
            self.n,
            self.rmse(),
            self.rmse().to_degrees(),
            self.relative_rmse() * 100.0,
            self.max_err,
            self.max_truth.to_degrees(),
        )
    }
}

/// 阶跃响应轨迹与指标（单轴：角度°/角速率/位置等）。
///
/// 存整条时间序列（host 侧内存无压力），指标按需算——比累加器式实现更灵活，
/// 也便于落盘做曲线回看。
///
/// 口径（与 `docs/test-roadmap.md` 阶段 2 一致）：
/// - **超调** = (朝目标方向的极值 − target) / |target − y0| × 100%
/// - **上升时间** = 10% → 90% 目标幅值（递增/递减均支持）
/// - **调节时间** = 首次进入且**此后不再离开** ±tol 带
/// - **稳态误差** = 末段（后 10%）均值 − target
#[derive(Clone, Debug)]
pub struct StepTrace {
    pub dt: f32,
    pub y0: f32,
    pub target: f32,
    /// 容差带，相对于阶跃幅值 `|target − y0|`。
    pub tol_frac: f32,
    pub y: Vec<f32>,
    nonfinite: Option<u32>,
}

impl StepTrace {
    pub fn new(dt: f32, y0: f32, target: f32, tol_frac: f32) -> Self {
        Self {
            dt,
            y0,
            target,
            tol_frac,
            y: Vec::new(),
            nonfinite: None,
        }
    }

    pub fn push(&mut self, y: f32) {
        if !y.is_finite() {
            if self.nonfinite.is_none() {
                self.nonfinite = Some(self.y.len() as u32);
            }
        }
        self.y.push(y);
    }

    pub fn amp(&self) -> f32 {
        (self.target - self.y0).abs().max(1e-9)
    }
    fn dir(&self) -> f32 {
        if self.target >= self.y0 {
            1.0
        } else {
            -1.0
        }
    }
    fn tol(&self) -> f32 {
        self.tol_frac * self.amp()
    }

    /// 超调百分比（不超过目标则为 0）。
    pub fn overshoot_pct(&self) -> f32 {
        if self.y.is_empty() {
            return f32::NAN;
        }
        let d = self.dir();
        let peak = self
            .y
            .iter()
            .fold(f32::NEG_INFINITY, |a, &v| a.max(d * v));
        let over = d * peak - self.target;
        if over <= 0.0 {
            0.0
        } else {
            over / self.amp() * 100.0
        }
    }

    /// 10% → 90% 上升时间（s）；未达到则 −1。
    pub fn rise_s(&self) -> f32 {
        let (y0, tg, a) = (self.y0, self.target, self.amp());
        let p10 = y0 + 0.10 * (tg - y0);
        let p90 = y0 + 0.90 * (tg - y0);
        let mut i10 = None;
        for (i, &v) in self.y.iter().enumerate() {
            if (v - p10).abs() <= 0.05 * a && (v - y0).abs() <= (p10 - y0).abs() + 0.05 * a {
                i10 = Some(i);
                break;
            }
        }
        let mut i90 = None;
        for (i, &v) in self.y.iter().enumerate() {
            if (v - tg).abs() <= 0.10 * a {
                i90 = Some(i);
                break;
            }
        }
        match (i10, i90) {
            (Some(a10), Some(a90)) if a90 >= a10 => (a90 - a10) as f32 * self.dt,
            _ => -1.0,
        }
    }

    /// 调节时间（s）：首次进入且**此后不再离开** ±tol 带；从未进入则 −1。
    pub fn settle_s(&self) -> f32 {
        let tol = self.tol();
        let n = self.y.len();
        if n == 0 {
            return -1.0;
        }
        // 从后往前找“连续在带内”的起点
        let mut start = n;
        for i in (0..n).rev() {
            if (self.y[i] - self.target).abs() <= tol {
                start = i;
            } else {
                break;
            }
        }
        if start == n {
            -1.0
        } else {
            start as f32 * self.dt
        }
    }

    /// 稳态误差：末段（后 10%，至少 1 点）均值 − target。
    pub fn ss_err(&self) -> f32 {
        let n = self.y.len();
        if n == 0 {
            return f32::NAN;
        }
        let k = (n / 10).max(1);
        let tail = &self.y[n - k..];
        let m = tail.iter().sum::<f32>() / k as f32;
        m - self.target
    }

    /// 全程最大偏差（相对 target）。
    pub fn max_err(&self) -> f32 {
        self.y
            .iter()
            .fold(0.0f32, |a, &v| a.max((v - self.target).abs()))
    }

    /// 末段峯値（用于“振荡是否衰减”：末段幅值 vs 全局幅值）。
    pub fn tail_peak(&self) -> f32 {
        let n = self.y.len();
        if n == 0 {
            return f32::NAN;
        }
        let k = (n / 4).max(1);
        self.y[n - k..]
            .iter()
            .fold(0.0f32, |a, &v| a.max((v - self.target).abs()))
    }

    /// 末段（后 1/4）**振荡幅度** = max − min。
    ///
    /// 与 `tail_peak()` 的区别：后者含**系统性偏移**（如旋翼阻力矩造成的倾斜静差），
    /// 会把它误判为“振荡”。判稳必须用本量。
    pub fn tail_osc(&self) -> f32 {
        let n = self.y.len();
        if n == 0 {
            return f32::NAN;
        }
        let k = (n / 4).max(1);
        let t = &self.y[n - k..];
        let hi = t.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let lo = t.iter().cloned().fold(f32::INFINITY, f32::min);
        hi - lo
    }

    /// 末段均值。
    pub fn tail_mean(&self) -> f32 {
        let n = self.y.len();
        if n == 0 {
            return f32::NAN;
        }
        let k = (n / 4).max(1);
        self.y[n - k..].iter().sum::<f32>() / k as f32
    }

    pub fn diverged(&self) -> bool {
        self.nonfinite.is_some()
    }
    pub fn first_nonfinite(&self) -> Option<u32> {
        self.nonfinite
    }

    pub fn summary(&self, tag: &str) -> String {
        format!(
            "{tag}: overshoot={:.1}% rise={:.3}s settle={:.3}s ss_err={:+.3} max|e|={:.3} tail_peak={:.3}{}",
            self.overshoot_pct(),
            self.rise_s(),
            self.settle_s(),
            self.ss_err(),
            self.max_err(),
            self.tail_peak(),
            match self.nonfinite {
                Some(s) => format!(" NONFINITE@{s}"),
                None => String::new(),
            }
        )
    }
}

/// 执行器饱和占比（任一路超出 [0,1] 或触边）。
#[derive(Clone, Copy, Debug, Default)]
pub struct SatMetrics {
    n: u32,
    at_limit: u32,
    sum_abs_diff: f64,
    max_abs_diff: f32,
}

impl SatMetrics {
    /// `motor` 为四路归一化指令（应在 [0,1]）。
    pub fn push(&mut self, motor: [f32; 4]) {
        let hi = motor.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let lo = motor.iter().cloned().fold(f32::INFINITY, f32::min);
        self.n += 1;
        if hi >= 0.999 || lo <= 0.001 {
            self.at_limit += 1;
        }
        let d = hi - lo;
        self.sum_abs_diff += d as f64;
        if d > self.max_abs_diff {
            self.max_abs_diff = d;
        }
    }

    /// 触边步数占比。
    pub fn ratio(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        self.at_limit as f32 / self.n as f32
    }

    /// 平均四路差动（执行器“出力不均匀”程度）。
    pub fn mean_diff(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.sum_abs_diff / self.n as f64) as f32
    }

    pub fn max_diff(&self) -> f32 {
        self.max_abs_diff
    }

    pub fn summary(&self, tag: &str) -> String {
        format!(
            "{tag}: sat_ratio={:.1}% mean_diff={:.3} max_diff={:.3}",
            self.ratio() * 100.0,
            self.mean_diff(),
            self.max_diff()
        )
    }
}

/// 位置/速度估计误差指标（阶段 3）。
///
/// 水平（N/E）与垂向（D）分开统计：EKF 两者的可观测性完全不同
/// （垂向靠 baro 绝对观测，水平靠 GPS/Doppler）。
#[derive(Clone, Debug, Default)]
pub struct PosVelMetrics {
    n: u32,
    pos_sq: f64,
    vel_sq: f64,
    pos_max: f32,
    vel_max: f32,
    h_sq: f64,
    h_max: f32,
    v_sq: f64,
    v_max: f32,
    nonfinite: Option<u32>,
    /// 位置误差的**环形缓冲**（最近 `TAIL_CAP` 个样本）——用于**末段包络 RMS**。
    ///
    /// 为何不用 `pos_max`：窗口内最大值对**振荡相位**极敏感（切在波峰还是波谷
    /// 能差几倍，见 `docs/stage4-outer-loop-findings.md` P10 的自我声明）。
    /// 末段 RMS 对相位不敏感，可稳定比较不同增益/配置。
    tail_ring: Vec<f32>,
}

impl PosVelMetrics {
    /// 环形缓冲容量（样本数）。250Hz 下 2048 ≈ 8.2s。
    pub const TAIL_CAP: usize = 2048;
}

impl PosVelMetrics {
    pub fn push(
        &mut self,
        est_p: [f32; 3],
        est_v: [f32; 3],
        true_p: [f32; 3],
        true_v: [f32; 3],
    ) {
        let mut dp = [0.0f32; 3];
        let mut dv = [0.0f32; 3];
        for k in 0..3 {
            if !est_p[k].is_finite()
                || !est_v[k].is_finite()
                || !true_p[k].is_finite()
                || !true_v[k].is_finite()
            {
                if self.nonfinite.is_none() {
                    self.nonfinite = Some(self.n);
                }
                return;
            }
            dp[k] = est_p[k] - true_p[k];
            dv[k] = est_v[k] - true_v[k];
        }
        let ep = (dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]).sqrt();
        let ev = (dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]).sqrt();
        let hp = (dp[0] * dp[0] + dp[1] * dp[1]).sqrt();
        self.n += 1;
        self.pos_sq += (ep as f64) * (ep as f64);
        self.vel_sq += (ev as f64) * (ev as f64);
        self.h_sq += (hp as f64) * (hp as f64);
        self.v_sq += (dp[2] as f64) * (dp[2] as f64);
        if ep > self.pos_max {
            self.pos_max = ep;
        }
        if ev > self.vel_max {
            self.vel_max = ev;
        }
        if hp > self.h_max {
            self.h_max = hp;
        }
        if dp[2].abs() > self.v_max {
            self.v_max = dp[2].abs();
        }
        if self.tail_ring.len() >= Self::TAIL_CAP {
            self.tail_ring.remove(0);
        }
        self.tail_ring.push(ep);
    }

    pub fn count(&self) -> u32 {
        self.n
    }
    pub fn pos_rmse(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.pos_sq / self.n as f64).sqrt() as f32
    }
    pub fn vel_rmse(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.vel_sq / self.n as f64).sqrt() as f32
    }
    pub fn pos_max(&self) -> f32 {
        self.pos_max
    }
    pub fn vel_max(&self) -> f32 {
        self.vel_max
    }
    /// 水平位置 RMSE。
    pub fn horiz_rmse(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.h_sq / self.n as f64).sqrt() as f32
    }
    /// 垂向位置（D）RMSE。
    pub fn vert_rmse(&self) -> f32 {
        if self.n == 0 {
            return f32::NAN;
        }
        (self.v_sq / self.n as f64).sqrt() as f32
    }
    /// **末段位置误差 RMS**（环形缓冲内）——对振荡相位不敏感，用于横向比较。
    pub fn tail_pos_rms(&self) -> f32 {
        if self.tail_ring.is_empty() {
            return f32::NAN;
        }
        let sq: f64 = self.tail_ring.iter().map(|&v| (v as f64) * (v as f64)).sum();
        (sq / self.tail_ring.len() as f64).sqrt() as f32
    }
    /// 末段**峰值**（环形缓冲内 max）——仍受相位影响，但比全窗口 max 稳。
    pub fn tail_pos_peak(&self) -> f32 {
        self.tail_ring.iter().cloned().fold(0.0f32, f32::max)
    }

    pub fn diverged(&self) -> bool {
        self.nonfinite.is_some()
    }
    pub fn summary(&self, tag: &str) -> String {
        format!(
            "{tag}: n={} pos_rmse={:.3}m (h={:.3} v={:.3}) pos_max={:.3}m vel_rmse={:.3}m/s vel_max={:.3}{}",
            self.n,
            self.pos_rmse(),
            self.horiz_rmse(),
            self.vert_rmse(),
            self.pos_max,
            self.vel_rmse(),
            self.vel_max,
            match self.nonfinite {
                Some(s) => format!(" NONFINITE@{s}"),
                None => String::new(),
            }
        )
    }
}

/// 跟踪误差（**被控量 vs 设定点**，阶段 4）。
///
/// 语义与 [`PosVelMetrics`] 完全一致，只是"参考量"从真值换成**设定点**。
/// 单独立名是为了避免调用点上的语义混淆（`push(est, sp)` vs `push(est, truth)`）。
pub type TrackMetrics = PosVelMetrics;

#[cfg(test)]
mod tests {
    use super::*;

    /// q 与 −q 是同一姿态：夹角误差必须为 0（双覆盖）。
    #[test]
    fn quat_double_cover_is_zero() {
        let q = [0.6f32, 0.8, 0.0, 0.0];
        let neg = [-0.6f32, -0.8, 0.0, 0.0];
        assert!(quat_angle_error_deg(q, neg) < 1e-3);
    }

    /// 90° 误差应被正确测出（不因欧拉角表述而失真）。
    #[test]
    fn quat_angle_error_measures_ninety_degrees() {
        let q_true = [1.0f32, 0.0, 0.0, 0.0];
        let s = (45.0f32).to_radians().sin();
        let c = (45.0f32).to_radians().cos();
        let q_hat = [c, s, 0.0, 0.0]; // 绕 x 转 90°
        assert!((quat_angle_error_deg(q_hat, q_true) - 90.0).abs() < 1e-2);
    }

    /// yaw 跨界（179° vs −179°）距离应为 2°，不是 358°。
    /// 符号约定为 `a−b` 的最短回绕方向（此处 179−(−179) → −2），故断言**幅值**。
    #[test]
    fn yaw_wrap_is_handled() {
        assert!((angle_diff_deg(179.0, -179.0).abs() - 2.0).abs() < 1e-3);
        assert!((angle_diff_deg(-179.0, 179.0).abs() - 2.0).abs() < 1e-3);
        assert!((angle_diff_deg(179.0, -179.0) + angle_diff_deg(-179.0, 179.0)).abs() < 1e-3);
    }

    #[test]
    fn att_metrics_accumulates() {
        let mut m = AttMetrics::default();
        let q = [1.0f32, 0.0, 0.0, 0.0];
        m.push(0, q, q);
        m.push(1, q, q);
        assert_eq!(m.count(), 2);
        assert!(m.rmse_deg() < 1e-3);
        assert!(!m.diverged());
    }

    #[test]
    fn nonfinite_is_recorded() {
        let mut m = AttMetrics::default();
        m.push(0, [f32::NAN; 4], [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(m.first_nan_step(), Some(0));
        assert!(m.diverged());
    }

    #[test]
    fn rate_metrics_relative() {
        let mut m = RateMetrics::default();
        let truth = [1.0f32, 0.0, 0.0];
        m.push([1.1, 0.0, 0.0], truth);
        assert!((m.rmse() - 0.1).abs() < 1e-5);
        assert!((m.relative_rmse() - 0.1).abs() < 1e-5);
    }
}
