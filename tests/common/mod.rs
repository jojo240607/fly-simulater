//! 共享测试工具：**TRU 真值有界**断言（验收判据推广，见 `FIDELITY_ROADMAP`）。
//!
//! 判据：收敛判定必须同时断言物理真值（TRU）有界——不只 `EST≈TRU`。
//! 判定量统一归一化为三个量：水平漂移 `h`、NED 下向 `d`、姿态倾角 `tilt°`。
//! 各场景测试负责把本机坐标系真值（NED `VehicleState` 或引擎世界系 `debug_up`）
//! 换算到这 3 个量，逐帧喂给 [`TruStats::sample`]，最后用 [`assert_tru_bounded`] 断言。
//! 这样把"有界判据"收敛到单一定义，避免各测试各自写一份且阈值不一致。

/// 逐帧 TRU 真值统计（归一化判定量）。
#[derive(Clone, Debug)]
pub struct TruStats {
    /// 是否出现 NaN/Inf（任一判定量非有限即置位）。
    pub nan: bool,
    /// 相对原点最大水平漂移（m）。
    pub h_max: f64,
    /// NED down 最浅（m）。
    pub d_min: f64,
    /// NED down 最深（m）。
    pub d_max: f64,
    /// 姿态偏离水平最大值（°）。
    pub tilt_max_deg: f64,
    /// 结束时 NED down（m）。
    pub end_d: f64,
}

impl Default for TruStats {
    fn default() -> Self {
        Self {
            nan: false,
            h_max: 0.0,
            d_min: f64::MAX,
            d_max: f64::MIN,
            tilt_max_deg: 0.0,
            end_d: 0.0,
        }
    }
}

impl TruStats {
    /// 记录一帧归一化真值测量：水平漂移 `h`、NED 下向 `d`、倾角 `tilt_deg`。
    /// 任一量非有限即置 `nan`（随后断言失败，防止静默发散）。
    pub fn sample(&mut self, h: f64, d: f64, tilt_deg: f64) {
        if !h.is_finite() || !d.is_finite() || !tilt_deg.is_finite() {
            self.nan = true;
            return;
        }
        self.h_max = self.h_max.max(h);
        self.d_min = self.d_min.min(d);
        self.d_max = self.d_max.max(d);
        self.tilt_max_deg = self.tilt_max_deg.max(tilt_deg);
        self.end_d = d;
    }
}

/// 断言 TRU 真值有界：不 NaN、高度不 runaway、水平漂移与姿态倾角受限。
///
/// - `label`：场景名（错误信息定位）；
/// - `sp_d`：设定点 NED down（如悬停 -5，仅用于错误信息提示）；
/// - `max_h`：允许最大水平漂移（m）；
/// - `max_tilt_deg`：允许最大倾角（°，>45 视为翻机）。
pub fn assert_tru_bounded(st: &TruStats, label: &str, sp_d: f64, max_h: f64, max_tilt_deg: f64) {
    assert!(!st.nan, "[{label}] TRU 出现 NaN/Inf");
    assert!(
        st.end_d.abs() < 10.0,
        "[{label}] TRU 高度 runaway：end d={:.1}（期望≈{sp_d}）",
        st.end_d
    );
    assert!(
        st.h_max < max_h,
        "[{label}] TRU 水平漂移过大：h_max={:.1}m（期望 < {max_h}）",
        st.h_max
    );
    assert!(
        st.tilt_max_deg < max_tilt_deg,
        "[{label}] TRU 姿态翻滚：max tilt={:.1}°（期望 < {max_tilt_deg}°）",
        st.tilt_max_deg
    );
}
