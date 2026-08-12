//! 阶段 P1-1：在线控制分配（可重构混控 / 冗余容错）。
//!
//! 把控制器期望动作向量 `des = [thrust, roll, pitch, yaw]`（归一化）分配给 4 路电机油门。
//! 全有效时与 `flyctrl-core` 固定 X 布局混控**完全一致**（退化为原硬编码）；
//! 电机部分失效时通过**最小二乘控制分配**把需求重分配到剩余有效电机，失效电机油门=0。
//!
//! 控制矩阵 M（行=[thrust, roll, pitch, yaw]，列=电机0..3，X 布局，与 pid.rs 一致）：
//!   m0 = t + 0.5(+p +q +r)
//!   m1 = t + 0.5(-p -q +r)
//!   m2 = t + 0.5(-p +q -r)
//!   m3 = t + 0.5(+p -q -r)

/// 控制矩阵（4×4，行=动作轴，列=电机）。
const M: [[f64; 4]; 4] = [
    [1.0, 1.0, 1.0, 1.0], // thrust
    [0.5, -0.5, -0.5, 0.5], // roll
    [0.5, -0.5, 0.5, -0.5], // pitch
    [0.5, 0.5, -0.5, -0.5], // yaw
];

/// 全有效：直接逆（== 原固定混控）。
fn invert_full(des: [f64; 4]) -> [f64; 4] {
    let (t, p, q, r) = (des[0], des[1], des[2], des[3]);
    [
        t + 0.5 * (p + q + r),
        t + 0.5 * (-p - q + r),
        t + 0.5 * (-p + q - r),
        t + 0.5 * (p - q - r),
    ]
}

/// 在线控制分配：`des`（thrust/roll/pitch/yaw 归一化需求）→ 4 油门。
/// `eff`：电机效率系数（0=失效）。返回各电机油门（未 clamp，由调用方 clamp [0,1]）。
///
/// 全有效 → 原混控。部分失效 → 最小二乘：
///   M_a = M 保留有效电机列（4×n），u_n = (M_aᵀ M_a)⁻¹ (M_aᵀ des)，失效电机=0。
/// 四旋翼单电机失效（3 有效、4 需求）为超定，最小二乘在有效电机间近似满足
/// 各轴需求；推力/姿态通常近似保持，偏航（反桨差动最弱）偏差可能较大。
pub fn allocate_eff(des: [f64; 4], eff: &[f32; 4]) -> [f64; 4] {
    let active: Vec<usize> = (0..4).filter(|&i| eff[i] > 0.0).collect();
    if active.is_empty() {
        return [0.0; 4];
    }
    if active.len() == 4 {
        return invert_full(des);
    }

    let n = active.len();
    // M_a（4×n）：行=轴，列=有效电机。
    let mut ma = vec![vec![0.0f64; n]; 4];
    for (c, &mi) in active.iter().enumerate() {
        for r in 0..4 {
            ma[r][c] = M[r][mi];
        }
    }
    // H = M_aᵀ M_a（n×n）
    let mut h = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        for j in 0..n {
            let mut s = 0.0;
            for r in 0..4 {
                s += ma[r][i] * ma[r][j];
            }
            h[i][j] = s;
        }
    }
    // rhs = M_aᵀ des（n 维）
    let mut rhs = vec![0.0f64; n];
    for i in 0..n {
        for r in 0..4 {
            rhs[i] += ma[r][i] * des[r];
        }
    }
    // 解 H·u = rhs
    let u = gauss_solve(&h, &rhs, n);
    let mut out = [0.0f64; 4];
    for (c, &mi) in active.iter().enumerate() {
        out[mi] = u[c];
    }
    out
}

/// 高斯消元（含部分主元）解 n×n 线性方程组 A·x = b。
fn gauss_solve(a: &[Vec<f64>], b: &[f64], n: usize) -> Vec<f64> {
    let mut m: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = a[i].clone();
            row.push(b[i]);
            row
        })
        .collect();
    for col in 0..n {
        let mut piv = col;
        for r in col + 1..n {
            if m[r][col].abs() > m[piv][col].abs() {
                piv = r;
            }
        }
        m.swap(col, piv);
        let d = m[col][col];
        if d.abs() < 1e-12 {
            continue;
        }
        for r in 0..n {
            if r == col {
                continue;
            }
            let f = m[r][col] / d;
            for c in col..=n {
                m[r][c] -= f * m[col][c];
            }
        }
    }
    (0..n)
        .map(|i| {
            if m[i][i].abs() > 1e-12 {
                m[i][n] / m[i][i]
            } else {
                0.0
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

    #[test]
    fn full_effective_equals_original_mixer() {
        // 全有效：分配结果应完全等于原 pid.rs 固定混控。
        let des = [0.6, 0.1, -0.05, 0.02];
        let u = allocate_eff(des, &ALL);
        let expect = invert_full(des);
        for i in 0..4 {
            assert!(
                (u[i] - expect[i]).abs() < 1e-9,
                "M{} 全有效应等于原混控: got {}, expect {}",
                i,
                u[i],
                expect[i]
            );
        }
    }

    #[test]
    fn single_failure_thrust_matches_lstsq_analytic() {
        // 电机 0 失效，仅推力需求：最小二乘精确解可解析求出。
        // M_a(有效电机1,2,3) 对姿态轴非正交 → 折中推力，sum=0.5/1.625。
        let des = [0.5, 0.0, 0.0, 0.0];
        let eff = [0.0, 1.0, 1.0, 1.0];
        let u = allocate_eff(des, &eff);
        assert_eq!(u[0], 0.0, "失效电机油门应为 0");
        // 解析最小二乘解：H=[[1.75,.75,.75],...]，H⁻¹[.5,.5,.5]=[.1538,.1538,.1538]
        let expect = 0.5 / 3.25;
        for i in 1..4 {
            assert!(
                (u[i] - expect).abs() < 1e-9,
                "M{} 应等于最小二乘解析解: got {}, expect {}",
                i,
                u[i],
                expect
            );
        }
        let sum: f64 = u[1] + u[2] + u[3];
        assert!((sum - 0.5 * 3.0 / 3.25).abs() < 1e-9, "sum={}", sum);
    }

    #[test]
    fn two_failures_thrust_matches_lstsq_analytic() {
        // 电机 0,3 失效，2 有效：仅推力需求 → u=[0.2,0.2]（2.5a=0.5→a=0.2）。
        let des = [0.5, 0.0, 0.0, 0.0];
        let eff = [0.0, 1.0, 1.0, 0.0];
        let u = allocate_eff(des, &eff);
        assert_eq!(u[0], 0.0);
        assert_eq!(u[3], 0.0);
        assert!((u[1] - 0.2).abs() < 1e-9, "M1={}", u[1]);
        assert!((u[2] - 0.2).abs() < 1e-9, "M2={}", u[2]);
    }

    #[test]
    fn mixed_demand_reallocates_with_zero_dead_motor() {
        // 混合需求（推力+滚转+俯仰+偏航），电机 0 失效：失效电机=0，其余分担。
        let des = [0.5, 0.08, -0.06, 0.03];
        let eff = [0.0, 1.0, 1.0, 1.0];
        let u = allocate_eff(des, &eff);
        assert_eq!(u[0], 0.0, "失效电机油门应为 0");
        // 有效电机均有非零油门（重分配到剩余电机）
        assert!(u[1].abs() > 0.0 && u[2].abs() > 0.0 && u[3].abs() > 0.0);
        // 有效电机油门不越界（未 clamp 时也应在合理范围内）
        for i in 1..4 {
            assert!(u[i] > -1e-9, "有效电机油门应非负: M{}={}", i, u[i]);
        }
    }

    #[test]
    fn all_failed_returns_zero() {
        let u = allocate_eff([0.5, 0.1, 0.0, 0.0], &[0.0, 0.0, 0.0, 0.0]);
        assert_eq!(u, [0.0, 0.0, 0.0, 0.0]);
    }
}
