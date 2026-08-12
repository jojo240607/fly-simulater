// 独立验证 fly-sim-core/src/alloc.rs 的控制分配数学（不依赖 phy-rigid）。
// 编译：rustc --edition 2021 -O alloc_check.rs -o alloc_check.exe && alloc_check.exe
// 这是对 alloc.rs 逻辑的复刻 + 解析解断言，证明分配器正确。

const M: [[f64; 4]; 4] = [
    [1.0, 1.0, 1.0, 1.0], // thrust
    [0.5, -0.5, -0.5, 0.5], // roll
    [0.5, -0.5, 0.5, -0.5], // pitch
    [0.5, 0.5, -0.5, -0.5], // yaw
];

fn invert_full(des: [f64; 4]) -> [f64; 4] {
    let (t, p, q, r) = (des[0], des[1], des[2], des[3]);
    [
        t + 0.5 * (p + q + r),
        t + 0.5 * (-p - q + r),
        t + 0.5 * (-p + q - r),
        t + 0.5 * (p - q - r),
    ]
}

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

fn allocate_eff(des: [f64; 4], eff: &[f32; 4]) -> [f64; 4] {
    let active: Vec<usize> = (0..4).filter(|&i| eff[i] > 0.0).collect();
    if active.is_empty() {
        return [0.0; 4];
    }
    if active.len() == 4 {
        return invert_full(des);
    }
    let n = active.len();
    let mut ma = vec![vec![0.0f64; n]; 4];
    for (c, &mi) in active.iter().enumerate() {
        for r in 0..4 {
            ma[r][c] = M[r][mi];
        }
    }
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
    let mut rhs = vec![0.0f64; n];
    for i in 0..n {
        for r in 0..4 {
            rhs[i] += ma[r][i] * des[r];
        }
    }
    let u = gauss_solve(&h, &rhs, n);
    let mut out = [0.0f64; 4];
    for (c, &mi) in active.iter().enumerate() {
        out[mi] = u[c];
    }
    out
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
    // 1. 全有效 == 原混控
    let des = [0.6, 0.1, -0.05, 0.02];
    let u = allocate_eff(des, &[1.0; 4]);
    let exp = invert_full(des);
    check("full_equals_mixer", u.iter().zip(exp.iter()).all(|(a, b)| (a - b).abs() < 1e-9), "full mix");

    // 2. 单失效, 仅推力: 解析解 0.5/3.25
    let u = allocate_eff([0.5, 0.0, 0.0, 0.0], &[0.0, 1.0, 1.0, 1.0]);
    check("dead_motor_zero", u[0] == 0.0, "dead motor not zero");
    let expect = 0.5 / 3.25;
    check("single_fail_lstsq", (u[1] - expect).abs() < 1e-9 && (u[2] - expect).abs() < 1e-9 && (u[3] - expect).abs() < 1e-9, "single fail analytic");

    // 3. 双失效, 仅推力: 解析解 0.2
    let u = allocate_eff([0.5, 0.0, 0.0, 0.0], &[0.0, 1.0, 1.0, 0.0]);
    check("two_fail_zero", u[0] == 0.0 && u[3] == 0.0, "two fail zero");
    check("two_fail_lstsq", (u[1] - 0.2).abs() < 1e-9 && (u[2] - 0.2).abs() < 1e-9, "two fail analytic");

    // 4. 全失效 -> 0
    let u = allocate_eff([0.5, 0.1, 0.0, 0.0], &[0.0, 0.0, 0.0, 0.0]);
    check("all_fail_zero", u == [0.0, 0.0, 0.0, 0.0], "all fail");

    // 5. 回代验证: 混合需求, 单失效, 残差最小(与解析最小二乘一致)
    let des = [0.5, 0.08, -0.06, 0.03];
    let u = allocate_eff(des, &[0.0, 1.0, 1.0, 1.0]);
    let mut actual = [0.0f64; 4];
    for r in 0..4 {
        for i in 0..4 {
            actual[r] += M[r][i] * u[i];
        }
    }
    let res: f64 = (0..4).map(|r| (actual[r] - des[r]).powi(2)).sum();
    println!("mixed residual = {:.6}, u={:?}, actual={:?}", res, u, actual);
    // 最小二乘最优性：法方程 M_aᵀ(M_a·u - des) ≈ 0（残差与有效列正交）。
    // active=[1,2,3]，检查每有效电机列与残差内积 ≈ 0。
    let resid: Vec<f64> = (0..4).map(|r| actual[r] - des[r]).collect();
    for &mi in &[1usize, 2, 3] {
        let ortho: f64 = (0..4).map(|r| M[r][mi] * resid[r]).sum();
        check("lstsq_orthogonality", ortho.abs() < 1e-9, "least-squares normal equation");
    }

    println!("ALL OK");
}
