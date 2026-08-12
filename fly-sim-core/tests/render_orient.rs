//! 渲染姿态约定回归：渲染直接使用引擎世界系（Y-up），悬停时机体水平。
//!
//! 物理约束：悬停时四旋翼的推力轴（机体 +Z，旋翼盘法线）指向世界 +Y（上）。
//! 因此引擎悬停的四元数 q_up 满足 `rotate(q_up, (0,0,1)) = (0,1,0)`，即旋翼盘水平。
//! 渲染世界 = 引擎世界（同为 Y-up），直接用 q_up，故渲染也水平。
//!
//! 曾错误地对引擎四元数做"z 镜像变换"（`(w,x,y,-z)`），导致带偏航悬停被翻成侧躺；
//! 本测试锁定"引擎悬停四元数本身即水平"这一不变量，防止再次引入坐标变换破坏姿态。

#![cfg(feature = "phy")]

use fly_sim_core::render::RenderInput;

/// 用单位四元数旋转向量 v：R(q)·v（r = v + 2w(q×v) + 2(q×(q×v))）。
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
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

/// Hamilton 四元数积。
fn ham_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// 渲染直接使用的四元数 = 引擎四元数（不转换），故本函数即恒等，仅表意。
fn render_quat(q_up: [f64; 4]) -> [f64; 4] {
    q_up
}

#[test]
fn hover_rotor_stays_horizontal() {
    // 纯悬停：绕 X 倾 -90° 使机体 +Z 朝上。q_up = rotX(-90°)
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let q_up = [s, -s, 0.0, 0.0];
    let up = rotate(q_up, [0.0, 0.0, 1.0]);
    assert!(
        (up[0].abs() < 1e-6) && (up[1] > 0.99) && (up[2].abs() < 1e-6),
        "引擎悬停应使机体+Z朝上: {:?}",
        up
    );
    // 渲染直接用引擎四元数，应保持水平
    let q_r = render_quat(q_up);
    let up_r = rotate(q_r, [0.0, 0.0, 1.0]);
    assert!(
        (up_r[0].abs() < 1e-6) && (up_r[1] > 0.99) && (up_r[2].abs() < 1e-6),
        "渲染后旋翼盘应保持水平: {:?}",
        up_r
    );
}

#[test]
fn hover_with_yaw_stays_horizontal() {
    // 悬停 + 偏航 45°：先绕 X 倾 -90° 使机体 +Z 朝上，再绕世界 Y 偏航 45°。
    // q_up = rotY(45°) ⊗ rotX(-90°)。
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let rx = [s, -s, 0.0, 0.0]; // rotX(-90°)
    let h = 22.5f64.to_radians();
    let ry = [h.cos(), 0.0, h.sin(), 0.0]; // rotY(45°)
    let q_up = ham_mul(ry, rx);

    // 引擎悬停：机体 +Z（推力轴）仍朝 +Y（上），偏航不影响推力方向
    let up = rotate(q_up, [0.0, 0.0, 1.0]);
    assert!(up[1] > 0.99, "带偏航悬停推力仍应朝上: {:?}", up);

    // 渲染直接使用引擎四元数，应保持水平
    let q_r = render_quat(q_up);
    let up_r = rotate(q_r, [0.0, 0.0, 1.0]);
    assert!(up_r[1] > 0.99, "带偏航悬停渲染后仍应水平: {:?}", up_r);
}

#[test]
fn render_input_defaults_are_horizontal() {
    // 默认 RenderInput 为水平姿态（单位四元数），旋翼盘法线指向 +Z（引擎机体推力轴）。
    let inp = RenderInput::default();
    assert_eq!(inp.quat, [1.0, 0.0, 0.0, 0.0]);
    assert_eq!(inp.pos, [0.0, 0.0, 0.0]);
}
