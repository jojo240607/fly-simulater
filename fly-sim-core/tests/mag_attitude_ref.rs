//! **2.2e 零件验证**：磁场矢量作为**全姿态参考**的精度（硬铁补偿前/后）。
//!
//! # 为何先验这个零件
//! 路线 2.2e 打算把磁参考从"只修 yaw"扩为"修全姿态（含 roll/pitch）"，用来替代
//! 会被水平加速度污染的**比力锚定**（这正是"H 场湍流风劣化"的根源）。
//! 但它的成立前提是"**磁场方向能反映姿态**"——而硬铁会破坏这一点。
//!
//! 解析预期（已算）：`|B|=0.640`、`realistic()` 硬铁 `|h|=0.539` ⇒ `|h|/|B|=0.84`
//! ⇒ 方向偏差最大 `atan(0.84)=40.1°`（最坏；取决于朝向不可预测）。
//! **本测例把它变成实测**，并给出补偿后的残差，作为 2.2e 是否可动的判据。
//!
//! 判据：**补偿后磁姿态参考误差应 << 1°**（无噪声、无软铁的理想条件下的理论上限；
//! 真实系统还要叠加噪声与标定残差）。

use flyctrl_core::vehicle::{rotate_vec_by_quat, rotate_vec_by_quat_inverse, Quaternion};
use flyctrl_core::units::Radian;

/// 世界磁场（与 `plant.rs::m_world_ned` 一致：水平 0.5、垂向 0.4，NED 向下为正）。
const B_WORLD: [f32; 3] = [0.5, 0.0, 0.4];
/// `SensorConfig::realistic()` 的硬铁（机体系常量偏置）。
const HARD_IRON: [f32; 3] = [0.3, -0.2, 0.4];

fn norm(v: [f32; 3]) -> f32 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}
fn angle_deg(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (norm(a) * norm(b));
    d.clamp(-1.0, 1.0).acos().to_degrees()
}

/// 给定真实姿态、硬铁是否被补偿，返回"由机体磁测量反推的世界场方向"与真值方向的夹角。
fn ref_error_deg(q: Quaternion, compensate: bool) -> f32 {
    // 仿真侧：机体测量 = R^T·B + 硬铁（无噪声、无软铁）
    let m_true = rotate_vec_by_quat_inverse(q, B_WORLD);
    let m_meas = [
        m_true[0] + HARD_IRON[0],
        m_true[1] + HARD_IRON[1],
        m_true[2] + HARD_IRON[2],
    ];
    // 估计器侧：扣除离线标定的硬铁（本测例假设标定值 == 真值，即标定完美）
    let m_used = if compensate {
        [
            m_meas[0] - HARD_IRON[0],
            m_meas[1] - HARD_IRON[1],
            m_meas[2] - HARD_IRON[2],
        ]
    } else {
        m_meas
    };
    // 用**真值姿态**把机体测量旋回世界系（隔离"参考误差"本身，不含姿态估计误差）
    let b_est = rotate_vec_by_quat(q, m_used);
    angle_deg(b_est, B_WORLD)
}

#[test]
fn mag_vector_as_full_attitude_reference() {
    println!("\n2.2e 零件验证：磁场矢量作姿态参考的误差（硬铁补偿前/后）");
    let bn = norm(B_WORLD);
    let hn = norm(HARD_IRON);
    let ratio = hn / bn;
    let worst_pred = ratio.atan().to_degrees();
    println!("世界磁场 模长 {bn}   硬铁 模长 {hn}   比值 {ratio}");
    println!("解析预期 最坏方向偏差 {} 度\n", worst_pred);
    println!("{:>10} {:>10} {:>10} | {:>14} {:>14}", "roll(度)", "pitch(度)", "yaw(度)", "未补偿误差", "补偿后误差");
    println!("{}", "-".repeat(68));

    let (mut worst_raw, mut worst_comp) = (0.0f32, 0.0f32);
    for &(r, p, y) in &[
        (0.0f32, 0.0f32, 0.0f32),
        (0.0, 0.0, 45.0),
        (0.0, 0.0, 90.0),
        (0.0, 0.0, 180.0),
        (10.0, 0.0, 0.0),
        (0.0, 10.0, 0.0),
        (20.0, -15.0, 30.0),
        (-30.0, 25.0, -120.0),
    ] {
        let q = Quaternion::from_euler(Radian(r.to_radians()), Radian(p.to_radians()), Radian(y.to_radians()));
        let e_raw = ref_error_deg(q, false);
        let e_comp = ref_error_deg(q, true);
        worst_raw = worst_raw.max(e_raw);
        worst_comp = worst_comp.max(e_comp);
        println!("{:>10.1} {:>10.1} {:>10.1} | {:>14.4} {:>14.6}", r, p, y, e_raw, e_comp);
    }
    println!("{}", "-".repeat(68));
    println!("最坏 未补偿 {} 度   补偿后 {} 度", worst_raw, worst_comp);
    println!("\n判据：补偿后应 << 1°（无噪声/无软铁的理想要素）；未补偿应达到 ~40° 量级。");

    // ---- 判据（零件级，非验收级）----
    assert!(
        worst_raw > 5.0,
        "未补偿时方向误差应显著（预期 ~40° 量级），实测最坏 {worst_raw:.2}° —— 若很小说明本测例没构造出硬铁影响"
    );
    assert!(
        worst_comp < 0.1,
        "补偿后方向误差应为 0（无噪声/无软铁的理想条件），实测 {worst_comp:.4}°"
    );
    // 补偿后的残差在真实系统里还会叠加：磁噪声（realistic() 有 σ）、软铁（本仓未建模）、
    // 以及标定残差 ⇒ 2.2e 的实际精度需在闭环里再量（下一步）。
}
