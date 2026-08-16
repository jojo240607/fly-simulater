//! P0-A IMU 标准误差树回归测试：验证 `SensorConfig::realistic()` 下
//! 加速度计/陀螺仪的高阶误差（随机游走、偏置不稳定性、振动整流）被正确注入，
//! 且结果有界、确定性可复现。
//!
//! 用法：cargo test --test imu_error_tree --features phy

#![cfg(feature = "phy")]

use fly_sim_core::sensor::{SensorConfig, SensorModel};

const DT: f64 = 0.004;

fn mean_std(xs: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    let v = xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n;
    (m, v.sqrt())
}

/// 在相同真实值下跑 N 步，收集某轴加速度计读数。
fn accel_series(cfg: &SensorConfig, axis: usize, n: usize) -> Vec<f64> {
    let mut s = SensorModel::new(cfg.clone(), DT);
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let (imu, _pos) = s.process(
            DT,
            [0.0, 0.0, 9.81], // 真值比力（悬停向上 9.81）
            [0.0, 0.0, 0.0],
            [0.0, 0.0, -5.0],
            [0.0, 0.0, 0.0],
        );
        out.push(imu.accel[axis].0 as f64);
    }
    out
}

#[test]
fn realistic_imu_is_deterministic() {
    // 同种子两次运行必须逐字节一致（确定性可复现）。
    let a = accel_series(&SensorConfig::realistic(), 2, 1000);
    let b = accel_series(&SensorConfig::realistic(), 2, 1000);
    assert!(a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() < 1e-12));
}

#[test]
fn realistic_imu_has_nonzero_noise() {
    // realistic() 必须产生非零噪声（否则误差树未生效）。
    let s = accel_series(&SensorConfig::realistic(), 2, 50_000);
    let (_m, sd) = mean_std(&s);
    assert!(sd > 0.02, "IMU noise std too small (error tree not active): {}", sd);
}

#[test]
fn default_imu_is_clean() {
    // 默认配置（噪声全 0）应保持干净，SIL 场景不受影响。
    let s = accel_series(&SensorConfig::default(), 2, 1000);
    let (_m, sd) = mean_std(&s);
    assert!(sd < 1e-9, "default IMU should be noiseless: {}", sd);
}

#[test]
fn gyro_bias_instability_bounded() {
    // 陀螺偏置不稳定性：长时 std 有界（不发散到 clamp 边界）。
    let cfg = SensorConfig::realistic();
    let mut s = SensorModel::new(cfg.clone(), DT);
    let n = 300_000; // ~1200s
    let mut gx = Vec::with_capacity(n);
    for _ in 0..n {
        let (imu, _pos) = s.process(DT, [0.0; 3], [0.0; 3], [0.0, 0.0, -5.0], [0.0; 3]);
        gx.push(imu.gyro[0].0 as f64);
    }
    let (_m, sd) = mean_std(&gx);
    // 有界：远小于 clamp 0.05，且显著大于 0（非冻结）。
    assert!(sd > 0.1 * cfg.gyro_bias_inst, "BI frozen: {}", sd);
    assert!(sd < 0.05, "BI diverged to clamp: {}", sd);
}
