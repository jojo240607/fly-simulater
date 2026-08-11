//! 阶段 6：CSV 日志写出（runner 侧 I/O，依赖 `fly-sim-core` 的 `LogRow` 纯数据）。
//!
//! 序列化逻辑放在 runner（bin）侧，核心只产出 `LogRow`。这样核心保持纯计算，
//! 不被任何文件/格式绑定。

use fly_sim_core::LogRow;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// CSV 表头（28 列，与 `LogRow` 字段一一对应）。
pub const LOG_HEADER: &str =
    "step,t,\
     true_n,true_e,true_d,true_vn,true_ve,true_vd,true_qw,true_qx,true_qy,true_qz,true_wx,true_wy,true_wz,\
     est_n,est_e,est_d,est_vn,est_ve,est_vd,est_qw,est_qx,est_qy,est_qz,est_wx,est_wy,est_wz,\
     m0,m1,m2,m3,\
     imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz";

/// CSV 写出器（行缓冲，析构自动 flush）。
pub struct CsvLogger {
    w: BufWriter<File>,
    ok: bool,
}

impl CsvLogger {
    pub fn new<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let f = File::create(path)?;
        let mut w = BufWriter::new(f);
        writeln!(w, "{}", LOG_HEADER)?;
        Ok(Self { w, ok: true })
    }

    /// 写入一帧 `LogRow`。
    pub fn write(&mut self, row: &LogRow) -> std::io::Result<()> {
        if !self.ok {
            return Ok(());
        }
        let t = &row.true_state;
        let e = &row.est_state;
        let c = &row.cmd;
        let imu = &row.imu;
        // 防御性：任何非有限值替换为 0.0，避免下游 CSV 解析因 "NaN"/"inf" 文本而出错。
        let f = |v: f64| -> f64 { if v.is_finite() { v } else { 0.0 } };
        let s = format!(
            "{},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            row.step, row.t,
            f(t.pos[0].0 as f64), f(t.pos[1].0 as f64), f(t.pos[2].0 as f64),
            f(t.vel[0].0 as f64), f(t.vel[1].0 as f64), f(t.vel[2].0 as f64),
            f(t.att.w as f64), f(t.att.x as f64), f(t.att.y as f64), f(t.att.z as f64),
            f(t.omega[0].0 as f64), f(t.omega[1].0 as f64), f(t.omega[2].0 as f64),
            f(e.pos[0].0 as f64), f(e.pos[1].0 as f64), f(e.pos[2].0 as f64),
            f(e.vel[0].0 as f64), f(e.vel[1].0 as f64), f(e.vel[2].0 as f64),
            f(e.att.w as f64), f(e.att.x as f64), f(e.att.y as f64), f(e.att.z as f64),
            f(e.omega[0].0 as f64), f(e.omega[1].0 as f64), f(e.omega[2].0 as f64),
            f(c.motor[0] as f64), f(c.motor[1] as f64), f(c.motor[2] as f64), f(c.motor[3] as f64),
            f(imu.accel[0].0 as f64), f(imu.accel[1].0 as f64), f(imu.accel[2].0 as f64),
            f(imu.gyro[0].0 as f64), f(imu.gyro[1].0 as f64), f(imu.gyro[2].0 as f64),
        );
        // 字段数自检：正常 38 列；若异常（理论不应发生）补齐零，避免下游解析错位。
        let cols = s.split(',').count();
        if cols != 38 {
            eprintln!(
                "[log] WARN step {}: malformed row ({} cols, expected 38) — padding",
                row.step, cols
            );
            let mut padded = s;
            while padded.split(',').count() < 38 {
                padded.push_str(",0.000000");
            }
            writeln!(self.w, "{}", padded)?;
            return Ok(());
        }
        writeln!(self.w, "{}", s)?;
        // 每帧落盘：main 用 std::process::exit 会跳过 Drop（不 flush BufWriter），
        // 逐帧 flush 确保日志完整（SIL 非实时场景，开销可接受）。
        self.w.flush()?;
        Ok(())
    }

    pub fn flush(&mut self) {
        let _ = self.w.flush();
    }
}

impl Drop for CsvLogger {
    fn drop(&mut self) {
        self.flush();
    }
}
