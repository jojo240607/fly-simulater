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
        let line = format!(
            "{}", row.step
        );
        // 用 writeln 拼字段避免长 format 串出错；直接逐字段写更清晰。
        let _ = line;
        writeln!(
            self.w,
            "{},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            row.step, row.t,
            t.pos[0].0, t.pos[1].0, t.pos[2].0,
            t.vel[0].0, t.vel[1].0, t.vel[2].0,
            t.att.w, t.att.x, t.att.y, t.att.z,
            t.omega[0].0, t.omega[1].0, t.omega[2].0,
            e.pos[0].0, e.pos[1].0, e.pos[2].0,
            e.vel[0].0, e.vel[1].0, e.vel[2].0,
            e.att.w, e.att.x, e.att.y, e.att.z,
            e.omega[0].0, e.omega[1].0, e.omega[2].0,
            c.motor[0], c.motor[1], c.motor[2], c.motor[3],
            imu.accel[0].0, imu.accel[1].0, imu.accel[2].0,
            imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0,
        )?;
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
