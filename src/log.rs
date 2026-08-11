//! 阶段 6：CSV 日志与回放。
//!
//! 把每帧真值(NED)/估计(NED)/指令(CMD)/IMU 写入 CSV，供复现、回归与控制器量化对比。
//! 格式：逗号分隔，首行表头。所有量单位明确（m / m/s / rad / s / 归一化油门）。

use std::fs::File;
use std::io::Write;
use std::path::Path;

use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, VehicleState};

/// 单帧日志条目（阶段 6 字段全集）。
pub struct LogRow {
    pub step: u64,
    pub t: f64,
    pub true_state: VehicleState, // 物理引擎真值（NED）
    pub est_state: VehicleState,  // 估计器输出（NED）
    pub cmd: ActuatorCmd,
    pub imu: ImuSample,
}

/// 轻量 CSV 记录器（带表头，缓冲刷盘）。
pub struct CsvLogger {
    file: File,
}

impl CsvLogger {
    const HEADER: &'static str = "step,t,true_n,true_e,true_d,est_n,est_e,est_d,\
vx,vy,vz,attw,attx,atty,attz,omg_p,omg_q,omg_r,\
cmd0,cmd1,cmd2,cmd3,accelx,accely,accelz,gyrox,gyroy,gyroz";

    pub fn new<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let mut file = File::create(path)?;
        writeln!(file, "{}", Self::HEADER)?;
        Ok(Self { file })
    }

    pub fn write(&mut self, r: &LogRow) -> std::io::Result<()> {
        let ts = &r.true_state;
        let es = &r.est_state;
        let c = r.cmd.motor;
        let a = r.imu.accel;
        let g = r.imu.gyro;
        writeln!(
            self.file,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            r.step,
            r.t,
            ts.pos[0].0, ts.pos[1].0, ts.pos[2].0,
            es.pos[0].0, es.pos[1].0, es.pos[2].0,
            ts.vel[0].0, ts.vel[1].0, ts.vel[2].0,
            ts.att.w, ts.att.x, ts.att.y, ts.att.z,
            ts.omega[0].0, ts.omega[1].0, ts.omega[2].0,
            c[0], c[1], c[2], c[3],
            a[0].0, a[1].0, a[2].0,
            g[0].0, g[1].0, g[2].0,
        )
    }

    /// 强制刷盘（场景结束调用，避免进程退出丢失尾部缓冲）。
    pub fn flush(&mut self) -> std::io::Result<()> {
        self.file.flush()
    }
}
