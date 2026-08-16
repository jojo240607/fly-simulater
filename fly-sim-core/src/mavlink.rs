//! P2-2 续：MAVLink 标准消息协议链路（遥测下行）。
//!
//! 把仿真机体的 [`VehicleState`] 按标准 MAVLink v2 编码成遥测流（HEARTBEAT /
//! ATTITUDE / LOCAL_POSITION_NED / SYS_STATUS / VFR_HUD / GLOBAL_POSITION_INT），
//! 经 [`LoopbackLink`] 字节链路发出，可被标准地面站（QGC/PX4）解析——
//! 补齐路线图差距清单 #8（"无标准消息协议 MAVLink"）。
//!
//! 复用 `flyctrl-core` 既有的 MAVLink v2 编解码（`crc16_x25` + `CRC_EXTRA` 字节级
//! 兼容），本模块只做"仿真状态 → 帧流 → 链路 → 回环解析"的编排 + 自验证。
//!
//! 注意：本模块是 **host 侧遥测桥**（仿真/地面站联调用），非嵌入式链路实现
//! （后者在 `flyctrl-core::comm::link::stm32f407` 占位，落地时接 joc-base HAL）。

use flyctrl_core::comm::link::{Frame, Link, LoopbackLink, MAX_FRAME_LEN};
use flyctrl_core::comm::mavlink::{
    self, encode_attitude, encode_global_position_int, encode_heartbeat, encode_local_pos,
    encode_sys_status, encode_vfr_hud,
};
use flyctrl_core::vehicle::{VehicleState, MeterPerSecond};

/// 标准 MAVLink 遥测下行桥：维护 sys_id/comp_id/seq，把 [`VehicleState`] 编码成帧流。
#[derive(Clone, Debug)]
pub struct MavlinkBridge {
    sys_id: u8,
    comp_id: u8,
    seq: u8,
    fb: [u8; MAX_FRAME_LEN],
}

impl MavlinkBridge {
    pub fn new(sys_id: u8, comp_id: u8) -> Self {
        Self {
            sys_id,
            comp_id,
            seq: 0,
            fb: [0u8; MAX_FRAME_LEN],
        }
    }

    /// 当前帧序号（跨帧递增，MAVLink 要求每帧 +1 回绕）。
    pub fn seq(&self) -> u8 {
        self.seq
    }

    /// 编码一帧 LOCAL_POSITION_NED（机体 NED 位置/速度）。
    fn push_local_pos(&mut self, st: &VehicleState, out: &mut Vec<u8>) {
        let n = encode_local_pos(st, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 编码一帧 ATTITUDE（欧拉角 + 角速度）。
    fn push_attitude(&mut self, st: &VehicleState, out: &mut Vec<u8>) {
        let n = encode_attitude(st, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 编码一帧 SYS_STATUS（机载传感器健康位，统一 0x1F = 5 路 OK）。
    fn push_sys_status(&mut self, sensors_ok: bool, out: &mut Vec<u8>) {
        let n = encode_sys_status(sensors_ok, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 编码一帧 VFR_HUD（地速 / 航向 / 油门%）。
    fn push_vfr_hud(&mut self, st: &VehicleState, throttle_pct: u16, out: &mut Vec<u8>) {
        let n = encode_vfr_hud(st, throttle_pct, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 编码一帧 GLOBAL_POSITION_INT（相对高度 + 航向，GPS 纬度/经度为 0）。
    fn push_global_pos(&mut self, st: &VehicleState, out: &mut Vec<u8>) {
        let n = encode_global_position_int(st, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 编码一帧 HEARTBEAT（模式/解锁态）。
    fn push_heartbeat(&mut self, mode: u8, armed: bool, out: &mut Vec<u8>) {
        let n = encode_heartbeat(mode, armed, self.seq, &mut self.fb);
        out.extend_from_slice(&self.fb[..n]);
        self.seq = self.seq.wrapping_add(1);
    }

    /// 生成一拍完整遥测流（6 帧，顺序：HB/ATT/LOCAL/SYS/VFR/GLOBAL）。
    ///
    /// 返回字节流（可直接喂给 [`LoopbackLink::send_frame`] 或文件/串口）。
    /// `mode`/`armed`/`throttle_pct` 由上层（飞控状态机）提供；`sensors_ok` 标记
    /// 机载传感器健康（这里统一置 0x1F = IMU/GPS/罗盘/气压/空速全 OK）。
    pub fn emit_telemetry(
        &mut self,
        st: &VehicleState,
        mode: u8,
        armed: bool,
        throttle_pct: u16,
        sensors_ok: bool,
    ) -> Vec<u8> {
        let mut out = Vec::with_capacity(6 * 40);
        self.push_heartbeat(mode, armed, &mut out);
        self.push_attitude(st, &mut out);
        self.push_local_pos(st, &mut out);
        self.push_sys_status(sensors_ok, &mut out);
        self.push_vfr_hud(st, throttle_pct, &mut out);
        self.push_global_pos(st, &mut out);
        out
    }
}

/// 增量式 MAVLink v2 字节流解析器：逐字节喂入，命中完整帧即回调。
///
/// 解析规则（与 `LoopbackLink::recv_frame` 同语义）：以 `0xFD` 起始，读 9 字节头部，
/// 按 `len` 域收齐 `10 + len + 2` 字节为一帧。仅做**组帧**，CRC/CRC_EXTRA 校验交给
/// `flyctrl_core::comm::mavlink::decode`（权威）。
pub struct MavlinkStreamParser {
    buf: [u8; MAX_FRAME_LEN],
    len: usize,
    need: usize, // 当前帧还需收集的字节数（0=等待起始符）
}

impl MavlinkStreamParser {
    pub fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_LEN],
            len: 0,
            need: 0,
        }
    }

    /// 喂入一批字节，返回本批解析出的完整帧（已校验长度边界，CRC 由调用方 decode）。
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for &b in bytes {
            if self.need == 0 {
                // 等待起始符
                if b == mavlink::MAVLINK_MAGIC {
                    self.buf[0] = b;
                    self.len = 1;
                    self.need = 9; // 还需 9 字节头部
                }
                continue;
            }
            // 收集中：先收齐 9 字节头部以确定 payload 长度
            if self.len < 10 {
                self.buf[self.len] = b;
                self.len += 1;
                self.need -= 1;
                if self.len == 10 {
                    let plen = self.buf[1] as usize;
                    let total = 10 + plen + 2;
                    self.need = total - self.len; // 还需 payload + 2 CRC
                }
            } else {
                self.buf[self.len] = b;
                self.len += 1;
                self.need -= 1;
                if self.need == 0 {
                    // 一帧完整
                    frames.push(Frame::from_bytes(&self.buf[..self.len]));
                    self.len = 0;
                    self.need = 0;
                }
            }
        }
        frames
    }
}

/// 便捷：把遥测字节流经 [`LoopbackLink`] 回环，解析回 (msgid, payload) 列表。
///
/// 用于自验证——证明仿真端发出的遥测能被标准 MAVLink 解析器（含 CRC_EXTRA）接受。
pub fn loopback_telemetry(stream: &[u8]) -> Vec<(u32, Vec<u8>)> {
    let mut parser = MavlinkStreamParser::new();
    let frames = parser.feed(stream); // 先按字节流组帧
    let mut out = Vec::new();
    let mut link = LoopbackLink::new();
    for f in &frames {
        link.send_frame(f);
        let back = link.recv_frame();
        if back.is_empty() {
            continue;
        }
        if let Some((id, payload)) = mavlink::decode(&back) {
            out.push((id, payload.to_vec()));
        }
    }
    out
}
