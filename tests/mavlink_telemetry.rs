//! P2-2 续：MAVLink 遥测下行链路自验证（无头）。
//!
//! 把已知 [`VehicleState`] 经 [`MavlinkBridge`] 编码成 MAVLink v2 遥测流，
//! 再经 [`loopback_telemetry`]（标准 MAVLink 解析器，含 CRC_EXTRA）回环解析，
//! 断言：
//! 1) 6 帧全部被成功解析（CRC_EXTRA 字节级兼容，证明仿真端能"说"标准 MAVLink）；
//! 2) msg_id 集合正确（HEARTBEAT/ATTITUDE/LOCAL_POSITION_NED/SYS_STATUS/VFR_HUD/
//!    GLOBAL_POSITION_INT）；
//! 3) 载荷字段与原始状态一致（ATTITUDE 的 roll/pitch/yaw、LOCAL_POSITION_NED 的
//!    NED 位置）——证明遥测下行语义正确。
//!
//! 用法：cargo test --test mavlink_telemetry -- --nocapture

use fly_sim_core::mavlink::{loopback_telemetry, MavlinkBridge};
use flyctrl_core::comm::mavlink::msg_id;
use flyctrl_core::units::{Meter, MeterPerSecond, Radian, RadianPerSecond};
use flyctrl_core::vehicle::{Quaternion, VehicleState};

fn known_state() -> VehicleState {
    VehicleState {
        time_boot_ms: 1234,
        pos: [Meter(-1.5), Meter(2.5), Meter(-5.0)],
        vel: [MeterPerSecond(0.3), MeterPerSecond(-0.4), MeterPerSecond(0.1)],
        att: Quaternion::from_euler(Radian(0.1), Radian(-0.2), Radian(1.57)), // roll/pitch/yaw
        omega: [RadianPerSecond(0.01), RadianPerSecond(-0.02), RadianPerSecond(0.03)],
        airspeed: MeterPerSecond(1.2),
        accel_bias: [0.0; 3],
    }
}

fn le_f32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

#[test]
fn mavlink_telemetry_roundtrips() {
    let st = known_state();
    let mut bridge = MavlinkBridge::new(7, 1); // sys_id=7
    let stream = bridge.emit_telemetry(&st, 0, true, 42, true);

    // 回环解析：标准 MAVLink 解码（含 CRC_EXTRA）。
    let parsed = loopback_telemetry(&stream);
    println!(
        "[mavlink] stream_len={} bytes, parsed frames={}",
        stream.len(),
        parsed.len()
    );

    // 1) 6 帧全部成功解析（CRC_EXTRA 兼容）。
    assert_eq!(parsed.len(), 6, "应解析出 6 帧，实得 {}", parsed.len());

    // 2) msg_id 集合正确且唯一。
    let ids: std::collections::HashSet<u32> = parsed.iter().map(|(id, _)| *id).collect();
    for want in [
        msg_id::HEARTBEAT,
        msg_id::ATTITUDE,
        msg_id::LOCAL_POSITION_NED,
        msg_id::SYS_STATUS,
        msg_id::VFR_HUD,
        msg_id::GLOBAL_POSITION_INT,
    ] {
        assert!(ids.contains(&want), "缺失 msg_id={}", want);
    }

    // 3) 载荷字段与原始状态一致。
    for (id, payload) in &parsed {
        match *id {
            msg_id::ATTITUDE => {
                // ATTITUDE payload: time_boot_ms i32, roll f32@4, pitch f32@8, yaw f32@12
                let roll = le_f32(payload, 4);
                let pitch = le_f32(payload, 8);
                let yaw = le_f32(payload, 12);
                assert!((roll - st.att.roll()).abs() < 1e-3, "roll 不一致 {} vs {}", roll, st.att.roll());
                assert!((pitch - st.att.pitch()).abs() < 1e-3, "pitch 不一致 {} vs {}", pitch, st.att.pitch());
                assert!((yaw - st.att.yaw()).abs() < 1e-3, "yaw 不一致 {} vs {}", yaw, st.att.yaw());
            }
            msg_id::LOCAL_POSITION_NED => {
                // LOCAL_POSITION_NED payload: time i32, x@4, y@8, z@12, vx@16, vy@20, vz@24
                let x = le_f32(payload, 4);
                let y = le_f32(payload, 8);
                let z = le_f32(payload, 12);
                assert!((x - st.pos[0].0).abs() < 1e-3, "x 不一致 {} vs {}", x, st.pos[0].0);
                assert!((y - st.pos[1].0).abs() < 1e-3, "y 不一致 {} vs {}", y, st.pos[1].0);
                assert!((z - st.pos[2].0).abs() < 1e-3, "z 不一致 {} vs {}", z, st.pos[2].0);
            }
            msg_id::SYS_STATUS => {
                // 机载传感器健康位 = 0x1F（5 路 OK）。
                let present = i32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                assert_eq!(present, 0x1F, "传感器健康位应为 0x1F");
            }
            _ => {}
        }
    }
}

#[test]
fn mavlink_stream_parser_is_incremental() {
    // 逐字节喂入也应正确组帧（模拟串口字节流）。
    let st = known_state();
    let mut bridge = MavlinkBridge::new(1, 1);
    let stream = bridge.emit_telemetry(&st, 0, true, 10, true);

    let mut parser = fly_sim_core::mavlink::MavlinkStreamParser::new();
    let mut n_frames = 0usize;
    for b in &stream {
        n_frames += parser.feed(&[*b]).len(); // 一次喂一字节
    }
    assert_eq!(n_frames, 6, "增量解析应得到 6 帧，实得 {}", n_frames);
}
