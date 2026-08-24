//! 主机侧 MAVLink 链路（UDP，SIL）。
//!
//! 让仿真作为 MAVLink 端点，可被真实地面站（QGroundControl / MAVProxy）经 UDP 连接。
//! 仅依赖 `std::net`，不参与嵌入式构建（编译期用 feature 隔离，但本仿真工程恒为主机）。

use flyctrl_core::comm::link::Link;
use mavlink_core::frame::{Frame, MAX_FRAME_LEN};
use std::net::{SocketAddr, UdpSocket};

/// UDP 链路：每个 MAVLink 帧作为一个 datagram 收发。
/// 接收端 `recv_from` 不限源（GCS 可从任意端口发包），发送端 `send_to` 默认对端。
pub struct UdpLink {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    buf: [u8; 2048],
}

impl UdpLink {
    /// 绑定监听地址（如 `0.0.0.0:14551`），非阻塞以便主循环不卡在 recv。
    pub fn bind(listen: &str) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(listen)?;
        // 非阻塞：poll 在没有入站帧时立即返回空帧，不阻塞控制循环。
        let _ = socket.set_nonblocking(true);
        Ok(Self { socket, peer: None, buf: [0u8; 2048] })
    }

    /// 设定默认对端（GCS）地址，使 send_frame 走 `send_to`（不调用 connect，
    /// 从而 recv 仍可从任意源接收 GCS 命令，即便 GCS 从随机端口发包）。
    pub fn connect_peer(&mut self, peer: &str) -> std::io::Result<()> {
        self.peer = Some(peer.parse().map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::Other, format!("bad peer addr: {}", e))
        })?);
        Ok(())
    }
}

impl Link for UdpLink {
    fn send_frame(&mut self, frame: &Frame) -> usize {
        let n = frame.len.min(MAX_FRAME_LEN);
        match self.peer {
            Some(addr) => self.socket.send_to(&frame.data[..n], addr).unwrap_or(0),
            None => 0,
        }
    }

    fn recv_frame(&mut self) -> Frame {
        match self.socket.recv_from(&mut self.buf) {
            Ok((n, _src)) if n > 0 => {
                let mut d = [0u8; MAX_FRAME_LEN];
                let m = n.min(MAX_FRAME_LEN);
                d[..m].copy_from_slice(&self.buf[..m]);
                Frame { data: d, len: m }
            }
            _ => Frame::default(),
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}
