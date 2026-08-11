//! 极简 WebSocket 服务端（RFC6455），零外部依赖（仅 std）。
//!
//! 仅实现本仿真服务所需的最小子集：
//! - 握手（Sec-WebSocket-Accept = base64(sha1(key + GUID))）
//! - 服务端→客户端：未掩码文本(0x1) / 二进制(0x2) / pong(0xA) / close(0x8)
//! - 客户端→服务端：解析掩码文本(0x1) / 二进制(0x2) / close(0x8) / ping(0x9)
//!
//! 帧负载 < 2^31，支持 126/65536 长度前缀；不支持分片续帧（单帧即整条消息）。

use std::io::{Read, Write};
use std::net::TcpStream;

const GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// 服务端握手：从已读取的 HTTP 请求字节中解析 `Sec-WebSocket-Key`，回 101。
/// 注意：**请求字节必须由调用方已就绪**（不得在握手内再从流读取，否则会与前置
/// HTTP 解析重复消费）。返回 `Err` 表示缺少必要头。
pub fn handshake(stream: &mut TcpStream, request: &[u8]) -> std::io::Result<()> {
    let text = String::from_utf8_lossy(request);
    let mut key = None;
    for line in text.lines() {
        if let Some(v) = line.strip_prefix("Sec-WebSocket-Key:") {
            key = Some(v.trim().to_string());
        }
    }
    let key = key.ok_or_else(|| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, "缺少 Sec-WebSocket-Key")
    })?;
    let accept = accept_key(&key);

    let resp = format!(
        "HTTP/1.1 101 Switching Protocols\r\n\
         Upgrade: websocket\r\n\
         Connection: Upgrade\r\n\
         Sec-WebSocket-Accept: {}\r\n\r\n",
        accept
    );
    stream.write_all(resp.as_bytes())?;
    stream.flush()?;
    Ok(())
}

/// 计算 Sec-WebSocket-Accept。
fn accept_key(key: &str) -> String {
    let mut ctx = Sha1::new();
    ctx.update(key.as_bytes());
    ctx.update(GUID.as_bytes());
    let digest = ctx.finish();
    base64_encode(&digest)
}

/// 服务端发送一帧（未掩码）。opcode: 0x1 文本, 0x2 二进制, 0x8 close, 0xA pong。
pub fn send_frame(stream: &mut TcpStream, opcode: u8, payload: &[u8]) -> std::io::Result<()> {
    let mut hdr = Vec::with_capacity(10);
    hdr.push(0x80 | opcode); // FIN=1
    let len = payload.len();
    if len < 126 {
        hdr.push(len as u8);
    } else if len < 65536 {
        hdr.push(126);
        hdr.extend_from_slice(&[(len >> 8) as u8, (len & 0xFF) as u8]);
    } else {
        hdr.push(127);
        let mut l = [0u8; 8];
        for i in 0..8 {
            l[i] = ((len as u64) >> (56 - i * 8)) as u8;
        }
        hdr.extend_from_slice(&l);
    }
    stream.write_all(&hdr)?;
    stream.write_all(payload)?;
    stream.flush()?;
    Ok(())
}

/// 读取一帧（客户端→服务端，必带掩码）。返回 (opcode, payload)。
/// 遇 close 返回 opcode=0x8。
pub fn read_frame(stream: &mut TcpStream) -> std::io::Result<(u8, Vec<u8>)> {
    let mut b0 = [0u8; 1];
    stream.read_exact(&mut b0)?;
    let opcode = b0[0] & 0x0F;
    let masked = (b0[0] & 0x80) != 0;

    let mut b1 = [0u8; 1];
    stream.read_exact(&mut b1)?;
    let mut len = (b1[0] & 0x7F) as usize;
    if len == 126 {
        let mut ext = [0u8; 2];
        stream.read_exact(&mut ext)?;
        len = ((ext[0] as usize) << 8) | ext[1] as usize;
    } else if len == 127 {
        let mut ext = [0u8; 8];
        stream.read_exact(&mut ext)?;
        len = 0;
        for i in 0..8 {
            len = (len << 8) | ext[i] as usize;
        }
    }
    let mut mask = [0u8; 4];
    if masked {
        stream.read_exact(&mut mask)?;
    }
    let mut payload = vec![0u8; len];
    stream.read_exact(&mut payload)?;
    if masked {
        for i in 0..len {
            payload[i] ^= mask[i & 3];
        }
    }
    Ok((opcode, payload))
}

// ---------------- SHA1 + Base64（零依赖实现） ----------------

struct Sha1 {
    h: [u32; 5],
    msg: Vec<u8>,
    len: u64,
}

impl Sha1 {
    fn new() -> Self {
        Self {
            h: [0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0],
            msg: Vec::new(),
            len: 0,
        }
    }
    fn update(&mut self, data: &[u8]) {
        self.len += data.len() as u64;
        self.msg.extend_from_slice(data);
    }
    fn finish(mut self) -> [u8; 20] {
        let ml = self.len * 8;
        self.msg.push(0x80);
        while self.msg.len() % 64 != 56 {
            self.msg.push(0);
        }
        self.msg.extend_from_slice(&[
            (ml >> 56) as u8,
            (ml >> 48) as u8,
            (ml >> 40) as u8,
            (ml >> 32) as u8,
            (ml >> 24) as u8,
            (ml >> 16) as u8,
            (ml >> 8) as u8,
            ml as u8,
        ]);
        for chunk in self.msg.chunks(64) {
            let mut w = [0u32; 80];
            for i in 0..16 {
                w[i] = ((chunk[i * 4] as u32) << 24)
                    | ((chunk[i * 4 + 1] as u32) << 16)
                    | ((chunk[i * 4 + 2] as u32) << 8)
                    | chunk[i * 4 + 3] as u32;
            }
            for i in 16..80 {
                w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
            }
            let (mut a, mut b, mut c, mut d, mut e) =
                (self.h[0], self.h[1], self.h[2], self.h[3], self.h[4]);
            for i in 0..80 {
                let (f, k) = if i < 20 {
                    ((b & c) | ((!b) & d), 0x5A827999)
                } else if i < 40 {
                    (b ^ c ^ d, 0x6ED9EBA1)
                } else if i < 60 {
                    ((b & c) | (b & d) | (c & d), 0x8F1BBCDC)
                } else {
                    (b ^ c ^ d, 0xCA62C1D6)
                };
                let tmp = a
                    .rotate_left(5)
                    .wrapping_add(f)
                    .wrapping_add(e)
                    .wrapping_add(k)
                    .wrapping_add(w[i]);
                e = d;
                d = c;
                c = b.rotate_left(30);
                b = a;
                a = tmp;
            }
            self.h[0] = self.h[0].wrapping_add(a);
            self.h[1] = self.h[1].wrapping_add(b);
            self.h[2] = self.h[2].wrapping_add(c);
            self.h[3] = self.h[3].wrapping_add(d);
            self.h[4] = self.h[4].wrapping_add(e);
        }
        let mut out = [0u8; 20];
        for i in 0..5 {
            out[i * 4] = (self.h[i] >> 24) as u8;
            out[i * 4 + 1] = (self.h[i] >> 16) as u8;
            out[i * 4 + 2] = (self.h[i] >> 8) as u8;
            out[i * 4 + 3] = self.h[i] as u8;
        }
        out
    }
}

fn base64_encode(input: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for chunk in input.chunks(3) {
        let b0 = chunk[0];
        let b1 = *chunk.get(1).unwrap_or(&0);
        let b2 = *chunk.get(2).unwrap_or(&0);
        let n = ((b0 as u32) << 16) | ((b1 as u32) << 8) | (b2 as u32);
        out.push(T[((n >> 18) & 0x3F) as usize] as char);
        out.push(T[((n >> 12) & 0x3F) as usize] as char);
        if chunk.len() > 1 {
            out.push(T[((n >> 6) & 0x3F) as usize] as char);
        } else {
            out.push('=');
        }
        if chunk.len() > 2 {
            out.push(T[(n & 0x3F) as usize] as char);
        } else {
            out.push('=');
        }
    }
    out
}
