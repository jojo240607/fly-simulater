//! HIL 链路层：经 USB CDC（虚拟串口）与真实飞控 MCU 的 MAVLink 数据交互。
//!
//! 闭环方向（与 `tools/hil_mock.py` 同构，验证过的边界直接复用）：
//!   - 上行（PC -> 飞控）：`HIL_SENSOR`(107) 注入 IMU 真值 + 气压高度；
//!     `SET_POSITION_TARGET_LOCAL_NED`(84) 同时下发「期望状态」与「GPS 位置/速度真值」
//!     （固件 uplink 用它写 SENSOR_FRAME.gps 与 G_HIL_SETPOINT，见 uplink.rs）。
//!   - 下行（飞控 -> PC）：`HIL_ACTUATOR_CONTROLS`(93) 回传四电机归一化推力，
//!     用于驱动物理引擎；`ATTITUDE`(30) / `LOCAL_POSITION_NED`(32) 提供 MCU 估计，
//!     供 Web UI 遥测显示（渲染仍用物理真值）。
//!
//! 端口边界（与 hil_mock.py 一致，均属枚举/时机问题而非代码问题）：
//!   - USB-CDC 用 VID/PID 匹配（STM32 0483:5740），区别于日志口 CH340(1A86:7523)；
//!   - reset/重插后枚举需数秒，调用方需轮询等待；本模块只负责"打开后"的稳定通信；
//!   - MCU bulk-OUT 端点收满 64B 后需 ~ms 级时间供 uplink 消费并 re-arm，
//!     多包（如 HIL_SENSOR 74B=64+10）分块间必须留间隔，否则命中 NAK 写超时。

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use flyctrl_core::comm::mavlink::{self, Frame, MAX_FRAME_LEN};
use flyctrl_core::vehicle::ImuSample;

/// STM32 USB-CDC 的 VID/PID（HIL 上行口）。
pub const VID_STM32: u16 = 0x0483;
pub const PID_STM32_CDC: u16 = 0x5740;
/// HIL 上行口波特率（USB-CDC 恒为 115200）。
const BAUD: u32 = 115200;

/// MAV_MODE_FLAG_HIL_ENABLED（心跳 base_mode 位，标识当前为 HIL 仿真模式）。
pub const HIL_FLAG: u8 = 0x20;
/// MAV_CMD_COMPONENT_ARM_DISARM（解锁/上锁指令）。
const MAV_CMD_ARM_DISARM: u16 = 400;
const MAV_RESULT_ACCEPTED: u8 = 0;

/// MCU bulk-OUT 端点收满 64B 后需 ~ms 级时间供 uplink 消费并 re-arm；
/// 多包分块间留此间隔，避免命中 NAK 写超时（与 hil_mock.py --gap 同源）。
/// 原 15ms 使每物理步 HIL_SENSOR+SET_POSITION 两处分块各睡 15ms → 8 步/帧 ~240ms，
/// 远超飞控 4ms 控制周期（实测 1s 仿真耗时 ~9s 真实时间）。降为 2ms 后每步 ~4ms，
/// 近似实时；若命中 NAK 由 send_chunked 内 50ms 重试兜底。
///
/// 【HIL 发散根因验证】2ms 分块间隔仍使每物理步 HIL_SENSOR 睡 2ms、nav 步再叠
/// HIL_GPS+SET_POSITION 共 4-6 块 → 物理步实际耗时 >4ms，IMU 注入到 MCU 的到达率
/// < 控制拍率（250Hz）→ MCU 大量拍 `imu=None` → sample-and-hold 用旧 IMU → 姿态
/// 估计滞后物理旋转 → 控制反馈错误 → HIL 闭环发散（SIL 全同步无此问题，稳定悬停）。
/// 置 0 验证该假设：分块不再睡眠（NAK 由 50ms 重试兜底），物理步贴近 4ms 实时。
const OUT_CHUNK_GAP: Duration = Duration::ZERO;

// #region debug-point helper:dbg-report
/// 写入一条 HIL 调试会话（hil-cdc-stall）事件：HTTP POST 到本地调试服务器 127.0.0.1:7777，
/// 并同步追加到本地 NDJSON 文件（调试服务器可能未运行，文件证据始终可靠）。
/// `data` 为已格式化好的 JSON 对象体（不含外层花括号）。纯 std 实现，短超时非致命。
fn write_event(hyp: &str, loc: &str, msg: &str, data: &str) {
    use std::io::Write;
    use std::net::TcpStream;
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0);
    let body = format!(
        "{{\"sessionId\":\"hil-cdc-stall\",\"runId\":\"pre\",\"hypothesisId\":\"{hyp}\",\"location\":\"{loc}\",\"msg\":\"[DEBUG] {msg}\",\"ts\":{ts},\"data\":{{{data}}}}}"
    );
    // 本地落盘（始终可靠）：追加一行 NDJSON。
    let _ = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(".dbg/trae-debug-log-hil-cdc-stall.ndjson")
        .and_then(|mut f| writeln!(f, "{body}"));
    let _ = (|| -> std::io::Result<()> {
        let addr: std::net::SocketAddr = "127.0.0.1:7777".parse().unwrap();
        let mut s = TcpStream::connect_timeout(&addr, std::time::Duration::from_millis(30))?;
        let req = format!(
            "POST /event HTTP/1.1\r\nHost: 127.0.0.1:7777\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            body.len(),
            body
        );
        s.write_all(req.as_bytes())?;
        Ok(())
    })();
}

/// 整数键值对上报入口（原有调用点不变）。
pub fn dbg_report(hyp: &str, loc: &str, msg: &str, kv: &[(&str, i64)]) {
    let data = kv
        .iter()
        .map(|(k, v)| format!("\"{k}\":{v}"))
        .collect::<Vec<_>>()
        .join(",");
    write_event(hyp, loc, msg, &data);
}

/// 浮点键值对上报入口（保留 2 位小数，输出为合法 JSON number）。
pub fn dbg_report_f64(hyp: &str, loc: &str, msg: &str, kv: &[(&str, f64)]) {
    let data = kv
        .iter()
        .map(|(k, v)| format!("\"{k}\":{v:.2}"))
        .collect::<Vec<_>>()
        .join(",");
    write_event(hyp, loc, msg, &data);
}
// #endregion

// #region debug-point hil-input-replay:rec
/// f32 → 十六进制位串（精确回放，避免十进制舍入损失）。
fn f32hex(v: f32) -> String {
    format!("0x{:08x}", v.to_bits())
}

/// 录制文件路径（相对 cwd 的 .dbg 目录，open_impl 打开一次持久复用）。
const REC_PATH: &str = ".dbg/trae-debug-log-hil-input-replay.ndjson";
// #endregion

/// HIL 链路状态（供 main 渲染连接进度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LinkState {
    /// 尚未找到/打开 USB-CDC 端口（轮询探测中）。
    Connecting,
    /// 端口已打开，等待固件首个 HIL 心跳（确认链路 + HIL 固件）。
    WaitingHeartbeat,
    /// 心跳确认，进入闭环注入/解析。
    Running,
    /// 连接失败或链路中断（携带原因，展示给用户）。
    Error(&'static str),
}

/// USB-CDC 与真实飞控的 MAVLink 交互句柄。
///
/// 单一调用方（main 的 HIL 渲染线程）持有，发送/接收均在此串行完成，
/// 不需要内部锁（与固件侧 USB_TX_MTX 的 app 级互斥不在同一进程）。
pub struct HilLink {
    port: Box<dyn serialport::SerialPort>,
    seq: u8,
    /// 下行分包/粘包重组缓冲。
    rx_buf: Vec<u8>,
    /// 最近 HIL_ACTUATOR_CONTROLS 四电机归一化推力。
    actuator: [f32; 4],
    /// 是否已收到含 HIL flag 的心跳（链路建立判据）。
    hil_flagged: bool,
    /// 最近 MCU 估计姿态 / NED 位置速度（Web UI 遥测用）。
    mcu_att: Option<mavlink::Attitude>,
    mcu_local: Option<mavlink::LocalPositionNed>,
    last_rx: Instant,
    /// 临时诊断：记录解析到的帧（前 N 条）与 CRC 失败数。
    dbg_frames: u32,
    dbg_crc_fail: u32,
    /// 临时诊断：send_chunked 写/睡耗时累加（区分 USB 写阻塞 vs 分块间隔）。
    dbg_write_us: u64,
    dbg_sleep_us: u64,
    dbg_chunks: u64,
    /// 临时诊断（hil-input-replay）：注入序号与上次注入墙钟（测量真实注入节奏）。
    inj_seq: u64,
    last_inj_wall: Option<Instant>,
    /// 录制句柄（open_impl 打开一次，持久复用），避免每次 rec_event 重开文件拖慢 HIL 注入节奏。
    rec_file: Option<std::fs::File>,
}

/// HIL USB-CDC 端口后台探测。
///
/// 背景：`serialport::available_ports()` 在 USB 驱动卡死时可能长时间阻塞（hil-cdc-stall
/// 调试中主循环曾因此挂起）。把枚举放到独立线程，每 ~1.5s 刷新一次缓存结果；
/// 主线程只读缓存，永不阻塞，设备恢复后自动重连。
pub struct CdcProbe {
    last: Arc<Mutex<Option<String>>>,
    stop: Arc<AtomicBool>,
}

impl CdcProbe {
    /// 启动后台探测线程（进程生命周期内常驻）。
    pub fn start() -> Self {
        let last = Arc::new(Mutex::new(None::<String>));
        let stop = Arc::new(AtomicBool::new(false));
        let t_last = Arc::clone(&last);
        let t_stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            while !t_stop.load(Ordering::Relaxed) {
                // 枚举可能阻塞（驱动卡死）；只影响本线程，主循环仍能读到上一次结果。
                let found = serialport::available_ports().ok().and_then(|ports| {
                    ports.into_iter().find_map(|p| match p.port_type {
                        serialport::SerialPortType::UsbPort(info)
                            if info.vid == VID_STM32 && info.pid == PID_STM32_CDC =>
                        {
                            Some(p.port_name)
                        }
                        _ => None,
                    })
                });
                if let Ok(mut g) = t_last.lock() {
                    *g = found;
                }
                std::thread::sleep(Duration::from_millis(1500));
            }
        });
        CdcProbe { last, stop }
    }

    /// 最新探测到的 HIL CDC 端口名（无则 None）。非阻塞，主循环可任意调用。
    pub fn port_name(&self) -> Option<String> {
        self.last.lock().map(|g| g.clone()).unwrap_or(None)
    }
}

impl HilLink {
    /// 打开指定 HIL 上行口（带整体墙钟限时）。
    ///
    /// 驱动卡死时 `serialport::open()` 可能无超时阻塞（CreateFileW/SetCommState），
    /// 直接调用会饿死主循环。故放到独立线程执行并限时返回：
    /// 超时泄漏一个探测线程（重试已节流，可接受），调用方按重试节流继续探测。
    pub fn open(port_name: &str, open_timeout: Duration) -> Result<Self, String> {
        let (tx, rx) = std::sync::mpsc::channel();
        let name = port_name.to_string();
        std::thread::spawn(move || {
            let _ = tx.send(Self::open_impl(&name, open_timeout));
        });
        let wall = open_timeout + Duration::from_secs(2);
        match rx.recv_timeout(wall) {
            Ok(r) => r,
            Err(_) => {
                // #region debug-point D:open-timeout
                dbg_report(
                    "D",
                    "hil_link.rs:open",
                    "open timed out (driver stall)",
                    &[("wall_ms", wall.as_millis() as i64)],
                );
                // #endregion
                Err(format!("打开 {port_name} 超时（USB 驱动卡死，请重插或复位）"))
            }
        }
    }

    /// 打开实现：端口未就绪（复位枚举中）时在 `open_timeout` 内重试，
    /// 占用/禁用类错误直接返回 `Err`（提示 taskkill / Enable-PnpDevice，属边界问题）。
    fn open_impl(port_name: &str, open_timeout: Duration) -> Result<Self, String> {
        let t0 = Instant::now();
        let mut tries = 0u64;
        loop {
            tries += 1;
            // #region debug-point D:open-attempt
            dbg_report(
                "D",
                "hil_link.rs:open",
                "open attempt",
                &[
                    ("tries", tries as i64),
                    ("elapsed_ms", t0.elapsed().as_millis() as i64),
                ],
            );
            // #endregion
            match serialport::new(port_name, BAUD)
                .timeout(Duration::from_millis(50))
                .open()
            {
                Ok(port) => {
                    // #region debug-point D:open-ok
                    dbg_report("D", "hil_link.rs:open", "serial open ok", &[("tries", tries as i64)]);
                    // #endregion
                    return Ok(Self {
                        port,
                        seq: 0,
                        rx_buf: Vec::with_capacity(2048),
                        actuator: [0.0; 4],
                        hil_flagged: false,
                        mcu_att: None,
                        mcu_local: None,
                        last_rx: Instant::now(),
                        dbg_frames: 0,
                        dbg_crc_fail: 0,
                        dbg_write_us: 0,
                        dbg_sleep_us: 0,
                        dbg_chunks: 0,
                        inj_seq: 0,
                        last_inj_wall: None,
                        rec_file: std::fs::OpenOptions::new()
                            .create(true)
                            .append(true)
                            .open(REC_PATH)
                            .ok(),
                    });
                }
                Err(e) => {
                    // 已枚举但驱动未就绪（FileNotFoundError/找不到文件）→ 短暂重试；
                    // 占用（PermissionError/拒绝访问）→ 立即报错，给出可操作提示。
                    let msg = e.to_string();
                    let not_ready = msg.contains("FileNotFoundError")
                        || msg.contains("系统找不到指定的文件")
                        || msg.contains("No such file");
                    let occupied = msg.contains("PermissionError")
                        || msg.contains("拒绝访问")
                        || msg.contains("Access is denied")
                        || msg.contains("access is denied");
                    // #region debug-point D:open-fail
                    dbg_report(
                        "D",
                        "hil_link.rs:open",
                        "open failed",
                        &[
                            ("tries", tries as i64),
                            ("occupied", if occupied { 1 } else { 0 }),
                            ("not_ready", if not_ready { 1 } else { 0 }),
                        ],
                    );
                    // #endregion
                    if not_ready && t0.elapsed() < open_timeout {
                        std::thread::sleep(Duration::from_millis(500));
                        continue;
                    }
                    if occupied {
                        return Err(
                            "端口被占用或设备被禁用：用 `taskkill /F /T` 结束占用进程，\
                             或在设备管理器中启用该 COM 口（边界问题，非代码问题）"
                                .to_string(),
                        );
                    }
                    return Err(format!("打开 {port_name} 失败: {msg}"));
                }
            }
        }
    }

    /// 链路是否已确认（收到 HIL 心跳）。
    pub fn is_hil_ready(&self) -> bool {
        self.hil_flagged
    }

    /// 接收缓冲长度（诊断用）。
    pub fn rx_buf_len(&self) -> usize {
        self.rx_buf.len()
    }

    /// 最近 HIL_ACTUATOR_CONTROLS 电机指令（未收到时为全 0）。
    pub fn actuator(&self) -> [f32; 4] {
        self.actuator
    }

    /// 最近 MCU 估计姿态（ATTITUDE）。
    pub fn mcu_att(&self) -> Option<mavlink::Attitude> {
        self.mcu_att
    }

    /// 最近 MCU 估计 NED 位置/速度（LOCAL_POSITION_NED）。
    pub fn mcu_local(&self) -> Option<mavlink::LocalPositionNed> {
        self.mcu_local
    }

    /// 距最近一次收到下行帧的时间（链路活动度，>2s 可判离线）。
    pub fn idle(&self) -> Duration {
        self.last_rx.elapsed()
    }

    /// 读取并解析下行帧，更新 actuator / MCU 估计状态。
    ///
    /// 非阻塞（每帧最多一次 read）：主循环本就 ~30fps 节流，由调用方周期性调 poll 即可。
    /// 两个原因不能依赖 `read()` 超时或"排空到空"：
    /// 1. Windows serialport 的 `read()` 超时只在"收到首字节后"的空闲间隔生效，纯空读会无限阻塞；
    /// 2. 即便有数据，read 循环也会被下行持续流入同步（每个 read 等下一批字节），
    ///    积压较大时一次 poll 可阻塞数秒、饿死主循环（见 hil-cdc-stall 调试：8953ms/135KB）。
    /// 因此：先 `bytes_to_read()` 判空，无下行立即返回；有下行则只读一次即返回，
    /// 积压由后续帧分摊排空，主循环速率与 MCU 数据流完全解耦。
    pub fn poll(&mut self) {
        // 只读当前已缓冲的字节数（≤4096）：Windows 下 ReadFile 若请求的字节数已全部
        // 在驱动缓冲中会立即返回（50ms 超时只对"等待更多数据"生效）。若仍按满 4096B
        // 请求，下行持续有数据但不足 4096B 时会阻塞满 50ms（H10：实测单次 poll ~50ms）
        // → 主循环冻结 → IMU/GPS 注入饥饿 → 飞控回退 SimImu 反向重力 → 垂直/姿态发散。
        let avail = self.port.bytes_to_read().unwrap_or(0).min(4096) as usize;
        if avail == 0 {
            return;
        }
        // 单次读：只请求已缓冲字节，read 立即返回已到字节，不会长时间阻塞。
        let mut tmp = [0u8; 4096];
        match self.port.read(&mut tmp[..avail]) {
            Ok(0) => {}
            Ok(n) => self.rx_buf.extend_from_slice(&tmp[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => {}
        }
        self.scan_frames();
    }

    /// 注入一帧 HIL_SENSOR + HIL_GPS + SET_POSITION（PC -> 飞控，闭环上行）。
    ///
    /// `imu` 为物理引擎机体比力/角速度（FRD，与固件 `ImuSample` 同约定）；
    /// `yaw` 为引擎 NED 偏航（同时作为设定点期望偏航）；`pressure_alt` 为向上气压
    /// 高度（= -D，与 EKF `update_alt` 的 `z=-alt` 约定一致）；`pos`/`vel` 为物理引擎
    /// 真实 NED 位置/速度（作为 **GPS 真值** 经 HIL_GPS 注入，供 EKF 跟踪真实运动）。
    ///
    /// 设定点与真值解耦（关键）：
    /// - SET_POSITION 只当设定点 → 固定为悬停目标 NED (0,0,-5)，速度归零，给位置环
    ///   稳定目标；若把引擎实时位置当设定点，位置环无误差、无恢复力，垂直环失效（实测）。
    /// - HIL_GPS 携带引擎**真实**位置/速度 → EKF 能感知爬升/漂移并修正估计；若真值
    ///   被固定（旧实现与设定点共用 SET_POSITION 字段），EKF 认为机体静止 → 垂直环
    ///   持续高推力 → 无人机无反馈爬升撞顶（实测 alt 5→10m）。
    /// - yaw 仍传引擎真值：偏航环目标=当前偏航，不引入偏航扰动耦合进 R/P 收敛。
    /// `nav=false`（每物理步，4ms）仅发 HIL_SENSOR；`nav=true`（每 HIL_NAV_EVERY 步，
    /// ≈31Hz）追加 HIL_GPS + SET_POSITION。见调用方 `HIL_NAV_EVERY` 常量注释：多消息
    /// 分块发送叠加的睡眠时间会拖慢 IMU 注入节奏，使飞控多数 4ms 周期无新 IMU 回退
    /// SimImu → EKF 位置预测爆炸 → 姿态发散。导航观测需求远低于 IMU 节拍，故节流。
    ///
    /// 录制 HIL 输入/输出事件到 REC_PATH（本地落盘，可靠）。`data` 为 JSON 对象体
    /// （不含外层花括号）。使用 open_impl 打开的持久句柄（避免每次重开文件拖慢注入），
    /// 仅记录不改业务逻辑；用于"录制 MCU 实际输入 → 原样回放 SIL step_hil → 对比输出"。
    fn rec_event(&mut self, evt: &str, data: &str) {
        if let Some(f) = &mut self.rec_file {
            let ts = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis() as i64)
                .unwrap_or(0);
            let body = format!("{{\"evt\":\"{evt}\",\"ts\":{ts},{data}}}\n");
            let _ = f.write_all(body.as_bytes());
        }
    }

    pub fn inject(
        &mut self,
        time_usec: u64,
        imu: &ImuSample,
        yaw: f32,
        pressure_alt: f32,
        pos: [f32; 3],
        vel: [f32; 3],
        nav: bool,
    ) {
        let mut out = [0u8; MAX_FRAME_LEN];

        // HIL_SENSOR：accel/gyro 真值 + 气压高度。mag/pressure 无真值源填 0。每步必发。
        let n = mavlink::encode_hil_sensor(
            time_usec,
            imu.accel[0].0, imu.accel[1].0, imu.accel[2].0,
            imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0,
            0.0, 0.0, 0.0,
            1013.25, 0.0,
            pressure_alt,
            25,
            0,
            self.seq,
            &mut out,
        );
        self.send_chunked(&out[..n]);

        // #region debug-point hil-input-replay:inj
        // 记录注入序列（真实 MCU 输入）：序号/仿真时间/距上次注入墙钟/IMU/偏航/气压/GPS/是否导航步。
        self.inj_seq += 1;
        let d_us = self
            .last_inj_wall
            .map(|t| t.elapsed().as_micros() as i64)
            .unwrap_or(0);
        self.last_inj_wall = Some(Instant::now());
        self.rec_event(
            "inj",
            &format!(
                "\"seq\":{},\"sim_t_us\":{},\"d_us\":{},\"nav\":{},\"imu\":[{},{},{},{},{},{}],\"yaw\":{},\"baro\":{},\"pos\":[{},{},{}],\"vel\":[{},{},{}]",
                self.inj_seq, time_usec, d_us, if nav { 1 } else { 0 },
                f32hex(imu.accel[0].0), f32hex(imu.accel[1].0), f32hex(imu.accel[2].0),
                f32hex(imu.gyro[0].0), f32hex(imu.gyro[1].0), f32hex(imu.gyro[2].0),
                f32hex(yaw), f32hex(pressure_alt),
                f32hex(pos[0]), f32hex(pos[1]), f32hex(pos[2]),
                f32hex(vel[0]), f32hex(vel[1]), f32hex(vel[2]),
            ),
        );
        // #endregion

        // 导航消息（HIL_GPS / SET_POSITION）节流：仅 nav=true 时发送，避免每步分块叠加拖慢注入。
        if !nav {
            return;
        }

        // HIL_GPS：真实 NED 位置/速度真值（经 lat/lon/alt·1e7/1e3 与 vn/ve/vd·1e2 缩放）。
        // 固件 EKF 用它做位置观测（update_pos）+ Doppler 速度观测（update_vel），
        // 使姿态环外链的位置/速度估计跟踪真实运动（mavlink-core::codec::encode_hil_gps）。
        let n = mavlink::encode_hil_gps(
            time_usec,
            pos,
            vel,
            self.seq,
            &mut out,
        );
        self.send_chunked(&out[..n]);

        // SET_POSITION：type_mask=0 全有效，只当设定点（见函数头注释）。
        let n = mavlink::encode_set_position_target_local_ned(
            (time_usec / 1000) as u32,
            0,
            0.0, 0.0, -5.0, // 固定悬停目标 NED (0,0,-5)（= PC plant 初始位置）
            0.0, 0.0, 0.0,  // 设定点速度归零（悬停目标速度=0）
            0.0, 0.0, 0.0,
            yaw,
            0.0,
            self.seq,
            &mut out,
        );
        self.send_chunked(&out[..n]);
    }

    /// 发送 ARM/DISARM（COMMAND_LONG，直接写不经 chunk gap，避免与注入流穿插竞争）。
    pub fn send_arm(&mut self, armed: bool) -> Result<(), String> {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = mavlink::encode_command_long(
            MAV_CMD_ARM_DISARM,
            if armed { 1.0 } else { 0.0 },
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0,
            self.seq,
            &mut out,
        );
        self.seq = self.seq.wrapping_add(1);
        self.port.write_all(&out[..n]).map_err(|e| e.to_string())?;
        let _ = self.port.flush();
        Ok(())
    }

    /// 带 retry 的分块写（≤64B/块，块间留 re-arm 间隔）。
    fn send_chunked(&mut self, data: &[u8]) {
        for (i, chunk) in data.chunks(64).enumerate() {
            if i > 0 {
                let t0 = Instant::now();
                std::thread::sleep(OUT_CHUNK_GAP);
                self.dbg_sleep_us += t0.elapsed().as_micros() as u64;
            }
            let t0 = Instant::now();
            for attempt in 0..2 {
                match self.port.write_all(chunk) {
                    Ok(()) => break,
                    Err(_) if attempt == 0 => {
                        std::thread::sleep(Duration::from_millis(50));
                    }
                    Err(e) => {
                        // 写失败不影响读；留一条路让上层看到端口状态（丢弃该帧即可）。
                        let _ = e;
                        self.dbg_chunks += 1;
                        return;
                    }
                }
            }
            self.dbg_chunks += 1;
            self.dbg_write_us += t0.elapsed().as_micros() as u64;
        }
        let _ = self.port.flush();
        self.seq = self.seq.wrapping_add(1);
    }

    /// 临时诊断：send_chunked 累计写/睡耗时与块数（区分 USB 写阻塞 vs 分块间隔）。
    pub fn dbg_tx(&self) -> (u64, u64, u64) {
        (self.dbg_write_us, self.dbg_sleep_us, self.dbg_chunks)
    }

    /// 增量解析 rx_buf 中的 MAVLink v2 帧（处理粘包/分包/错位字节）。
    fn scan_frames(&mut self) {
        // 清除前导垃圾（非 0xFD）。
        while !self.rx_buf.is_empty() && self.rx_buf[0] != 0xFD {
            self.rx_buf.remove(0);
        }
        while self.rx_buf.len() >= 12 {
            let plen = self.rx_buf[1] as usize;
            let total = 10 + plen + 2;
            if self.rx_buf.len() < total {
                break; // 半帧，等待更多字节
            }
            let frame = Frame::from_bytes(&self.rx_buf[..total]);
            if let Some((id, payload)) = mavlink::decode(&frame) {
                if self.dbg_frames < 12 {
                    eprintln!("[hil][dbg] rx msgid={id} plen={}", payload.len());
                    self.dbg_frames += 1;
                }
                self.handle_frame(id, payload);
            } else {
                self.dbg_crc_fail += 1;
                if self.dbg_crc_fail <= 5 {
                    eprintln!("[hil][dbg] CRC 校验失败 len={total}");
                }
            }
            self.rx_buf.drain(..total);
            while !self.rx_buf.is_empty() && self.rx_buf[0] != 0xFD {
                self.rx_buf.remove(0);
            }
        }
    }

    fn handle_frame(&mut self, id: u32, payload: &[u8]) {
        use mavlink::msg_id;
        match id {
            msg_id::HIL_ACTUATOR_CONTROLS => {
                if let Some(act) = mavlink::decode_hil_actuator_controls(payload) {
                    self.actuator =
                        [act.controls[0], act.controls[1], act.controls[2], act.controls[3]];
                    // #region debug-point hil-input-replay:act
                    self.rec_event(
                        "act",
                        &format!(
                            "\"m\":[{},{},{},{}]",
                            f32hex(act.controls[0]),
                            f32hex(act.controls[1]),
                            f32hex(act.controls[2]),
                            f32hex(act.controls[3]),
                        ),
                    );
                    // #endregion
                }
            }
            msg_id::HEARTBEAT => {
                // base_mode 在 payload[2]（见 encode_heartbeat_ap）。
                if self.dbg_frames < 12 {
                    eprintln!(
                        "[hil][dbg] HEARTBEAT base_mode=0x{:02x} armed={}",
                        payload.get(2).copied().unwrap_or(0),
                        payload.get(7).copied().unwrap_or(0) == 4
                    );
                }
                if payload.len() > 2 && payload[2] & HIL_FLAG != 0 {
                    self.hil_flagged = true;
                }
            }
            msg_id::ATTITUDE => {
                self.mcu_att = mavlink::decode_attitude(payload);
                // #region debug-point hil-input-replay:att
                if let Some(a) = self.mcu_att {
                    self.rec_event(
                        "att",
                        &format!(
                            "\"r\":{},\"p\":{},\"y\":{}",
                            f32hex(a.roll), f32hex(a.pitch), f32hex(a.yaw),
                        ),
                    );
                }
                // #endregion
            }
            msg_id::LOCAL_POSITION_NED => {
                self.mcu_local = mavlink::decode_local_position_ned(payload);
                // #region debug-point hil-input-replay:lpos
                if let Some(l) = self.mcu_local {
                    self.rec_event(
                        "lpos",
                        &format!(
                            "\"x\":{},\"y\":{},\"z\":{},\"vx\":{},\"vy\":{},\"vz\":{}",
                            f32hex(l.x), f32hex(l.y), f32hex(l.z),
                            f32hex(l.vx), f32hex(l.vy), f32hex(l.vz),
                        ),
                    );
                }
                // #endregion
            }
            msg_id::COMMAND_ACK => {
                // 仅打印（不影响闭环）：确认 ARM/DISARM 是否被固件接受。
                if let Some((cmd, result)) = mavlink::decode_command_ack(payload) {
                    if cmd == MAV_CMD_ARM_DISARM && result == MAV_RESULT_ACCEPTED {
                        // ACK 成功（可选：main 据此判定 armed 生效）。
                    }
                }
            }
            _ => {}
        }
        self.last_rx = Instant::now();
    }
}
