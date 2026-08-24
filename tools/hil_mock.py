#!/usr/bin/env python3
"""HIL 闭环 mock：仿真器 -> 飞控 的 HIL_SENSOR / SET_POSITION_TARGET_LOCAL_NED 注入 + 执行器回传验证。

验证目标（MCU 端已编译 `hil` feature 的固件，经 USB CDC 连接）：
  1. 固件以 HEARTBEAT base_mode 的 MAV_MODE_FLAG_HIL_ENABLED(0x20) 标识 HIL 模式；
  2. 注入的 HIL_SENSOR（IMU 比力 + 气压高度）被固件解析、写入 SENSOR_FRAME；
  3. 注入的 SET_POSITION_TARGET_LOCAL_NED 设定点被固件用于控制律；
  4. COMMAND_LONG(400 ARM/DISARM) 解锁后，固件回传 HIL_ACTUATOR_CONTROLS 携带四电机归一化推力；
  5. 扰动注入（zacc 增大 3 m/s²）使估计速度下漂 -> 电机推力升高（闭环响应）；扰动撤销后回落。

用法：
  python hil_mock.py [--port COMx] [--duration 14] [--rate 50]

时间轴（秒，以收到首个 HIL 心跳为 0 点）：
  0-2    未解锁，悬停注入（z=-2m）           -> 期望电机 ~0
  2-5    ARM，稳定悬停                       -> 电机升至悬停油门（>0，有限）
  5-7    垂直扰动 zacc += 3 m/s²            -> 电机升高（抵抗"下坠"）
  7-10   扰动撤销，恢复悬停                   -> 电机回落接近基线
  10+    DISARM                              -> 电机归零
"""
import argparse
import math
import os
import sys
import threading
import time

# 共享 MAVLink 工具库（joc-app-rust/tools/mavlink.py），可用环境变量覆盖路径。
sys.path.insert(0, os.path.abspath(os.environ.get(
    "MAVLINK_TOOLS", r"d:\project\mcu\oop\joc-app-rust\tools")))
import mavlink  # noqa: E402

G = 9.81
HOVER_Z = -2.0      # 悬停高度（NED，向下为正 -> 负值表示向上 2m）
DISTURB = 3.0       # 扰动期 zacc 增量（m/s²）
MAV_CMD_ARM = 400   # MAV_CMD_COMPONENT_ARM_DISARM
HIL_FLAG = 0x20     # MAV_MODE_FLAG_HIL_ENABLED


def parse_args():
    ap = argparse.ArgumentParser(description="HIL 闭环 mock（仿真器 -> 飞控）")
    ap.add_argument("--port", default=None, help="USB CDC 串口（默认自动探测）")
    ap.add_argument("--duration", type=float, default=14.0, help="总运行秒数")
    ap.add_argument("--rate", type=float, default=50.0, help="HIL 注入频率 Hz")
    ap.add_argument("--gap", type=float, default=0.25,
                    help="多包(>64B)分块写入的块间间隔 s（MCU OUT re-arm 需要）")
    ap.add_argument("--wait", type=float, default=15.0,
                    help="自动探测时等待 USB CDC 枚举的超时 s（reset/重插后枚举需数秒，默认 15）")
    ap.add_argument("--com8", default=None,
                    help="诊断：同时抓日志口(如 COM8)的 uplink/ctrl 行，确认 ARM 是否被固件解析")
    return ap.parse_args()


def list_all_ports():
    """返回当前所有串口名列表（用于诊断，勿在探测逻辑里当判据）。"""
    try:
        import serial.tools.list_ports
        return [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        return []


def describe_ports():
    """返回当前所有串口的 (device, description, hwid) 列表（诊断用）。

    用于区分日志口（CH340，如 COM8）与 HIL 上行口（STM32 USB-CDC，如 COM12），
    避免把日志口误当 HIL 口。
    """
    try:
        import serial.tools.list_ports
        return [(p.device, p.description, p.hwid)
                for p in serial.tools.list_ports.comports()]
    except Exception:
        return []


def wait_for_cdc(timeout):
    """等待 STM32 USB-CDC（HIL 上行口）出现。

    边界：reset/重插 USB 后，主机枚举需数秒，且 COM 号可能变化（如 COM12→COM13）。
    不能只探测一次就判"设备不在"，也不能依赖固定 COM 号。这里周期用 find_cdc()
    （按 PID 匹配，已与日志口 CH340 区分）重试，并每 2s 打印全部端口供诊断。
    """
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < timeout:
        port = mavlink.find_cdc()
        if port is not None:
            print(f"[INFO] 探测到 HIL USB-CDC 端口: {port}（等待 {time.time()-t0:.1f}s）", flush=True)
            return port
        now = time.time()
        if now - last_print >= 2.0:
            last_print = now
            print(f"[WAIT] 已等 {now-t0:.0f}s，尚未发现 HIL USB-CDC；当前全部串口:", flush=True)
            for d, desc, hwid in describe_ports() or [("无", "", "")]:
                print(f"       {d} | {desc} | {hwid}", flush=True)
        time.sleep(0.5)
    return None


def open_port(port, wait):
    """打开 HIL 上行串口，处理复位重枚举的时序边界。

    边界分类：
      - 指定端口当前不在串口列表（复位后 COM 号可能变化）→ 回退自动探测；
      - 已枚举但驱动未就绪（FileNotFoundError/找不到文件）→ 在 wait 内重试；
      - 占用/被禁用（PermissionError/拒绝访问）→ 给出可操作提示，而非"代码问题"。
    """
    import serial
    if port is not None and port not in list_all_ports():
        print(f"[WARN] 指定端口 {port} 不在当前串口列表（复位重枚举后 COM 号可能变化），"
              f"改为自动探测", flush=True)
        port = None
    if port is None:
        port = wait_for_cdc(wait)
        if port is None:
            print(f"[FAIL] 等待 {wait:.0f}s 仍未探测到 HIL USB-CDC 端口", file=sys.stderr)
            print("       请按序排查（边界问题，非代码问题）：", file=sys.stderr)
            print("       1) 确认 MCU 已复位/重插 USB——枚举需数秒，可延长 --wait 再试；", file=sys.stderr)
            print("       2) 设备管理器能看到 COM 口但这里探测不到：核对其 VID/PID 是否"
                  "STM32 USB-CDC(0483:5740)，而非日志口 CH340(1A86:7523)；", file=sys.stderr)
            print("       3) 端口若被残留进程占用，用 `taskkill /F /T` 结束占用进程后重试；", file=sys.stderr)
            print("       4) 或用 --port 显式指定（先看设备管理器里实际的 COMx）。", file=sys.stderr)
            sys.exit(2)
    print(f"[INFO] 打开 {port} ...", flush=True)
    # write_timeout 放宽：多包 OUT（如 74B=64+10）需等 MCU uplink 消费并 re-arm
    # 端点，1s 内若未完成即抛 SerialTimeoutException（此时固件/端口均正常）。
    # 打开同样有枚举"中间态"：端口名刚分配但设备驱动未就绪 → 报
    # FileNotFoundError(2, 找不到文件)。此时短暂重试即可；真正的占用
    # （PermissionError/拒绝访问）才提示杀进程/启用设备。
    t_open = time.time()
    while True:
        try:
            return serial.Serial(port, baudrate=115200, timeout=0.2, write_timeout=5.0)
        except serial.SerialException as e:
            msg = str(e)
            is_not_ready = ('FileNotFoundError' in msg or '系统找不到指定的文件' in msg
                            or 'No such file' in msg)
            if is_not_ready and time.time() - t_open < wait:
                print(f"[WAIT] {port} 已枚举但设备未就绪，等待初始化... "
                      f"剩余 {wait-(time.time()-t_open):.0f}s", flush=True)
                time.sleep(0.5)
                continue
            is_occupied = ('PermissionError' in msg or '拒绝访问' in msg
                           or 'Access is denied' in msg)
            print(f"[FAIL] 打开 {port} 失败: {msg}", file=sys.stderr)
            if is_occupied:
                print("       端口被占用或设备被禁用（边界问题，非代码问题）：", file=sys.stderr)
                print("       a) 残留读进程占用 → `taskkill /F /T` 结束占用进程；", file=sys.stderr)
                print("       b) 设备管理器中被禁用(CM_PROB_DISABLED) → PowerShell 执行: "
                      "Enable-PnpDevice -InstanceId '<该 COM 口的设备实例 ID>'", file=sys.stderr)
            else:
                print("       非占用类错误，先核对端口身份与枚举状态（见上方端口表）。", file=sys.stderr)
            sys.exit(2)


class Stats:
    def __init__(self):
        self.hil_act = 0
        self.heartbeat = 0
        self.local_pos = 0
        self.hil_flagged = 0
        self.motor_sum = 0.0
        self.motor_n = 0
        self.motor_min = 1.0
        self.motor_max = 0.0
        self.nan_hit = 0
        self.est_z = 0.0

    def add_motor(self, motors):
        m = sum(motors[0:4]) / 4.0
        self.motor_sum += m
        self.motor_n += 1
        self.motor_min = min(self.motor_min, m)
        self.motor_max = max(self.motor_max, m)
        if not all(math.isfinite(x) for x in motors[0:4]):
            self.nan_hit += 1

    def avg_motor(self):
        return (self.motor_sum / self.motor_n) if self.motor_n else 0.0


def main():
    args = parse_args()
    ser = open_port(args.port, args.wait)
    seq = 0
    st = Stats()
    t0 = time.time()
    t_ref = None  # 以首个 HIL 心跳为 0 点
    last_report = 0.0
    last_send = 0.0
    interval = 1.0 / args.rate
    zacc_now = G
    armed = False
    disarmed = False

    # 诊断：--com8 抓固件日志口，确认 uplink 是否解析到 ARM / ctrl 的 armed/crit
    log_lines = []
    log_stop = threading.Event()
    log_fh = None

    def log_reader():
        nonlocal log_fh
        try:
            log_fh = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       '_hil_full_log.txt'), 'w')
        except Exception:
            log_fh = None
        try:
            import serial as _s
            s8 = _s.Serial(args.com8, 115200, timeout=0.2)
            s8.dtr = False
            s8.rts = False
            buf = b''
            while not log_stop.is_set():
                d = s8.read(512)
                if d:
                    buf += d
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        txt = line.decode('utf-8', 'replace')
                        log_lines.append(txt)
                        if log_fh:
                            try:
                                log_fh.write(txt + '\n')
                            except Exception:
                                pass
        except Exception as e:
            log_lines.append(f'[COM8 reader error] {e}')

    if args.com8:
        threading.Thread(target=log_reader, daemon=True).start()
        print(f"[INFO] 抓日志口 {args.com8}（uplink/ctrl/ARM 行）...", flush=True)

    # 下行解析：独立线程持续排空 USB 接收缓冲区。
    # 此前下行读取放在 main 循环里，与 send() 的 gap sleep(0.25s) 串行执行，
    # ARM 窗口内的非零电机帧被积压延迟，读到多为 ARM 前的 0 值缓存帧，
    # 导致 m_avg=0.000 而固件内电机实际正常。线程化后缓冲区被及时排空，
    # 统计与真实一致（pyserial 对读/写使用独立锁，双线程安全）。
    dl_stop = threading.Event()

    def downlink_reader():
        while not dl_stop.is_set():
            d = ser.read(1024)
            if not d:
                continue
            for f in mavlink.scan_frames(d):
                if not f.crc_ok:
                    continue
                if f.msgid == 93:  # HIL_ACTUATOR_CONTROLS
                    r = mavlink.dec_hil_actuator_controls(f.payload)
                    if r:
                        st.hil_act += 1
                        st.add_motor(r[1])
                elif f.msgid == 0:
                    st.heartbeat += 1
                    hb = mavlink.dec_heartbeat(f.payload)
                    if hb and (hb[3] & HIL_FLAG):
                        st.hil_flagged += 1
                elif f.msgid == 32:  # LOCAL_POSITION_NED
                    lp = mavlink.dec_local_position_ned(f.payload)
                    if lp:
                        st.local_pos += 1
                        st.est_z = lp[3]
                elif f.msgid == 77:  # COMMAND_ACK（确认 ARM/DISARM 是否被固件接收）
                    cmd = f.payload[0] | (f.payload[1] << 8)
                    result = f.payload[2]
                    if cmd == 400:
                        print(f"[ACK] ARM/DISARM command={cmd} result={result} "
                              f"(0=ACCEPTED)", flush=True)

    def elapsed():
        return (time.time() - t_ref) if t_ref else 0.0

    def send(blob):
        # blob 已在调用处用当前 seq 编码（enc_* 传 seq=seq），CRC 与 SEQ 一致。
        # 切勿在此改 b[4]=seq：帧是按 seq=0 算好 CRC 的，盲改 SEQ 会致 CRC 校验失败、
        # 整帧被固件丢弃（HIL_SENSOR/SET_POSITION 全部 decode FAILED 的根因）。
        nonlocal seq
        import serial as _serial
        data = bytes(blob)
        # MCU 的 bulk-OUT 端点每收满 64B 后需 ~0.2-0.3s 供 uplink 消费并 re-arm；
        # 多包(74B=64+10)与帧间首包若背靠背发送会命中 NAK 窗口致主机写超时。
        # 固件无需改动：每帧开头与分块之间均留 re-arm 间隔，失败重试一次。
        time.sleep(args.gap)
        for i in range(0, len(data), 64):
            chunk = data[i:i + 64]
            if i > 0:
                time.sleep(args.gap)
            for attempt in range(2):
                try:
                    ser.write(chunk)
                    break
                except _serial.SerialTimeoutException:
                    if attempt == 0:
                        time.sleep(0.2)
                        continue
                    raise
        seq = (seq + 1) & 0xFF

    def arm(do_arm):
        # target_system=1（mavlink.py enc_command_long 硬编码 1）
        # 与 _hil_ctrl_diag.py 一致：直接 write（不经 send 的 gap sleep / seq 覆盖），
        # 避免注入帧流穿插导致 ARM 帧与 64B 分块边界竞争。
        nonlocal seq
        seq = (seq + 1) & 0xFF
        ser.write(bytes(mavlink.enc_command_long(
            MAV_CMD_ARM, 1.0 if do_arm else 0.0, seq=seq)))
        print(f"[CMD] {'ARM' if do_arm else 'DISARM'} @ t={elapsed():.1f}s", flush=True)

    # 等待固件 HIL 心跳（HIL flag），建立 0 点
    print("[INFO] 等待固件 HIL 心跳（HEARTBEAT base_mode 含 0x20）...")
    n_bytes = 0
    n_frames = 0
    while t_ref is None and time.time() - t0 < 10:
        d = ser.read(512)
        n_bytes += len(d)
        for f in mavlink.scan_frames(d):
            n_frames += 1
            if f.msgid == 0 and f.crc_ok:
                hb = mavlink.dec_heartbeat(f.payload)
                if hb and (hb[3] & HIL_FLAG):
                    t_ref = time.time()
                    st.heartbeat += 1
                    print(f"[INFO] HIL 心跳确认 base_mode=0x{hb[3]:02x}（含 HIL flag）", flush=True)
                    break
        if t_ref is None:
            time.sleep(0.05)
    if t_ref is None:
        print("[FAIL] 10s 内未收到 HIL 心跳", file=sys.stderr)
        print("       边界排查（先确认端口/时机，再谈固件）：", file=sys.stderr)
        if n_bytes == 0:
            print("       - 端口 0 字节：很可能打开了错误端口（如日志口 CH340/COM8），或"
                  "设备刚复位未就绪。核对上方端口表，确认打开的是 HIL USB-CDC(0483:5740)。", file=sys.stderr)
        else:
            print(f"       - 端口收到 {n_bytes}B/{n_frames} 帧但无 HIL 心跳：确认固件已用 "
                  "`--features hil` 编译烧录，且连接的是 HIL 上行口而非日志口。", file=sys.stderr)
        print("       处理：taskkill /F /T 清理占用 → 硬件复位/重插 USB（枚举需数秒）→ 重跑。", file=sys.stderr)
        ser.close()
        sys.exit(3)

    # 收到首个 HIL 心跳后，启动下行解析线程（持续排空接收缓冲区）。
    threading.Thread(target=downlink_reader, daemon=True).start()

    # 主循环：注入（上行） + 阶段状态机
    while elapsed() < args.duration:
        t = elapsed()

        # --- 阶段状态机 ---
        if not armed and t >= 2.0:
            arm(True)
            armed = True
        elif armed and not disarmed and t >= 10.0:
            arm(False)
            disarmed = True
        # 扰动窗口 5-7s：zacc 增大（模拟额外下坠比力）
        zacc_now = G + (DISTURB if 5.0 <= t < 7.0 else 0.0)

        # --- HIL 注入（以 --rate 频率） ---
        now = time.time()
        if now - last_send >= interval:
            last_send = now
            time_usec = int((t_ref + t) * 1e6)
            send(mavlink.enc_hil_sensor(
                time_usec, 0.0, 0.0, zacc_now, 0.0, 0.0, 0.0,
                pressure_alt=100.0, temperature=25))
            send(mavlink.enc_set_position_target_local_ned(
                int(t * 1000), 0.0, 0.0, HOVER_Z))

        # --- 每秒一行报告 ---
        if t - last_report >= 1.0:
            last_report = t
            tag = "ARMED" if (armed and not disarmed) else "IDLE"
            print(
                f"t={t:5.1f}s {tag:5s} hb={st.heartbeat:4d} act={st.hil_act:4d} "
                f"lp={st.local_pos:4d} m_avg={st.avg_motor():.3f} "
                f"[{st.motor_min:.3f},{st.motor_max:.3f}] est_z={st.est_z:+.2f} "
                f"zacc={zacc_now:.2f}",
                flush=True)

    # 停下行线程：先置停止位，等 read(timeout=0.2) 自然返回后再关端口，
    # 避免线程正阻塞在 read 上时 close() 抛异常。
    dl_stop.set()
    time.sleep(0.3)
    ser.close()
    if args.com8:
        log_stop.set()
        time.sleep(0.3)
        if log_fh:
            try:
                log_fh.close()
            except Exception:
                pass
        print("\n==== 固件日志（uplink/ctrl/ARM 行） ====", flush=True)
        shown = 0
        for ln in log_lines:
            if ('uplink' in ln or 'ctrl' in ln or 'telem' in ln
                    or 'ARM' in ln or 'DECAY' in ln):
                print(ln, flush=True)
                shown += 1
        if shown == 0:
            print(f"(无相关日志；共 {len(log_lines)} 行)", flush=True)

    # --- 验收汇总 ---
    print("\n=== 汇总 ===")
    print(f"HIL 心跳: {st.hil_flagged}/{st.heartbeat} 带 HIL flag(0x20)")
    print(f"HIL_ACTUATOR_CONTROLS 回传帧数: {st.hil_act}")
    print(f"电机均值 min/max: {st.motor_min:.3f} / {st.motor_max:.3f}（须有限且在 [0,1]）")
    print(f"非有限(NaN/Inf)样本: {st.nan_hit}")
    print(f"末段估计高度 est_z: {st.est_z:+.2f} m（期望接近 {HOVER_Z}）")

    ok = True
    if st.hil_flagged == 0:
        print("[FAIL] 未收到带 HIL flag 的心跳")
        ok = False
    if st.hil_act == 0:
        print("[FAIL] 未收到 HIL_ACTUATOR_CONTROLS 回传")
        ok = False
    if st.nan_hit:
        print("[FAIL] 电机指令出现 NaN/Inf")
        ok = False
    if not (0.0 <= st.motor_min <= st.motor_max <= 1.0):
        print("[FAIL] 电机指令越界 [0,1]")
        ok = False
    if st.motor_max < 0.2:
        print("[WARN] 电机峰值 <0.2——ARM 后未见明显推力，检查解锁/健康状态")
    if st.local_pos == 0:
        print("[WARN] 未收到 LOCAL_POSITION_NED 下行")
    print("结论:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 4)


if __name__ == "__main__":
    main()
