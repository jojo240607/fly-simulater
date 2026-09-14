# Fly Simulator 仿真框架架构文档

> **本文档是运行 / 开发 / 诊断本仿真框架的权威参考。** 新对话从本文档开始理解整个系统，
> 不要再从头摸索。最后更新对应 git 提交：`ab136ad`（fly-simulater）+ `7a7089e`（flyctrl）。
>
> 交流语言：中文。所有路径、常量、参数、端口、踩坑记录以本文档为准（代码改动后请同步更新）。

---

## 1. 系统目标

- 在 Web 页面（浏览器 canvas）上展示一架四旋翼**仿真飞机**的实时画面与遥测。
- 仿真闭环由 **vperiph + Unicorn 模拟 MCU + 真实固件二进制（flyctrl）** 驱动：
  - Unicorn 模拟一块 **STM32F407** 微控制器，跑**真实的 PX4 风格飞控固件**（`flyctrl_clean.bin`，
    real-sensors 编译）；
  - 固件通过虚拟外设（SBUS 摇杆输入 / PWM 电机输出 / IMU / GPS / 气压计）与物理仿真闭环；
  - 服务端注入 8 字机动 + 定高外环，飞机会自动做出水平 8 字机动。
- 历史演进：PNG 推帧 → H.264 视频流（openh264 + 浏览器 WebCodecs）→ 3 线程解耦 → 20fps 稳定档。
- **当前核心难点已解决**：H.264 流在桌面 + 手机浏览器均能稳定解码显示（见第 8、13 节踩坑）。

---

## 2. 仓库与代码位置

| 路径 | 内容 |
|---|---|
| `/home/ubuntu/work/fly-simulater/` | 主仓库（服务端 + Web + 仿真核心），git 仓库 |
| `/home/ubuntu/work/fly-simulater/fly-sim-server/` | Rust 服务端（HTTP/WS/渲染/编码/仿真主循环），`main.rs` 是核心 |
| `/home/ubuntu/work/fly-simulater/fly-sim-core/` | 仿真核心：物理模型、渲染器、控制、遥测 |
| `/home/ubuntu/work/fly-simulater/web/` | 前端 `index.html` + `main.js`（浏览器解码/绘制/HUD） |
| `/home/ubuntu/work/fly-simulater/fly-sim-hil/` | HIL 链路 crate（USB-CDC 真机 HIL，已从 server 抽出独立 crate） |
| `/home/ubuntu/work/fly-simulater/certs/` | HTTPS 自签证书（cert.pem / key.pem） |
| `/home/ubuntu/work/flyctrl/` | 飞控固件源码（独立 git 仓库），编译产物是模拟的 app.bin |
| `/home/ubuntu/work/flyctrl/app/src/flyctrl/control.rs` | 固件控制律（速率模式关键改动在这） |
| `/home/ubuntu/work/flyctrl/core/src/controller/pid.rs` | PidController（rate_mode_xy、vmax_xy 等） |
| `/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf` | 模拟 MCU 的系统镜像（VP_SYS） |
| `/home/ubuntu/work/mcu_simulater/` | vperiph / mcu_simulater 环境（含 Unicorn engine） |

> 服务器还在运行时的当前工作目录必须是项目根（`/home/ubuntu/work/fly-simulater`），
> 否则 `web/` 静态目录定位会走回退路径。见 `serve_static` 的 candidates 逻辑。

---

## 3. 运行时拓扑（端口 / 进程 / 证书）

```
浏览器
  │  https://150.158.109.150:8081/   （自签证书，需“高级→继续访问”）
  ▼
socat TLS 终止代理（OPENSSL-LISTEN:8081, cert=cert.pem, verify=0, fork）
  │  TCP 明文 127.0.0.1:8082
  ▼
fly-sim-server（FLY_SIM_PORT=8082 启动）
  │  WS:  /ws?fmt=h264[&q=low|lowest]   视频/遥测
  │  HTTP: /  /main.js  /index.html     静态资源（每次请求读盘，改 JS 即时生效）
```

- **明文后端端口：8082**（用 `FLY_SIM_PORT=8082` 显式启动；代码默认 8080，但 8080 常被其他服务/Caddy 占用）。
- **公网 HTTPS 入口：8081**，由 socat 做 TLS 终止（**不要杀 8080 的 Caddy**）。
- **启动方式（关键，勿用 start_https.sh——后台 fd 继承会超时）**：
  1. 后台 job 1：`cd /home/ubuntu/work/fly-simulater && FLY_SIM_PORT=8082 ./target/release/fly-sim-server`
  2. 后台 job 2：`cd /home/ubuntu/work/fly-simulater/certs && socat OPENSSL-LISTEN:8081,cert=cert.pem,key=key.pem,verify=0,fork,reuseaddr TCP:127.0.0.1:8082`
- 改代码后：`cargo build --release -p fly-sim-server --features phy`（约 1 分钟）→ 杀旧 job → 重启 job 1。
- 改 `web/main.js` **无需重编译**（静态文件实时读盘），浏览器强刷（Ctrl+Shift+R）即可。
- 验证：`curl -sk -o /dev/null -w "%{http_code}" https://150.158.109.150:8081/` 应 200。

---

## 4. 线程与数据流（3 线程）

```
┌─ 主线程（仿真）─────────────────────────────────────────────┐
│ 场景循环：advance_vperiph()                                  │
│  ├─ 每物理步 DT=4ms（SimLoop step_hil，不耗 Unicorn）        │
│  ├─ 每 VP_RUN_EVERY=1 步：Unicorn run(200_000)（固件控制律） │
│  ├─ inject_maneuver()：注入 SBUS 摇杆（8 字 + 定高外环）     │
│  └─ 每 VP_STEPS_PER_FRAME=4 步：try_send 渲染输入帧          │
│      通道满 → 丢帧 + force_idr 标志（AtomicBool）            │
└───────────────┬─────────────────────────────────────────────┘
                │ sync_channel(4)：(fw, fh, RenderInput, tele)
                ▼
┌─ 渲染/编码线程（每连接一个）─────────────────────────────────┐
│  render_frame(fw,fh) → rgba → I420 → openh264 encode        │
│  丢帧后 force_intra_frame() 重置参考链                       │
│  → WS 0x2（视频帧）/ 0x1（遥测 JSON）                        │
└───────────────┬─────────────────────────────────────────────┘
                │ TCP → socat → 浏览器
                ▼
┌─ 浏览器（前端）─────────────────────────────────────────────┐
│  WebSocket onmessage → drawFrame                             │
│  fmt=2 H.264：WebCodecs VideoDecoder 解码                    │
│  解码 output → pushFrame（只存最新）→ requestAnimationFrame  │
│  绘制（rAF 节流，消除 burst）                                │
└─────────────────────────────────────────────────────────────┘
```

- 渲染线程与仿真解耦：渲染/编码/网络阻塞不会拖慢仿真。
- 编码器每 WS 连接一个；渲染线程串行做「渲染 + 编码 + 推帧」。

---

## 5. 仿真链路（vperiph + Unicorn + 固件）

- 场景名：`vperiph`（默认演示场景）。SIL（悬停/抗风）场景走另一条 PNG 路径。
- **常量（fly-sim-server/src/main.rs）**：
  - `VP_SYS = /home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf`（系统镜像）
  - `VP_APP = /tmp/flyctrl_clean.bin`（固件，real-sensors 编译）
  - `VP_LAT0=31.2304 / VP_LON0=121.4737 / VP_ALT0=4.0`（起飞基准，上海）
  - `DT = 0.004`（物理步长 4ms）
  - `VP_STEPS_PER_FRAME = 4`（每显示帧推进 4 物理步 → **20fps** 推帧）
  - `VP_RUN_EVERY = 1`（每物理步都 run 固件——**必须 1**，见踩坑 13.3）
  - run 步数 `200_000`（Unicorn 单步成本 ≈12ms，是帧率硬下限）
- **物理推进**（SimLoop step_hil，DT=4ms）不耗 Unicorn；**Unicorn run(200k) 是唯一 CPU 成本**。
- 固件 SBUS 摇杆 → 控制律 → PWM → 物理仿真 → 传感器（IMU/GPS/baro）→ 固件 EKF → 闭环。
- 固件编译（每次改固件后）：
  `cd /home/ubuntu/work/flyctrl && python3 build_app.py --features real-sensors --out /tmp/flyctrl_clean.bin`
  **必须是 `--features real-sensors`**（非 HIL、非默认；默认无 feature 编译产物 113KB 不对，
  real-sensors ≈58KB 才匹配原 clean.bin）。改固件需重编 + 重启服务器才生效。

---

## 6. 固件（flyctrl）控制律与速率模式

固件：`flyctrl`（PX4 风格），`PidController` 在 `core/src/controller/pid.rs`，
非 HIL setpoint 在 `app/src/flyctrl/control.rs`。

### 速率模式（用户要求“大疆航拍机手感”，git `7a7089e`）
- `PidController` 增加 `rate_mode_xy: bool`（默认 false）+ `pub fn set_rate_mode_xy(&mut self, on: bool)`；
  `vmax_xy: 2.0 → 3.5`。
- 非 HIL setpoint（control.rs）：
  - `pos = [速度外推预测位置(×0.25s), target_alt]`
  - `vel = [rc.pitch × 1.7, -rc.roll × 1.7, 0]` m/s（摇杆 → 期望速度）
  - `yaw = rc.yaw * 0.5`
- 位置外环在 rate_mode_xy 下旁路：`des_vx = clamp(sp.vel, ±vmax)`（速度由摇杆严格决定，
  消除 kp_xy*ex 的正反馈放大——早期半径冲到 5.4-5.8m 的根因）。
- **重要**：open264 速率放大倍率实测 1.84-2.3×（EKF 速度标定 vs 物理偏差），
  所以增益最终从 3.0 调到 **1.7** 才得到 8 字半径 ≈3.28m（用户要 3m）。
- 高度语义：`target_alt = hold_alt - thr_off*2`，`thr_off=(throttle-0.5)*2`，**油门中位 = 定高**。

### 历史（勿回退）
- `8b2c447`：setpoint 水平系数 ×8.5 → 半径 3m 级。
- `00284d3`：非 HIL 摇杆水平机动（此前 PidController **完全忽略 rc.roll/pitch**，setpoint 水平固定 (0,0)
  ——这是“8 字摇杆从未生效”的根因，amp 实验证实）。

---

## 7. 服务端注入层（定高外环 + 8 字机动）

`fly-sim-server/src/main.rs` 的 `inject_maneuver(&mut self, t_sim, z)`（每物理步调用）：
- **8 字机动**（SBUS 摇杆注入）：
  - `roll_amp = 0.75`、`pitch_amp = 0.75`（对称）
  - `w = 2π / 8.0`（**8 字周期 8s 仿真**；曾为 16s，为“感知更快”减半）
  - `rc_ch[0] = 1500 + roll_amp·sin(wt)·500`；`rc_ch[1] = 1500 + pitch_amp·cos(wt)·500`
  - 速率模式下：速度 ≈ 0.75×1.7×~2 ≈ 2.55m/s → 半径 ≈ 2.55÷(2π/8) ≈ 3.25m
- **定高外环**（注入层，经油门通道与固件内环级联）：
  - `hold_z`：起飞脱离瞬间锁定（`hold_z = Some(z)` 当 throttle 松杆判定）
  - `z_filt = 0.85·z_filt + 0.15·z`（低通）
  - `err = z_filt - hold_z`（NED z 向下为正，err>0=偏低）
  - `alt_int = clamp(alt_int + err·DT, -0.8, 0.8)`（积分）
  - 前馈 `ff = tilt·0.14`（机动倾斜越大升力损失越大，同步推油）
  - `thr = clamp(0.5 + ff + (0.55·err + 0.10·alt_int), 0.25, 0.95)` → `rc_ch[3]`
  - 实测起伏：0.64-0.92m（视 8 字幅度）

---

## 8. 视频流链路（H.264 编码 / 解码 / 降级）

### 编码（服务器）
- openh264 编码器（`new_h264_encoder(bitrate_bps, intra_period)`）：
  - `max_frame_rate = FrameRate::from_hz(8.0)`（**固定 8，勿改**——见踩坑 13.1）
  - `profile = Profile::Baseline`（**无 B 帧**——见踩坑 13.4）
  - `intra_frame_period = 15`（GOP 15——见踩坑 13.2）
  - bitrate 档位：桌面 640×480 `1_200_000`、手机 `800_000`、lowest 320×240 `500_000`
- 帧格式：`w(4B LE) + h(4B LE) + fmt(1B) + [fmt=2: key(1B) + H.264 Annex-B NAL]`
- 分辨率档（WS 查询参数协商）：
  - 桌面 `?fmt=h264`（无 q）→ 640×480
  - 手机 `?fmt=h264&q=low` → 640×480 @800k（UA 检测自动）
  - 手机解码 640 失败 → 前端自动退 `q=lowest` → 320×240 @500k（仍是 H.264 视频流）
  - 再失败 → PNG（`forcePng`，fmt=1，320×240）
- 背压：渲染 channel(4) 满 → 丢输入帧 + `force_idr` → 下一编码帧 `force_intra_frame()`
  （open264 API，重置参考链，防止浏览器因缺参考帧报错）。

### 解码 + 绘制（前端 web/main.js）
- WebCodecs `VideoDecoder`（Secure Context 限定：HTTPS/localhost 才可用；明文 HTTP 自动退 PNG）。
- 首帧提取 SPS/PPS → 打包 avcC description → `vdec.configure({codec, description, codedWidth, codedHeight})`。
- **解码输入用 length-prefixed**（每 NAL 4 字节长度前缀），与 avcC description 一致——Annex-B 直喂会报错。
- **rAF 节流绘制**：output 回调只 `pushFrame(frame)`（存最新、close 旧帧），
  `requestAnimationFrame` 统一 `drawImage`——消除解码器 burst 导致的画面卡顿。
- **降级链**（`resLevel` 状态机）：0=640×480 → 解码无输出/错误 → 1=320×240（q=lowest）→ 再失败 → PNG。
- **无输出兜底**：H.264 连接后 4s 解码器仍无输出（静默故障/手机硬解 640 失败）→ 沿降级链降。
- **偶发错误静默**：解码错误 console.warn + 自动重同步（≤0.75s 等关键帧），不再刷状态栏；
  仅连续失败（vdecErrors≥2）才提示 + 降级。
- 等待关键帧期间状态栏提示“等待视频流关键帧…”，出帧后恢复。

---

## 9. 前端（web）

- 单页 `index.html` + `main.js`。WS 推帧 → canvas 渲染 + HUD（高度/水平位移/速度/姿态/电机）+ 控制面板。
- `fmt`/`q` 协商：`?fmt=${canH264 ? "h264" : "png"}${isMobile && canH264 ? "&q=low" : ""}`
  - `isMobile` 正则：`/Android|iPhone|iPad|iPod|Mobile|HarmonyOS/i`
- 遥测 JSON 字段（fmt=1 消息）：`alt`、`horiz`、`speed`、`att`（姿态）、`m`（电机）、`t` 等；HIL 真机模式另有 `hil` 字段。
- 降级路径：`canH264=false`（无 WebCodecs/非 HTTPS）→ PNG；H.264 持续失败 → forcePng。

---

## 10. 关键参数速查表

| 参数 | 值 | 说明 |
|---|---|---|
| `DT` | 0.004s | 物理步长 |
| `VP_STEPS_PER_FRAME` | 4 | 每显示帧物理步数 → 20fps |
| `VP_RUN_EVERY` | 1 | 每物理步 run 固件（必须 1） |
| run 步数 | 200_000 | 固件控制周期（~12ms/步） |
| 8 字周期 | 8s | roll/pitch amp 0.75 |
| 速率增益 | 1.7 | 摇杆→期望速度 m/s |
| 定高外环 | kp 0.55 / ki 0.10 / ff tilt×0.14 | z_filt 0.85/0.15、thr clamp 0.25-0.95 |
| 编码 bitrate | 桌面 1200k / 手机 800k / lowest 500k | openh264 |
| 编码 GOP | 15 | 关键帧间隔 |
| 编码 max_frame_rate | 8.0 | **固定，勿改** |
| 编码 profile | Baseline | **固定，勿改** |
| 渲染 channel | 4 | 背压丢帧 + force_idr |
| 端口 | 8082 明文 / 8081 HTTPS | socat TLS 代理 |

---

## 11. 性能实测基线（本地，127.0.0.1）

| 指标 | 值 |
|---|---|
| 推帧率 | 桌面/手机均 ~24fps（20fps 档稳定） |
| 带宽 | 桌面 ~3.0Mbps、手机 ~2.3Mbps、lowest ~1Mbps |
| 本地解码验证 | 40-45 帧 0 错误全解 |
| 定高 | alt 起伏 0.64m |
| 8 字半径 | ~3.3-3.8m |
| 绝对 fps 上限 | ~80fps（run 12ms 是硬下限，1 步/帧）——**但 40fps+ 档 open264 跳帧不稳，已弃用** |

---

## 12. 调试工具与复现方法

- **WS 抓流/测 fps/带宽**：用 Python socket + WS 帧解析脚本（`send_text` 发控制 JSON
  `{"scenario":"vperiph","controller":"pid","wind":0.5,...}`，收 0x2 视频帧 / 0x1 遥测）。
  抓帧结构：`w(4)+h(4)+fmt(1)+[key(1)+NAL]`，fmt=2 才是 H.264。
- **本地解码验证服务器流**（关键：验证器必须按“完整 access unit 喂 + flush_remaining”）：
  `/tmp/vdec2/`（Rust，openh264 0.7 默认 features）。抓流存成 `u32 LE 长度前缀 + NAL`，
  `cargo run --release /tmp/x.lp` → 看“解码 N 帧 / 0 错误”。**不要逐 NAL 切分喂**（会报 dsNoParamSets）。
- **编码参数对比测试**：`/tmp/venc/`（Rust，复制了服务器的 I420Source + rgba_to_i420，
  EncoderConfig 参数化）生成各参数流 → vdec2 解码对比。
- **手工验证**：`curl -sk https://150.158.109.150:8081/` 应 200；`curl -s http://127.0.0.1:8082/main.js | grep ...` 看是否已 serve 新 JS。
- 每轮改编码/固件参数后：build → 重启两 job → 抓流本地解码验证 → 让用户强刷验证。

---

## 13. 踩坑记录（重要——不要重复踩）

1. **open264 `max_frame_rate` 20/40 的流 WebCodecs 不兼容**：
   40fps 声明本地解码仅 38/60（丢帧）、浏览器直接解码失败。**必须固定 8.0**（SPS VUI 兼容，
   用户正常显示过）。码控按 8fps 标定（实际码率偏宽松 ~3Mbps），推帧率不受限（由 VP_STEPS_PER_FRAME 决定）。
2. **GOP 过长 → P 帧链累积错误 → 周期性卡顿**：GOP 30 时“每 ~3s 卡一下”（周期=关键帧间隔）。
   **GOP 必须 15**（短 P 链 + IDR 0.75s@20fps 频繁重同步）。
3. **40fps 推帧 → open264 跳帧 → 解码错误**：800k÷40fps=20k/帧预算不足，动态场景主动跳帧 →
   参考帧缺失 → 周期性报错。**统一 20fps（VP_STEPS_PER_FRAME=4）**，40fps 档已弃用。
4. **High profile 的 B 帧 → 偶发解码错误**：open264 在 20fps 输入下输出 B 帧，
   前端单调 timestamp 与其 POC 偶发不一致 → WebCodecs 偶发报错。**用 Baseline（无 B 帧）**。
5. **丢输入帧 → 参考链缺失 → 零星错误**：渲染线程慢时 channel 满丢输入 → 编码 P 帧链断。
   **channel 2→4 + 丢帧后 force_intra_frame()**。
6. **VP_RUN_EVERY=2（每 2 物理步 run）→ 控制律滞后 → 发散坠机**（alt 起伏 9.9m）。必须 1。
7. **run(150k) 不稳定**（控制律 18ms/周期超临界，alt 起伏 4.4m）。run(200k)=13ms/周期是稳定下限。
8. **Annex-B 直喂 WebCodecs 报错**：前端必须转 length-prefixed（与 avcC description 一致）。
9. **解码器输出 burst → 画面卡**（不是帧率问题）：output 回调直接 drawImage 会把 burst 抖到画面，
   必须 rAF 节流（只画最新帧）。
10. **8 字摇杆从未生效**：固件 PidController 原忽略 rc.roll/pitch（setpoint 固定 (0,0)）。
    需改固件 setpoint 水平=摇杆。
11. **速率模式位置项正反馈**：kp_xy*ex 与速度同向 → 半径 5.4-5.8m。需 rate_mode_xy 旁路位置外环。
12. **降级死循环**：降级 PNG 后重连 canH264 仍 true 又回 h264。需 forcePng 记忆降级状态。
13. **start_https.sh 超时**：后台 job fd 继承问题，勿用——直接起两个后台 job（socat + server）。
14. **SIL 悬停场景带宽异常 22Mbps**（渲染每帧全变→每帧大帧）。仅影响 SIL 场景，未修。

---

## 14. Git 版本史摘要

**fly-simulater**（核心演进，最新在前）：
- `ab136ad` 零星解码错误：丢帧 force IDR + 前端静默自恢复
- `7db3fda` 偶发解码错误：Baseline（无 B 帧）
- `d24b8db` 周期性 3s 卡顿：GOP 回 15 + High
- `b289580` 40fps 跳帧：统一 20fps + Baseline
- `ab6faa2` 解码失败根因：max_frame_rate 20/40 → 固定 8
- `e0b04f4` 解码降级链 640→320→PNG
- `f9f568e` 视频流无显示：GOP 缩短 + 无输出兜底 + 降级防死循环
- `f9542b7` 移动端分辨率提到 640×480
- `a5d51d9` 分辨率不变降码率（bitrate/GOP 参数化）
- `de6cc9d` 移动端降档 q=low
- `51f968e` 帧率档位选 40fps（上限实测 80fps）
- `63ee694` rAF 节流绘制（消除 burst 卡顿）
- `bb431d3` HIL 链路抽独立 crate fly-sim-hil + 路径修正
- `4a786e8` vperiph 提速 2.6×（帧步 8→4 + 8 字周期 16s→8s）
- `fa2fad2`/`bcbd699`/`8d5ceb7` 8 字幅度 + 定高外环
- `5b34430` Annex-B→length-prefixed
- `ee1462f` 去掉背压丢帧 + 解码错误自恢复
- `5b35c58` HTTPS：自签证书 + socat TLS 代理
- `ec67c64`/`f223538`/`0e30a76` H.264 视频流 + WebCodecs + 分辨率 640×480
- `2e5c169` PNG 推帧（公网 0.37→8fps）
- `fdad568` 渲染/推帧独立线程

**flyctrl**：
- `7a7089e` 速率模式（摇杆→期望速度，位置外环旁路）
- `8b2c447` 8 字半径 ~3m（setpoint 水平系数 ×8.5）
- `00284d3` 非 HIL 摇杆水平机动
- `ec2cd60` GPS Doppler 速度链路 + 垂向观测隔离
- `04b79b5` HIL 闭环诊断观测口

---

## 15. 已知限制与后续方向

- **带宽偏高**（桌面 ~3M）：IDR 密集（GOP15）+ 码控按 8fps 宽松。稳定优先，暂不追低。
  后续可在“稳定”前提下尝试 GOP 15-25 折中或降手机码率（需用户实测浏览器）。
- **40fps+ 档不可用**（open264 跳帧限制）。若未来要更高帧率：换 x264 软编（开销大）或硬件编码。
- **SIL 悬停场景带宽异常（22Mbps）**未修。
- 遗留：fly-sim-hil 独立 crate 已提交；若做真机 HIL 需接 USB-CDC（见 fly-sim-hil）。
- 公网 RTT 抖动未做播放缓冲（当前 rAF 只画最新帧，延迟最小；极端弱网可加 2-3 帧 jitter buffer）。
