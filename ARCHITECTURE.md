# 飞控建模仿真平台 — 架构文档

> **本文档是理解本平台的权威入口。** 平台定位：**飞控建模仿真平台**——用「物理世界仿真 +
> MCU 指令级仿真 + 总线协议级外设仿真」三层栈，把**真实飞控固件二进制（零改动）**跑在
> 模拟硬件上并形成闭环。Web 页面（浏览器看飞机画面/遥测）只是这个平台的一个**消费端应用**，
> 不是平台本体。后续任务围绕平台本身的完善展开。
>
> 最后更新对应 git：`ab136ad`（fly-simulater）/ `7a7089e`（flyctrl）/ mcu_simulater（M17）。
> 交流语言：中文。

---

## 1. 平台定位与三层仿真栈

```
┌────────────────────────────────────────────────────────────────┐
│  固件层（被测对象）：真实飞控固件 flyctrl（.bin，real-sensors）  │
│   —— 在模拟 MCU 上零改动运行，跑自己的驱动/EKF/控制律/状态机      │
├────────────────────────────────────────────────────────────────┤
│  第 2 层：MCU 仿真（mcu_simulater）                             │
│   —— Unicorn Engine 执行 Cortex-M4F 指令（STM32F407VET6 目标）   │
│   —— Rust 实现内存映射 / MMIO 外设 / NVIC 中断 / 虚拟时钟         │
│   —— 事件总线挂载 25+ 个外设（GPIO/USART/I2C/SPI/TIM/ADC/DAC/…） │
├────────────────────────────────────────────────────────────────┤
│  第 3 层：物理世界仿真（fly-sim-core）                          │
│   —— 四旋翼刚体动力学（桨叶素动量理论 + 失速模型）               │
│   —— 接触/地面/障碍物/避障 / 风场（恒风+阵风+湍流）/ 传感器噪声   │
│   —— 时间步 DT=4ms 推进，SIL 与 HIL 两种驱动模式                 │
└────────────────────────────────────────────────────────────────┘
        ▲ 闭环方向：物理 → 传感器真值 → 外设仿真 → 固件感知/控制 → PWM → 物理
```

**平台核心价值**：外设是**总线协议级**仿真（真实 I2C/SPI/UART 从机行为），固件驱动
**零改动**跑通——这是与"把传感器读数直接喂给控制律"的玩具仿真的根本区别。

---

## 2. 仓库与代码位置

| 路径 | 角色 |
|---|---|
| `/home/ubuntu/work/mcu_simulater/` | **MCU 仿真器**（仿 Renode）：Unicorn CPU + Rust 外设/总线/中断/时序，独立 git |
| `/home/ubuntu/work/fly-simulater/` | **物理仿真 + 应用**：fly-sim-core（物理/控制/渲染）+ fly-sim-server（平台驱动 + Web 后端）+ web（消费端） |
| `/home/ubuntu/work/fly-simulater/fly-sim-core/` | 物理世界：`physics.rs`（刚体/接触/碰撞/障碍）、`plant.rs`（四旋翼桨模型）、`sim.rs`（SimLoop 闭环）、`wind.rs`（风场）、`sensor.rs`（噪声）、`render.rs` |
| `/home/ubuntu/work/fly-simulater/fly-sim-server/src/main.rs` | 平台驱动：场景循环、vperiph 虚拟 MCU 装配（boot/EKF/ARM）、机动注入、渲染/推帧线程 |
| `/home/ubuntu/work/fly-simulater/fly-sim-hil/` | 真机 USB-CDC HIL 链路 crate（虚拟 MCU 的姊妹通路） |
| `/home/ubuntu/work/fly-simulater/web/` | 消费端：浏览器 canvas + HUD + H.264 视频流 |
| `/home/ubuntu/work/flyctrl/` | 被测固件源码（独立 git），编译产物加载到模拟 MCU |
| `/home/ubuntu/work/joc-base/build_rel/stm32f407_minimal.elf` | 模拟 MCU 的系统镜像（VP_SYS，含 RTOS/app 分区 loader） |

---

## 3. MCU 仿真（mcu_simulater）—— 第 2 层

### 3.1 职责边界（关键设计）

- **Unicorn 只管**：执行指令（THUMB+MCLASS+VFP4）、访问已 map 内存、寄存器读写、触发 hook。
- **Rust 管**：MMIO 外设、NVIC（挂起/抢占/异常入栈出栈 + `BX LR` 返回拦截）、内核私密外设
  （SysTick/SCB/MPU）、虚拟时钟、事件总线互联、调试、固件加载。
- 内存总线：RAM/Flash 直接 map 给 Unicorn；MMIO 经 mem hook 转发到 Rust 外设；MPU 8 region
  访问控制（取指 XN / 数据三入口）+ MemManage fault。

### 3.2 时序模型

- **块级加权虚拟时钟**：每执行块按指令/周期加权推进虚拟时间；
- **外设 tick**：TIM/DMA/DAC/RTC/IWDG/WWDG 各自按周期触发（更新中断、更新事件 DMA）。
- **性能**：纯计算负载 **105 MIPS**（min-of-3）；Unicorn 比真机慢 ~5-10×，是帧率的主要成本。

### 3.3 已实现外设（STM32F407VET6，M0→M17 全落地）

核心 NVIC/SCB/MPU/SysTick；系统 RCC/PWR/CRC/RNG/EXTI+SYSCFG；GPIOA-I（含 M17 寄存器级
pinmux AFRL/AFRH）；USART1-6/UART4-5（TX/RX+DMA+IRQ）；I2C1-3、SPI1-3（EV 中断+DMA）；
TIM1-14（更新中断 + 更新事件 DMA）；ADC1-3（DMA2）、DAC1（定时器触发+DMA）；DMA1/2；
IWDG/WWDG（超时复位）；RTC+BKP；DCMI（帧注入+DMA2）；FSMC（Bank1-4）；SDIO；CAN1/2
（3 邮箱+滤波+总线互联）；USB OTG FS（设备模式+虚拟主机注入）；虚拟 Console/Terminal。

### 3.4 架构分层

```
前端层：CLI / Monitor(REPL) / GDB Server / 日志        （Monitor/GDB 为占位，见 §9）
配置层：Machine 描述（Board / 外设挂载 / connect）      （类 Renode DSL，占位）
仿真内核层：仿真循环 / 时间模型 / 事件调度器 / status.rs 全局状态位域
设备层：Peripheral trait + 注册表 + 具体外设
核心层：Unicorn（CPU 执行）+ Rust Memory Bus
```

---

## 4. 外设仿真（vperiph）—— 总线协议级虚拟外设

### 4.1 设计原则

- **总线协议级**：从设备模拟真实 I2C/SPI/UART 从机行为（地址匹配、寄存器指针、ACK/NACK），
  固件真实驱动（flyctrl real-sensors）**零改动**直接跑通。
- **挂载在总线外设内部直路由**：i2c/spi/usart 外设持有从设备表，事务时直接路由到匹配从设备
  （不走 EventBus 字节流解析——避免无 addr/无方向歧义）；EventBus 保留供外部注入/观测。
- **数据源抽象** `DataSource`：`Const`（固定寄存器值/WHO_AM_I）+ `Math`（物理模型，随仿真时间
  step）；后续可扩展 `Replay`（真机数据回放）/ `Script`（脚本注入）——**平台扩展点**。

### 4.2 器件清单（一器件一文件，可插拔）

| 总线 | 器件 | 说明 |
|---|---|---|
| I2C | mpu6050(0x68) / bmp280(0x76) / qmc5883(0x0D) / sht30 / vl53l1x / at24cxx | IMU/气压/磁力/温湿/测距/EEPROM |
| SPI | bmi088（双片选 ACCEL_CS/GYRO_CS）/ flash / pmw3901 | 六轴 IMU / 存储 / 光流 |
| UART | nmea_gps（$GNGGA 推流，ublox 协议）/ sbus（SBUS 遥控帧） | GPS / 遥控输入 |
| 专用 | esc（PWM→电机语义）/ fsmc/st7789（LCD） | 输出侧 / 显示 |

### 4.3 FlySimState 直通注入（物理 ↔ 外设桥）

- `FlySimState`（Arc<Mutex>）是物理世界与外设仿真的**共享状态总线**：
  `imu_acc/imu_gyr/baro_pa/gps_lat/lon/alt/fix/vel/rc_ch[16]`。
- 服务器每物理步：物理推进 → 噪声化真值（`SensorConfig::realistic`）→ 写 FlySimState；
  固件经虚拟 I2C/UART 事务读到的就是这些值（`FlySimSource` 按角色解析）。
- **一致性不变量**：`FlySimState` **只能在两次 `run()` 之间写入**（run 期间冻结）——
  否则固件读到的寄存器中途变化，破坏总线语义。见 `docs/virtual_direct_mode.md` §8.1。
- **ESC/PWM 读回**：固件写 TIM 的 CCR/ARR（内存映射地址 VP_TIM1-4），服务器读
  `duty = CCR/ARR` → 归一化电机指令（`read_thrust`）→ 施加到物理。这就是"指令来源"。

---

## 5. 物理世界仿真（fly-sim-core）—— 第 3 层

### 5.1 四旋翼动力学（plant.rs）

- 刚体：质量/惯量（cfg.inertia）、半隐式欧拉积分（`apply_impulse`/`apply_torque_impulse`）。
- **电机一阶滞后**：`thrust_actual` 指数趋近目标油门（tau=cfg.motor_tau）。
- **桨模型（叶素动量理论 Bet + 失速）**：由转速 ω、桨半径/盘面积、空气密度 ρ 求
  推力 `thrust = ct_eff·ρ·A·vt²`、反扭矩 `torque_factor = 1 + 2·s`（s=失速深度随前进比 μ）——
  水平速度大 → 失速加深 → 反扭矩剧增 + 水平力增大（真实飞行特性，非线性表）。
- 满油门单电机推力由 `thrust_coeff` 标定（k_t per (rad/s)²）。

### 5.2 世界/接触/障碍（physics.rs）

- `RigidBodyWorld`（PhySdkWorld / ToyWorld）：刚体注册、积分、接触求解。
- `ContactModel`/`ground height_at`：地面高程 + 障碍物（静态 + 动态 `DynamicObstacle::at(t)`）。
- 地面接触 `resolve_ground_contact`、机体互撞 `resolve_body_peer_collisions`、
  射线测距 `ray_obstacle_distance`（避障场景）。

### 5.3 风场（wind.rs）

- 恒风 base + 阵风 `gust`（带随机相位正弦，可多频叠加）+ **湍流 `turb`（Dryden 简化：
  一阶低通白噪声，各轴独立）** + 阵风突发 `gust_burst`（时间窗脉冲）。

### 5.4 传感器噪声（sensor.rs / SensorConfig）

- `SensorConfig::realistic`（vperiph 全场景固定）：IMU/GPS/气压叠加噪声、偏置、延迟；
  GPS Doppler 速度（`$GNRMC`）约束 EKF 水平速度漂移。
- SIL 控制律与 vperiph 外设注入**同源**（同一个噪声化真值），保证两通路一致。

### 5.5 两种驱动模式（sim.rs / SimLoop）

| 模式 | 控制律位置 | 推进方式 | 用途 |
|---|---|---|---|
| **SIL** | PC（flyctrl_core 控制律在服务器进程内） | `step_frame(sp)` 每显示帧推进 | 多场景快速仿真（hover/wind/degraded/avoidance） |
| **HIL** | 真实 MCU（固件） | `step_hil(cmd)`：外部电机指令驱动单步物理 | 真机 USB-CDC HIL / **虚拟 vperiph（Unicorn）** |

`step_hil` 是平台核心：PC 只把外部（真机或虚拟 MCU）回传的电机指令施加到被控对象并推进
物理一步，同时刷新 IMU/GPS/气压真值供注入——**控制律完全在固件里**。

---

## 6. 虚拟 MCU 闭环（vperiph）—— 平台默认演示链路

`VperiphMc`（fly-sim-server/main.rs）：把 mcu_simulater 的 Machine 与物理 SimLoop 接成闭环。

### 6.1 装配流程（boot，每次场景切换重建）

1. `Machine::new_m4f()` + `map_stm32f407_layout()`（内存布局）
2. `attach_flysim_sensors`（I2C/SPI 传感器直通）+ `attach_flysim_uart_slaves`（GPS/SBUS）
3. 注入初始真值（静止水平悬停 + baro h=0 + GPS 有效）→ 加载 VP_SYS 系统镜像 + VP_APP 固件
   → `reset()` → boot run
4. **EKF 收敛**：循环 run + 读固件 EKF 高度（内存地址 `0x2000_9074+28`）直至 |z|<0.6
5. **ARM + 解锁**：写 ARM 标志内存位（`0x2000_b669`=1）+ rc_ch[4]=2000（解锁通道）
6. Unicorn 偶发 `UC_ERR_INSN_INVALID`（M4F 瞬时不稳定）→ 内部重建重试（最多 8 次）

### 6.2 每物理步闭环（advance_vperiph，DT=4ms）

```
read_thrust() 读固件 PWM（TIM1-4 CCR/ARR → duty → 0..1）
  → ActuatorCmd → sim.step_hil(cmd) 推进物理（起飞台保持：thrust<0.05 前机体静止）
  → inject(sim) 噪声化 IMU/GPS/baro 真值写 FlySimState
  → inject_maneuver() 注入 SBUS 摇杆（8 字机动 + 定高外环，见 §7）
  → vp.run_step() = Unicorn run(200_000)（固件控制律运行，唯一 CPU 成本）
  → 循环 VP_STEPS_PER_FRAME=4 次 → 渲染/推帧
```

### 6.3 与真机 HIL 的异同

- 同：闭环骨架（step_hil + 起飞台保持 + 渲染/遥测）与 `advance_hil` 共用。
- 异：指令来源（读 Unicorn 内 PWM vs USB 下行）、注入通路（FlySimState vs HIL_SENSOR 消息）。

---

## 7. 机动注入与定高外环（服务器注入层）

- **8 字机动**：`roll_amp=0.75 / pitch_amp=0.75`、`w=2π/8`（周期 8s 仿真），
  `rc_ch[0]=1500+roll_amp·sin(wt)·500`、`rc_ch[1]=1500+pitch_amp·cos(wt)·500`。
- **定高外环**（经油门通道与固件内环级联）：`z_filt=0.85/0.15` 低通 → err →
  `alt_int=clamp(+err·DT, ±0.8)` → 前馈 `ff=tilt·0.14` →
  `thr=clamp(0.5+ff+(0.55·err+0.10·alt_int), 0.25, 0.95)` → rc_ch[3]。
  实测 alt 起伏 0.64m；hold_z 在脱离起飞台瞬间锁定。
- **固件侧**（flyctrl）：速率模式（大疆手感）——`rate_mode_xy` 旁路位置外环、
  `vel=[rc.pitch·1.7, -rc.roll·1.7, 0]`、`pos=[速度外推预测, hold_alt]`、油门中位=定高。

---

## 8. 场景与配置

| 场景 | 说明 |
|---|---|
| `vperiph` | 默认：Unicorn 虚拟 MCU 闭环（真实固件），全场景 realistic 噪声 + 风场 |
| `hover` / `wind` / `degraded` / `avoidance` | SIL（PC 控制律）演示场景：悬停 / 抗风 / 电机退化 / 避障 |

- 控制参数经 WS JSON：`{"scenario","controller","wind","fail_motor","degrade","sensor_noise"}`。
- 场景/配置变更 → 虚拟 MCU 重建（下次 advance_vperiph 重新 boot）。

---

## 9. 消费端（web 应用）—— 平台的一个功能

浏览器（`web/`）只是消费端：WS 收渲染帧 + 遥测 JSON → canvas 绘制 + HUD + 视频流。

- 渲染帧：`w(4)+h(4)+fmt(1)+[fmt=1:PNG | fmt=2:key(1)+H.264 Annex-B]`；遥测 op=0x1。
- 视频流：openh264 编码（**Baseline、GOP15、max_frame_rate=8 固定、桌面 1200k/手机 800k/320p 500k**）
  → 浏览器 WebCodecs 解码（length-prefixed、rAF 节流绘制）→ 降级链 640→320→PNG。
- **踩坑（应用层，勿重踩）**：open264 max_frame_rate 20/40 的流 WebCodecs 不兼容（必须 8）；
  GOP 30 长 P 链周期卡顿（必须 15）；40fps 推帧 open264 跳帧（必须 20fps 档）；
  High profile B 帧偶发错误（必须 Baseline）；丢输入帧后必须 force_intra_frame() 重置参考链。
- 运行拓扑：fly-sim-server（明文 8082，`FLY_SIM_PORT=8082`）+ socat TLS 终止（8081，
  `certs/cert.pem`）；**勿用 start_https.sh**（后台 fd 继承超时），直接起两个后台 job。
- 改 web/main.js 无需重编译（静态读盘），浏览器强刷即可。

---

## 10. 关键参数速查

| 参数 | 值 | 说明 |
|---|---|---|
| `DT` | 0.004s | 物理步长 |
| `VP_STEPS_PER_FRAME` | 4 | 每显示帧物理步数（20fps 推帧） |
| `VP_RUN_EVERY` | 1 | 每物理步 run 固件（**必须 1**，2 会发散） |
| Unicorn run 步数 | 200_000 | 固件控制周期（~12ms/步；150k 不稳） |
| 8 字周期 / 幅度 | 8s / 0.75 | roll & pitch |
| 速率增益 | 1.7 | 摇杆→期望速度 m/s |
| 定高外环 | kp 0.55 / ki 0.10 / ff tilt×0.14 | thr clamp 0.25-0.95 |
| 编码 | Baseline / GOP15 / mfps 8 | 桌面 1200k / 手机 800k / 320p 500k |
| 端口 | 8082 明文 / 8081 HTTPS | socat TLS 代理 |
| 固件 | /tmp/flyctrl_real.bin | real-sensors 编译（≈58KB） |
| 系统镜像 | joc-base/build_rel/stm32f407_minimal.elf | 模拟 MCU 系统 |

---

## 11. 性能基线

- MCU 仿真：**105 MIPS**（纯计算）；Unicorn 比真机慢 ~5-10×。
- 虚拟 MCU 闭环：20fps 推帧稳定（run 200k ≈12ms/步是硬下限）；80fps 绝对上限（1 步/帧）但
  open264 40fps+ 跳帧不稳，已弃用。
- 定高 alt 起伏 0.64m；8 字半径 ~3.3-3.8m。
- 视频带宽：桌面 ~3Mbps / 手机 ~2.3Mbps（IDR 密集为主因，稳定优先）。

---

## 12. 调试工具

- **MCU 仿真器**：`mcu_simulater/tests/`（m0-m17 里程碑集成测试 + bench_mips/bench_probe/bench_tb）、
  `firmware/`（30 个验收固件覆盖各外设 demo）。
- **虚拟闭环**：`[vperiph]` 前缀 eprintln 日志（boot/收敛/run 失败）；
  EKF 高度直读 `0x2000_9074+28`；ARM 标志 `0x2000_b669`。
- **WS 抓流/测帧率带宽**：Python socket + WS 帧解析（fmt=2 才是 H.264）。
- **H.264 流本地解码验证**：`/tmp/vdec2/`（openh264，**按完整 access unit 喂 + flush_remaining**，
  勿逐 NAL 切分）；`/tmp/venc/` 编码参数对比。
- 真机 HIL：fly-sim-hil crate（USB-CDC，与虚拟通路共用闭环骨架）。

---

## 13. 平台踩坑记录（重要）

**MCU/外设层**：
1. `UC_ERR_INSN_INVALID` 偶发（M4F 模拟瞬时不稳定）→ boot 重试 ≤8 次。
2. `FlySimState` 只能在两次 `run()` 之间写入（run 期间冻结），否则总线语义破坏。
3. GPS Doppler 速度必须走 `$GNRMC` 下发，否则 EKF 水平速度纯积分漂移。
4. 固件必须 `--features real-sensors` 编译（默认/其他 feature 产物体积不对、跑不通）。

**物理层**：
5. 速率模式位置项正反馈（kp_xy·ex 与速度同向）→ 8 字半径冲到 5.4-5.8m；需旁路位置外环。
6. `VP_RUN_EVERY=2` 控制律滞后 → 发散坠机；`run(150k)` 超临界不稳——200k/每步必守。
7. 起飞台保持必须（固件未产生推力前机体静止），否则自由落体撞地尖峰。

**应用层（视频流）**：见 §9 踩坑（max_frame_rate=8 / GOP15 / 20fps / Baseline / force IDR）。

---

## 14. 平台完善方向（后续任务围绕这里）

1. **数据源扩展**：`DataSource` 增加 `Replay`（真机飞行数据回放驱动外设）/ `Script`
   （脚本注入机动/故障）——外设仿真从"物理模型驱动"走向"任意数据驱动"。
2. **MCU 仿真器补全**：Monitor REPL、GDB Server（源码级调试固件）、类 Renode DSL 配置
   （Machine 描述从代码走向配置）、新外设（ETH/DAC 深化/USB 深化）、MPU/NVIC 时序深化。
3. **物理模型深化**：地面效应、空气动力（机身阻力）、螺旋桨滑流、温度/电池模型、
   更多机型（固定翼/倾转）、多机编队。
4. **场景库**：故障注入（传感器失效/电机卡死/通信中断）、任务脚本（起飞-巡航-降落）、
   自动化测试（回归：参数扫描 + 稳定性断言）。
5. **性能**：Unicorn 多核/缓存提速、物理并行、降低 20fps 帧率的 CPU 占用。
6. **消费端增强**（次要）：视频带宽优化（GOP 折中）、弱网 jitter buffer、飞行数据记录回放。

---

## 15. Git 版本史摘要

- **mcu_simulater**：M0→M17（Cortex-M4F/内存总线/MPU/NVIC/外设集/事件总线/虚拟时钟，
  105 MIPS）；vperiph 虚拟外设层（I2C/SPI/UART 从设备 + DataSource）。
- **fly-simulater**：`fdad568` 渲染线程解耦 → `2e5c169` PNG 推帧 → `ec67c64` H.264 视频流 →
  `5b35c58` HTTPS → `4a786e8` vperiph 提速 2.6× → `bb431d3` HIL 独立 crate →
  `51f968e` 40fps 档 → 视频流稳定性系列（ab6faa2/b289580/d24b8db/7db3fda/ab136ad）。
- **flyctrl**：`00284d3` 摇杆水平机动 → `8b2c447` 半径 ~3m → `7a7089e` 速率模式。
