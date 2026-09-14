// Fly Simulator Web 前端：WebSocket 收帧 → canvas 渲染 + 遥测面板 + 控制面板。
(() => {
  "use strict";
  const canvas = document.getElementById("view");
  const ctx = canvas.getContext("2d");
  const statusEl = document.getElementById("status");

  // 复用后端帧尺寸（与 fly-sim-server FRAME_W/H 一致）
  const FRAME_W = 720, FRAME_H = 540;
  // 后端二进制帧 = w(4 LE) + h(4 LE) + RGBA 字节(w*h*4)。
  let imgData = ctx.createImageData(FRAME_W, FRAME_H);

  // 相机状态（前端拖拽/滚轮维护，发给后端）
  let cam = { yaw: 0.6, pitch: 0.45, distance: 28.0 };

  // 控制面板当前值
  const ctl = {
    scenario: "vperiph", // 默认虚拟 MCU 闭环（与下拉 selected 一致）；可选 hil/hover/wind/degraded/avoidance
    controller: "pid",
    wind: 0.5,
    fail_motor: null,   // 整数或 null
    degrade: null,      // [m, e] 或 null
  };

  let ws = null;
  let lastTele = 0;

  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    // WebCodecs（VideoDecoder）是 Secure Context 限定 API：HTTPS/localhost 才可用。
    // 明文 HTTP 下自动回退 PNG 推帧（服务器按 ?fmt= 协商编码格式）。
    const canH264 = window.isSecureContext && window.VideoDecoder;
    ws = new WebSocket(`${proto}://${location.host}/ws?fmt=${canH264 ? "h264" : "png"}`);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
      if (!canH264 && !window.VideoDecoder) {
        statusEl.textContent = "已连接（PNG 推帧：浏览器无 WebCodecs）";
      } else if (!canH264) {
        statusEl.textContent = "已连接（PNG 推帧：需 HTTPS 才支持视频流）";
      } else {
        statusEl.textContent = "已连接（H.264 视频流）";
      }
      statusEl.style.color = "var(--accent)";
      sendControl();
    };
    ws.onclose = () => {
      statusEl.textContent = "断开，重连中…";
      statusEl.style.color = "var(--warn)";
      setTimeout(connect, 1000);
    };
    ws.onerror = () => { statusEl.textContent = "连接错误"; };

    ws.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) {
        drawFrame(new Uint8Array(ev.data));
      } else if (typeof ev.data === "string") {
        applyTelemetry(ev.data);
      }
    };
  }

  // ---- H.264 视频流解码（WebCodecs，fmt=2）----
  let vdec = null;
  let vdecPending = [];
  let h264UnsupportedShown = false;

  function hex2(v) { return v.toString(16).padStart(2, "0"); }

  // 从 Annex-B NAL 流提取 SPS(7)/PPS(8)。
  function extractSpsPps(nal) {
    let i = 0, sps = null, pps = null;
    while (i < nal.length - 3) {
      if (nal[i] === 0 && nal[i + 1] === 0 && nal[i + 2] === 1) i += 3;
      else if (nal[i] === 0 && nal[i + 1] === 0 && nal[i + 2] === 0 && nal[i + 3] === 1) i += 4;
      else { i++; continue; }
      const type = nal[i] & 0x1f;
      let j = i + 1;
      while (j < nal.length - 3 && !(nal[j] === 0 && nal[j + 1] === 0 && (nal[j + 2] === 1 || (nal[j + 2] === 0 && nal[j + 3] === 1)))) j++;
      const body = nal.subarray(i, j);
      if (type === 7) sps = body;
      else if (type === 8) pps = body;
      i = j;
    }
    return { sps, pps };
  }

  function initVDecoder(sps, pps, w, h) {
    // avcC description（WebCodecs 需要，从 SPS/PPS 打包）
    const avcc = new Uint8Array(11 + sps.length + pps.length);
    avcc[0] = 1;
    avcc[1] = sps[1]; avcc[2] = sps[2]; avcc[3] = sps[3]; // profile/compat/level
    avcc[4] = 0xff; avcc[5] = 0xe1;
    avcc[6] = (sps.length >> 8) & 0xff; avcc[7] = sps.length & 0xff;
    avcc.set(sps, 8);
    avcc[8 + sps.length] = 1;
    const po = 9 + sps.length;
    avcc[po] = (pps.length >> 8) & 0xff; avcc[po + 1] = pps.length & 0xff;
    avcc.set(pps, po + 2);
    const codec = "avc1." + hex2(sps[1]) + hex2(sps[2]) + hex2(sps[3]);
    vdec = new VideoDecoder({
      output: (frame) => {
        ctx.drawImage(frame, 0, 0, canvas.width, canvas.height);
        frame.close();
      },
      error: (e) => { console.error("VideoDecoder error:", e); },
    });
    vdec.configure({ codec, description: avcc, codedWidth: w, codedHeight: h });
    for (const c of vdecPending) { try { vdec.decode(c); } catch (e) {} }
    vdecPending = [];
  }

  function drawH264(buf) {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    const w = dv.getUint32(0, true), h = dv.getUint32(4, true);
    const key = buf[9] === 1;
    const data = buf.subarray(10);
    if (!window.VideoDecoder) {
      // 老浏览器无 WebCodecs：vperiph 视频流无法解码，提示换场景（SIL 走 PNG 可用）
      if (!h264UnsupportedShown) {
        h264UnsupportedShown = true;
        statusEl.textContent = "当前浏览器不支持视频流解码(WebCodecs)，请用 Chrome/Edge，或切换到悬停/抗风等场景（PNG 推帧）";
      }
      return;
    }
    if (!vdec) {
      if (!key) return; // 等关键帧（含 SPS/PPS）
      const { sps, pps } = extractSpsPps(data);
      if (!sps || !pps) return;
      initVDecoder(sps, pps, w, h);
    }
    if (vdec.decodeQueueSize > 8) return; // 背压：解码队列满丢帧
    const chunk = new EncodedVideoChunk({
      type: key ? "key" : "delta",
      timestamp: performance.now() * 1000,
      data: data.slice(0),
    });
    try { vdec.decode(chunk); } catch (e) {}
  }

  // 解析二进制帧：前 8 字节 = (w,u32)(h,u32)，第 9 字节 = fmt，
  // fmt=0 → 余下 RGBA；fmt=1 → 余下 PNG；fmt=2 → key(1B)+H.264 Annex-B。
  function drawFrame(buf) {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    const w = dv.getUint32(0, true);
    const h = dv.getUint32(4, true);
    const fmt = buf[8];
    if (fmt === 2) { drawH264(buf); return; }
    if (fmt === 1) {
      const blob = new Blob([buf.subarray(9)], { type: "image/png" });
      createImageBitmap(blob).then((bmp) => {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(bmp, 0, 0, canvas.width, canvas.height);
        bmp.close();
      }).catch(() => {});
      return;
    }
    const px = buf.subarray(9, 9 + w * h * 4);
    if (!imgData || imgData.width !== w || imgData.height !== h) {
      imgData = ctx.createImageData(w, h);
    }
    // 后端像素是 u32(0xAARRGGBB) 的小端字节 = [B,G,R,A]；ImageData 需要 [R,G,B,A]，
    // 故交换每像素的 R/B。
    const d = imgData.data;
    for (let i = 0, j = 0; i < px.length; i += 4, j += 4) {
      d[j] = px[i + 2];       // R = B(后端)
      d[j + 1] = px[i + 1];   // G
      d[j + 2] = px[i];       // B = R(后端)
      d[j + 3] = px[i + 3];   // A
    }
    // 若后端帧尺寸与 canvas 不同，缩放绘制
    if (canvas.width !== w || canvas.height !== h) {
      const tmp = document.createElement("canvas");
      tmp.width = w; tmp.height = h;
      tmp.getContext("2d").putImageData(imgData, 0, 0);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(tmp, 0, 0, canvas.width, canvas.height);
    } else {
      ctx.putImageData(imgData, 0, 0);
    }
  }

  function applyTelemetry(json) {
    let t;
    try { t = JSON.parse(json); } catch (e) { return; }
    // HIL 真机模式：链路状态 + MCU 估计（遥测数字）+ 物理真值（对照）。
    if (t.hil) {
      applyHilTelemetry(t);
      drawMotors(t.m);
      return;
    }
    setText("t-alt", t.alt.toFixed(2) + " m");
    setText("t-horiz", t.horiz.toFixed(2) + " m");
    setText("t-spd", t.speed.toFixed(2) + " m/s");
    setText("t-roll", (t.roll * 180 / Math.PI).toFixed(1) + "°");
    setText("t-pitch", (t.pitch * 180 / Math.PI).toFixed(1) + "°");
    setText("t-yaw", (t.yaw * 180 / Math.PI).toFixed(1) + "°");
    setText("t-hil", "–");
    setText("t-truth", "–");
    const st = document.getElementById("t-state");
    if (t.diverged) {
      st.innerHTML = '<span class="badge bad">发散</span>';
    } else {
      st.innerHTML = '<span class="badge ok">稳定</span>';
    }
    drawMotors(t.m);
  }

  // HIL 遥测：状态徽章 + MCU 估计 + 真值对照。
  function applyHilTelemetry(t) {
    const h = t.hil;
    const st = document.getElementById("t-state");
    const hilEl = document.getElementById("t-hil");
    const stateMap = {
      connecting: ["探测 USB-CDC…", "var(--warn)"],
      waiting_heartbeat: ["等待心跳…", "var(--warn)"],
      running: ["已连接 · 闭环运行", "var(--accent)"],
      error: ["连接失败", "var(--bad)"],
    };
    const [label, color] = stateMap[h.state] || [h.state, "var(--warn)"];
    hilEl.textContent = label;
    hilEl.style.color = color;
    if (h.state === "error") {
      st.innerHTML = '<span class="badge bad">链路异常</span>';
      setText("t-truth", h.msg || "");
    } else {
      st.innerHTML = h.state === "running"
        ? '<span class="badge ok">HIL</span>'
        : '<span class="badge" style="background:rgba(227,179,65,.15);color:var(--warn)">连接中</span>';
    }
    // 遥测数字 = MCU 估计（真实飞控的"所见"），未到回退真值。
    const mcu = t.mcu;
    if (mcu) {
      setText("t-alt", mcu.alt.toFixed(2) + " m");
      setText("t-horiz", "–");
      setText("t-spd", mcu.speed.toFixed(2) + " m/s");
      setText("t-roll", (mcu.roll * 180 / Math.PI).toFixed(1) + "°");
      setText("t-pitch", (mcu.pitch * 180 / Math.PI).toFixed(1) + "°");
      setText("t-yaw", (mcu.yaw * 180 / Math.PI).toFixed(1) + "°");
    } else if (t.truth) {
      const tr = t.truth;
      setText("t-alt", tr.alt.toFixed(2) + " m");
      setText("t-horiz", tr.horiz.toFixed(2) + " m");
      setText("t-spd", tr.speed.toFixed(2) + " m/s");
      setText("t-roll", (tr.roll * 180 / Math.PI).toFixed(1) + "°");
      setText("t-pitch", (tr.pitch * 180 / Math.PI).toFixed(1) + "°");
      setText("t-yaw", (tr.yaw * 180 / Math.PI).toFixed(1) + "°");
    }
    // 真值对照（物理引擎）：仅数字，简短展示。
    if (t.truth) {
      const tr = t.truth;
      setText(
        "t-truth",
        `alt ${tr.alt.toFixed(1)} · R/P ${(tr.roll * 180 / Math.PI).toFixed(0)}/${(tr.pitch * 180 / Math.PI).toFixed(0)}°`
      );
    }
  }

  function drawMotors(m) {
    // 电机推力条
    const wrap = document.getElementById("motors");
    if (!wrap._bars) {
      wrap._bars = [];
      for (let i = 0; i < 4; i++) {
        const bar = document.createElement("div");
        bar.className = "motorbar";
        const fill = document.createElement("span");
        bar.appendChild(fill);
        const lbl = document.createElement("div");
        lbl.style.fontSize = "10px";
        lbl.style.color = "#8b949e";
        wrap.appendChild(bar);
        wrap.appendChild(lbl);
        wrap._bars.push({ fill, lbl });
      }
    }
    for (let i = 0; i < 4; i++) {
      const val = m ? m[i] : 0;
      const mm = val || 0;
      wrap._bars[i].fill.style.width = Math.max(0, Math.min(1, mm)) * 100 + "%";
      wrap._bars[i].fill.style.background = mm > 0.5 ? "var(--accent)" : "var(--warn)";
      wrap._bars[i].lbl.textContent = `M${i}: ${mm.toFixed(2)}`;
    }
  }

  function setText(id, v) { document.getElementById(id).textContent = v; }

  // ---- 控制指令发送 ----
  function sendControl() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const msg = {
      scenario: ctl.scenario,
      controller: ctl.controller,
      wind: ctl.wind,
      fail_motor: ctl.fail_motor,
      degrade: ctl.degrade,
      cam_yaw: cam.yaw,
      cam_pitch: cam.pitch,
      cam_distance: cam.distance,
    };
    ws.send(JSON.stringify(msg));
  }

  // ---- 面板绑定 ----
  const scenarioSel = document.getElementById("scenario");
  const controllerSel = document.getElementById("controller");
  const failSel = document.getElementById("fail");
  const degSlider = document.getElementById("deg");
  const degVal = document.getElementById("degv");
  const windSlider = document.getElementById("wind");
  const windVal = document.getElementById("windv");

  scenarioSel.onchange = () => {
    ctl.scenario = scenarioSel.value;
    setHilMode(ctl.scenario === "hil");
    sendControl();
  };
  controllerSel.onchange = () => { ctl.controller = controllerSel.value; sendControl(); };

  // HIL 真机模式：控制律/故障/风场由 MCU 接管，面板只读。
  function setHilMode(active) {
    controllerSel.disabled = active;
    failSel.disabled = active;
    degSlider.disabled = active;
    windSlider.disabled = active;
    const hint = document.querySelector(".hint");
    if (hint) {
      hint.textContent = active
        ? "HIL 真机：PC 物理引擎 ↔ USB-CDC 真实飞控闭环。\n" +
          "渲染=物理真值，遥测=MCU 估计（真值对照）。\n" +
          "先确认 MCU 已烧录 --features hil 固件并已连接 USB。"
        : "拖拽画面：旋转视角 · 滚轮：缩放\n" +
          "退化场景：先 4s 稳态再注入故障，发散后自动重演。\n" +
          "避障场景：橙色=障碍（球/盒），青色射线=测距（绿=有效命中），蓝箭头=风场。";
    }
  }

  failSel.onchange = () => {
    const v = parseInt(failSel.value, 10);
    if (v < 0) { ctl.fail_motor = null; }
    else { ctl.fail_motor = v; ctl.degrade = null; degSlider.value = "0.0"; degVal.textContent = "0.00"; }
    sendControl();
  };

  degSlider.oninput = () => {
    const e = parseFloat(degSlider.value);
    degVal.textContent = e.toFixed(2);
    if (e < 1.0) {
      ctl.degrade = [0, e];
      ctl.fail_motor = null;
      failSel.value = "-1";
    } else {
      ctl.degrade = null;
    }
    sendControl();
  };

  windSlider.oninput = () => {
    ctl.wind = parseFloat(windSlider.value);
    windVal.textContent = ctl.wind.toFixed(1);
    sendControl();
  };

  // 周期性把相机状态推给后端（拖拽/滚轮时）
  setInterval(() => sendControl(), 100);

  // ---- 相机交互 ----
  let dragging = false, lastX = 0, lastY = 0;
  canvas.addEventListener("mousedown", (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
  window.addEventListener("mouseup", () => { dragging = false; });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    const dx = e.clientX - lastX, dy = e.clientY - lastY;
    lastX = e.clientX; lastY = e.clientY;
    cam.yaw += dx * 0.01;
    cam.pitch = Math.max(-1.5, Math.min(1.5, cam.pitch + dy * 0.01));
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    cam.distance = Math.max(2, Math.min(200, cam.distance + (e.deltaY > 0 ? 2 : -2)));
  }, { passive: false });

  connect();
})();
