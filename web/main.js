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
    scenario: "hover",
    controller: "pid",
    wind: 0.0,
    fail_motor: null,   // 整数或 null
    degrade: null,      // [m, e] 或 null
  };

  let ws = null;
  let lastTele = 0;

  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
      statusEl.textContent = "已连接";
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

  // 解析二进制帧：前 8 字节 = (w,u32)(h,u32)，余下 = RGBA。
  function drawFrame(buf) {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    const w = dv.getUint32(0, true);
    const h = dv.getUint32(4, true);
    const px = buf.subarray(8, 8 + w * h * 4);
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
    setText("t-alt", t.alt.toFixed(2) + " m");
    setText("t-horiz", t.horiz.toFixed(2) + " m");
    setText("t-spd", t.speed.toFixed(2) + " m/s");
    setText("t-roll", (t.roll * 180 / Math.PI).toFixed(1) + "°");
    setText("t-pitch", (t.pitch * 180 / Math.PI).toFixed(1) + "°");
    setText("t-yaw", (t.yaw * 180 / Math.PI).toFixed(1) + "°");
    const st = document.getElementById("t-state");
    if (t.diverged) {
      st.innerHTML = '<span class="badge bad">发散</span>';
    } else {
      st.innerHTML = '<span class="badge ok">稳定</span>';
    }
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
      const m = t.m[i] || 0;
      wrap._bars[i].fill.style.width = Math.max(0, Math.min(1, m)) * 100 + "%";
      wrap._bars[i].fill.style.background = m > 0.5 ? "var(--accent)" : "var(--warn)";
      wrap._bars[i].lbl.textContent = `M${i}: ${m.toFixed(2)}`;
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

  scenarioSel.onchange = () => { ctl.scenario = scenarioSel.value; sendControl(); };
  controllerSel.onchange = () => { ctl.controller = controllerSel.value; sendControl(); };

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
