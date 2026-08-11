//! 阶段 7+：纯软件 3D 渲染模块（零 GPU 依赖）。
//!
//! 把四旋翼的渲染系位姿 + 电机/速度/失效数据渲染成一帧 `Vec<u32>` 像素
//! （`0xAARRGGBB`，小端内存即 B,G,R,A，可直接作为 canvas `ImageData` 的字节）。
//!
//! 该模块**不依赖任何窗口/事件系统**（无 winit/softbuffer），因此是单一渲染源：
//! - 原生窗口前端（`src/view.rs`）复用本模块 + softbuffer 上传；
//! - Web 后端（`fly-sim-server`）复用本模块 + 经 WebSocket 把像素推给浏览器 canvas。
//!
//! 复用物理引擎 `phy-demo` 的 `Camera`（轨道视角）与 `Framebuffer`（软件光栅化）。
//! NED 世界系 → 渲染系（右手 Y-up）固定轴映射：render = (N, -D, E)。

use phy_demo::{Camera, Framebuffer};
use phy_math::na::{Matrix4, Vector4};

/// 一帧渲染所需的全部输入（渲染系，已由调用方完成 NED→render 转换）。
pub struct RenderInput {
    /// 渲染系位置 (x=north, y=up, z=east)。
    pub pos: [f64; 3],
    /// 渲染系姿态 (w,x,y,z)，机体→渲染。
    pub quat: [f64; 4],
    /// 四路电机归一化推力 [0,1]。
    pub motors: [f64; 4],
    /// 渲染系速度 (x=north, y=up, z=east) m/s。
    pub vel: [f64; 3],
    /// 四路电机效率系数 [0,1]（1=正常，0=完全停转）。
    pub eff: [f32; 4],
    /// 历史渲染系位置（轨迹拖尾），为空则不画。
    pub trail: Vec<[f64; 3]>,
    /// 机体臂长（米，渲染系坐标尺度）。
    pub arm: f64,
    /// 视觉放大系数（机体在画面中的大小）。
    pub visual_scale: f32,
    /// 相机轨道角 yaw（绕 Y）。
    pub cam_yaw: f32,
    /// 相机俯仰 pitch。
    pub cam_pitch: f32,
    /// 相机距离。
    pub cam_distance: f32,
    /// 闪烁相位（完全停转电机高亮用）。
    pub blink: f64,
}

impl Default for RenderInput {
    fn default() -> Self {
        Self {
            pos: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0],
            motors: [0.0; 4],
            vel: [0.0; 3],
            eff: [1.0; 4],
            trail: Vec::new(),
            arm: 0.5,
            visual_scale: 2.5,
            cam_yaw: 0.6,
            cam_pitch: 0.45,
            cam_distance: 28.0,
            blink: 0.0,
        }
    }
}

// ---- NED → 渲染坐标 固定世界变换：绕 X 轴 +90° 主动旋转 ----
// NED (n, e, d) -> render (n, -d, e)
pub fn ned_to_render(pos_ned: [f64; 3]) -> [f64; 3] {
    [pos_ned[0], -pos_ned[2], pos_ned[1]]
}

/// Hamilton 四元数积（a∘b）。
fn ham_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// NED 姿态（机体→NED）转渲染系姿态（机体→render）：q_r = q_T * q_ned。
pub fn ned_quat_to_render(q_ned: flyctrl_core::vehicle::Quaternion) -> [f64; 4] {
    let q_t = [
        std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        0.0,
    ];
    let qn = [q_ned.w as f64, q_ned.x as f64, q_ned.y as f64, q_ned.z as f64];
    ham_mul(q_t, qn)
}

/// 单位四元数 → 列主序旋转矩阵（nalgebra `Matrix4::new` 按列填充）。
fn quat_to_mat4(q: [f64; 4]) -> Matrix4<f32> {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt().max(1e-9);
    let (w, x, y, z) = (q[0] / n, q[1] / n, q[2] / n, q[3] / n);
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - z * w);
    let r02 = 2.0 * (x * z + y * w);
    let r10 = 2.0 * (x * y + z * w);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - x * w);
    let r20 = 2.0 * (x * z - y * w);
    let r21 = 2.0 * (y * z + x * w);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    Matrix4::new(
        r00 as f32, r10 as f32, r20 as f32, 0.0,
        r01 as f32, r11 as f32, r21 as f32, 0.0,
        r02 as f32, r12 as f32, r22 as f32, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )
}

/// 模型空间点 → 屏幕像素（含深度 1/w）。近裁剪 + NDC 裁剪。
fn project(
    p_model: [f32; 3],
    vp: &Matrix4<f32>,
    model: &Matrix4<f32>,
    w: u32,
    h: u32,
) -> Option<(i32, i32, f32)> {
    let world = *model * Vector4::new(p_model[0], p_model[1], p_model[2], 1.0);
    let clip = *vp * world;
    if clip.w <= 1e-5 {
        return None;
    }
    let inv = 1.0 / clip.w;
    let ndc_x = clip.x * inv;
    let ndc_y = clip.y * inv;
    let ndc_z = clip.z * inv;
    if ndc_z < -1.0 || ndc_z > 1.0 {
        return None;
    }
    let sx = ((ndc_x * 0.5 + 0.5) * (w as f32 - 1.0)) as i32;
    let sy = ((1.0 - (ndc_y * 0.5 + 0.5)) * (h as f32 - 1.0)) as i32;
    Some((sx, sy, inv))
}

/// 画地面网格（render 系 y=0 平面，即 NED d=0 水平面）。
fn draw_ground(fb: &mut Framebuffer, vp: &Matrix4<f32>) {
    let model = Matrix4::<f32>::identity();
    let span = 10i32;
    for i in -span..=span {
        let a = [i as f32, 0.0, -span as f32];
        let b = [i as f32, 0.0, span as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, [38, 46, 58]);
        }
        let a = [-span as f32, 0.0, i as f32];
        let b = [span as f32, 0.0, i as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, [38, 46, 58]);
        }
    }
}

/// 画轨迹拖尾：历史渲染系位置连成渐隐线（越旧越暗）。
fn draw_trail(fb: &mut Framebuffer, vp: &Matrix4<f32>, trail: &[[f64; 3]], w: u32, h: u32) {
    let n = trail.len();
    if n < 2 {
        return;
    }
    let model = Matrix4::<f32>::identity();
    for i in 1..n {
        let a = [trail[i - 1][0] as f32, trail[i - 1][1] as f32, trail[i - 1][2] as f32];
        let b = [trail[i][0] as f32, trail[i][1] as f32, trail[i][2] as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, w, h),
            project(b, vp, &model, w, h),
        ) {
            let t = i as f32 / (n as f32 - 1.0);
            let g = (60.0 + 160.0 * t) as u8;
            let col = [40, g, 90u8.saturating_add((60.0 * t) as u8)];
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, col);
        }
    }
}

/// 画三维世界线段（model 空间两点 → 屏幕）。
fn draw_line_world(
    fb: &mut Framebuffer,
    vp: &Matrix4<f32>,
    model: &Matrix4<f32>,
    a: [f32; 3],
    b: [f32; 3],
    col: [u8; 3],
) {
    if let (Some(pa), Some(pb)) = (
        project(a, vp, model, fb.width, fb.height),
        project(b, vp, model, fb.width, fb.height),
    ) {
        fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, col);
    }
}

/// 画四旋翼：中心盒 + 4 臂线 + 旋翼盘 + 机体坐标轴 + 失效高亮 + 速度箭头。
fn draw_quad(fb: &mut Framebuffer, vp: &Matrix4<f32>, inp: &RenderInput, aspect: f32) {
    let _ = aspect; // 模型空间与宽高比无关；宽高比已体现在 vp。
    let t = Matrix4::new(
        1.0, 0.0, 0.0, inp.pos[0] as f32,
        0.0, 1.0, 0.0, inp.pos[1] as f32,
        0.0, 0.0, 1.0, inp.pos[2] as f32,
        0.0, 0.0, 0.0, 1.0,
    );
    let r = quat_to_mat4(inp.quat);
    let model = t * r;

    let arm = inp.arm as f32 * inp.visual_scale;
    let s = 0.18f32 * inp.visual_scale;

    let corners = [
        [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
        [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s],
    ];
    let edges = [
        [0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7],
    ];
    for e in edges {
        let (a, b) = (corners[e[0]], corners[e[1]]);
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, [120, 140, 180]);
        }
    }

    let rotor = [
        [-arm, -arm, 0.0],
        [arm, arm, 0.0],
        [arm, -arm, 0.0],
        [-arm, arm, 0.0],
    ];
    let center = project([0.0, 0.0, 0.0], vp, &model, fb.width, fb.height);
    for i in 0..4 {
        let rr = rotor[i];
        let pr = project(rr, vp, &model, fb.width, fb.height);
        if let (Some(pc), Some(pr)) = (center, pr) {
            fb.draw_line(pc.0, pc.1, pr.0, pr.1, (pc.2 + pr.2) * 0.5, [80, 90, 110]);
            let m = inp.motors[i] as f32;
            let rad = (2.0 + m * 6.0).max(1.0) as i32;
            let eff = inp.eff[i];
            let col = if eff <= 0.02 {
                let on = (inp.blink.sin() * 0.5 + 0.5) > 0.5;
                if on { [230, 50, 50] } else { [120, 30, 30] }
            } else if eff < 0.98 {
                [230, 150, 40]
            } else if m > 0.05 {
                [70, 200, 120]
            } else {
                [110, 110, 110]
            };
            fb.fill_circle(pr.0, pr.1, rad, pr.2, col);
        }
    }

    let axes = [
        ([0.6, 0.0, 0.0], [220, 70, 70]),
        ([0.0, 0.6, 0.0], [70, 220, 70]),
        ([0.0, 0.0, 0.6], [70, 70, 220]),
    ];
    if let Some(pc) = center {
        for (ax, col) in axes {
            let pa = project(
                [ax[0] * inp.visual_scale, ax[1] * inp.visual_scale, ax[2] * inp.visual_scale],
                vp,
                &model,
                fb.width,
                fb.height,
            );
            if let Some(pa) = pa {
                fb.draw_line(pc.0, pc.1, pa.0, pa.1, (pc.2 + pa.2) * 0.5, col);
            }
        }
    }

    // 速度矢量箭头
    if let Some(pc) = center {
        let v = inp.vel;
        let speed = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if speed > 0.05 {
            let len = (speed * 0.6).min(3.0) as f32;
            let dir = [v[0] as f32 / speed as f32, v[1] as f32 / speed as f32, v[2] as f32 / speed as f32];
            let tip = [dir[0] * len, dir[1] * len, dir[2] * len];
            draw_line_world(fb, vp, &model, [0.0, 0.0, 0.0], tip, [40, 200, 220]);
            let up = [0.0f32, 1.0, 0.0];
            let n = [
                dir[1] * up[2] - dir[2] * up[1],
                dir[2] * up[0] - dir[0] * up[2],
                dir[0] * up[1] - dir[1] * up[0],
            ];
            let nl = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-6);
            let n = [n[0] / nl, n[1] / nl, n[2] / nl];
            let back = 0.3f32 * len;
            let wing = 0.18f32 * len;
            let base = [tip[0] - dir[0] * back, tip[1] - dir[1] * back, tip[2] - dir[2] * back];
            draw_line_world(fb, vp, &model, tip, [base[0] + n[0] * wing, base[1] + n[1] * wing, base[2] + n[2] * wing], [40, 200, 220]);
            draw_line_world(fb, vp, &model, tip, [base[0] - n[0] * wing, base[1] - n[1] * wing, base[2] - n[2] * wing], [40, 200, 220]);
        }
    }
}

/// 渲染一帧：返回 `width*height` 个 `0xAARRGGBB` 像素（`Vec<u32>`）。
/// 像素内存布局为小端 B,G,R,A，可直接作为 canvas `ImageData` 的字节源。
pub fn render_frame(width: u32, height: u32, inp: &RenderInput) -> Vec<u32> {
    let mut fb = Framebuffer::new(width, height);
    fb.clear();
    let mut cam = Camera::default();
    cam.yaw = inp.cam_yaw;
    cam.pitch = inp.cam_pitch;
    cam.distance = inp.cam_distance;
    cam.target = phy_math::na::Point3::new(inp.pos[0] as f32, inp.pos[1] as f32, inp.pos[2] as f32);
    let aspect = width as f32 / height as f32;
    let vp = cam.view_proj(aspect);

    draw_ground(&mut fb, &vp);
    draw_trail(&mut fb, &vp, &inp.trail, width, height);
    draw_quad(&mut fb, &vp, inp, aspect);

    fb.pixels
}
