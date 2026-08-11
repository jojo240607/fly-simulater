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

use phy_demo::raster::pack;
use phy_demo::{Camera, Framebuffer};
use phy_math::na::{Matrix4, Vector4};
use std::sync::OnceLock;

/// 一帧渲染所需的全部输入（渲染世界 = 物理引擎世界系，右手 Y-up，上=+Y）。
///
/// 注意：**渲染直接使用引擎位姿，不做任何坐标/四元数变换**。调用方应直接填
/// `SimLoop::debug_up()` 返回的引擎世界坐标与四元数，保证悬停时机体水平。
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

// ---- 渲染世界系约定 ----
// 渲染世界 = 物理引擎世界系（同为 Y-up，上=+Y）。因此**渲染直接使用引擎位姿**，
// 不做任何坐标/四元数变换：引擎悬停时机体 +Z（推力轴）指向 +Y，旋翼盘水平，
// 渲染即水平。任何 NED 或 z 镜像变换都会破坏"上"方向，把水平机体翻成侧躺。
//
// 以下 NED 转换保留给原生窗口路径（`view.rs` 早期实现），Web 端不再使用。
// NED (n, e, d) -> render (n, -d, e)，对应 q_T = rotX(+90°)。
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
    // 网格线用柔和低对比色（比地面略亮一点但不抢眼）。
    let grid_col = [50, 59, 72];
    let span = 10i32;
    for i in -span..=span {
        let a = [i as f32, 0.0, -span as f32];
        let b = [i as f32, 0.0, span as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, grid_col);
        }
        let a = [-span as f32, 0.0, i as f32];
        let b = [span as f32, 0.0, i as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, grid_col);
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

// ================= 真实纹理贴图（软件 UV 光栅化） =================

/// 位图纹理：`data` 为 `0xAARRGGBB` 像素（与帧缓冲同格式）。
#[derive(Clone)]
pub struct Texture {
    pub w: u32,
    pub h: u32,
    pub data: Vec<u32>,
}

impl Texture {
    pub fn new(w: u32, h: u32, data: Vec<u32>) -> Self {
        Self { w, h, data }
    }
    /// 最近邻采样，uv ∈ [0,1]。
    fn sample(&self, u: f32, v: f32) -> [u8; 3] {
        let x = ((u * self.w as f32).clamp(0.0, self.w as f32 - 1.0)) as u32;
        let y = ((v * self.h as f32).clamp(0.0, self.h as f32 - 1.0)) as u32;
        let px = self.data[(y * self.w + x) as usize];
        let (r, g, b) = ((px >> 16) & 0xFF, (px >> 8) & 0xFF, px & 0xFF);
        [r as u8, g as u8, b as u8]
    }
}

/// 一个带 UV 的三角形（模型空间）。
struct TexTri {
    pos: [[f32; 3]; 3],
    uv: [[f32; 2]; 3],
}

/// 纹理三角形光栅化：透视校正的 UV 插值 + 纹理采样，复用 Framebuffer 的深度测试。
/// `depth`（1/w，越小越近）与 `Framebuffer::set_depth` 语义一致。
fn draw_textured_tri(
    fb: &mut Framebuffer,
    vp: &Matrix4<f32>,
    model: &Matrix4<f32>,
    tri: &TexTri,
    tex: &Texture,
) {
    // 顶点变换 → 屏幕坐标 + 1/w（深度）。
    let mut sp = [[0.0f32; 2]; 3];
    let mut invw = [0.0f32; 3];
    for i in 0..3 {
        match project(tri.pos[i], vp, model, fb.width, fb.height) {
            Some((sx, sy, iw)) => {
                sp[i] = [sx as f32, sy as f32];
                invw[i] = iw;
            }
            None => return,
        }
    }
    let minx = sp.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min).floor().max(0.0) as i32;
    let maxx = sp.iter().map(|p| p[0]).fold(f32::NEG_INFINITY, f32::max).ceil().min(fb.width as f32) as i32;
    let miny = sp.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min).floor().max(0.0) as i32;
    let maxy = sp.iter().map(|p| p[1]).fold(f32::NEG_INFINITY, f32::max).ceil().min(fb.height as f32) as i32;

    let area = edge2(sp[0], sp[1], sp[2]);
    if area.abs() < 1e-6 {
        return;
    }
    // 透视校正：每个像素插值 (uv/w, 1/w)，再 uv = (uv/w)/(1/w)。
    let uv0w = [tri.uv[0][0] * invw[0], tri.uv[0][1] * invw[0]];
    let uv1w = [tri.uv[1][0] * invw[1], tri.uv[1][1] * invw[1]];
    let uv2w = [tri.uv[2][0] * invw[2], tri.uv[2][1] * invw[2]];

    for y in miny..=maxy {
        for x in minx..=maxx {
            let px = x as f32 + 0.5;
            let py = y as f32 + 0.5;
            let w0 = edge2(sp[1], sp[2], [px, py]) / area;
            let w1 = edge2(sp[2], sp[0], [px, py]) / area;
            let w2 = edge2(sp[0], sp[1], [px, py]) / area;
            if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                continue;
            }
            let iw = w0 * invw[0] + w1 * invw[1] + w2 * invw[2];
            if iw <= 1e-6 {
                continue;
            }
            let u = (w0 * uv0w[0] + w1 * uv1w[0] + w2 * uv2w[0]) / iw;
            let v = (w0 * uv0w[1] + w1 * uv1w[1] + w2 * uv2w[1]) / iw;
            let col = tex.sample(u, v);
            fb.set_depth(x, y, iw, col);
        }
    }
}

/// 画一个贴纹理的四边形（两个三角形）。
fn draw_textured_quad(
    fb: &mut Framebuffer,
    vp: &Matrix4<f32>,
    model: &Matrix4<f32>,
    corners: [[f32; 3]; 4],
    uvs: [[f32; 2]; 4],
    tex: &Texture,
) {
    // 三角1: 0,1,2；三角2: 0,2,3（逆时针保证正面）。
    let t1 = TexTri {
        pos: [corners[0], corners[1], corners[2]],
        uv: [uvs[0], uvs[1], uvs[2]],
    };
    let t2 = TexTri {
        pos: [corners[0], corners[2], corners[3]],
        uv: [uvs[0], uvs[2], uvs[3]],
    };
    draw_textured_tri(fb, vp, model, &t1, tex);
    draw_textured_tri(fb, vp, model, &t2, tex);
}

/// 2D 边函数（有符号面积，判断点在三角形内/插值权重）。
fn edge2(a: [f32; 2], b: [f32; 2], c: [f32; 2]) -> f32 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// 画屏幕空间圆环（Bresenham 式逐点描边），带深度。
fn draw_circle_outline(fb: &mut Framebuffer, cx: i32, cy: i32, r: i32, depth: f32, color: [u8; 3]) {
    let r = r.max(1);
    let mut x = 0;
    let mut y = r;
    let mut d = 3 - 2 * r;
    while x <= y {
        let pts = [
            (cx + x, cy + y), (cx - x, cy + y), (cx + x, cy - y), (cx - x, cy - y),
            (cx + y, cy + x), (cx - y, cy + x), (cx + y, cy - x), (cx - y, cy - x),
        ];
        for (px, py) in pts {
            if px >= 0 && py >= 0 && (px as u32) < fb.width && (py as u32) < fb.height {
                fb.set_depth(px, py, depth, color);
            }
        }
        if d < 0 {
            d += 4 * x + 6;
        } else {
            d += 4 * (x - y) + 10;
            y -= 1;
        }
        x += 1;
    }
}

/// 程序化生成一张"碳纤维 + 警示条"机身纹理（避免依赖外部图片，后端离线可用）。
/// 用伪随机碳纤维纹路 + 中心警示色块。
pub fn make_fuselage_texture() -> Texture {
    let (w, h) = (64u32, 64u32);
    let mut data = vec![0u32; (w * h) as usize];
    let mut rng: u64 = 0x9E3779B97F4A7C15;
    let cy = h as f32 * 0.5;
    for y in 0..h {
        for x in 0..w {
            let fx = x as f32;
            let fy = y as f32;
            rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
            let n = (rng & 0xFF) as f32 / 255.0;
            // 碳纤维斜纹编织：交错 45° 亮/暗细线（更真实的编织观感）
            let diag1 = ((fx + fy) as i32) % 6 == 0;
            let diag2 = ((fx - fy) as i32) % 6 == 0;
            let mut r = 34.0 + n * 10.0;
            let mut g = 40.0 + n * 10.0;
            let mut b = 46.0 + n * 10.0;
            if diag1 || diag2 {
                r += 14.0; g += 15.0; b += 16.0;
            }
            // 边缘黑边（机身边框）
            let edge = fx < 3.0 || fx > w as f32 - 3.0 || fy < 3.0 || fy > h as f32 - 3.0;
            // 中央横向红色装饰条（DJI 风格红色机头带），横向中带
            let band = (fy - cy).abs() < 5.0;
            if band {
                r = 200.0; g = 48.0; b = 42.0;
            }
            // 圆形 Logo 白点在右上
            let logo = ((fx - w as f32 * 0.78).powi(2) + (fy - h as f32 * 0.30).powi(2)).sqrt() < 4.0;
            if logo {
                r = 240.0; g = 240.0; b = 245.0;
            }
            if edge {
                r = 16.0; g = 18.0; b = 22.0;
            }
            let c = ((0xFFu32) << 24) | ((r as u32 & 0xFF) << 16) | ((g as u32 & 0xFF) << 8) | (b as u32 & 0xFF);
            data[(y * w + x) as usize] = c;
        }
    }
    Texture::new(w, h, data)
}

/// 程序化生成一张"螺旋桨叶片"纹理：径向渐变 + 3 片桨叶扇区，模拟高速旋转。
/// 惰性获取机身纹理（线程安全，仅生成一次）。
fn fuselage_tex() -> &'static Texture {
    static T: OnceLock<Texture> = OnceLock::new();
    T.get_or_init(make_fuselage_texture)
}

/// 画四旋翼：中心盒 + 4 臂线 + 旋翼盘 + 失效高亮 + 速度箭头。
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
    let center = project([0.0, 0.0, 0.0], vp, &model, fb.width, fb.height);

    // ---- 机架：中心扁平机身（贴纹理的菱形平板）+ 4 臂 ----
    // 机身平板（X-Y 平面，z≈0）：贴碳纤维+警示条纹理。
    let body = [
        [arm * 0.45, 0.0, 0.0],   // 前
        [0.0, arm * 0.45, 0.0],   // 右
        [-arm * 0.45, 0.0, 0.0],  // 后
        [0.0, -arm * 0.45, 0.0],  // 左
    ];
    let body_uvs = [
        [0.5, 0.0], [1.0, 0.5], [0.5, 1.0], [0.0, 0.5],
    ];
    draw_textured_quad(fb, vp, &model, body, body_uvs, fuselage_tex());

    // 机头标记（前）红色小圆点
    if let Some(pc) = center {
        if let Some(pn) = project([arm * 0.45, 0.0, 0.0], vp, &model, fb.width, fb.height) {
            fb.fill_circle(pn.0, pn.1, 3, pn.2, [230, 80, 70]);
        }
        let _ = pc;
    }

    // 4 臂 + 电机座 + 旋翼盘 + 螺旋桨叶
    let rotor = [
        [-arm, -arm, 0.0],
        [arm, arm, 0.0],
        [arm, -arm, 0.0],
        [-arm, arm, 0.0],
    ];
    for i in 0..4 {
        let rr = rotor[i];
        let pr = project(rr, vp, &model, fb.width, fb.height);
        if let (Some(pc), Some(pr)) = (center, pr) {
            // 臂线
            fb.draw_line(pc.0, pc.1, pr.0, pr.1, (pc.2 + pr.2) * 0.5, [96, 108, 128]);
            // 旋翼盘：高速旋转轨迹画成"圈圈"（淡色实心圆盘 + 外圈），随推力大小缩放。
            let m = inp.motors[i] as f32;
            let r = arm * 0.34 * (0.6 + m * 0.5); // 转速越高盘越明显
            let rad = (r * inp.visual_scale) as i32;
            let spin_col = if m > 0.05 { [140, 170, 190] } else { [70, 80, 95] };
            // 淡色圆盘（旋转圈）+ 外圈亮边（更明显的旋转轨迹边界）
            if rad > 2 {
                fb.fill_circle(pr.0, pr.1, rad, pr.2, spin_col);
                draw_circle_outline(fb, pr.0, pr.1, rad, pr.2, [190, 215, 230]);
            } else {
                fb.fill_circle(pr.0, pr.1, rad, pr.2, spin_col);
            }
            // 电机座 + 失效/退化高亮
            let eff = inp.eff[i];
            if eff <= 0.02 {
                let on = (inp.blink.sin() * 0.5 + 0.5) > 0.5;
                fb.fill_circle(pr.0, pr.1, 5, pr.2, if on { [230, 50, 50] } else { [120, 30, 30] });
            } else if eff < 0.98 {
                fb.fill_circle(pr.0, pr.1, 5, pr.2, [230, 150, 40]);
            } else {
                fb.fill_circle(pr.0, pr.1, 2, pr.2, [60, 66, 76]);
            }
        }
    }

    // 机体朝向指示由屏幕角落的 draw_compass（北/上/东）提供，不再在机体上画长坐标轴。

    // 速度矢量箭头
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

/// 天空渐变背景：垂直三色带（顶=天空蓝，中=地平线淡蓝，底=地面暗色）。
/// 逐行填充像素，取代 `clear()` 的纯深蓝。
fn draw_sky(fb: &mut Framebuffer) {
    let h = fb.height;
    let horizon = (h as f64 * 0.58) as i32; // 地平线大致位置
    let ground = (h as f64 * 0.90) as i32;
    for y in 0..h {
        // 0..horizon 天空渐变，horizon..ground 地面渐变
        let c = if y < horizon as u32 {
            let t = y as f64 / horizon.max(1) as f64;
            let sky_top = [82.0, 132.0, 210.0]; // 天蓝
            let sky_hor = [190.0, 210.0, 235.0]; // 地平线淡蓝
            lerp3(sky_top, sky_hor, t)
        } else if y < ground as u32 {
            let t = (y as f64 - horizon as f64) / (ground - horizon).max(1) as f64;
            let land_hor = [120.0, 130.0, 150.0];
            let land_dark = [28.0, 34.0, 44.0];
            lerp3(land_hor, land_dark, t)
        } else {
            [24.0, 28.0, 36.0]
        };
        let col = pack(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0);
        let base = (y as usize) * fb.width as usize;
        for x in 0..fb.width as usize {
            fb.pixels[base + x] = col;
        }
    }
}

fn lerp3(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// 地面参考：在原点 (0,0,0) 画醒目的十字标（红），便于观察机体相对起飞点位置。
fn draw_ground_marker(fb: &mut Framebuffer, vp: &Matrix4<f32>, w: u32, h: u32) {
    let model = Matrix4::<f32>::identity();
    let m = 0.6f32; // 十字半长（米）
    // 沿 north(x) 和 east(z) 两条线，y=0 平面。
    draw_line_world(fb, vp, &model, [-m, 0.0, 0.0], [m, 0.0, 0.0], [230, 80, 70]);
    draw_line_world(fb, vp, &model, [0.0, 0.0, -m], [0.0, 0.0, m], [230, 80, 70]);
    // 中心圆点
    if let Some(pc) = project([0.0, 0.0, 0.0], vp, &model, w, h) {
        fb.fill_circle(pc.0, pc.1, 4, pc.2, [235, 220, 90]);
    }
}

/// 朝向图例：屏幕左下角画三个固定的小色点 + 短线段（北N/上U/东E），
/// **完全脱离世界投影**，作为纯屏幕图例，不会延伸到场景深处。
fn draw_compass(fb: &mut Framebuffer, _vp: &Matrix4<f32>, _w: u32, h: u32) {
    let x = 24i32;
    let y0 = (h as i32) - 40;
    let dirs: [(&str, [u8; 3]); 3] = [
        ("N", [230, 90, 90]),
        ("U", [90, 230, 90]),
        ("E", [90, 130, 230]),
    ];
    for (i, (_label, col)) in dirs.iter().enumerate() {
        let y = y0 + i as i32 * 14;
        fb.draw_line(x, y, x + 14, y, 0.0, *col);
        fb.fill_circle(x, y, 2, 0.0, *col);
    }
}

/// 渲染一帧：返回 `width*height` 个 `0xAARRGGBB` 像素（`Vec<u32>`）。
/// 像素内存布局为小端 B,G,R,A，可直接作为 canvas `ImageData` 的字节源。
pub fn render_frame(width: u32, height: u32, inp: &RenderInput) -> Vec<u32> {
    let mut fb = Framebuffer::new(width, height);
    // 环境：天空渐变背景（取代纯色 clear）。
    draw_sky(&mut fb);
    let mut cam = Camera::default();
    cam.yaw = inp.cam_yaw;
    cam.pitch = inp.cam_pitch;
    cam.distance = inp.cam_distance;
    cam.target = phy_math::na::Point3::new(inp.pos[0] as f32, inp.pos[1] as f32, inp.pos[2] as f32);
    let aspect = width as f32 / height as f32;
    let vp = cam.view_proj(aspect);

    draw_ground(&mut fb, &vp);
    draw_ground_marker(&mut fb, &vp, width, height);
    draw_trail(&mut fb, &vp, &inp.trail, width, height);
    draw_quad(&mut fb, &vp, inp, aspect);
    draw_compass(&mut fb, &vp, width, height);

    fb.pixels
}
