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

/// 画一个实心四边形（世界/model 空间 4 顶点 → 屏幕，逐像素填充）。
/// 用于画立体机架（机身平板/电机座/螺旋桨叶），带 Z 深度测试。
fn fill_quad_world(
    fb: &mut Framebuffer,
    vp: &Matrix4<f32>,
    model: &Matrix4<f32>,
    corners: [[f32; 3]; 4],
    color: [u8; 3],
) {
    let mut proj = Vec::with_capacity(4);
    for c in corners {
        if let Some(p) = project(c, vp, model, fb.width, fb.height) {
            proj.push(p);
        } else {
            return; // 任一顶点被裁剪则整体跳过（简化）
        }
    }
    if proj.len() < 4 {
        return;
    }
    let xs: Vec<i32> = proj.iter().map(|p| p.0).collect();
    let ys: Vec<i32> = proj.iter().map(|p| p.1).collect();
    let minx = *xs.iter().min().unwrap();
    let maxx = *xs.iter().max().unwrap();
    let miny = *ys.iter().min().unwrap();
    let maxy = *ys.iter().max().unwrap();
    // 深度取四角平均（简化，够用）。
    let depth = (proj[0].2 + proj[1].2 + proj[2].2 + proj[3].2) * 0.25;
    for y in miny..=maxy {
        for x in minx..=maxx {
            if point_in_quad(x, y, &proj) {
                fb.set_depth(x, y, depth, color);
            }
        }
    }
}

/// 判断屏幕点 (x,y) 是否在凸四边形（4 个屏幕投影点）内（含边）。
fn point_in_quad(x: i32, y: i32, quad: &[(i32, i32, f32)]) -> bool {
    let mut sign = None;
    for i in 0..4 {
        let a = (quad[i].0, quad[i].1);
        let b = (quad[(i + 1) % 4].0, quad[(i + 1) % 4].1);
        let cross = (b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0);
        let s = if cross > 0 { 1 } else if cross < 0 { -1 } else { 0 };
        if s == 0 {
            continue;
        }
        match sign {
            None => sign = Some(s),
            Some(prev) if prev != s => return false,
            _ => {}
        }
    }
    true
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
    let s = 0.20f32 * inp.visual_scale; // 机身平板半宽
    let center = project([0.0, 0.0, 0.0], vp, &model, fb.width, fb.height);

    // ---- 机架：中心扁平机身（实心菱形平板）+ 4 臂 ----
    // 机身平板（X-Y 平面，z≈0），做一个圆角菱形机身：沿臂对角线方向拉长。
    let body = [
        [arm * 0.42, 0.0, 0.0],   // 前
        [0.0, arm * 0.42, 0.0],   // 右
        [-arm * 0.42, 0.0, 0.0],  // 后
        [0.0, -arm * 0.42, 0.0],  // 左
    ];
    fill_quad_world(fb, vp, &model, body, [150, 175, 210]); // 机身淡蓝
    // 机身下沿加一点厚度感（z 偏移）
    let body_low = [
        [arm * 0.42, 0.0, -0.05 * inp.visual_scale],
        [0.0, arm * 0.42, -0.05 * inp.visual_scale],
        [-arm * 0.42, 0.0, -0.05 * inp.visual_scale],
        [0.0, -arm * 0.42, -0.05 * inp.visual_scale],
    ];
    fill_quad_world(fb, vp, &model, body_low, [110, 130, 165]);

    // 机头标记（前）红色小圆点
    if let Some(pc) = center {
        if let Some(pn) = project([arm * 0.42, 0.0, 0.0], vp, &model, fb.width, fb.height) {
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
            let m = inp.motors[i] as f32;
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
            // 电机座（小实心圆，暗色）
            fb.fill_circle(pr.0, pr.1, 2, pr.2, [60, 66, 76]);
            // 旋翼盘：十字叶片（随相角转动，模拟高速旋转）+ 盘心
            let blade = (3.0 + m * 8.0) as i32;
            let ang = inp.blink * 3.0 + i as f64 * 1.5708;
            let cxx = (ang.cos() * blade as f64) as i32;
            let cyy = (ang.sin() * blade as f64) as i32;
            fb.draw_line(pr.0 - cxx, pr.1 - cyy, pr.0 + cxx, pr.1 + cyy, pr.2, col);
            fb.draw_line(pr.0 + cyy, pr.1 - cxx, pr.0 - cyy, pr.1 + cxx, pr.2, col);
            fb.fill_circle(pr.0, pr.1, 2, pr.2, col);
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

/// 朝向指示器：屏幕左下角固定锚点，画世界系北(红)/上(绿)/东(蓝) 三条方向线。
/// 方向来自世界原点沿各轴延伸在相机投影下的屏幕位置，随相机旋转真实反映世界朝向。
fn draw_compass(fb: &mut Framebuffer, vp: &Matrix4<f32>, w: u32, h: u32) {
    let model = Matrix4::<f32>::identity();
    // 屏幕锚点（左下角）。
    let ax = 70i32;
    let ay = (h as i32) - 60;
    // 世界原点沿各轴 0.5m 的点投影。
    let north = project([0.5, 0.0, 0.0], vp, &model, w, h);
    let up = project([0.0, 0.5, 0.0], vp, &model, w, h);
    let east = project([0.0, 0.0, 0.5], vp, &model, w, h);
    let dirs: [(&str, Option<(i32, i32, f32)>, [u8; 3]); 3] = [
        ("N", north, [230, 90, 90]),
        ("U", up, [90, 230, 90]),
        ("E", east, [90, 130, 230]),
    ];
    for (label, proj, col) in dirs {
        if let Some((px, py, _)) = proj {
            // 从锚点画到投影点
            fb.draw_line(ax, ay, px, py, 0.0, col);
            // 端点小圆
            fb.fill_circle(px, py, 2, 0.0, col);
            // 标签在锚点处简单标注（用短线区分三色即可，文字由前端可加）
            let _ = label;
        }
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
