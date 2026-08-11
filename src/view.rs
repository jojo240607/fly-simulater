//! 阶段 7：实时 3D 可视化前端。
//!
//! 架构：**后台仿真线程全速推进** + **渲染线程按显示帧率采样最新位姿**。
//! - 复用物理引擎 `phy-demo` 的 `Camera`（轨道视角）与 `Framebuffer`（软件光栅化）原语；
//! - 四旋翼以线框 + 旋翼盘绘制（机体坐标：前+X / 右+Y / 下+Z）；
//! - NED 世界系 → 渲染系（右手 Y-up）的固定轴映射：render = (N, -D, E)。
//!
//! 物理引擎自身不带"通用渲染器"——`phy-demo` 的 `Scene` 与它的 `World` 绑定，无法直接
//! 渲染我们的四旋翼世界。因此我们复用其相机/光栅化数学，自己把 `PhySdkWorld` 的机体
//! 真值投影成线框，核心仿真逻辑零改动。

use std::sync::Arc;
use std::sync::Mutex;

use phy_demo::{Camera, Framebuffer};
use phy_math::na::{Matrix4, Point3, Vector4};

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::PhySdkWorld;
use fly_sim_core::sensor;
use fly_sim_core::sim;
use fly_sim_core::wind::WindField;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{ActuatorCmd, Quaternion, VehicleState};

/// 渲染共享状态：已转换到渲染坐标系（右手 Y-up）的机体位姿 + 电机指令。
pub struct RenderState {
    /// 渲染系位置 (x=north, y=up, z=east)。
    pub pos: [f64; 3],
    /// 渲染系姿态 (w,x,y,z)，机体→渲染。
    pub quat: [f64; 4],
    /// 四路电机归一化推力 [0,1]，用于旋翼盘视觉。
    pub motors: [f64; 4],
}

// ---- NED → 渲染坐标 固定世界变换：绕 X 轴 +90° 主动旋转 ----
// NED (n, e, d) -> render (n, -d, e)
// 对应四元数 q_T = rotX(+90°) = (√2/2, √2/2, 0, 0)
fn ned_to_render(pos_ned: [f64; 3]) -> [f64; 3] {
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
fn ned_quat_to_render(q_ned: Quaternion) -> [f64; 4] {
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
    // 标准旋转矩阵（行主序视图）：
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - z * w);
    let r02 = 2.0 * (x * z + y * w);
    let r10 = 2.0 * (x * y + z * w);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - x * w);
    let r20 = 2.0 * (x * z - y * w);
    let r21 = 2.0 * (y * z + x * w);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    // 转列主序：new(col0, col1, col2, col3)。nalgebra Matrix4<f32> 需 f32 分量。
    Matrix4::new(
        r00 as f32, r10 as f32, r20 as f32, 0.0, // col0
        r01 as f32, r11 as f32, r21 as f32, 0.0, // col1
        r02 as f32, r12 as f32, r22 as f32, 0.0, // col2
        0.0, 0.0, 0.0, 1.0, // col3
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
        // 沿 east(z) 的线，位于 north=i
        let a = [i as f32, 0.0, -span as f32];
        let b = [i as f32, 0.0, span as f32];
        if let (Some(pa), Some(pb)) = (
            project(a, vp, &model, fb.width, fb.height),
            project(b, vp, &model, fb.width, fb.height),
        ) {
            fb.draw_line(pa.0, pa.1, pb.0, pb.1, (pa.2 + pb.2) * 0.5, [38, 46, 58]);
        }
        // 沿 north(x) 的线，位于 east=i
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

/// 画四旋翼：中心盒 + 4 臂线 + 旋翼盘 + 机体坐标轴。
fn draw_quad(
    fb: &mut Framebuffer,
    vp: &Matrix4<f32>,
    rs: &RenderState,
    arm: f64,
    visual_scale: f32,
) {
    // 模型矩阵 = 平移(render pos) * 旋转(render 姿态)
    let t = Matrix4::new(
        1.0, 0.0, 0.0, rs.pos[0] as f32,
        0.0, 1.0, 0.0, rs.pos[1] as f32,
        0.0, 0.0, 1.0, rs.pos[2] as f32,
        0.0, 0.0, 0.0, 1.0,
    );
    let r = quat_to_mat4(rs.quat);
    let model = t * r;

    let arm = arm as f32 * visual_scale;
    let s = 0.18f32 * visual_scale; // 中心盒半边长

    // 中心盒 8 角 + 12 棱
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

    // 4 旋翼（X 型四象限）+ 臂线 + 旋翼盘
    let rotor = [
        [-arm, -arm, 0.0], // 后右
        [arm, arm, 0.0],   // 前左
        [arm, -arm, 0.0],  // 前右
        [-arm, arm, 0.0],  // 后左
    ];
    let center = project([0.0, 0.0, 0.0], vp, &model, fb.width, fb.height);
    for i in 0..4 {
        let rr = rotor[i];
        let pr = project(rr, vp, &model, fb.width, fb.height);
        if let (Some(pc), Some(pr)) = (center, pr) {
            fb.draw_line(pc.0, pc.1, pr.0, pr.1, (pc.2 + pr.2) * 0.5, [80, 90, 110]);
            let m = rs.motors[i] as f32;
            let rad = (2.0 + m * 6.0).max(1.0) as i32;
            let col = if m > 0.05 { [70, 200, 120] } else { [110, 110, 110] };
            fb.fill_circle(pr.0, pr.1, rad, pr.2, col);
        }
    }

    // 机体坐标轴：X 前(红) / Y 右(绿) / Z 下(蓝)
    let axes = [
        ([0.6, 0.0, 0.0], [220, 70, 70]),
        ([0.0, 0.6, 0.0], [70, 220, 70]),
        ([0.0, 0.0, 0.6], [70, 70, 220]),
    ];
    if let Some(pc) = center {
        for (ax, col) in axes {
            let pa = project(
                [ax[0] * visual_scale, ax[1] * visual_scale, ax[2] * visual_scale],
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
}

/// 入口：启动后台仿真 + 渲染窗口。
pub fn run_view(
    cfg: &VehicleConfig,
    dt: f64,
    wind: Option<WindField>,
    sensor_cfg: sensor::SensorConfig,
    kind: ControllerKind,
    scenario: &str,
    eff_mask: [f32; 4],
) {
    let state = Arc::new(Mutex::new(RenderState {
        pos: [0.0, 0.0, 0.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        motors: [0.0; 4],
    }));
    // 后台仿真线程：全速推进整个场景，逐物理步把真值推入共享状态。
    let sim_state = state.clone();
    let cfg2 = cfg.clone();
    let scenario_owned = scenario.to_string();
    std::thread::spawn(move || {
        let mut loop_sim = sim::SimLoop::new(
            PhySdkWorld::create_empty(),
            &cfg2,
            dt,
            wind,
            sensor_cfg,
            kind,
        );
        loop_sim.set_motor_eff(eff_mask);
        loop_sim.set_on_frame(Box::new(move |st: &VehicleState, cmd: ActuatorCmd| {
            let mut g = sim_state.lock().unwrap();
            g.pos = ned_to_render([
                st.pos[0].0 as f64,
                st.pos[1].0 as f64,
                st.pos[2].0 as f64,
            ]);
            g.quat = ned_quat_to_render(st.att);
            for i in 0..4 {
                g.motors[i] = cmd.motor[i] as f64;
            }
        }));
        let ok = match scenario_owned.as_str() {
            "wind" => loop_sim.run_hover_wind(15.0),
            _ => loop_sim.run_hover(10.0),
        };
        println!("[view] 仿真线程结束，结果 = {}", if ok { "PASS" } else { "FAIL" });
    });

    // 主线程：渲染窗口（winit + softbuffer），按显示帧率采样共享状态。
    run_window(state, cfg.arm_length as f64);
}

/// 渲染窗口主循环（winit 0.30 `ApplicationHandler` 标准写法，无 deprecated API）。
fn run_window(state: Arc<Mutex<RenderState>>, arm: f64) {
    use std::num::NonZeroU32;
    use winit::application::ApplicationHandler;
    use winit::dpi::LogicalSize;
    use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
    use winit::event_loop::{ActiveEventLoop, EventLoop};
    use winit::window::{Window, WindowId};

    struct App {
        state: Arc<Mutex<RenderState>>,
        arm: f64,
        window: Option<&'static Window>,
        ctx: Option<softbuffer::Context<&'static Window>>,
        surface: Option<softbuffer::Surface<&'static Window, &'static Window>>,
        cam: Camera,
        dragging: bool,
        last_x: f64,
        last_y: f64,
    }

    impl ApplicationHandler for App {
        fn resumed(&mut self, el: &ActiveEventLoop) {
            if self.window.is_some() {
                return; // 已有窗口（某些平台会多次触发 resumed）
            }
            let window = el
                .create_window(
                    Window::default_attributes()
                        .with_title("Fly Simulator 3D — 拖拽旋转 / 滚轮缩放")
                        .with_inner_size(LogicalSize::new(960, 720)),
                )
                .expect("无法创建窗口");
            // 泄露到 'static，使 softbuffer 的 Context/Surface 可持久持有对窗口的借用
            // （GUI 生命周期 = 进程，泄漏可接受）。
            let window: &'static Window = Box::leak(Box::new(window));
            let ctx = softbuffer::Context::new(window).expect("无法创建 softbuffer context");
            let surface =
                softbuffer::Surface::new(&ctx, window).expect("无法创建 surface");
            self.window = Some(window);
            self.ctx = Some(ctx);
            self.surface = Some(surface);
        }

        fn window_event(
            &mut self,
            el: &ActiveEventLoop,
            _id: WindowId,
            event: WindowEvent,
        ) {
            let window = match self.window {
                Some(w) => w,
                None => return,
            };
            match event {
                WindowEvent::CloseRequested => {
                    el.exit();
                }
                WindowEvent::MouseInput { button: MouseButton::Left, state: s, .. } => {
                    self.dragging = s == ElementState::Pressed;
                }
                WindowEvent::CursorMoved { position, .. } => {
                    if self.dragging {
                        let dx = position.x - self.last_x;
                        let dy = position.y - self.last_y;
                        self.cam.yaw += (dx as f32) * 0.01;
                        self.cam.pitch =
                            (self.cam.pitch + (dy as f32) * 0.01).clamp(-1.5, 1.5);
                    }
                    self.last_x = position.x;
                    self.last_y = position.y;
                }
                WindowEvent::MouseWheel { delta, .. } => {
                    let d = match delta {
                        MouseScrollDelta::LineDelta(_, y) => y,
                        MouseScrollDelta::PixelDelta(p) => p.y as f32 * 0.01,
                    };
                    self.cam.distance =
                        (self.cam.distance * (1.0 + d * 0.1)).clamp(2.0, 200.0);
                }
                WindowEvent::RedrawRequested => {
                    let sz = window.inner_size();
                    let (w, h) = (sz.width, sz.height);
                    if w == 0 || h == 0 {
                        return;
                    }
                    let surface = self.surface.as_mut().expect("surface 未初始化");
                    surface
                        .resize(NonZeroU32::new(w).unwrap(), NonZeroU32::new(h).unwrap())
                        .expect("surface resize 失败");
                    let mut fb = Framebuffer::new(w, h);
                    fb.clear();

                    // 相机 target 跟随机体（渲染系）。
                    {
                        let g = self.state.lock().unwrap();
                        self.cam.target =
                            Point3::new(g.pos[0] as f32, g.pos[1] as f32, g.pos[2] as f32);
                    }
                    let vp = self.cam.view_proj(w as f32 / h as f32);

                    draw_ground(&mut fb, &vp);
                    {
                        let g = self.state.lock().unwrap();
                        draw_quad(&mut fb, &vp, &g, self.arm, 2.5);
                    }

                    let mut buf = surface.buffer_mut().expect("buffer_mut 失败");
                    buf.copy_from_slice(&fb.pixels);
                    buf.present().expect("present 失败");
                }
                _ => {}
            }
        }

        fn about_to_wait(&mut self, el: &ActiveEventLoop) {
            if let Some(w) = self.window {
                // 持续动画：请求下一帧重绘（默认 Wait 下也保持流动）。
                w.request_redraw();
            }
            let _ = el;
        }
    }

    let mut app = App {
        state,
        arm,
        window: None,
        ctx: None,
        surface: None,
        cam: {
            let mut c = Camera::default();
            c.distance = 12.0;
            c.target = Point3::new(0.0, 0.0, 0.0);
            c
        },
        dragging: false,
        last_x: 0.0,
        last_y: 0.0,
    };

    let event_loop = EventLoop::builder()
        .build()
        .expect("无法创建事件循环");
    event_loop.run_app(&mut app).expect("事件循环运行失败");
}
