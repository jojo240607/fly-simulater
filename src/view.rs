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
//!
//! 此模块依赖 `phy` feature（真实渲染原语 + 真实物理引擎适配器），非 `phy` 构建中整模块禁用。

#![cfg(feature = "phy")]

use std::sync::Arc;
use std::sync::Mutex;

use phy_demo::Camera;
use phy_math::na::Point3;

use fly_sim_core::controller::ControllerKind;
use fly_sim_core::physics::{ContactModel, PhySdkWorld};
use fly_sim_core::sensor;
use fly_sim_core::sim;
use fly_sim_core::wind::WindField;
use flyctrl_core::config::VehicleConfig;
use flyctrl_core::vehicle::{ActuatorCmd, VehicleState};

/// 渲染共享状态：已转换到渲染坐标系（右手 Y-up）的机体位姿 + 电机指令。
pub struct RenderState {
    /// 渲染系位置 (x=north, y=up, z=east)。
    pub pos: [f64; 3],
    /// 渲染系姿态 (w,x,y,z)，机体→渲染。
    pub quat: [f64; 4],
    /// 四路电机归一化推力 [0,1]，用于旋翼盘视觉。
    pub motors: [f64; 4],
    /// 渲染系速度 (x=north, y=up, z=east) m/s，用于速度矢量箭头。
    pub vel: [f64; 3],
    /// 四路电机效率系数 [0,1]（1=正常，0=完全停转），用于失效/退化高亮。
    pub eff: [f32; 4],
}

/// 入口：启动后台仿真 + 渲染窗口。
///
/// `degrade` = `Some((motor_idx, eff))` 时运行退化场景（先稳态再注入，实时高亮失效电机）；
/// 否则按 `scenario` 跑 "wind" / 默认悬停。
pub fn run_view(
    cfg: &VehicleConfig,
    dt: f64,
    wind: Option<WindField>,
    sensor_cfg: sensor::SensorConfig,
    kind: ControllerKind,
    scenario: &str,
    eff_mask: [f32; 4],
    degrade: Option<(usize, f32)>,
) {
    let state = Arc::new(Mutex::new(RenderState {
        pos: [0.0, 0.0, 0.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        motors: [0.0; 4],
        vel: [0.0, 0.0, 0.0],
        eff: eff_mask,
    }));
    // 后台仿真线程：全速推进整个场景，逐物理步把真值推入共享状态。
    let sim_state = state.clone();
    let sim_state_eff = sim_state.clone(); // 供 degraded 注入后同步 eff 高亮使用
    let cfg2 = cfg.clone();
    let scenario_owned = scenario.to_string();
    let degrade_owned = degrade;
    std::thread::spawn(move || {
        let mut loop_sim = sim::SimLoop::new(
            PhySdkWorld::create_empty(),
            &cfg2,
            dt,
            wind,
            sensor_cfg,
            kind,
            Some(ContactModel::default()),
            Vec::new(),
        );
        loop_sim.set_motor_eff(eff_mask);
        loop_sim.set_on_frame(Box::new(move |st: &VehicleState, cmd: ActuatorCmd| {
            let mut g = sim_state.lock().unwrap();
            g.pos = fly_sim_core::render::ned_to_render([
                st.pos[0].0 as f64,
                st.pos[1].0 as f64,
                st.pos[2].0 as f64,
            ]);
            g.quat = fly_sim_core::render::ned_quat_to_render(st.att);
            g.vel = fly_sim_core::render::ned_to_render([
                st.vel[0].0 as f64,
                st.vel[1].0 as f64,
                st.vel[2].0 as f64,
            ]);
            for i in 0..4 {
                g.motors[i] = cmd.motor[i] as f64;
            }
        }));
        let ok = match scenario_owned.as_str() {
            "wind" => loop_sim.run_hover_wind(15.0),
            "degraded" => {
                if let Some((m, e)) = degrade_owned {
                    // 复刻 run_hover_degraded：先稳态再注入，并同步渲染态的 eff 高亮。
                    let ok = loop_sim.run_hover_degraded(4.0, 8.0, m, e);
                    {
                        let mut g = sim_state_eff.lock().unwrap();
                        g.eff = [1.0f32; 4];
                        g.eff[m] = e;
                    }
                    ok
                } else {
                    loop_sim.run_hover(10.0)
                }
            }
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
        trail: Vec<[f64; 3]>,
        blink: f64,
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

                    // 相机 target 跟随机体（渲染系），并记录轨迹拖尾。
                    let mut inp = fly_sim_core::render::RenderInput {
                        arm: self.arm,
                        cam_yaw: self.cam.yaw,
                        cam_pitch: self.cam.pitch,
                        cam_distance: self.cam.distance,
                        ..Default::default()
                    };
                    {
                        let g = self.state.lock().unwrap();
                        self.cam.target =
                            Point3::new(g.pos[0] as f32, g.pos[1] as f32, g.pos[2] as f32);
                        inp.pos = g.pos;
                        inp.quat = g.quat;
                        inp.motors = g.motors;
                        inp.vel = g.vel;
                        inp.eff = g.eff;
                        // 轨迹拖尾：每帧记录当前渲染系位置（封顶 120 点 ≈ 历史轨迹）。
                        self.trail.push(g.pos);
                        if self.trail.len() > 120 {
                            self.trail.remove(0);
                        }
                        self.blink += 0.08;
                    }
                    inp.trail = self.trail.clone();
                    inp.blink = self.blink;

                    // 复用 fly-sim-core 单一渲染源（软件光栅化）。
                    let pixels = fly_sim_core::render::render_frame(w, h, &inp);

                    let mut buf = surface.buffer_mut().expect("buffer_mut 失败");
                    buf.copy_from_slice(&pixels);
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
        trail: Vec::new(),
        blink: 0.0,
    };

    let event_loop = EventLoop::builder()
        .build()
        .expect("无法创建事件循环");
    event_loop.run_app(&mut app).expect("事件循环运行失败");
}
