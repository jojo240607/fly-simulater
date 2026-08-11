//! 仿真主循环与场景。
//!
//! 当前实现 SIL 模式：物理引擎在 PC，控制律经 `FlyController`(`HilContext`) 在 PC。
//! HIL 模式（接真实飞控 USB）见 DESIGN.md §9，后续在 `hil_link.rs` 扩展。

use flyctrl_core::invariants;
use flyctrl_core::vehicle::VehicleState;

use crate::controller::{hover_setpoint, FlyController};
use crate::plant::QuadrotorPlant;
use flyctrl_core::config::VehicleConfig;

pub struct SimLoop {
    ctrl: FlyController,
    cfg: VehicleConfig,
    dt: f64,
    steps: u64,
}

impl SimLoop {
    pub fn new(cfg: &VehicleConfig, dt: f64) -> Self {
        Self {
            ctrl: FlyController::new(cfg, dt),
            cfg: cfg.clone(),
            dt,
            steps: 0,
        }
    }

    /// 跑一个悬停场景：设定点在 (0,0,-5)，验证收敛 + 不变量。
    pub fn run_hover(&mut self, seconds: f64) -> bool {
        let total = (seconds / self.dt) as u64;
        let sp = hover_setpoint(0.0, 0.0, -5.0);
        let mut all_ok = true;

        for _ in 0..total {
            let st = self.ctrl.step(&sp);
            self.steps += 1;

            if self.steps <= 5 || self.steps % 500 == 0 {
                let w = self.ctrl.world_state();
                let (up, qup) = self.ctrl.debug_up();
                let cmd = self.ctrl.last_cmd();
                println!(
                    "  step {}: TRUE_NED=({:.3},{:.3},{:.3}) EST_NED=({:.3},{:.3},{:.3}) UP=({:.3},{:.3},{:.3}) att=({:.2},{:.2},{:.2},{:.2}) cmd=({:.3},{:.3},{:.3},{:.3})",
                    self.steps,
                    w.pos[0].0, w.pos[1].0, w.pos[2].0,
                    st.pos[0].0, st.pos[1].0, st.pos[2].0,
                    up[0], up[1], up[2],
                    w.att.w, w.att.x, w.att.y, w.att.z,
                    cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                );
            }

            // 不变量：状态有限、指令有界。
            if !invariants::state_finite(&st) {
                eprintln!("[FAIL] step {}: state not finite", self.steps);
                all_ok = false;
                break;
            }
            let cmd = self.ctrl.last_cmd();
            if !invariants::actuator_bounded(&cmd) {
                eprintln!("[FAIL] step {}: actuator out of bounds", self.steps);
                all_ok = false;
                break;
            }
        }

        // 收敛判定：末态位置接近设定点。
        let end = self.ctrl.world_state();
        let dz = (end.pos[2].0 - (-5.0)).abs();
        let horiz = (end.pos[0].0).hypot(end.pos[1].0);
        println!(
            "[hover] end pos NED = ({:.2},{:.2},{:.2})m  |dz|={:.2} horiz={:.2}",
            end.pos[0].0, end.pos[1].0, end.pos[2].0, dz, horiz
        );
        let converged = dz < 0.5 && horiz < 0.5;
        all_ok && converged
    }

    pub fn steps(&self) -> u64 { self.steps }
}

// 抑制未使用告警：QuadrotorPlant 在 controller 内部已使用，这里保留引用便于扩展。
#[allow(dead_code)]
fn _assert_plant(_: QuadrotorPlant) {}
#[allow(dead_code)]
fn _assert_state(_: VehicleState) {}
