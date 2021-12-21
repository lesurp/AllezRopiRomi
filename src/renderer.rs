use crate::agent::{AgentMessage, Cell, Grid, Kinematics};
use crate::consts::*;
use crate::missions::Mission;
use kiss3d::event::Action;
use kiss3d::text::Font;
use kiss3d::{scene::PlanarSceneNode, window::Window};
use nalgebra::{Matrix2x1, Point2, Point3, Translation2, UnitComplex, Vector2};
use std::collections::HashMap;
use std::f32::consts::FRAC_1_SQRT_2;
use std::f32::consts::FRAC_PI_2;
use std::rc::Rc;
use std::sync::mpsc::Receiver;
use std::sync::Mutex;
use std::time::Duration;

struct TargetNode {
    target_cross: PlanarSceneNode,
    target_line: PlanarSceneNode,
}

impl TargetNode {
    pub fn new(w: &mut Window) -> Self {
        let mut target_cross = w.add_rectangle(5.0, 5.0);
        let mut target_line = w.add_rectangle(LINE_WIDTH, 1.0);

        target_cross.set_color(0.1, 0.1, 0.1);
        target_line.set_color(0.1, 0.1, 0.1);
        Self {
            target_cross,
            target_line,
        }
    }
}

pub struct AgentNode {
    main: PlanarSceneNode,
    velocity: PlanarSceneNode,
    accel: PlanarSceneNode,
    to_target: TargetNode,
}

struct RendererConfig {
    with_target: bool,
}

pub struct Renderer {
    window: Window,
    agent_nodes: HashMap<usize, AgentNode>,
    config: Mutex<RendererConfig>,
    font: Rc<kiss3d::text::Font>,
    rx: Receiver<AgentMessage>,
}

impl Renderer {
    pub fn new(grid: &Grid, rx: Receiver<AgentMessage>) -> Self {
        let mut window = Window::new("Allez Opi, Omi !");
        for (k, cell) in grid.cells.iter().enumerate() {
            let col = k % grid.width;
            let row = k / grid.width;
            let mut rect = window.add_rectangle(CELL_SIZE, CELL_SIZE);
            match &cell {
                Cell::Uncrossable => rect.set_color(0.686, 0.2, 0.0),
                Cell::Crossable(cost) => {
                    let cost = *cost;
                    let reduced = (cost - HALF_COST) / HALF_COST;
                    let (r, g, b) = if cost > HALF_COST {
                        (1.0, 1.0 - reduced, 1.0 - reduced)
                    } else {
                        (1.0 + reduced, 1.0, 1.0 + reduced)
                    };
                    rect.set_color(r, g, b);
                }
            }
            rect.append_translation(&Translation2::new(
                col as f32 * CELL_SIZE - GRID_HALF_SIZE,
                row as f32 * CELL_SIZE - GRID_HALF_SIZE,
            ));
        }

        let config = Mutex::new(RendererConfig { with_target: true });

        Renderer {
            window,
            config,
            agent_nodes: HashMap::new(),
            font: Font::default(),
            rx,
        }
    }

    pub fn toggle_target(&mut self) {
        let mut c = self.config.get_mut().unwrap();
        c.with_target = !c.with_target;
    }

    pub fn run(mut self) {
        while self.render_one() {}
    }

    pub fn render_one(&mut self) -> bool {
        for mut event in self.window.events().iter() {
            if let kiss3d::event::WindowEvent::Key(button, Action::Press, _) = event.value {
                event.inhibited = true;
                match button {
                    kiss3d::event::Key::A => todo!(), // accel
                    kiss3d::event::Key::T => self.toggle_target(),
                    kiss3d::event::Key::V => todo!(), // velocity
                    _ => event.inhibited = false,
                }
            }
        }
        loop {
            match self.rx.recv_timeout(Duration::from_millis(0)) {
                Ok(agent_message) => {
                    match self.agent_nodes.get_mut(&agent_message.id) {
                        Some(node) => {
                            Renderer::update_agent(
                                node,
                                &agent_message.kinematics,
                                &agent_message.mission,
                                &self.config.lock().unwrap(),
                            );
                        }
                        None => self.add_agent(&agent_message),
                    }
                    self.window.draw_text(
                        &agent_message.id.to_string(),
                        &(Point2::origin()
                            + Vector2::new(
                                agent_message.kinematics.p.x,
                                -agent_message.kinematics.p.y,
                            )),
                        10.0,
                        &self.font,
                        &Point3::new(1.0, 0.0, 0.0),
                    )
                }
                Err(e) => match e {
                    std::sync::mpsc::RecvTimeoutError::Timeout => break,
                    std::sync::mpsc::RecvTimeoutError::Disconnected => {}
                },
            }
        }
        self.window.render()
    }

    fn update_agent(
        agent_node: &mut AgentNode,
        kinematics: &Kinematics,
        mission: &Option<Mission>,
        config: &RendererConfig,
    ) {
        let agent_t = Translation2::new(kinematics.p.x, kinematics.p.y);

        if let Some(mission) = mission {
            let delta = mission.target - kinematics.p;
            let center_target_line = delta / 2.0 + kinematics.p;
            agent_node
                .to_target
                .target_line
                .set_local_rotation(UnitComplex::new(delta.y.atan2(delta.x) - FRAC_PI_2));
            agent_node
                .to_target
                .target_cross
                .set_local_translation(mission.target.into());
            agent_node
                .to_target
                .target_line
                .set_local_translation(Translation2::new(
                    center_target_line.x,
                    center_target_line.y,
                ));
            agent_node
                .to_target
                .target_line
                .set_local_scale(1.0, delta.norm());

            agent_node
                .to_target
                .target_line
                .set_visible(config.with_target);
            agent_node
                .to_target
                .target_cross
                .set_visible(config.with_target);
        } else {
            agent_node.to_target.target_line.set_visible(false);
            agent_node.to_target.target_cross.set_visible(false);
        }

        agent_node.main.set_local_translation(agent_t);
        agent_node
            .main
            .set_local_rotation(UnitComplex::new(kinematics.theta - FRAC_PI_2));

        agent_node.velocity.set_local_rotation(UnitComplex::new(
            kinematics.v.y.atan2(kinematics.v.x) - FRAC_PI_2,
        ));
        agent_node.velocity.set_local_translation(Translation2::new(
            kinematics.p.x + kinematics.v.x / 2.0,
            kinematics.p.y + kinematics.v.y / 2.0,
        ));
        agent_node
            .velocity
            .set_local_scale(1.0, kinematics.v.norm());

        agent_node.accel.set_local_rotation(UnitComplex::new(
            kinematics.a.y.atan2(kinematics.a.x) - FRAC_PI_2,
        ));
        agent_node.accel.set_local_translation(Translation2::new(
            kinematics.p.x + kinematics.a.x / 2.0,
            kinematics.p.y + kinematics.a.y / 2.0,
        ));
        agent_node.accel.set_local_scale(1.0, kinematics.a.norm());
    }

    pub fn add_agent(&mut self, agent_message: &AgentMessage) {
        let mut main = self.window.add_planar_group();

        let mut main_radius_out = main.add_circle(AGENT_RADIUS);
        let mut main_radius_in = main.add_circle(AGENT_RADIUS * 0.9);
        let mut main_triangle = main.add_convex_polygon(
            vec![
                Point2::new(0.0, 1.0),
                Point2::new(-FRAC_1_SQRT_2, FRAC_1_SQRT_2),
                Point2::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2),
            ],
            Matrix2x1::new(AGENT_RADIUS, AGENT_RADIUS),
        );
        let mut velocity = self.window.add_rectangle(LINE_WIDTH, 1.0);
        let mut accel = self.window.add_rectangle(LINE_WIDTH, 1.0);
        let to_target = TargetNode::new(&mut self.window);

        accel.set_color(1.0, 0.0, 0.0);
        velocity.set_color(0.0, 0.0, 1.0);
        main_triangle.set_color(0.5, 0.5, 1.0);
        main_radius_out.set_color(0.0, 0.0, 0.0);
        main_radius_in.set_color(1.0, 1.0, 1.0);

        let mut agent_node = AgentNode {
            main,
            velocity,
            accel,
            to_target,
        };
        Renderer::update_agent(
            &mut agent_node,
            &agent_message.kinematics,
            &None,
            &self.config.lock().unwrap(),
        );
        assert!(self
            .agent_nodes
            .insert(agent_message.id, agent_node)
            .is_none());
    }
}
