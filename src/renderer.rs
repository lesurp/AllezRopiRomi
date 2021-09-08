use crate::agent::{Agent, Cell, Grid, Kinematics, Mission};
use crate::consts::*;
use kiss3d::event::Action;
use kiss3d::{scene::PlanarSceneNode, window::Window};
use nalgebra::{Matrix2x1, Point2, Translation2, UnitComplex};
use std::collections::HashMap;
use std::f32::consts::FRAC_1_SQRT_2;
use std::f32::consts::FRAC_PI_2;
use std::sync::Arc;
use std::sync::Mutex;

use crate::consts::*;

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
    agents: Mutex<Vec<Arc<Agent>>>,
    config: Mutex<RendererConfig>
}

impl Renderer {
    pub fn new(grid: &Grid) -> Self {
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

        let config = Mutex::new(RendererConfig {
            with_target: true,
        });

        Renderer {
            window,
            config,
            agent_nodes: HashMap::new(),
            agents: Mutex::new(Vec::new()),
        }
    }

    pub fn toggle_target(&mut self) {
        let mut c = self.config.get_mut().unwrap();
        c.with_target = !c.with_target;
    }

    pub fn run(mut self) {
        while self.window.render() {
            for mut event in self.window.events().iter() {
                if let kiss3d::event::WindowEvent::Key(button, Action::Press, _) = event.value {
                    match button {
                        kiss3d::event::Key::A => todo!(), // accel
                        kiss3d::event::Key::T => self.toggle_target(),
                        kiss3d::event::Key::V => todo!(), // velocity
                        _ => {}
                    }
                    println!("You pressed the button: {:?}", button);
                    println!("Do not try to press escape: the event is inhibited!");
                    event.inhibited = true // override the default keyboard handler
                }
            }
            for agent in self.agents.lock().unwrap().iter() {
                if let Ok(kinematics) = agent.kinematics.try_read() {
                    if let Ok(mission) = agent.mission.try_read() {
                        Renderer::update_agent(
                            self.agent_nodes.get_mut(&agent.id).unwrap(),
                            &kinematics,
                            &mission,
                            &self.config.lock().unwrap(),
                        )
                    }
                }
            }
        }
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

            agent_node.to_target.target_line.set_visible(true && config.with_target);
            agent_node.to_target.target_cross.set_visible(true && config.with_target);
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

    pub fn add_agent(&mut self, agent: Arc<Agent>) {
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
        let mut to_target = TargetNode::new(&mut self.window);
        //let mut to_target = self.window.add_planar_group();

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
        Renderer::update_agent(&mut agent_node, &agent.kinematics.read().unwrap(), &None, &self.config.lock().unwrap());
        assert!(self.agent_nodes.insert(agent.id, agent_node).is_none());
        self.agents.lock().unwrap().push(agent);
    }
}
