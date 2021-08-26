use crate::agent::{Agent, Cell, Grid, Kinematics};
use crate::consts::*;
use kiss3d::{scene::PlanarSceneNode, window::Window};
use nalgebra::{Matrix2x1, Point2, Translation2, UnitComplex};
use std::collections::HashMap;
use std::sync::Arc;
use std::{thread, time};

use crate::consts::*;

pub struct AgentNode {
    main: PlanarSceneNode,
    velocity: PlanarSceneNode,
    accel: PlanarSceneNode,
    to_target: PlanarSceneNode,
}

pub struct Renderer {
    window: Window,
    agents: HashMap<usize, AgentNode>,
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
        Renderer {
            window,
            agents: HashMap::new(),
        }
    }

    pub fn run(mut self, agents: Arc<Vec<Agent>>) {
        while self.window.render() {
            for agent in agents.iter() {
                if let Ok(kinematics) = agent.kinematics.try_read() {
                    self.update_agent(agent.id, &kinematics)
                }
            }
        }
    }

    pub fn update_agent(&mut self, id: usize, kinematics: &Kinematics) {
        let agent = self.agents.get_mut(&id).unwrap();
        agent.main.set_color(0.5, 0.5, 1.0);
        agent
            .main
            .set_local_translation(Translation2::new(kinematics.p.x, kinematics.p.y));
        agent.main.set_local_rotation(UnitComplex::new(
            kinematics.theta - std::f32::consts::FRAC_PI_2,
        ));

        agent.velocity.set_color(0.0, 1.0, 1.0);
        agent.velocity.set_local_rotation(UnitComplex::new(
            kinematics.v.y.atan2(kinematics.v.x) - std::f32::consts::FRAC_PI_2,
        ));
        agent.velocity.set_local_translation(Translation2::new(
            kinematics.p.x + kinematics.v.x / 2.0,
            kinematics.p.y + kinematics.v.y / 2.0,
        ));
        agent.velocity.set_local_scale(1.0, kinematics.v.norm());

        agent.accel.set_color(1.0, 0.5, 0.5);
        agent.accel.set_local_rotation(UnitComplex::new(
            kinematics.a.y.atan2(kinematics.a.x) - std::f32::consts::FRAC_PI_2,
        ));
        agent.accel.set_local_translation(Translation2::new(
            kinematics.p.x + kinematics.a.x / 2.0,
            kinematics.p.y + kinematics.a.y / 2.0,
        ));
        agent.accel.set_local_scale(1.0, kinematics.a.norm());
    }

    pub fn add_agent(&mut self, id: usize, kinematics: &Kinematics) {
        let main = self.window.add_convex_polygon(
            vec![
                Point2::new(0.0, 2.0),
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
            ],
            Matrix2x1::new(20.0, 20.0),
        );
        let velocity = self.window.add_rectangle(LINE_WIDTH, 1.0);
        let accel = self.window.add_rectangle(LINE_WIDTH, 1.0);
        let to_target = self.window.add_planar_group();
        let agent = AgentNode {
            main,
            velocity,
            accel,
            to_target,
        };
        assert!(self.agents.insert(id, agent).is_none());
        self.update_agent(id, kinematics);
    }
}
