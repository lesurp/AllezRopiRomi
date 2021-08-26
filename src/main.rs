mod agent;
mod consts;
mod renderer;

use std::sync::RwLock;
use std::sync::Arc;

use agent::{Agent, Cell, Grid, Kinematics};
use consts::*;
use nalgebra::Vector2;
use renderer::Renderer;


fn init_grid() -> Grid {
    let height = GRID_SPLIT as usize;
    let width = GRID_SPLIT as usize;
    let mut cells = Vec::with_capacity(height * width);
    for i in 0..height {
        for j in 0..width {
            if i == 0 || j == 0 || i == height - 1 || j == height - 1 {
                cells.push(Cell::Uncrossable);
            } else if j > (0.4 * width as f32) as usize && j < (0.6 * width as f32) as usize {
                cells.push(Cell::Crossable(MAX_COST * i as f32 / height as f32));
            } else {
                cells.push(Cell::Crossable(MAX_COST / 2.0));
            }
        }
    }
    Grid { cells, width }
}

fn init_agents() -> Vec<Agent> {
    let mut out = Vec::new();
    for i in 0..2 {
        for j in 0..2 {
            let id = i * 2 + j;
            let kinematics = Kinematics {
                v: Vector2::new(20.0, 50.0),
                a: Vector2::new(0.0, 0.0),
                p: Vector2::new(
                    (j + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                    (i + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                ),
                theta: j as f32 * std::f32::consts::PI,
                radius: 10.0,
            };
            let agent = Agent { id, kinematics: RwLock::new(kinematics), mission: RwLock::new(None) };
            out.push(agent)
        }
    }
    out
}

fn main() {
    let grid = init_grid();
    let mut renderer = Renderer::new(&grid);
    let agents = Arc::new(init_agents());
    for agent in agents.iter() {
        renderer.add_agent(agent.id, &agent.kinematics.read().unwrap())
    }
    let m : agent::MotionSimulator<u64> = agent::MotionSimulator::spawn(agents.clone());
    renderer.run(agents);
}
