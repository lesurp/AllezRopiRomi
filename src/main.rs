mod agent;
mod consts;
mod missions;
mod motion;
mod renderer;
mod system;

use std::sync::Arc;

use system::SystemManager;
use agent::{Cell, Grid, Kinematics};
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

fn init_agent_kinematics() -> Vec<Kinematics> {
    let mut out = Vec::new();
    for i in 0..2 {
        for j in 0..2 {
            let id = i * 2 + j;
            let kinematics = Kinematics {
                v: Vector2::zeros(),
                a: Vector2::zeros(),
                p: Vector2::new(
                    (j + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                    (i + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                ),
                theta: j as f32 * std::f32::consts::PI,
                radius: 10.0,
            };
            out.push(kinematics)
        }
    }
    out
}

fn main() {
    env_logger::init();
    let grid = Arc::new(init_grid());
    let mut renderer = Renderer::new(&grid);
    let agent_kinematics = init_agent_kinematics();

    let mut system = SystemManager::new();
    let mut agents = Vec::new();
    let mut connection_handlers = Vec::new();
    agent_kinematics.into_iter().for_each(|agent_kinematic| {
        let (a, ch) = system.add_agent(agent_kinematic);
        agents.push(Arc::new(a));
        connection_handlers.push(ch);
    });

    let mut m = motion::MotionSimulator::new();

    for agent in agents.iter() {
        renderer.add_agent(Arc::clone(agent));
        m.add_agent(Arc::clone(agent));
    }

    let _system_thread = std::thread::spawn(move || system.run());
    for (a, mut ch) in agents.into_iter().zip(connection_handlers) {
        let g = Arc::clone(&grid);
        std::thread::spawn(move || a.run(&mut ch, g));
    }
    renderer.run();

    m.stop();
}
