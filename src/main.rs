mod agent;
mod consts;
mod missions;
mod renderer;
mod system;

use std::sync::Arc;

use agent::{Cell, Grid, Kinematics};
use consts::*;
use nalgebra::Vector2;
use renderer::Renderer;
use std::sync::mpsc::channel;
use system::SystemManager;

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
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::DEBUG)
        .with_thread_ids(true)
        .with_thread_names(true)
        .init();
    let grid = Arc::new(init_grid());
    let agent_kinematics = init_agent_kinematics();

    let (renderer_tx, rendered_rx) = channel();
    let renderer = Renderer::new(&grid, rendered_rx);
    let mut system = SystemManager::new(renderer_tx);
    let mut agents = Vec::new();
    let mut connection_handlers = Vec::new();
    agent_kinematics.into_iter().for_each(|agent_kinematic| {
        let (a, ch) = system.add_agent(agent_kinematic);
        agents.push(a);
        connection_handlers.push(ch);
    });

    let _system_thread = std::thread::Builder::new()
        .name("SystemManager".to_owned())
        .spawn(move || system.run())
        .unwrap();
    for (i, (mut a, mut ch)) in agents.into_iter().zip(connection_handlers).enumerate() {
        let grid = grid.clone();
        std::thread::Builder::new()
            .name(format!("Agent {}", i))
            .spawn(move || a.run(&mut ch, &grid))
            .unwrap();
    }
    renderer.run();
}
