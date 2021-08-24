use kiss3d::ncollide3d::math::Point;
use kiss3d::{scene::PlanarSceneNode, window::Window};
use nalgebra::{Matrix2x1, Point2, Vector2};
use nalgebra::{Translation2, UnitComplex};
use std::collections::HashMap;

struct AgentMessage {
    id: usize,
    kinematics: Kinematics,
    mission: Mission,
}
struct ConnectionManager;
impl ConnectionManager {}

struct Mission {
    target: Vector2<f32>,
}
struct MissionManager;
struct Agent {
    id: usize,
    kinematics: Kinematics,
}

const CELL_SIZE: f32 = 10.0;
const GRID_SPLIT: f32 = 100.0;
const MAX_COST: f32 = 1000.0;
const HALF_COST: f32 = MAX_COST / 2.0;
const GRID_SIZE: f32 = GRID_SPLIT * CELL_SIZE;
const GRID_HALF_SIZE: f32 = GRID_SIZE / 2.0;
struct Grid {
    cells: Vec<Cell>,
    width: usize,
}

impl Grid {
    pub fn new() -> Self {
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
}

enum Cell {
    Uncrossable,
    Crossable(f32),
}

struct Kinematics {
    p: Vector2<f32>,
    v: Vector2<f32>,
    a: Vector2<f32>,
    theta: f32,
    radius: f32,
}

impl Agent {
    fn run(
        self,
        connection_manager: &mut ConnectionManager,
        mission_manager: &mut MissionManager,
        grid: &Grid,
    ) {
    }
}

struct AgentNode {
    main: PlanarSceneNode,
    speed: PlanarSceneNode,
    accel: PlanarSceneNode,
    to_target: PlanarSceneNode,
}
struct Renderer {
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
                    let reduced = (cost - 500.0) / 500.0;
                    let (r, g, b) = if cost > 500.0 {
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

    pub fn run(mut self) {
        while self.window.render() {}
    }

    pub fn add_agent(&mut self, id: usize, kinematics: Kinematics) {
        let mut main = self.window.add_planar_group();
        let mut main_triangle = main.add_convex_polygon(
            vec![
                Point2::new(0.0, 2.0),
                Point2::new(-0.5, -0.5),
                Point2::new(0.5, -0.5),
            ],
            Matrix2x1::new(20.0, 20.0),
        );
        main_triangle.set_color(0.5, 0.5, 1.0);
        main_triangle.set_local_translation(Translation2::new(kinematics.p.x, kinematics.p.y));
        main_triangle.set_local_rotation(UnitComplex::new(kinematics.theta));
        let mut speed = self.window.add_planar_group();
        let mut accel = self.window.add_planar_group();
        let mut to_target = self.window.add_planar_group();
        let an = AgentNode {
            main,
            speed,
            accel,
            to_target,
        };
        // TODO check already exists
        assert!(self.agents.insert(id, an).is_none());
    }
}

fn init_agents() -> Vec<Agent> {
    let mut out = Vec::new();
    for i in 0..2 {
        for j in 0..2 {
            let id = i * 2 + j;
            let kinematics = Kinematics {
                v: Vector2::zeros(),
                a: Vector2::zeros(),
                p: Vector2::new(
                    (i + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                    (j + 1) as f32 * GRID_SIZE / 3.0 - GRID_HALF_SIZE,
                ),
                theta: j as f32 * std::f32::consts::PI,
                radius: 10.0,
            };
            let agent = Agent { id, kinematics };
            out.push(agent)
        }
    }
    out
}

fn main() {
    let grid = Grid::new();
    let mut renderer = Renderer::new(&grid);
    for agent in init_agents() {
        renderer.add_agent(agent.id, agent.kinematics)
    }
    renderer.run();
}
