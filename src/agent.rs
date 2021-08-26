use nalgebra::Vector2;
use std::sync::Arc;
use std::sync::RwLock;
use std::{thread, time};

pub struct AgentMessage {
    id: usize,
    pub kinematics: Kinematics,
    pub mission: Option<Mission>,
}
pub struct ConnectionManager;
impl ConnectionManager {}

pub struct Mission {
    target: Vector2<f32>,
}
pub struct MissionManager;
pub struct Agent {
    pub id: usize,
    pub kinematics: RwLock<Kinematics>,
    pub mission: RwLock<Option<Mission>>,
}

pub struct Grid {
    pub cells: Vec<Cell>,
    pub width: usize,
}

impl Grid {}

pub enum Cell {
    Uncrossable,
    Crossable(f32),
}

#[derive(Clone)]
pub struct Kinematics {
    pub p: Vector2<f32>,
    pub v: Vector2<f32>,
    pub a: Vector2<f32>,
    pub theta: f32,
    pub radius: f32,
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

pub struct MotionSimulator<T> {
    t: std::thread::JoinHandle<T>,
}

const FRICTION_FACTOR: f32 = 0.2;
impl<T: Send + 'static> MotionSimulator<T> {
    pub fn spawn(agents: Arc<Vec<Agent>>) -> Self {
        let t = std::thread::spawn(move || {
            let mut prev_time = time::Instant::now();
            loop {
                let now = time::Instant::now();
                let dt = (now - prev_time).as_secs_f32();
                prev_time = now;
                for agent in agents.iter() {
                    let mut k = agent.kinematics.read().unwrap().clone();
                    k.p += dt * (k.v + dt * k.a / 2.0);
                    k.v += dt * (k.a - FRICTION_FACTOR * k.v);
                    *agent.kinematics.write().unwrap() = k;
                }
                let paf = time::Duration::from_micros(100);
                thread::sleep(paf);
            }
        });

        MotionSimulator { t }
    }
}
