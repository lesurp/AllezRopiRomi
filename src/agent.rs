use nalgebra::Vector2;
use std::collections::HashMap;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::sync::RwLock;
use std::{thread, time};

pub struct SystemManager {
    connection_manager: ConnectionManager,
    mission_manager: MissionManager,
    id_counter: usize,
}

impl SystemManager {
    pub fn new() -> Self {
        SystemManager {
            connection_manager: ConnectionManager::new(),
            mission_manager: MissionManager::new(),
            id_counter: 0,
        }
    }

    pub fn add_agent(&mut self, kinematics: Kinematics) -> (Agent, ConnectionHandle) {
        let connection_handle = self.connection_manager.create_new_handle();
        self.id_counter += 1;
        (Agent {
            id: self.id_counter,
            kinematics: RwLock::new(kinematics),
            mission: RwLock::new(None),
        }, connection_handle)
    }

    pub fn run(mut self) -> ! {
        loop {
            if self.mission_manager.number_missions_left() < 2 * self.id_counter {
                let new_missions = self.mission_manager.create_new_missions(self.id_counter);
                self.connection_manager.send_new_missions(new_missions);
            }
        }
    }
}

pub enum Message {
    Mission(MissionMessage),
    Agent(AgentMessage),
}

pub struct AgentMessage {
    id: usize,
    pub kinematics: Kinematics,
    pub mission: Option<Mission>,
}

pub struct ConnectionManager {
    rx: Receiver<AgentMessage>,
    tx: Sender<AgentMessage>,
    txs: Vec<Sender<Message>>,
}

pub struct ConnectionHandle {
    tx: Sender<AgentMessage>,
    rx: Receiver<Message>,
}

impl ConnectionManager {
    pub fn new() -> Self {
        let (tx, rx) = channel();
        ConnectionManager {
            tx,
            rx,
            txs: Vec::new(),
        }
    }

    pub fn create_new_handle(&mut self) -> ConnectionHandle {
        let (tx, rx) = channel();
        self.txs.push(tx);
        ConnectionHandle {
            tx: self.tx.clone(),
            rx,
        }
    }

    pub fn send_new_missions(&mut self, new_missions: Vec<Mission>) {
        for tx in &self.txs {
            tx.send(Message::Mission(MissionMessage(new_missions.clone())));
        }
    }
}

pub struct MissionMessage(Vec<Mission>);
#[derive(Clone)]
pub struct Mission {
    id: usize,
    agent: Option<usize>,
    target: Vector2<f32>,
}

pub struct MissionManager {
    missions: HashMap<usize, Mission>,
    id_counter: usize,
}

impl MissionManager {
    pub fn new() -> Self {
        MissionManager {
            missions: HashMap::new(),
            id_counter: 0,
        }
    }

    pub fn create_new_missions(&mut self, n: usize) -> Vec<Mission> {
        let mut out = Vec::new();
        for i in 0..n {
            let mission = Mission {
                id: self.id_counter,
                agent: None,
                target: Vector2::zeros(),
            };
            self.missions.insert(self.id_counter, mission.clone());
            out.push(mission);
            self.id_counter += 1;
        }
        out
    }

    pub fn finish_mission(&mut self, id: usize) {
        self.missions.remove(&id);
    }

    pub fn number_missions_left(&self) -> usize {
        self.missions.len()
    }
}

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
    pub fn run(
        &self,
        connection_handle: &mut ConnectionHandle,
        grid: Arc<Grid>,
    ) {
        loop {}
    }
}

pub struct MotionSimulator<T> {
    t: std::thread::JoinHandle<T>,
    agents: Arc<Mutex<Vec<Arc<Agent>>>>
}

const FRICTION_FACTOR: f32 = 0.2;
impl<T: Send + 'static> MotionSimulator<T> {
    pub fn add_agent(&mut self, agent: Arc<Agent>) {
        self.agents.lock().unwrap().push(agent)
    }

    pub fn new() -> Self {
        let agents : Arc<Mutex<Vec<Arc<Agent>>>> = Arc::new(Mutex::new(Vec::new()));
        let c = Arc::clone(&agents);
        let t = std::thread::spawn(move || {
            let mut prev_time = time::Instant::now();
            loop {
                let now = time::Instant::now();
                let dt = (now - prev_time).as_secs_f32();
                prev_time = now;
                for agent in c.lock().unwrap().iter() {
                    let mut k = agent.kinematics.read().unwrap().clone();
                    k.p += dt * (k.v + dt * k.a / 2.0);
                    k.v += dt * (k.a - FRICTION_FACTOR * k.v);
                    *agent.kinematics.write().unwrap() = k;
                }
                let paf = time::Duration::from_micros(100);
                thread::sleep(paf);
            }
        });

        MotionSimulator { t, agents }
    }
}
