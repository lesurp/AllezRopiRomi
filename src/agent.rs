use log::*;
use nalgebra::Vector2;
use rand::distributions::{Distribution, Uniform};
use std::collections::HashMap;
use std::fmt;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::RwLock;
use std::sync::{Arc, Mutex};
use std::thread::sleep_ms;
use std::time::Duration;

use crate::consts::{AGENT_RADIUS, CELL_SIZE, GRID_HALF_SIZE, GRID_SIZE};

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
        (
            Agent {
                id: self.id_counter,
                kinematics: RwLock::new(kinematics),
                mission: RwLock::new(None),
            },
            connection_handle,
        )
    }

    pub fn run(mut self) -> ! {
        loop {
            let number_missions_left = self.mission_manager.number_missions_left();
            debug!("Missions left in the pool: {}", number_missions_left);
            if number_missions_left < 2 * self.id_counter {
                info!("Creating new batch of missions");
                let new_missions = self.mission_manager.create_new_missions(self.id_counter);
                self.connection_manager.send_new_missions(new_missions);
            }

            sleep_ms(10);
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
            tx.send(Message::Mission(MissionMessage(new_missions.clone()))).unwrap();
        }
    }
}

pub struct MissionMessage(Vec<Mission>);

#[derive(Clone, Debug)]
pub struct Mission {
    pub id: usize,
    pub agent: Option<usize>,
    pub target: Vector2<f32>,
}
impl fmt::Display for Mission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[id: {}, agent: {}, target: {}]",
            self.id,
            match self.agent {
                Some(id) => id.to_string(),
                None => "None".to_owned(),
            },
            self.target
        )
    }
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
        let between = Uniform::from(CELL_SIZE + AGENT_RADIUS / 2.0..GRID_SIZE - AGENT_RADIUS / 2.0 - CELL_SIZE);
        let mut rng = rand::thread_rng();
        for _i in 0..n {
            let mission = Mission {
                id: self.id_counter,
                agent: None,
                target: Vector2::new(
                    between.sample(&mut rng) - GRID_HALF_SIZE,
                    between.sample(&mut rng) - GRID_HALF_SIZE,
                ),
            };
            info!("Mission created with target: {}", mission.target);
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
    pub fn run(&self, connection_handle: &mut ConnectionHandle, _grid: Arc<Grid>) {
        loop {
            loop {
                match connection_handle.rx.recv_timeout(Duration::from_millis(10)) {
                    Ok(message) => match message {
                        Message::Mission(mission_message) => self.handle_mission(mission_message),
                        Message::Agent(_agent_message) => todo!(),
                    },
                    Err(err) => match err {
                        std::sync::mpsc::RecvTimeoutError::Timeout => {
                            debug!("Agent {}'s rx channel timed out", self.id);
                            break;
                        }
                        std::sync::mpsc::RecvTimeoutError::Disconnected => {
                            error!("Agent {} could not retrieve message fmor channel", self.id)
                        }
                    },
                }
            }

            if let Some(mission) = self.mission.read().unwrap().clone() {
                let mut k = self.kinematics.read().unwrap().clone();
                let dt = 1.0;
                let a = (2.0 / dt * (mission.target - k.p) - k.v) / dt;
                k.a = a / 20.0;
                debug!("Agent {}'s new acceleration is: {}", self.id, k.a);
                *self.kinematics.write().unwrap() = k;
            }
        }
    }

    fn handle_mission(&self, mission_message: MissionMessage) {
        let mut best_dist = std::f32::MAX;
        let mut best_mission = None;
        let p = self.kinematics.read().unwrap().p;
        for mission in mission_message.0 {
            let n = (p - mission.target).norm_squared();
            if n < best_dist {
                best_dist = n;
                best_mission = Some(mission)
            }
        }

        match &best_mission {
            Some(best_mission) => {
                debug!("Agent {} chose mission {}", self.id, best_mission);
            }
            None => {
                debug!("Agent {} has no mission", self.id);
            }
        }
        *self.mission.write().unwrap() = best_mission;
    }
}
