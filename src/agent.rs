use crate::missions::*;
use crate::system::*;
use log::*;
use nalgebra::Vector2;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::sync::RwLock;
use std::time::Duration;

pub enum Message {
    Mission(MissionMessage),
    Agent(AgentMessage),
}

#[derive(Clone, Debug)]
pub struct AgentMessage {
    pub id: usize,
    pub kinematics: Kinematics,
    pub mission: Option<Mission>,
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

#[derive(Clone, Debug)]
pub struct Kinematics {
    pub p: Vector2<f32>,
    pub v: Vector2<f32>,
    pub a: Vector2<f32>,
    pub theta: f32,
    pub radius: f32,
}

impl Agent {
    pub fn run(&self, connection_handle: &mut ConnectionHandle, _grid: Arc<Grid>) {
        info!("Starting agent");
        let mut agents = HashMap::new();
        let mut missions = HashMap::new();
        loop {
            loop {
                match connection_handle.rx.recv_timeout(Duration::from_millis(10)) {
                    Ok(message) => match message {
                        Message::Mission(mission_message) => {
                            self.handle_mission(&mission_message, &agents);
                            for m in mission_message.0 {
                                missions.insert(m.id, m);
                            }
                        }
                        Message::Agent(agent_message) => {
                            debug!("Updating info from agent {}", agent_message.id);
                            agents.insert(agent_message.id, agent_message);
                        }
                    },
                    Err(err) => match err {
                        std::sync::mpsc::RecvTimeoutError::Timeout => {
                            debug!("Rx channel timed out");
                            break;
                        }
                        std::sync::mpsc::RecvTimeoutError::Disconnected => {
                            error!("Could not retrieve message from channel")
                        }
                    },
                }
            }

            self.check_missions(connection_handle, &missions, &mut agents);

            if let Some(mission) = self.mission.read().unwrap().clone() {
                let mut k = self.kinematics.read().unwrap().clone();
                let dt = 1.0;
                let a = (2.0 / dt * (mission.target - k.p) - k.v) / dt;
                k.a = a / 20.0;
                debug!("New target is at {}", mission.target);
                debug!("New acceleration is: {}", k.a);
                *self.kinematics.write().unwrap() = k;
            } else {
                *self.kinematics.write().unwrap().a = *Vector2::zeros();
                debug!("New acceleration is null, because it has no associated mission",);
            }

            let our_state = self.state();
            debug!("Sending new state {:?}", our_state);
            connection_handle.tx.send(our_state).unwrap();

            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn state(&self) -> AgentMessage {
        AgentMessage {
            id: self.id,
            kinematics: self.kinematics.read().unwrap().clone(),
            mission: self.mission.read().unwrap().clone(),
        }
    }

    fn handle_mission(
        &self,
        mission_message: &MissionMessage,
        _agents: &HashMap<usize, AgentMessage>,
    ) {
        debug!("Received new mission: {:?}", mission_message);
        let mut best_dist = std::f32::MAX;
        let mut best_mission = None;
        let p = self.kinematics.read().unwrap().p;
        for mission in &mission_message.0 {
            let n = (p - mission.target).norm_squared();
            if n < best_dist {
                best_dist = n;
                best_mission = Some(mission.clone())
            }
        }

        match &*self.mission.read().unwrap() {
            Some(m) => {
                let current_mission_cost = (m.target - p).norm_squared();
                if current_mission_cost < best_dist {
                    debug!("Current mission is closer than any other mission: not changing");

                    return;
                }
            }
            None => {}
        }

        match &best_mission {
            Some(best_mission) => {
                debug!("Chose mission {}", best_mission);
            }
            None => {
                debug!("Has no mission");
            }
        }
        *self.mission.write().unwrap() = best_mission;
        debug!("Chosen mission {:?}", self.mission.read());
    }

    fn check_missions(
        &self,
        _connection_handle: &mut ConnectionHandle,
        missions: &HashMap<usize, Mission>,
        agents: &mut HashMap<usize, AgentMessage>,
    ) {
        let k = self.kinematics.read().unwrap().clone();
        let mut assigned_missions = HashSet::new();
        let g = self.mission.read().unwrap();
        let m = g.clone();
        drop(g); // TODO: is there a better way to enforce the lock dtor?
                 // e.g. some getter function? this is *very* bug-prone :(
        if let Some(curr_m) = m {
            let mut reassign = false;
            for (_, a) in agents.iter_mut() {
                if a.id == self.id {
                    continue;
                }
                match &a.mission {
                    Some(m) => {
                        assigned_missions.insert(m.id);
                        if m.id == curr_m.id {
                            let other_cost =
                                (missions[&m.id].target - a.kinematics.p).norm_squared();
                            let my_cost = (missions[&m.id].target - k.p).norm_squared();
                            debug!(
                                "Agent {} (cost {}) works on the same mission ({}) as us (our cost {})",
                                a.id, other_cost, m.id , my_cost,
                            );
                            reassign = my_cost > other_cost;
                            break;
                        }
                    }
                    None => {}
                }
            }

            debug!("Is looking for a new mission: {}", reassign);
            if reassign {
                let mut best_score = std::f32::MAX;
                let mut best_mission = None;
                for m in missions.values() {
                    if assigned_missions.contains(&m.id) {
                        continue;
                    }

                    let score = (k.p - m.target).norm_squared();
                    if score < best_score {
                        best_score = score;
                        best_mission = Some(m.clone());
                    }
                }

                match &best_mission {
                    Some(bm) => debug!("Reassigned itself to {}", bm),
                    None => debug!("Did not reassign itself"),
                };
                *self.mission.write().unwrap() = best_mission.clone();
            }
        }
    }
}
