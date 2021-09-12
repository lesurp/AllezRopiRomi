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
                            agents.insert(agent_message.id, agent_message);
                        }
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

            self.check_missions(connection_handle, &missions, &mut agents);

            if let Some(mission) = self.mission.read().unwrap().clone() {
                let mut k = self.kinematics.read().unwrap().clone();
                let dt = 1.0;
                let a = (2.0 / dt * (mission.target - k.p) - k.v) / dt;
                k.a = a / 20.0;
                debug!("Agent {}'s new acceleration is: {}", self.id, k.a);
                *self.kinematics.write().unwrap() = k;
            } else {
                *self.kinematics.write().unwrap().a = *Vector2::zeros();
                debug!("Agent {}'s new acceleration is null, because it has no associated mission", self.id);
            }

            connection_handle
                .tx
                .send(AgentMessage {
                    id: self.id,
                    kinematics: self.kinematics.read().unwrap().clone(),
                    mission: self.mission.read().unwrap().clone(),
                })
                .unwrap();
        }
    }

    fn handle_mission(
        &self,
        mission_message: &MissionMessage,
        agents: &HashMap<usize, AgentMessage>,
    ) {
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

    fn check_missions(
        &self,
        connection_handle: &mut ConnectionHandle,
        missions: &HashMap<usize, Mission>,
        agents: &mut HashMap<usize, AgentMessage>,
    ) {
        let k = self.kinematics.read().unwrap().clone();
        let mut assigned_missions = HashSet::new();
        if let Some(curr_m) = self.mission.read().unwrap().clone() {
            let mut reassign = false;
            for (_, a) in agents.iter_mut() {
                match &a.mission {
                    Some(m) => {
                        assigned_missions.insert(m.id);
                        if m.id == curr_m.id {
                            let other_cost =
                                (missions[&m.id].target - a.kinematics.p).norm_squared();
                            let my_cost = (missions[&m.id].target - k.p).norm_squared();
                            debug!(
                                "Agents {} (cost {}) and {} (cost {}) work on the same mission {}",
                                self.id, my_cost, a.id, other_cost, m.id
                            );
                            reassign = my_cost > other_cost;
                            break;
                        }
                    }
                    None => {}
                }
            }

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
                    Some(bm) => debug!("Agent {} reassigned itself to {}", self.id, bm),
                    None => debug!("Agent {} reassigned itself to no mission", self.id),
                };
                *self.mission.write().unwrap() = best_mission.clone();
                agents.get_mut(&self.id).unwrap().mission = best_mission;
            }
        }
    }
}
