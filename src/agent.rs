use crate::missions::*;
use crate::system::*;
use log::*;
use nalgebra::Vector2;
use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

pub enum Message {
    Mission(MissionMessage),
    MissionFinished(usize),
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
    pub kinematics: Kinematics,
    pub mission: Option<Mission>,
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
    pub fn simulate_motion(&mut self, old: Instant) -> (Instant, f32) {
        let friction = (0.8f32).ln();
        let now = Instant::now();
        let dt = (now - old).as_secs_f32();
        debug!("wat: {}", (dt / friction).exp());
        let k = &mut self.kinematics;
        k.p += dt * (k.v + dt * k.a / 2.0);
        k.v = dt * k.a + (dt * friction).exp() * k.v;
        (now, dt)
    }

    pub fn run(&mut self, connection_handle: &mut ConnectionHandle, _grid: &Grid) {
        info!("Starting agent");
        let mut agents = HashMap::new();
        let mut missions = HashMap::new();
        let mut now = Instant::now();
        loop {
            let (new_now, dt) = self.simulate_motion(now);
            now = new_now;
            loop {
                match connection_handle.rx.recv_timeout(Duration::from_millis(10)) {
                    Ok(message) => match message {
                        Message::Mission(mission_message) => {
                            debug!("Received new mission: {:?}", mission_message);
                            for m in mission_message.0 {
                                missions.insert(m.id, m);
                            }
                            self.get_new_mission(&missions, &agents);
                        }
                        Message::Agent(agent_message) => {
                            debug!("Updating info from agent {}", agent_message.id);
                            agents.insert(agent_message.id, agent_message);
                        }
                        Message::MissionFinished(mission_id) => {
                            if let Some(mission) = &self.mission {
                                if mission.id == mission_id {
                                    self.mission = None;
                                    self.get_new_mission(&missions, &agents);
                                }
                            }
                            missions.remove(&mission_id);
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

            debug!("Current mission: {:?}", self.mission);
            if let Some(mission) = &self.mission {
                let k = &mut self.kinematics;
                let m = mission.target - k.p;
                let mut ppart = (2.0 / dt) * (m / dt);
                if ppart.norm() > 2.0 * 100.0 {
                    ppart *= 2.0 * 100.0 / ppart.norm();
                }
                let mut vpart = -(2.0 / dt) * k.v;
                if vpart.norm() > 100.0 {
                    vpart *= 100.0 / vpart.norm();
                }
                let a = ppart + vpart;
                k.a = if a.norm() > 100.0 {
                    a * 100.0 / a.norm()
                } else {
                    a
                };
                debug!("dt:\t{}", dt);
                debug!("target:\t{}", mission.target);
                debug!("Acceleration:\t{}", k.a);
                debug!("Position:\t{}", k.p);
                debug!("Velocity:\t{}", k.v);
            } else {
                self.kinematics.a = Vector2::zeros();
                debug!("New acceleration is null, because it has no associated mission",);
            }

            let our_state = self.state();
            debug!("Sending new state {:?}", our_state);
            connection_handle.tx.send(our_state).unwrap();
        }
    }

    fn state(&self) -> AgentMessage {
        AgentMessage {
            id: self.id,
            kinematics: self.kinematics.clone(),
            mission: self.mission.clone(),
        }
    }

    fn get_new_mission(
        &mut self,
        missions: &HashMap<usize, Mission>,
        _agents: &HashMap<usize, AgentMessage>,
    ) {
        let mut best_dist = std::f32::MAX;
        let mut best_mission = None;
        let p = self.kinematics.p;
        for mission in missions.values() {
            let n = (p - mission.target).norm_squared();
            if n < best_dist {
                best_dist = n;
                best_mission = Some(mission.clone())
            }
        }

        match &self.mission {
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
        self.mission = best_mission;
        debug!("Chosen mission {:?}", self.mission);
    }

    fn check_missions(
        &mut self,
        _connection_handle: &mut ConnectionHandle,
        missions: &HashMap<usize, Mission>,
        agents: &mut HashMap<usize, AgentMessage>,
    ) {
        let k = &self.kinematics;
        let mut assigned_missions = HashSet::new();
        if let Some(curr_m) = &self.mission {
            let mut reassign = false;
            for (_, a) in agents.iter_mut() {
                if a.id == self.id {
                    continue;
                }
                match &a.mission {
                    Some(m) => {
                        assigned_missions.insert(m.id);
                        if m.id == curr_m.id {
                            match missions.get(&m.id) {
                                Some(other_mission) => {
                                    let other_cost =
                                        (other_mission.target - a.kinematics.p).norm_squared();
                                    let my_cost = (missions[&m.id].target - k.p).norm_squared();
                                    debug!(
                                "Agent {} (cost {}) works on the same mission ({}) as us (our cost {})",
                                a.id, other_cost, m.id , my_cost,
                            );
                                    reassign = my_cost > other_cost;
                                    break;
                                }
                                None => warn!(
                                    "Agent {} appears to still be working on mission {}",
                                    a.id,
                                    m.id
                                ),
                            }
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
                self.mission = best_mission;
            }
        }
    }
}
