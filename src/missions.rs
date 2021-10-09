use crate::consts::{AGENT_RADIUS, CELL_SIZE, GRID_HALF_SIZE, GRID_SIZE};
use log::*;
use nalgebra::Vector2;
use rand::distributions::{Distribution, Uniform};
use rand_pcg::Pcg64;
use std::{collections::HashMap, fmt};

pub struct MissionManager {
    missions: HashMap<usize, Mission>,
    id_counter: usize,
    rng: Pcg64,
    between: Uniform<f32>,
}

impl MissionManager {
    pub fn new() -> Self {
        MissionManager {
            missions: HashMap::new(),
            id_counter: 0,
            between: Uniform::from(
                CELL_SIZE + AGENT_RADIUS / 2.0..GRID_SIZE - AGENT_RADIUS / 2.0 - CELL_SIZE,
            ),
            rng: rand_pcg::Pcg64::new(0, 0),
        }
    }

    pub fn create_new_missions(&mut self, n: usize) -> Vec<Mission> {
        let mut out = Vec::new();
        for _i in 0..n {
            let mission = Mission {
                id: self.id_counter,
                agent: None,
                target: Vector2::new(
                    self.between.sample(&mut self.rng) - GRID_HALF_SIZE,
                    self.between.sample(&mut self.rng) - GRID_HALF_SIZE,
                ),
            };
            info!(
                "Mission {} created with target: {}",
                self.id_counter, mission.target
            );
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

#[derive(Debug)]
pub struct MissionMessage(pub Vec<Mission>);

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
