use log::*;
use crate::consts::{AGENT_RADIUS, CELL_SIZE, GRID_HALF_SIZE, GRID_SIZE};
use nalgebra::Vector2;
use std::{collections::HashMap, fmt};
use rand::distributions::{Distribution, Uniform};

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
        let between = Uniform::from(
            CELL_SIZE + AGENT_RADIUS / 2.0..GRID_SIZE - AGENT_RADIUS / 2.0 - CELL_SIZE,
        );
        let mut rng = rand_pcg::Pcg64::new(0, 0);
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
