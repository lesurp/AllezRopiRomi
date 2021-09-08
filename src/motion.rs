use log::*;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::{thread, time};

use crate::agent::Agent;

pub struct MotionSimulator {
    t: std::thread::JoinHandle<()>,
    agents: Arc<Mutex<Vec<Arc<Agent>>>>,
    stop_flag: Arc<AtomicBool>,
}

const FRICTION_FACTOR: f32 = 0.2;
impl MotionSimulator {
    pub fn add_agent(&mut self, agent: Arc<Agent>) {
        self.agents.lock().unwrap().push(agent)
    }

    pub fn new() -> Self {
        let agents: Arc<Mutex<Vec<Arc<Agent>>>> = Arc::new(Mutex::new(Vec::new()));
        let c = Arc::clone(&agents);
        let stop_flag = Arc::new(AtomicBool::new(true));
        let sf_c = Arc::clone(&stop_flag);
        let t = std::thread::spawn(move || {
            let mut prev_time = time::Instant::now();
            while sf_c.load(std::sync::atomic::Ordering::Relaxed) {
                let now = time::Instant::now();
                let dt = (now - prev_time).as_secs_f32();
                debug!("Delta T: {}", dt);
                prev_time = now;
                for agent in c.lock().unwrap().iter() {
                    let mut k = agent.kinematics.read().unwrap().clone();
                    debug!("Processing agent {}", agent.id);
                    debug!("Old p: {}", k.p);
                    debug!("Old v: {}", k.v);
                    k.p += dt * (k.v + dt * k.a / 2.0);
                    k.v += dt * (k.a - FRICTION_FACTOR * k.v);
                    debug!("New p: {}", k.p);
                    debug!("New v: {}", k.v);
                    *agent.kinematics.write().unwrap() = k;
                }
                let update_period = time::Duration::from_micros(100);
                thread::sleep(update_period);
            }
        });

        MotionSimulator {
            t,
            agents,
            stop_flag,
        }
    }

    pub fn stop(self) {
        self.stop_flag
            .store(false, std::sync::atomic::Ordering::Relaxed);
        self.t.join().unwrap();
    }
}
