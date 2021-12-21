use crate::agent::{Agent, AgentMessage, Kinematics, Message};
use crate::missions::*;
use log::*;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Duration;

pub struct SystemManager {
    connection_manager: ConnectionManager,
    mission_manager: MissionManager,
    rendered_tx: Sender<AgentMessage>,
    id_counter: usize,
}

impl SystemManager {
    pub fn new(rendered_tx: Sender<AgentMessage>) -> Self {
        SystemManager {
            connection_manager: ConnectionManager::new(),
            mission_manager: MissionManager::new(),
            id_counter: 0,
            rendered_tx,
        }
    }

    pub fn add_agent(&mut self, kinematics: Kinematics) -> (Agent, ConnectionHandle) {
        let connection_handle = self.connection_manager.create_new_handle();
        let out = (
            Agent {
                id: self.id_counter,
                kinematics,
                mission: None,
            },
            connection_handle,
        );
        self.id_counter += 1;
        out
    }

    pub fn run(mut self) {
        loop {
            let number_missions_left = self.mission_manager.number_missions_left();
            debug!("Missions left in the pool: {}", number_missions_left);
            if number_missions_left < 2 * self.id_counter {
                info!("Creating new batch of missions");
                let new_missions = self.mission_manager.create_new_missions(self.id_counter);
                self.connection_manager.send_new_missions(new_missions);
            }

            loop {
                match self
                    .connection_manager
                    .rx
                    .recv_timeout(Duration::from_millis(10))
                {
                    Ok(agent_message) => {
                        let to_cancel = self.mission_manager.mission_to_finish(&agent_message);
                        for (i, tx) in self.connection_manager.txs.iter().enumerate() {
                            if i != agent_message.id {
                                debug!("Sending message from {} to {}", agent_message.id, i);
                                tx.send(Message::Agent(agent_message.clone())).unwrap();
                            }
                            if let Some(mission_id) = to_cancel {
                                tx.send(Message::MissionFinished(mission_id)).unwrap();
                            }
                        }
                        self.rendered_tx.send(agent_message).unwrap();
                    }
                    Err(e) => match e {
                        std::sync::mpsc::RecvTimeoutError::Timeout => break,
                        std::sync::mpsc::RecvTimeoutError::Disconnected => {}
                    },
                }
            }
        }
    }
}

pub struct ConnectionManager {
    rx: Receiver<AgentMessage>,
    tx: Sender<AgentMessage>,
    txs: Vec<Sender<Message>>,
}

pub struct ConnectionHandle {
    pub tx: Sender<AgentMessage>,
    pub rx: Receiver<Message>,
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
            tx.send(Message::Mission(MissionMessage(new_missions.clone())))
                .unwrap();
        }
    }
}
