extern crate alloc;
use alloc::{boxed::Box, vec::Vec};
use rp_pico::hal::pio::{Running, Rx, StateMachine, Stopped, Tx};

/// Number of PIOs on board
const NUM_PIO: usize = 8;

/// Top-level struct containing all PIOs on the board
pub struct AllPioStateMachines {
    pub state_machines: [Box<dyn AnyPioStateMachine>; NUM_PIO]
}
impl AllPioStateMachines{
    pub fn pause_all(&mut self) {
        for sm in self.state_machines.iter_mut() {
            sm.pause();
        }
    }
    pub fn restart_all(&mut self) {
        for sm in self.state_machines.iter_mut() {
            sm.restart();
        }
    }
    pub fn write_all(&mut self, val: u32) -> [bool; NUM_PIO] {
        self.state_machines.iter_mut()
            .map(|sm| sm.write(val))
            .collect::<Vec<_>>().try_into().unwrap()

    }
    pub fn read_all(&mut self) -> [Option<u32>; NUM_PIO] {
        self.state_machines.iter_mut()
            .map(|sm| sm.read())
            .collect::<Vec<_>>().try_into().unwrap()
    }
}

/// Trait for a PIO state machine, implemented on all PIO state machine instances.
/// 
/// Useful for storing all PIO state machines in an array, since they're all different types
pub trait AnyPioStateMachine {
    fn restart(&mut self);
    fn pause(&mut self);
    fn write(&mut self, val: u32) -> bool;
    fn read(&mut self) -> Option<u32>;
}
impl<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex> AnyPioStateMachine for PioStateMachine<PIO, SM> {
    fn restart(&mut self) {
        if self.state == PioState::Stopped {
            self.state_machine_on = Some(self.state_machine_off.take().unwrap().start());
            self.state = PioState::Running;
        }
        self.state_machine_on.as_mut().unwrap().restart();
    }
    fn pause(&mut self) {
        if self.state == PioState::Running {
            self.state_machine_off = Some(self.state_machine_on.take().unwrap().stop());
            self.state = PioState::Stopped;
        }
    }
    fn write(&mut self, val: u32) -> bool {
        self.tx.write(val)
    }
    fn read(&mut self) -> Option<u32> {
        self.rx.read()
    }
}

/// Struct housing all the relevant information and buffers for a single PIO state machine. Running type hidden so they're easier to carry around
pub struct PioStateMachine<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex> {
    pub state_machine_on: Option<StateMachine<(PIO, SM), Running>>,
    pub state_machine_off: Option<StateMachine<(PIO, SM), Stopped>>,
    pub rx: Rx<(PIO, SM)>,
    pub tx: Tx<(PIO, SM)>,
    pub state: PioState,
}

#[derive(PartialEq)]
pub enum PioState{
    Stopped,
    Running,
}

