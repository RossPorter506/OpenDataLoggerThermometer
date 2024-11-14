extern crate alloc;
use alloc::boxed::Box;
use rp_pico::pac::{PIO0, PIO1};
use rp_pico::hal::pio::{StateMachine, Running, Stopped, Rx, Tx, SM0, SM1, SM2, SM3, PIOExt, StateMachineIndex};

/// Number of PIOs on board
const NUM_PIO: usize = 8;

/// Top-level struct containing all PIOs on the board
pub struct AllPioStateMachines {
    state_machines: [Box<dyn AnyPioStateMachine>; NUM_PIO]
}
impl AllPioStateMachines{
    #[allow(clippy::too_many_arguments)]
    pub fn new(p0sm0: PioStateMachine<PIO0,SM0>, p0sm1: PioStateMachine<PIO0,SM1>, p0sm2: PioStateMachine<PIO0,SM2>, p0sm3: PioStateMachine<PIO0,SM3>,
        p1sm0: PioStateMachine<PIO1,SM0>, p1sm1: PioStateMachine<PIO1,SM1>, p1sm2: PioStateMachine<PIO1,SM2>, p1sm3: PioStateMachine<PIO1,SM3>) -> Self {
        AllPioStateMachines{state_machines: [
            Box::new(p0sm0), Box::new(p0sm1), Box::new(p0sm2), Box::new(p0sm3), 
            Box::new(p1sm0), Box::new(p1sm1), Box::new(p1sm2), Box::new(p1sm3)]}
    }
    /// Pauses execution of all state machines
    pub fn pause_all(&mut self) {
        for sm in self.state_machines.iter_mut() {
            sm.pause();
        }
    }
    /// Start stopped state machines and restarts execution from `wrap_target`
    pub fn restart_all(&mut self) {
        for sm in self.state_machines.iter_mut() {
            sm.restart();
        }
    }
    /// Attempt to write to all state machine FIFOs. Each state machine returns true if data was fed into the FIFO, false otherwise
    pub fn write_all(&mut self, val: u32) -> [bool; NUM_PIO] {
        core::array::from_fn(|i| self.state_machines[i].write(val))

    }
    /// Attempt to write from all state machine FIFOs. Some(data) if successful, None if nothing to read
    pub fn read_all(&mut self) -> [Option<u32>; NUM_PIO] {
        core::array::from_fn(|i| self.state_machines[i].read())
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
impl<PIO:PIOExt, SM: StateMachineIndex> AnyPioStateMachine for PioStateMachine<PIO, SM> {
    fn restart(&mut self) {
        let mut state_machine = match self.state_machine.take() {
            PioState::Stopped(sm) => sm.start(),
            PioState::Running(sm) => sm,
            PioState::Taken => unreachable!(),
        };
        state_machine.restart();
        self.state_machine = PioState::Running(state_machine);
    }
    fn pause(&mut self) {
        if let PioState::Running(state_machine) = self.state_machine.take() {
            self.state_machine = PioState::Stopped(state_machine.stop());
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
pub struct PioStateMachine<PIO: PIOExt, SM: StateMachineIndex> {
    state_machine: PioState<PIO, SM>,
    rx: Rx<(PIO, SM)>,
    tx: Tx<(PIO, SM)>,
}
impl<PIO:PIOExt, SM: StateMachineIndex> PioStateMachine<PIO, SM>{
    pub fn new(state_machine: StateMachine<(PIO,SM), Stopped>, rx: Rx<(PIO,SM)>, tx: Tx<(PIO,SM)>) -> Self {
        PioStateMachine{ state_machine: PioState::Stopped(state_machine), rx, tx }
    }
}

enum PioState<PIO:PIOExt, SM: StateMachineIndex>{
    Stopped(StateMachine<(PIO, SM), Stopped>),
    Running(StateMachine<(PIO, SM), Running>),
    Taken, // Instead of requiring wrapping in an Option
}
impl<PIO:PIOExt, SM: StateMachineIndex> PioState<PIO, SM> {
    /// Takes the value out, leaving `Taken` in its place
    fn take(&mut self) -> Self {
        core::mem::replace(self, PioState::Taken)
    }
}
