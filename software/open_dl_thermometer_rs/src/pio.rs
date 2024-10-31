use pac::{PIO0, PIO1};
use rp_pico::{hal::pio::{InstalledProgram, PIOExt, Running, Rx, StateMachine, StateMachineGroup4, Stopped, Tx, UninitStateMachine, SM0, SM1, SM2, SM3}, pac};

use crate::pcb_mapping::TempSensePins;

pub struct PioStateMachine<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex> {
    state_machine_on: Option<StateMachine<(PIO, SM), Running>>,
    state_machine_off: Option<StateMachine<(PIO, SM), Stopped>>,
    pub rx: Rx<(PIO, SM)>,
    pub tx: Tx<(PIO, SM)>,
    state: PioState,
}
impl<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex> PioStateMachine<PIO,SM> {
    pub fn restart(&mut self) {
        if self.state == PioState::Stopped {
            self.state_machine_on = Some(self.state_machine_off.take().unwrap().start());
            self.state = PioState::Running;
        }
        self.state_machine_on.as_mut().unwrap().restart();
    }
    pub fn resume(&mut self) {
        if self.state == PioState::Stopped {
            self.state_machine_on = Some(self.state_machine_off.take().unwrap().start());
            self.state = PioState::Running;
        }
    }
    pub fn pause(&mut self) {
        if self.state == PioState::Running {
            self.state_machine_off = Some(self.state_machine_on.take().unwrap().stop());
            self.state = PioState::Stopped;
        }
    }
    pub fn toggle(&mut self) {
        match self.state {
            PioState::Stopped => self.resume(),
            PioState::Running => self.pause(),
        }
    }
}

pub struct PioStateMachines {
    pub p0sm0: PioStateMachine<PIO0, SM0>,
    pub p0sm1: PioStateMachine<PIO0, SM1>,
    pub p0sm2: PioStateMachine<PIO0, SM2>,
    pub p0sm3: PioStateMachine<PIO0, SM3>,
    pub p1sm0: PioStateMachine<PIO1, SM0>,
    pub p1sm1: PioStateMachine<PIO1, SM1>,
    pub p1sm2: PioStateMachine<PIO1, SM2>,
    pub p1sm3: PioStateMachine<PIO1, SM3>,
}
impl PioStateMachines{
    pub fn resume(&mut self) {
        self.p0sm0.resume();
        self.p0sm1.resume();
        self.p0sm2.resume();
        self.p0sm3.resume();
        self.p1sm0.resume();
        self.p1sm1.resume();
        self.p1sm2.resume();
        self.p1sm3.resume();
    }
    pub fn pause(&mut self) {
        self.p0sm0.pause();
        self.p0sm1.pause();
        self.p0sm2.pause();
        self.p0sm3.pause();
        self.p1sm0.pause();
        self.p1sm1.pause();
        self.p1sm2.pause();
        self.p1sm3.pause();
    }
    pub fn toggle(&mut self) {
        self.p0sm0.toggle();
        self.p0sm1.toggle();
        self.p0sm2.toggle();
        self.p0sm3.toggle();
        self.p1sm0.toggle();
        self.p1sm1.toggle();
        self.p1sm2.toggle();
        self.p1sm3.toggle();
    }
    pub fn restart(&mut self) {
        self.p0sm0.restart();
        self.p0sm1.restart();
        self.p0sm2.restart();
        self.p0sm3.restart();
        self.p1sm0.restart();
        self.p1sm1.restart();
        self.p1sm2.restart();
        self.p1sm3.restart();
    }
}
#[derive(PartialEq)]
enum PioState{
    Stopped,
    Running,
}

pub fn configure_pios(pio0: pac::PIO0, pio1: pac::PIO1, resets: &mut pac::RESETS, sense_pins: &TempSensePins) -> PioStateMachines {
    let program = pio_proc::pio_file!("src/lmt01_pulse_count.pio");
    
    let (mut pio0, p0sm0, p0sm1, p0sm2, p0sm3) = pio0.split(resets);
    let (mut pio1, p1sm0, p1sm1, p1sm2, p1sm3) = pio1.split(resets);
    let installed0 = pio0.install(&program.program).unwrap();
    let installed1 = pio1.install(&program.program).unwrap();

    let p0sm0 = configure_sm(p0sm0, sense_pins.vn1.id().num, unsafe{installed0.share()});
    let p0sm1 = configure_sm(p0sm1, sense_pins.vn2.id().num, unsafe{installed0.share()});
    let p0sm2 = configure_sm(p0sm2, sense_pins.vn3.id().num, unsafe{installed0.share()});
    let p0sm3 = configure_sm(p0sm3, sense_pins.vn4.id().num, unsafe{installed0.share()});

    let p1sm0 = configure_sm(p1sm0, sense_pins.vn5.id().num, unsafe{installed1.share()});
    let p1sm1 = configure_sm(p1sm1, sense_pins.vn6.id().num, unsafe{installed1.share()});
    let p1sm2 = configure_sm(p1sm2, sense_pins.vn7.id().num, unsafe{installed1.share()});
    let p1sm3 = configure_sm(p1sm3, sense_pins.vn8.id().num, unsafe{installed1.share()});

    PioStateMachines {p0sm0, p0sm1, p0sm2, p0sm3, p1sm0, p1sm1, p1sm2, p1sm3}
}

fn configure_sm<PIO:rp_pico::hal::pio::PIOExt, SM: rp_pico::hal::pio::StateMachineIndex>(uninit_sm: UninitStateMachine<(PIO,SM)>, id_num: u8, prog: InstalledProgram<PIO>) -> PioStateMachine<PIO,SM> {
    let (mut state_machine, rx, tx) = rp_pico::hal::pio::PIOBuilder::from_installed_program(prog)
        .set_pins(id_num, 1)
        .build(uninit_sm);
    // The GPIO pin needs to be configured as an input
    state_machine.set_pindirs([(id_num, rp_pico::hal::pio::PinDir::Input)]);
    let state_machine_on = None;
    let state_machine_off = Some(state_machine);
    PioStateMachine{state_machine_on, state_machine_off, rx, tx, state: PioState::Stopped}
}