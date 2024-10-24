#![no_std]
#![no_main]
use rp_pico::entry;
use panic_halt as _;

#[entry]
fn main() -> ! {
  loop {}
}
