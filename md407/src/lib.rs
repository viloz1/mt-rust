#![no_main]
#![no_std]

use panic_halt as _;

pub use stm32f4xx_hal as hal; // memory layout


/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
