#![no_main]
#![no_std]

use hal::rng::Rng;
use rand_core::RngCore;
use panic_halt as _;

pub use stm32f4xx_hal as hal; // memory layout


/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub fn get_random_byte(rng: &mut Rng) -> u8 {
    let mut buffer: [u8; 1] = [0];
    rng.try_fill_bytes(&mut buffer).ok();
    return buffer[0];
}

pub fn get_random_u16(rng: &mut Rng) -> u16 {
    let mut buffer: [u8; 2] = [0, 0];
    rng.try_fill_bytes(&mut buffer).ok();
    return u16::from_le_bytes(buffer);
}

pub fn get_random_u32(rng: &mut Rng) -> u32 {
    rng.next_u32()
}

pub fn get_random_u64(rng: &mut Rng) -> u64 {
    rng.next_u64()
}

