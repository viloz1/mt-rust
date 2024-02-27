#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

use rp2040_hal::rosc::{RingOscillator, Enabled};
use rp_pico::hal as _; // memory layout

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]

fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

pub fn get_random_byte(rosc: &RingOscillator<Enabled>) -> u8 {
    let mut num: u8 = 0;
    for i in 0..8 {
        if rosc.get_random_bit() {
            num += 1 << (7 - i)
        }
    }
    return num
}

pub fn get_random_u16(rosc: &RingOscillator<Enabled>) -> u16 {
    let mut array: [u8; 2] = [0; 2];
    for i in 0..2 {
        array[i] = get_random_byte(&rosc)    
    }
    return u16::from_le_bytes(array)
}

pub fn get_random_u32(rosc: &RingOscillator<Enabled>) -> u32 {
    let mut array: [u8; 4] = [0; 4];
    for i in 0..4 {
        array[i] = get_random_byte(&rosc)    
    }
    return u32::from_le_bytes(array)
}

pub fn get_random_u64(rosc: &RingOscillator<Enabled>) -> u64 {
    let mut array: [u8; 8] = [0; 8];
    for i in 0..8 {
        array[i] = get_random_byte(&rosc)    
    }
    return u64::from_le_bytes(array)
}

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}