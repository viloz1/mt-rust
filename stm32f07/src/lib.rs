#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use embedded_hal::adc::OneShot;
use panic_probe as _;

use stm32f0xx_hal::{self as _, adc::Adc, gpio::{gpioa::PA1, Analog}}; // memory layout

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

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub fn get_random_byte(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u8 {
    let mut result = 0;
    for i in 0..8 {
        let value: u16 = adc.read(in_pin).unwrap();
        if (value % 10) < 5 {
            result += 1 << (7 - i)
        }
    }
    return result;
}

pub fn get_random_u16(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u16 {
    let mut array: [u8; 2] = [0; 2];
    for i in 0..2 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    return u16::from_le_bytes(array)
}

pub fn get_random_u32(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u32 {
    let mut array: [u8; 4] = [0; 4];
    for i in 0..4 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    return u32::from_le_bytes(array)
}

pub fn get_random_u64(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u64 {
    let mut array: [u8; 8] = [0; 8];
    for i in 0..8 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    return u64::from_le_bytes(array)
}

