#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m_rt::pre_init;
use embedded_hal::adc::OneShot;
use panic_probe as _;

use stm32f0xx_hal::{self as _, adc::Adc, gpio::{gpioa::PA1, Analog}, pac::TIM2, rcc::Clocks, time::Hertz}; // memory layout

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked

fn panic() -> ! {
    cortex_m::asm::udf()
}


/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub const SYSTICK_FREQ: u32 = 48_000_000;
pub const ADDRESS: *mut u32 = 0x2000_0000 as *mut u32;

#[pre_init]
unsafe fn startup() {
    let test = cortex_m::register::msp::read();
    core::ptr::write_volatile(ADDRESS, test);
}

pub fn get_random_byte(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u8 {
    let mut result = 0;
    for i in 0..8 {
        let value: u16 = adc.read(in_pin).unwrap();
        if (value % 10) < 5 {
            result += 1 << (7 - i)
        }
    }
    result
}

pub fn get_random_u16(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u16 {
    let mut array: [u8; 2] = [0; 2];
    for i in 0..2 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    u16::from_le_bytes(array)
}

pub fn get_random_u32(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u32 {
    let mut array: [u8; 4] = [0; 4];
    for i in 0..4 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    u32::from_le_bytes(array)
}

pub fn get_random_u64(adc: &mut Adc, in_pin: &mut PA1<Analog>) -> u64 {
    let mut array: [u8; 8] = [0; 8];
    for i in 0..8 {
        array[i] = get_random_byte(adc, in_pin);    
    }
    u64::from_le_bytes(array)
}

pub fn setup_tim2<T>(tim2: &TIM2, clocks: &Clocks, timeout: T) where T: Into<Hertz> {
    tim2.cnt.reset();
    let tclk = if clocks.hclk().0 == clocks.pclk().0 {
        clocks.pclk().0
    } else {
        clocks.pclk().0 * 2
    };
    let frequency = timeout.into().0;
    let ticks = tclk / frequency;

    let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
    tim2.psc.write(|w| w.psc().bits(psc));

    tim2.cr1.modify(|_, w| w.cen().set_bit());
}

pub fn time_us(tim2: &TIM2) -> u64 {
    (tim2.cnt.read().bits() as u64 * 1_000_000) / SYSTICK_FREQ as u64
}

pub fn tick(current: &mut u32) {
    let stack_end = cortex_m::register::msp::read();

    if stack_end < *current {
        *current = stack_end;
    }
}

pub fn get_stack() -> usize {
    let start_stack: usize;
    unsafe {
        let value = core::ptr::read_volatile(ADDRESS);
        start_stack = value as usize;
    }
    start_stack
}
