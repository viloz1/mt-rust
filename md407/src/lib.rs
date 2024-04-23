#![no_main]
#![no_std]

use cortex_m_rt::pre_init;
use fugit::MicrosDurationU32;
use hal::{gpio::{Alternate, Pin}, pac::{SCB, TIM2, USART1}, rcc::Clocks, rng::Rng, serial::SerialExt, timer::CounterUs, uart::{Config, Event, Serial}, Listen};
use rand_core::RngCore;
use panic_halt as _;
pub use stm32f4xx_hal as hal; // memory layout
use stm32f4xx_hal::time::U32Ext;

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
    }
}

pub const ADDRESS: *mut u32 = 0x2000_0000 as *mut u32;

#[pre_init]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
    let test = cortex_m::register::msp::read();
    core::ptr::write_volatile(ADDRESS, test);
}

pub fn get_stack() -> usize {
    let start_stack: usize;
    unsafe {
        let value = core::ptr::read_volatile(ADDRESS);
        start_stack = value as usize;
    }
    start_stack
}

pub fn tick(current: &mut u32) {
    let stack_end = cortex_m::register::msp::read();

    if stack_end < *current {
        *current = stack_end;
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

pub fn ticks_to_secs(ticks: u32, freq: u32) -> f32 {
    (ticks / freq) as f32
}

pub fn setup_usart(usart: USART1, tx_pin: Pin<'A',9, Alternate<7>> , rx_pin:  Pin<'A',10, Alternate<7>>, clocks: Clocks) -> Serial<USART1> {
    let mut serial = usart.serial((tx_pin, rx_pin), Config::default()
        .baudrate(115200_u32.bps())
        .wordlength_8()
        .stopbits(hal::uart::config::StopBits::STOP1)
        .parity_none(), &clocks).unwrap().with_u8_data();
    serial.listen(Event::RxNotEmpty);
    serial
}

pub fn time_us_64(tim2: &CounterUs<TIM2>) -> u64 {
    MicrosDurationU32::from_ticks(tim2.now().ticks()).to_micros().into()
}

pub fn reset(tim2: &CounterUs<TIM2>) {
    let a = time_us_64(tim2);
    while  a + 500_000 > time_us_64(tim2) {}
    SCB::sys_reset();
}


