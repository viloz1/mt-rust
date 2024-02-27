#![no_main]
#![no_std]

use hal::{rng::Rng, gpio::{Alternate, Pin}, pac::USART1, uart::{Serial, Config, Event}, rcc::Clocks, serial::SerialExt, Listen};
use rand_core::RngCore;
use panic_halt as _;
pub use stm32f4xx_hal as hal; // memory layout
use stm32f4xx_hal::time::U32Ext;

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

