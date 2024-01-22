#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use cortex_m::peripheral::SCB;
use cortex_m_semihosting::hprintln;
use hal::{uart::{Config, Serial, Tx}, pac::USART1};
use panic_halt as _;
//
//
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;
use cortex_m_rt::{entry, pre_init};
use crate::hal::{pac, prelude::*};
use core::fmt::Write;
use cortex_m_semihosting::syscall;

#[pre_init]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
}

#[entry]
fn main() -> ! {
    if let Some(dp) = 
        pac::Peripherals::take()
     {
         let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();
        let gpiob = dp.GPIOB.split();
        let mut led = gpiob.pb1.into_push_pull_output();
        // Create a delay abstraction based on SysTick
        let mut delay = dp.TIM5.delay_us(&clocks);

        
        let tx_pin = gpiob.pb6.into_alternate();
        let mut tx = dp
        .USART1
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();
        
        
        writeln!(tx, "wooooow\r").unwrap();

        loop {
            led.set_high();
            delay.delay_ms(300);
            led.set_low();
            delay.delay_ms(300);
        }
    }
    loop {
    } 

}
