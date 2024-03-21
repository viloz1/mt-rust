#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use cortex_m::peripheral::SCB;
use cortex_m_rt::pre_init;
use md407 as _;
use stm32f4xx_hal::{pac::RCC, rcc};


#[pre_init]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
    (*RCC::PTR).apb1enr.modify(|_, w| w.dacen().set_bit());
}

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4],
    peripherals = true

)]
mod app {

    use hal::dac::DacOut;
    use hal::dac::DacPin;
    use hal::dac::Pins;
    use hal::gpio::Analog;
    use hal::gpio::PA4;
    use hal::gpio::PA5;
    use hal::rcc::{Enable, Reset};
    use hal::pac::DAC;
    use md407::hal as hal;

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::USART1;
    use hal::prelude::*;
    use stm32f4xx_hal::uart::{Config, Tx};
    use systick_monotonic::*;
    use core::fmt::Write;
    // Shared resources go here
    #[shared]
    struct Shared {

        usart: Tx<USART1>,
    }

    // Local resources go here
    #[local]
    struct Local {
        red_led: Pin<'B', 1, Output>,
        green_led: Pin<'B', 0, Output>,
        peak: bool

    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let rcc_uninit = dp.RCC;
        

        let rcc: hal::rcc::Rcc = rcc_uninit.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();



        let gpiob = dp.GPIOB.split();
        let gpioa = dp.GPIOA.split();
        let red_led = gpiob.pb1.into_push_pull_output();
        let green_led = gpiob.pb0.into_push_pull_output();

        unsafe{DAC::enable_unchecked(); DAC::reset_unchecked()};

        let pa5: PA5<Analog> = gpioa.pa5.into_analog();
        PA5::init();

        //Enable buffer
        dp.DAC.cr.modify(|_, w| w.boff2().set_bit());

        //Disable DAC trigger
        dp.DAC.cr.modify(|_, w| w.ten2().clear_bit());

        //Disable wave generation
        dp.DAC.cr.modify(|_, w| w.wave2().disabled());
        
        //Send enable command
        dp.DAC.cr.modify(|_, w| w.dmaen2().set_bit());

        dp.DAC.dhr8r2.modify(|_, w| w.dacc2dhr().bits(0));


    

        dp.DAC.cr.modify(|_, w| w.en2().set_bit()); 
        

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
        
        let systick = ctx.core.SYST;
        let mono = Systick::new(systick, 168_000_000);
        
        writeln!(tx, "\rwooooow lets gooo\r").unwrap();
        toggle_red_led::spawn().unwrap();
        toggle_green_led::spawn_after((1 as u64).secs()).unwrap();
        generate_sound::spawn().unwrap();

        (
            Shared {
                usart: tx,
              
                // Initialization of shared resources go here
            },
            Local {
              red_led,
              green_led,
                peak: false

            },
            init::Monotonics(mono)

        )
    }

    #[task(local = [red_led], shared = [usart])]
    fn toggle_red_led(mut ctx: toggle_red_led::Context) {
        ctx.shared.usart.lock(|usart| { 
            writeln!(usart,"red started\r\0").unwrap();
        });
            
        ctx.local.red_led.toggle();
             
        toggle_red_led::spawn_after((1 as u64).secs()).unwrap();
    }

    #[task(local = [green_led], shared = [usart])]
    fn toggle_green_led(mut ctx: toggle_green_led::Context) {
        ctx.shared.usart.lock(|usart| { 
            writeln!(usart,"green started\r").unwrap();
        });
            
        ctx.local.green_led.toggle();

        toggle_green_led::spawn_after((1 as u64).secs()).unwrap();
    }

    // TODO: Add tasks
     
    #[task(local = [peak], shared = [usart])]
    fn generate_sound(ctx: generate_sound::Context) {
        
        
        if *ctx.local.peak {
            let dac = unsafe { &(*DAC::ptr()) };
            dac.dhr8r2.write(|w| unsafe { w.bits(15 as u32) });
            *ctx.local.peak = false;
        } else {
            let dac = unsafe { &(*DAC::ptr()) };
            dac.dhr8r2.write(|w| unsafe { w.bits(0 as u32) });
            *ctx.local.peak = true;
        }

        generate_sound::spawn_after((803 as u64).micros()).unwrap();
             
    }
    


}
