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
    use md407::hal as hal;

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::{USART1, TIM5};
    use hal::prelude::*;
    use stm32f4xx_hal::timer::Delay;
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
        dac: hal::dac::C2

    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let rcc_uninit = dp.RCC;
        

        let rcc: hal::rcc::Rcc = rcc_uninit.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();



        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let red_led = gpiob.pb1.into_push_pull_output();
        let green_led = gpiob.pb0.into_push_pull_output();

        let pa5 = gpioa.pa5.into_analog();
        let mut dac: hal::dac::C2 = dp.DAC.constrain(pa5);
        dac.enable();
        

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
              dac

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
     
    #[task(local = [dac], shared = [usart])]
    fn generate_sound(ctx: generate_sound::Context) {
        
        let dac = ctx.local.dac;
        
        let current = dac.get_value();
        if current > 0 {
            
            dac.set_value(7);
        } else {
            dac.set_value(0);
        }

        generate_sound::spawn_after((803 as u64).micros()).unwrap();
             
    }
    


}
