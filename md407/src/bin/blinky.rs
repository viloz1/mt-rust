#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use cortex_m::peripheral::SCB;
use cortex_m_rt::pre_init;
use md407 as _;


#[pre_init]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
}

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4],
    peripherals = true

)]
mod app {

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
        // TODO: Add resources
        delay: Delay<TIM5, 1000000>,
        usart: Tx<USART1>,
    }

    // Local resources go here
    #[local]
    struct Local {
        red_led: Pin<'B', 1, Output>,
        green_led: Pin<'B', 0, Output>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();
        let gpiob = dp.GPIOB.split();
        let red_led = gpiob.pb1.into_push_pull_output();
        let green_led = gpiob.pb0.into_push_pull_output();
        // Create a delay abstraction based on SysTick
        let delay = dp.TIM5.delay_us(&clocks);

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

        (
            Shared {
                usart: tx,
                delay,
                // Initialization of shared resources go here
            },
            Local {
                red_led,
                green_led

            },
            init::Monotonics(mono)

        )
    }

    // TODO: Add tasks

    #[task(local = [red_led], shared = [delay, usart])]
    fn toggle_red_led(mut ctx: toggle_red_led::Context) {
        ctx.shared.usart.lock(|usart| { 
            writeln!(usart,"red started\r\0").unwrap();
        });
            
        ctx.local.red_led.toggle();
             
        toggle_red_led::spawn_after((1 as u64).secs()).unwrap();
    }

    #[task(local = [green_led], shared = [delay, usart])]
    fn toggle_green_led(mut ctx: toggle_green_led::Context) {
        ctx.shared.usart.lock(|usart| { 
            writeln!(usart,"green started\r").unwrap();
        });
            
        ctx.local.green_led.toggle();

        toggle_green_led::spawn_after((1 as u64).secs()).unwrap();
    }
}

