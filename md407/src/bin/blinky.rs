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

    use cortex_m::interrupt;
    use hal::hal_02::blocking::serial::write;
    use hal::pac::usart1::cr2::STOP_A;
    use hal::uart::{Serial, Event};
    use md407::hal as hal;

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::{USART1, TIM5};
    use hal::prelude::*;
    use stm32f4xx_hal::timer::Delay;
    use stm32f4xx_hal::uart::{Config, Tx};
    use systick_monotonic::*;
    use core::fmt::Write;
    use core::str::from_utf8;
    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        delay: Delay<TIM5, 1000000>,
        usart: Serial<USART1>,
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
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).use_hse(25.MHz()).freeze();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let red_led = gpiob.pb1.into_push_pull_output();
        let green_led = gpiob.pb0.into_push_pull_output();
        // Create a delay abstraction based on SysTick
        let delay = dp.TIM5.delay_us(&clocks);

        let swdio = gpioa.pa13;
        let swclk = gpioa.pa14;

        let _a = swdio.into_alternate();
        let _b = swclk.into_alternate();
        

        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        
        let mut serial = usart1.serial((tx_pin, rx_pin), Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .stopbits(hal::uart::config::StopBits::STOP1)
                .parity_none(), &clocks).unwrap().with_u8_data();
        serial.listen(Event::RxNotEmpty);
        
        let systick = ctx.core.SYST;
        let mono = Systick::new(systick, 168_000_000);
        
        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        toggle_red_led::spawn().unwrap();

        if !serial.is_rx_not_empty() {
            writeln!(serial, "hmm");
        }

        (
            Shared {
                usart: serial,
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
    #[task(binds = USART1, local = [green_led], shared = [delay, usart])]
    fn read_usart(mut ctx: read_usart::Context) {
        ctx.local.green_led.toggle();
        ctx.shared.usart.lock(|usart| {
            while (usart.is_rx_not_empty()) {
                let input = usart.read();
                writeln!(usart, "Input: {:?}", input);

            }
        }); 
    }


    #[task(local = [red_led], shared = [delay, usart])]
    fn toggle_red_led(ctx: toggle_red_led::Context) {
        ctx.local.red_led.toggle();
             
        toggle_red_led::spawn_after((1 as u64).secs()).unwrap();
    }

}

