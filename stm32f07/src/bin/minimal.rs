#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1],
    peripherals = true

)]
mod app {
    use stm32f0xx_hal::adc::Adc;
    use stm32f0xx_hal::delay::Delay;
    use stm32f0xx_hal::gpio::Analog;
    use stm32f0xx_hal::gpio::gpioa::PA1;
    use stm32f0xx_hal::pac::{Interrupt, EXTI};
    use stm32f0xx_hal::prelude::*;
    use test_app::{get_random_byte, get_random_u64, setup_tim2}; 
    
    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        button: EXTI,
        delay: Delay,
        adc: Adc,
        an_in: PA1<Analog> 
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");
        let mut p = ctx.device;
        let cp = ctx.core;
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
        
        let gpioa = p.GPIOA.split(&mut rcc);
        let syscfg = p.SYSCFG;
        let exti = p.EXTI;
        let button = gpioa.pa0;

        let delay = Delay::new(cp.SYST, &rcc);
        let adc = Adc::new(p.ADC, &mut rcc);
        let mut an_in = None;
        
        cortex_m::interrupt::free(|cs| {
            an_in = Some(gpioa.pa1.into_analog(cs));

            button.into_pull_down_input(cs);
            syscfg.exticr1.modify(|_, w| unsafe { w.exti0().bits(1) });

            // Set interrupt request mask for line 1
            exti.imr.modify(|_, w| w.mr0().set_bit());

            // Set interrupt rising trigger for line 1
            exti.rtsr.modify(|_, w| w.tr0().set_bit());
            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::EXTI0_1, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0_1);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI0_1);
        });

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                button: exti,
                delay,
                adc,
                an_in: an_in.unwrap()

            },
            init::Monotonics()

        )
    }

    // TODO: Add tasks

    #[task(binds = EXTI0_1, local = [button, delay, an_in, adc])]
    fn toggle_task(ctx: toggle_task::Context) {
        let a: u64 = get_random_u64(ctx.local.adc, ctx.local.an_in);
        defmt::info!("button pressed: {}", a);
        cortex_m::interrupt::free(|_cs| { 
           ctx.local.button.pr.write(|w| w.pr0().set_bit()); 
        });
    }
}
