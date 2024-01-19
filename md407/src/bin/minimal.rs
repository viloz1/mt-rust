#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [USART1],
    peripherals = true

)]
mod app {
    use stm32f4xx_hal::pac::{Interrupt, EXTI};
    use stm32f4xx_hal::prelude::*; 

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        button: EXTI,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");
        let mut p = ctx.device;
        let cp = ctx.core;
        let mut rcc = p.RCC.constrain();
        
        let gpioa = p.GPIOA.split();
        let syscfg = p.SYSCFG;
        let exti = p.EXTI;
        let button = gpioa.pa0;
        
        

        cortex_m::interrupt::free(|cs| {
            button.into_pull_down_input();
            syscfg.exticr1.modify(|_, w| unsafe { w.exti0().bits(1) });

            // Set interrupt request mask for line 1
            exti.imr.modify(|_, w| w.mr0().set_bit());

            // Set interrupt rising trigger for line 1
            exti.rtsr.modify(|_, w| w.tr0().set_bit());
            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::EXTI0, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI0);
        });

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                button: exti,

            },
            init::Monotonics()

        )
    }

    // TODO: Add tasks

    #[task(binds = EXTI0, local = [button])]
    fn toggle_task(ctx: toggle_task::Context) {
        defmt::info!("button pressed");
        cortex_m::interrupt::free(|_cs| { 
           ctx.local.button.pr.write(|w| w.pr0().set_bit()); 
        });
    }
}
