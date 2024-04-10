#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1, USART2],
    peripherals = true

)]
mod app {
    use stm32f0xx_hal::adc::Adc;
    use stm32f0xx_hal::gpio::Analog;
    use stm32f0xx_hal::pac::{Interrupt, EXTI, TIM2};
    use stm32f0xx_hal::prelude::*;
    use systick_monotonic::{
    fugit::ExtU32,
    Systick,
};
    use test_app::{setup_tim2, SYSTICK_FREQ, time_us, get_random_byte};
    
    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        button: EXTI,
        tim2: TIM2,
        sleep_time: u64,
        interrupt_start_time: u64
    }


    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
        adc: Adc,
        an_in: stm32f0xx_hal::gpio::gpioa::PA1<Analog>,
        background_tasks: u8
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);
        
        let gpioa = p.GPIOA.split(&mut rcc);
        let syscfg = p.SYSCFG;
        let exti = p.EXTI;
        let button = gpioa.pa0;

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 2.mhz());

        let mono = Systick::new(cp.SYST, SYSTICK_FREQ);

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

        trigger_interrupt::spawn().ok();

        (
            Shared {
                button: exti,
                tim2,
                interrupt_start_time: 0,
                sleep_time: 0
            },
            Local {
                last_timer_value: 0,
                adc,
                an_in: an_in.unwrap(),
                background_tasks: 0
            },
            init::Monotonics(mono)

        )
    }

    #[idle(shared = [sleep_time, tim2])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
           cortex_m::interrupt::free(|_cs| {
                let start_time = ctx.shared.tim2.lock(|tim| {
                    time_us(tim)
                });

                rtic::export::wfi();

                let end_time =ctx.shared.tim2.lock(|tim| {
                    time_us(tim)
                });

                let sleep_time = end_time - start_time;

                 ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }
    

    #[task(priority = 2, shared = [button, tim2, interrupt_start_time])]
    fn trigger_interrupt(mut ctx: trigger_interrupt::Context) {
        ctx.shared.button.lock(| button | {
            let start_time = ctx.shared.interrupt_start_time;
            let tim2 = ctx.shared.tim2;
            (tim2, start_time).lock(|tim2, start_time| {
                cortex_m::interrupt::free(|_cs| {
                    button.swier.write(|w| w.swier0().set_bit());
                    *start_time = time_us(tim2)
                });
            });
            
        });
        trigger_interrupt::spawn_after(3.secs().into()).ok();
       

    }

    #[task(priority = 1, local = [adc, an_in], shared = [tim2], capacity = 10)]
    fn background_task(mut ctx: background_task::Context) {
        let start = ctx.shared.tim2.lock(|tim| {
            time_us(tim)
        });

        let sleep_time = get_random_byte(ctx.local.adc, ctx.local.an_in) % 10;
        let spawn_after = get_random_byte(ctx.local.adc, ctx.local.an_in) % 10;
        loop {
            let current_time = ctx.shared.tim2.lock(|tim| {
                time_us(tim)
            });
            if current_time - start > (sleep_time as u64) * 1_000_000 {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u32).secs().into());
    }


    #[task(binds = EXTI0_1, shared = [button, tim2, interrupt_start_time, sleep_time], local = [background_tasks, last_timer_value])]
    fn toggle_task(mut ctx: toggle_task::Context) {    
        let end_time = ctx.shared.tim2.lock(|tim| {
            time_us(tim)
        });

        ctx.shared.button.lock(| button | {
            cortex_m::interrupt::free(|_cs| { 
                button.pr.write(|w| w.pr0().set_bit()); 
           });
        });

        let start_time = ctx.shared.interrupt_start_time.lock(|start_time| {
            *start_time
        });

        let difference = end_time - start_time;

        let cpu_usage = (ctx.shared.sleep_time).lock(|sleep_time| {
            let last_timer_value = *ctx.local.last_timer_value;
            let current_timer_value = end_time;
            let elapsed_time = current_timer_value - last_timer_value;

            let cpu_usage = (elapsed_time as f64 - *sleep_time as f64) / (elapsed_time as f64);
            *sleep_time = 0;
            
            *ctx.local.last_timer_value = end_time;
            cpu_usage

        });
        //defmt::info!("Interrupt time: {}us, background tasks: {}, CPU usage: {}", difference, ctx.local.background_tasks, cpu_usage * 100 as f64);
        *ctx.local.background_tasks = *ctx.local.background_tasks + 1;
        background_task::spawn().ok();
    }
}
