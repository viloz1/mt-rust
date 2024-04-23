#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use md407 as _;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3],
    peripherals = true

)]
mod app {
    use cortex_m::register::msp;
    use hal::rng::Rng;
    use hal::timer::CounterUs;
    
    use hal::uart::Serial;
    use md407::{get_random_byte, hal as hal, setup_usart, tick, time_us_64};

    use stm32f4xx_hal::gpio::Pin;
    use hal::pac::{DAC, TIM2, USART1};
    use hal::prelude::*;
    use systick_monotonic::*;
    use core::fmt::Write;

    const BACKGROUND_TASKS: usize = 50;


    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        usart: Serial<USART1>,
        rng: Rng,
        button: Pin<'B', 7>,
        timer: CounterUs<TIM2>,
        sleep_time: u64,
        interrupt_start_time: u64,
        #[lock_free]
        largest_stack: u32
    }

    // Local resources go here
    #[local]
    struct Local {
        background_tasks: u64,
        last_timer_value: u64,
        
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity
    

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let cp = ctx.core;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.require_pll48clk().sysclk(168.MHz()).pclk1(8.MHz()).use_hse(25.MHz()).freeze();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
       
        // Create a delay abstraction based on SysTick
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;

        let mut button = gpiob.pb7.into_pull_down_input();
        button.make_interrupt_source(&mut syscfg);
        button.trigger_on_edge(&mut exti, hal::gpio::Edge::Rising);
        button.enable_interrupt(&mut exti);
        
        unsafe {
            cortex_m::peripheral::NVIC::unmask(button.interrupt());
        }

        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        
        let serial = setup_usart(usart1, tx_pin, rx_pin, clocks);
        
        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }
        
        issue_interrupt::spawn_after((1 as u64).secs()).ok();
        for _ in 0..BACKGROUND_TASKS {
            background_task::spawn().ok();
        }

        let rng = dp.RNG.constrain(&clocks);

        (
            Shared {
                usart: serial,
                rng,
                button,
                timer,
                sleep_time: 0,
                interrupt_start_time: 0,
                largest_stack: msp::read()
            },
            Local {
                background_tasks: 0,
                last_timer_value: 0,
                
            },
            init::Monotonics(mono)

        )
    }

    #[task(priority = 1, shared = [rng, timer], capacity = 50)]
    fn background_task(mut ctx: background_task::Context) {
        let mut timer = ctx.shared.timer;

        let start_time = timer.lock(|timer| {
            return time_us_64(timer);
        });

        let (sleep_time, spawn_after) = (ctx.shared.rng).lock(|rng| {
            return (get_random_byte(rng) % 10, get_random_byte(rng) % 10);
        });

        loop {
            let current_time = timer.lock(|timer| {
                time_us_64(timer)
            });
            if current_time - start_time > (sleep_time as u64) * 1_000  {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u64).millis());
    }

    #[task(priority = 2, shared = [button, timer, interrupt_start_time, largest_stack])]
    fn issue_interrupt(ctx: issue_interrupt::Context) {
        let button = ctx.shared.button;
        let timer = ctx.shared.timer;
        let start_time = ctx.shared.interrupt_start_time;
        
        issue_interrupt::spawn_after((1 as u64).secs()).ok();
        (button, timer, start_time).lock(|button, timer, start_time| {
            //tick(ctx.shared.largest_stack);
            cortex_m::peripheral::NVIC::pend(button.interrupt());
            *start_time = time_us_64(timer);
        });
    }

    #[task(
        priority = 2, 
        binds = EXTI9_5, 
        shared = [timer, button, usart, interrupt_start_time, sleep_time, largest_stack], 
        local = [background_tasks, last_timer_value]
    )]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        let end_time = ctx.shared.timer.lock(|timer| {
            time_us_64(timer)
        });

        //tick(ctx.shared.largest_stack);

        let start_time = ctx.shared.interrupt_start_time.lock(|start_time| {
            *start_time
        }); 

        let differnece = end_time - start_time;

        //tick(ctx.shared.largest_stack);

        ctx.shared.usart.lock(|usart| {
            writeln!(usart, "{}", differnece).ok();
            //writeln!(usart, "{:08x}", *ctx.shared.largest_stack).ok();
        });
        
        ctx.shared.button.lock(|button| {
            button.clear_interrupt_pending_bit();
        });
    }
}

