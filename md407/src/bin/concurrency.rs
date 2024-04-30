#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use cortex_m_rt::pre_init;
use md407 as _;
use stm32f4xx_hal::pac::SCB;

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3],
    peripherals = true
)]
mod app {
    use cortex_m::register::msp;
    use hal::timer::CounterUs;

    use hal::uart::Serial;
    use md407::{hal, reset, setup_usart, tick, time_us_64};

    use core::fmt::Write;
    use hal::pac::{SCB, TIM2, USART1};
    use hal::prelude::*;
    use systick_monotonic::*;

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    // Shared resources go here
    type NumberType = u32;
    const LIMIT: NumberType = 100_000;
    // Shared resources go here
    #[shared]
    struct Shared {
        shared_num: NumberType,
        timer: CounterUs<TIM2>,
        done: bool,
        usart: Serial<USART1>,
        largest_stack: u32,
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        reference_num: NumberType,
        reference_done: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let largest_stack = msp::read();
        let dp = ctx.device;
        let cp = ctx.core;
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .require_pll48clk()
            .sysclk(168.MHz())
            .pclk1(8.MHz())
            .use_hse(25.MHz())
            .freeze();

        let gpioa = dp.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;

        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);

        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);

        reference_task::spawn().ok();

        //low_priority_task::spawn().ok();
        //high_priority_task::spawn().ok();

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        (
            Shared {
                shared_num: 1,
                timer,
                done: false,
                usart: serial,
                largest_stack
            },
            Local {
                reference_num: 0,
                reference_done: false,
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [shared_num, timer, done, usart], local = [reference_num, reference_done])]
    fn reference_task(mut ctx: reference_task::Context) {
        let tim2 = &mut ctx.shared.timer;
        let num = ctx.local.reference_num;
        let usart = &mut ctx.shared.usart;
        let done = ctx.local.reference_done;
        let start = tim2.lock(|tim| {time_us_64(tim)});
        
        loop {
            if *num == LIMIT && !*done {
                (&mut *tim2, &mut *usart).lock(|tim2, usart| {
                    let end = time_us_64(tim2);
                    writeln!(usart, "{}", end-start).ok();
                    reset(tim2);
                });
                *done = true;
            } else if !*done {
                *num = *num + 1;
            }
        }
        
    }

    #[task(shared = [shared_num, timer, done, usart, largest_stack], priority = 1)]
    fn low_priority_task(mut ctx: low_priority_task::Context) {
        let tim2 = &mut ctx.shared.timer;
        let shared_num = &mut ctx.shared.shared_num;
        let done = &mut ctx.shared.done;
        let usart = &mut ctx.shared.usart;
        let l = &mut ctx.shared.largest_stack;

        loop {
            (&mut *shared_num, &mut *tim2, &mut *done, &mut *usart, &mut *l).lock(|num, tim, done, usart, l| {
                //tick(l);
                increase(num, tim, done, usart, l);
                //tick(l);
            });
        }
    }

    #[task(shared = [shared_num, timer, done, usart, largest_stack], priority = 2)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.timer;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;
        let l = ctx.shared.largest_stack;

        (shared_num, tim2, done, usart, l).lock(|num, tim2, done, usart, l| {
            //tick(l);
            increase(num, tim2, done, usart, l);
            //tick(l);
        });
        low_priority_task::spawn_after((1 as u64).millis()).ok();
    }

    fn increase(
        num: &mut NumberType,
        tim: &mut CounterUs<TIM2>,
        done: &mut bool,
        usart: &mut Serial<USART1>,
        largest_stack: &mut u32,
    ) {
        let old = *num;
        if old == LIMIT && !*done {
            let end = time_us_64(tim);
            writeln!(*usart, "{:08x}", largest_stack).ok();
            //writeln!(*usart, "{}", end).ok();
            *done = true;
            reset(tim); 
        }

        if !*done {
            let new_value = old + 1;
            *num = new_value;
        }
    }
}
