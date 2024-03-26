#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use md407 as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [USART6, USART2, USART3],
    peripherals = true

)]
mod app {
    use hal::timer::CounterUs;
    use md407::{hal, setup_usart, time_us_64};
    use stm32f4xx_hal::prelude::*;
    use systick_monotonic::Systick;
    use hal::pac::{USART1, TIM2};
    use core::fmt::Write;

    use hal::uart::Serial;
    // Shared resources go here
    #[shared]
    struct Shared {
        shared_num: u16,
        timer: CounterUs<TIM2>,
        done: bool,
        usart: Serial<USART1>,
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        let gpioa = dp.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);

        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);
        for _ in 0..10 {
            low_priority_task::spawn().ok();
            medium_priority_task::spawn().ok();
            high_priority_task::spawn().ok();
        }

        writeln!(serial, "Go\n\r").ok();

        (
            Shared {
                shared_num: 1,
                timer,
                done: false,
                usart: serial,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    // TODO: Add tasks
    //

    #[task(shared = [shared_num, timer, done, usart], priority = 1, capacity = 10)]
    fn low_priority_task(ctx: low_priority_task::Context) {
        let tim2 = ctx.shared.timer;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            writeln!(usart, "low prio").ok();
            increase(num, tim2, done, usart);
        });
        low_priority_task::spawn_after(systick_monotonic::ExtU64::micros(50)).ok();
    }

    #[task(shared = [shared_num, timer, done, usart], priority = 2, capacity = 10)]
    fn medium_priority_task(ctx: medium_priority_task::Context) {
        let tim2 = ctx.shared.timer;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });
        medium_priority_task::spawn_after(systick_monotonic::ExtU64::micros(100)).ok();
    }

    #[task(shared = [shared_num, timer, done, usart], priority = 3, capacity = 10)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.timer;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });
        high_priority_task::spawn_after(systick_monotonic::ExtU64::micros(150)).ok();
    }

    fn increase(
        num: &mut u16,
        tim: &mut CounterUs<TIM2>,
        done: &mut bool,
        usart: &mut Serial<USART1>,
    ) {
        if *num == u16::MAX && !*done {
            let end = time_us_64(tim);
            //writeln!(*usart, "Time: {:?}", end as f64);
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
}
