#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1, USART2, USART3_4_5_6_7_8],
    peripherals = true

)]
mod app {
    use stm32f0xx_hal::pac::TIM2;
    use systick_monotonic::Systick;

    use stm32f0xx_hal::prelude::*;
    use test_app::{setup_tim2, time_us};

    // Shared resources go here
    #[shared]
    struct Shared {
        shared_num: u16,
        tim2: TIM2,
        done: bool,
    }

    // Local resources go here
    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        //defmt::info!("init");
        let mut p = ctx.device;
        let cp = ctx.core;

        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

        let rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 1.mhz());

        let mono = Systick::new(cp.SYST, test_app::SYSTICK_FREQ);

        for _ in 0..10 {
            low_priority_task::spawn().ok();
            medium_priority_task::spawn().ok();
            high_priority_task::spawn().ok();
        }

        (
            Shared {
                shared_num: 1,
                tim2,
                done: false,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    // TODO: Add tasks
    //

    #[task(shared = [shared_num, tim2, done], priority = 1, capacity = 10)]
    fn low_priority_task(ctx: low_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;

        (shared_num, tim2, done).lock(|num, tim2, done| {
            increase(num, tim2, done);
        });

        low_priority_task::spawn_after(systick_monotonic::ExtU64::micros(50)).ok();
    }

    #[task(shared = [shared_num, tim2, done], priority = 2, capacity = 100)]
    fn medium_priority_task(ctx: medium_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;

        (shared_num, tim2, done).lock(|num, tim2, done| {
            increase(num, tim2, done);
        });

        medium_priority_task::spawn_after(systick_monotonic::ExtU64::micros(100)).ok();
    }

    #[task(shared = [shared_num, tim2, done], priority = 3, capacity = 10)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;

        (shared_num, tim2, done).lock(|num, tim2, done| {
            increase(num, tim2, done);
        });

        high_priority_task::spawn_after(systick_monotonic::ExtU64::micros(150)).ok();
    }

    fn increase(num: &mut u16, tim: &mut TIM2, done: &mut bool) {
        if *num == u16::MAX && !*done {
            let end = time_us(tim);
            //defmt::info!("MAX: {}", end);
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
}
