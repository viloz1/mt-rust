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
    use stm32f0xx_hal::{
        gpio::{
            gpioa::{PA10, PA9},
            Alternate, AF1,
        },
        pac::{TIM2, USART1},
        serial::Serial,
    };
    use systick_monotonic::Systick;

    use core::fmt::Write;
    use stm32f0xx_hal::prelude::*;
    use test_app::{setup_tim2, time_us};

    type NumberType = u32;
    const LIMIT: NumberType = 1_000;

    // Shared resources go here
    #[shared]
    struct Shared {
        serial: Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,

        shared_num: NumberType,
        tim2: TIM2,
        done: bool,
    }

    // Local resources go here
    #[local]
    struct Local {
        reference_num: NumberType,
        reference_done: bool,
    }

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

        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 1.mhz());

        let mono = Systick::new(cp.SYST, test_app::SYSTICK_FREQ);

        let mut serial: Option<Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>> = None;

        let gpioa = p.GPIOA.split(&mut rcc);
        let _ = cortex_m::interrupt::free(|cs| {
            let tx = gpioa.pa9.into_alternate_af1(cs);
            let rx = gpioa.pa10.into_alternate_af1(cs);

            serial = Some(Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc));
        });

        let mut real_serial = serial.unwrap();

        reference_task::spawn().ok();
        //low_priority_task::spawn().ok();
        //high_priority_task::spawn().ok();
        writeln!(real_serial, "init").ok();

        (
            Shared {
                serial: real_serial,
                shared_num: 1,
                tim2,
                done: false,
            },
            Local {
                reference_num: 0,
                reference_done: false,
            },
            init::Monotonics(mono),
        )
    }

    // TODO: Add tasks
    //
    #[task(shared = [tim2, serial], local = [reference_num, reference_done], priority = 1)]
    fn reference_task(ctx: reference_task::Context) {
        let tim2 = ctx.shared.tim2;
        let num = ctx.local.reference_num;
        let usart = ctx.shared.serial;
        let done = ctx.local.reference_done;

        if *num == LIMIT && !*done {
            (tim2, usart).lock(|tim2, usart| {
                let end = time_us(tim2);
                writeln!(usart, "{}", end).ok();
            });
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }

        reference_task::spawn().ok();
    }

    #[task(shared = [shared_num, tim2, done, serial], priority = 1)]
    fn low_priority_task(ctx: low_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.serial;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });
    }

    #[task(shared = [shared_num, tim2, done, serial], priority = 2)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.serial;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });

        high_priority_task::spawn_after(systick_monotonic::ExtU64::millis(1)).ok();
    }

    fn increase(
        num: &mut NumberType,
        tim: &mut TIM2,
        done: &mut bool,
        usart: &mut Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,
    ) {
        if *num == LIMIT && !*done {
            let end = time_us(tim);
            writeln!(usart, "{}", end).ok();
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
}
