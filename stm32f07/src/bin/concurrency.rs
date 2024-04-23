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
    use test_app::{get_stack, setup_tim2, tick, time_us};

    type NumberType = u32;
    const LIMIT: NumberType = 100_000;

    // Shared resources go here
    #[shared]
    struct Shared {
        serial: Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,

        shared_num: NumberType,
        tim2: TIM2,
        done: bool,
        largest_stack: u32,

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
        let start_stack = get_stack() as u32;
        let largest_stack = start_stack;
        let mut p = ctx.device;
        let cp = ctx.core;

        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

        

        let mono = Systick::new(cp.SYST, test_app::SYSTICK_FREQ);

        let mut serial: Option<Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>> = None;

        let gpioa = p.GPIOA.split(&mut rcc);
        let _ = cortex_m::interrupt::free(|cs| {
            let tx = gpioa.pa9.into_alternate_af1(cs);
            let rx = gpioa.pa10.into_alternate_af1(cs);

            serial = Some(Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc));
        });

        let real_serial = serial.unwrap();

        reference_task::spawn().ok();
        //low_priority_task::spawn().ok();
        //high_priority_task::spawn().ok();

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 1.mhz());

        (
            Shared {
                serial: real_serial,
                shared_num: 1,
                tim2,
                done: false,
                largest_stack,
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
    fn reference_task(mut ctx: reference_task::Context) {
        let tim2 = &mut ctx.shared.tim2;
        let num = ctx.local.reference_num;
        let usart = &mut ctx.shared.serial;
        let done = ctx.local.reference_done;
        
        loop {
            if *num == LIMIT && !*done {
                (&mut *tim2, &mut *usart).lock(|tim2, usart| {
                    let end = time_us(tim2);
                    writeln!(usart, "{}", end).ok();
                });
                *done = true;
            } else if !*done {
                *num = *num + 1;
            }
        }
        
    }

    #[task(shared = [shared_num, tim2, done, serial, largest_stack], priority = 1)]
    fn low_priority_task(mut ctx: low_priority_task::Context) {
        let tim2 = &mut ctx.shared.tim2;
        let shared_num = &mut ctx.shared.shared_num;
        let done = &mut ctx.shared.done;
        let usart = &mut ctx.shared.serial;
        let l = &mut ctx.shared.largest_stack;

        loop {
            (&mut *shared_num, &mut *tim2, &mut *done, &mut *usart, &mut *l).lock(|num, tim2, done, usart, l| {
                //tick(l);
                increase(num, tim2, done, usart, l);
                //tick(l);
            });
        }
    }

    #[task(shared = [shared_num, tim2, done, serial, largest_stack], priority = 2)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.serial;
        let l = ctx.shared.largest_stack;

        
        (shared_num, tim2, done, usart, l).lock(|num, tim2, done, usart, l| {
            //tick(l);
            increase(num, tim2, done, usart, l);
            //tick(l);
        });

        high_priority_task::spawn_after(systick_monotonic::ExtU64::millis(1)).ok();
    }

    fn increase(
        num: &mut NumberType,
        tim: &mut TIM2,
        done: &mut bool,
        usart: &mut Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,
        largest_stack: &mut u32,
    ) {
        if *num == LIMIT && !*done {
            let end = time_us(tim);
            //tick(largest_stack);
            writeln!(usart, "{}", end).ok();
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
}
