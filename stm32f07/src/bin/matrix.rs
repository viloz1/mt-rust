#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

extern "C" {
    static mut _stack_start: u32;
}

const A_MATRIX_ROWS: usize = 2;
const SHARED_SIZE: usize = 2;

const A_MATRIX_COLUMNS: usize = SHARED_SIZE;

const B_MATRIX_ROWS: usize = SHARED_SIZE;
const B_MATRIX_COLUMNS: usize = 5;

const RESULT_MATRIX_ROWS: usize = A_MATRIX_ROWS;
const RESULT_MATRIX_COLUMNS: usize = B_MATRIX_COLUMNS;

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1, USART2],
    peripherals = true

)]
mod app {
    use core::fmt::Write;

    use stm32f0xx_hal::gpio::gpioa::{PA10, PA9};
    use stm32f0xx_hal::gpio::{Alternate, AF1};
    use stm32f0xx_hal::prelude::*;
    use stm32f0xx_hal::{pac::TIM2, pac::USART1, serial::Serial};
    use systick_monotonic::Systick;
    use test_app::{get_stack, setup_tim2, tick, time_us, SYSTICK_FREQ};

    use crate::_stack_start;

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        #[lock_free]
        tim2: TIM2,
        #[lock_free]
        concurrent_tasks: u8,
        #[lock_free]
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        #[lock_free]
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        #[lock_free]
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
        #[lock_free]
        serial: Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,
    }

    // Local resources go here
    #[local]
    struct Local {
        start_conc: u64,
        start_stack: u32,
        largest_stack: u32
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let start_stack = get_stack();
        let largest_stack = start_stack;
        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

        
        let mut serial: Option<Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>> = None;

        let gpioa = p.GPIOA.split(&mut rcc);
        let _ = cortex_m::interrupt::free(|cs| {
            let tx = gpioa.pa9.into_alternate_af1(cs);
            let rx = gpioa.pa10.into_alternate_af1(cs);

            serial = Some(Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc));
        });

        let real_serial = serial.unwrap();

        let mono = Systick::new(cp.SYST, SYSTICK_FREQ);


        /*for i in 0..crate::RESULT_MATRIX_ROWS {
            task_i_row::spawn(i).ok();
        }*/

        task_reference::spawn().ok();

        let start_stack = get_stack() as u32;
        
        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 2.mhz());

        (
            Shared {
                tim2,
                concurrent_tasks: crate::RESULT_MATRIX_ROWS as u8,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
                serial: real_serial,
                a_matrix: [1690.8640661047846, 5894.418762210852, 4105.601556185959, 3900.2683835766584],
                b_matrix: [6933.946414524028, 4775.034075590325, 2335.3851959129224, 3712.0637208157805, 6253.255027073763, 324.79736138169744, 9085.52784976614, 1889.6565720893182, 6003.231810790218, 648.5619461881109]
            },
            Local {
                largest_stack: start_stack,
                                start_stack,
                start_conc: 0
            },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, tim2, serial])]
    fn task_reference(ctx: task_reference::Context) {
        let usart = ctx.shared.serial;
        let tim = ctx.shared.tim2;
        let mut start = 0;
        for i in 0..crate::RESULT_MATRIX_ROWS {
            if i == 0 {
                start = time_us(tim);
            }
            for j in 0..crate::RESULT_MATRIX_COLUMNS {
                let mut tmp: f64 = 0.0;
                for k in 0..crate::A_MATRIX_COLUMNS {
                    tmp = tmp
                        + ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k]
                            * ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
                }

                ctx.shared.result_matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
            }
        }
        core::hint::black_box(ctx.shared.result_matrix);
        
        let end_time = time_us(tim);

        writeln!(usart, "{}", end_time-start).ok();
    }

    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, tim2, serial], local = [start_conc, largest_stack, start_stack], capacity = 2)]
    fn task_i_row(ctx: task_i_row::Context, i: usize) {
        tick(ctx.local.largest_stack);
        let tim = ctx.shared.tim2;
        if i == 0 {
            *ctx.local.start_conc = time_us(tim);
        }
        for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tick(ctx.local.largest_stack);
                tmp = tmp
                    + ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k]
                        * ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }

            ctx.shared.result_matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
        }
        let n_tasks = ctx.shared.concurrent_tasks;
        
        let usart = ctx.shared.serial;
        
        core::hint::black_box(ctx.shared.result_matrix);

        *n_tasks = *n_tasks - 1;
        tick(ctx.local.largest_stack);
        if *n_tasks == 0 {
            let end_time = time_us(tim);
            //writeln!(usart, "{}", end_time - *ctx.local.start_conc).ok();
            writeln!(usart, "{:#08x}", *ctx.local.largest_stack).ok();
        }
    }
}
