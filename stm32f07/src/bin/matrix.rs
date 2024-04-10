#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout


const SHARED_SIZE: usize = 3;

const A_MATRIX_ROWS: usize = 4;
const A_MATRIX_COLUMNS: usize = SHARED_SIZE;

const B_MATRIX_ROWS: usize = SHARED_SIZE;
const B_MATRIX_COLUMNS: usize = 3;

const RESULT_MATRIX_ROWS: usize = A_MATRIX_ROWS;
const RESULT_MATRIX_COLUMNS: usize = B_MATRIX_COLUMNS;

const ADDRESS: *mut u32 = 0x2000_0000 as *mut u32;


// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1, USART2],
    peripherals = true

)]
mod app {
    use core::fmt::Write;
    use core::sync::atomic::{AtomicU32, Ordering};

    use cortex_m_rt::pre_init;
    use stm32f0xx_hal::gpio::gpioa::{PA10, PA9};
    use stm32f0xx_hal::gpio::{Alternate, AF1};
    use stm32f0xx_hal::{pac::TIM2, pac::USART1, serial::Serial};
    use stm32f0xx_hal::prelude::*;
    use systick_monotonic::Systick;
    use test_app::{setup_tim2, time_us, SYSTICK_FREQ};

    use crate::ADDRESS;
    
/// Saves the current position of the stack. Any function
/// being profiled must call this macro.
    ///
    fn tick(current: &mut u32) {
        let stack_end = cortex_m::register::msp::read();
            
        if stack_end < *current {
            *current = stack_end;
        }
    }

    #[pre_init]
    unsafe fn startup() {
        let test = cortex_m::register::msp::read();
            core::ptr::write_volatile(ADDRESS, test);
        let p = stm32f0xx_hal::pac::Peripherals::steal();
        let mut flash = p.FLASH;
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut flash);

        let gpioa = p.GPIOA.split(&mut rcc);

        
        

    }

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        tim2: TIM2,
        sleep_time: u64,
        concurrent_tasks: u8,
        #[lock_free]
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        #[lock_free]
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
    }

    // Local resources go here
    #[local]
    struct Local {
        start_stack: u32,
        largest_stack: u32,
        last_timer_value: u64,
        serial: Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 2.mhz());
        let mut serial: Option<Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>> = None;

        let gpioa = p.GPIOA.split(&mut rcc);
        let _ = cortex_m::interrupt::free(|cs| {
            let tx = gpioa.pa9.into_alternate_af1(cs);
            let rx = gpioa.pa10.into_alternate_af1(cs);

            serial = Some(Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc));
        });

        let mut real_serial = serial.unwrap();

        let mono = Systick::new(cp.SYST, SYSTICK_FREQ);

        let mut concurrent_tasks = 0;

        for i in 0..crate::RESULT_MATRIX_ROWS {
            concurrent_tasks += 1;
            task_i_row::spawn(i).ok();
        }


        let start_stack;
        unsafe {
            let value = core::ptr::read_volatile(ADDRESS);
            start_stack = value as u32;
        }

        writeln!(real_serial, "hello! {}", start_stack).ok();
    
        
        (
            Shared {
                tim2,
                sleep_time: 0,
                concurrent_tasks,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
                a_matrix: [
                    8022.401392711306,
                    6269.699646497047,
                    5919.486621679387,
                    7680.534704603009,
                    5608.435969144818,
                    8685.173505108116,
                    8796.865635113609,
                    3887.08577452924,
                    5141.32977276882,
                    5141.32977276882,
                    5141.32977276882,
                    5141.32977276882,
                ],
                b_matrix: [
                    8017.724296626506,
                    1515.4120723153633,
                    2041.0881516122372,
                    490.6566012387737,
                    1392.3503802878192,
                    6361.270938080724,
                    7399.567517801178,
                    1498.1163840916624,
                    6450.972180635455,
                ],
            },
            Local {
                last_timer_value: 0,
                largest_stack: u32::MAX,
                serial: real_serial,
                start_stack
            },
            init::Monotonics(mono),
        )
    }

    #[idle(shared = [sleep_time, tim2])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            cortex_m::interrupt::free(|_cs| {
                let start_time = ctx.shared.tim2.lock(|tim| time_us(tim));

                rtic::export::wfi();

                let end_time = ctx.shared.tim2.lock(|tim| time_us(tim));

                let sleep_time = end_time - start_time;

                ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }

    #[task(shared = [tim2, sleep_time], local = [last_timer_value])]
    fn calc_cpu(mut ctx: calc_cpu::Context) {
        let end_time = ctx.shared.tim2.lock(|tim| time_us(tim));

        let cpu_usage = (ctx.shared.sleep_time).lock(|sleep_time| {
            let last_timer_value = *ctx.local.last_timer_value;
            let current_timer_value = end_time;
            let elapsed_time = current_timer_value - last_timer_value;

            let cpu_usage = (elapsed_time as f64 - *sleep_time as f64) / (elapsed_time as f64);
            *sleep_time = 0;

            *ctx.local.last_timer_value = end_time;
            cpu_usage
        });
        //defmt::info!("CPU usage: {}", cpu_usage * 100 as f64);
    }

    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, tim2], local=[serial, start_stack, largest_stack], capacity = 15)]
    fn task_i_row(mut ctx: task_i_row::Context, i: usize) {
        tick(ctx.local.largest_stack);
        for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tmp = tmp
                    + ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k]
                        * ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }

            ctx.shared.result_matrix.lock(|matrix| {
                matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
            });
        }
        core::hint::black_box(&ctx.shared.result_matrix);

        let n_tasks = ctx.shared.concurrent_tasks;
        let tim = ctx.shared.tim2;
        (n_tasks, tim).lock(|n_tasks, tim2| {
            *n_tasks = *n_tasks - 1;

            if *n_tasks == 0 {
                let end_time = time_us(tim2);
                tick(ctx.local.largest_stack);

                let diff = *ctx.local.start_stack - *ctx.local.largest_stack;
                writeln!(ctx.local.serial, "Stack value: {diff}, hm {}\r\n", *ctx.local.start_stack);
               // defmt::info!("End_time: {}", end_time);
            }
        });
    }
}
