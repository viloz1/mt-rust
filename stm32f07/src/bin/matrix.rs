#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout


const SHARED_SIZE: usize = 2;

const A_MATRIX_ROWS: usize = 2;
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
        tim2: TIM2,
        sleep_time: u64,
        concurrent_tasks: u8,
        #[lock_free]
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        #[lock_free]
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS]
    }

    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
        adc: Adc,
        an_in: stm32f0xx_hal::gpio::gpioa::PA1<Analog>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");
        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);
        
        let gpioa = p.GPIOA.split(&mut rcc);

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 2.mhz());

        let mono = Systick::new(cp.SYST, SYSTICK_FREQ);

        let adc = Adc::new(p.ADC, &mut rcc);
        let mut an_in = None;
        
        cortex_m::interrupt::free(|cs| {
            an_in = Some(gpioa.pa1.into_analog(cs));
        });

        let mut concurrent_tasks = 0;

        for i in 0..crate::RESULT_MATRIX_ROWS {
            concurrent_tasks += 1;
            task_i_row::spawn(i).ok();
        };

        (
            Shared {
                tim2,
                sleep_time: 0,
                concurrent_tasks,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
                a_matrix: [1.0, 3.0, 12.3, 3.0],
                b_matrix: [7.5, 4.6, 7.0, 9.0, 0.0, 32.3, 64.64, 1.0, 356.74, 3.0]

            },
            Local {
                last_timer_value: 0,
                adc,
                an_in: an_in.unwrap(),
            },
            init::Monotonics(mono)

        )
    }

    #[idle(shared = [sleep_time, tim2])]
    fn idle(mut ctx: idle::Context) -> ! {
        defmt::info!("idle");
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
    
    #[task(shared = [tim2, sleep_time], local = [last_timer_value])]
    fn calc_cpu(mut ctx: calc_cpu::Context) {    
        let end_time = ctx.shared.tim2.lock(|tim| {
            time_us(tim)
        });

        let cpu_usage = (ctx.shared.sleep_time).lock(|sleep_time| {
            let last_timer_value = *ctx.local.last_timer_value;
            let current_timer_value = end_time;
            let elapsed_time = current_timer_value - last_timer_value;

            let cpu_usage = (elapsed_time as f64 - *sleep_time as f64) / (elapsed_time as f64);
            *sleep_time = 0;
            
            *ctx.local.last_timer_value = end_time;
            cpu_usage

        });
        defmt::info!("CPU usage: {}", cpu_usage * 100 as f64);
    }
    
    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, tim2], capacity = 10)]
    fn task_i_row(mut ctx: task_i_row::Context, i: usize) {
       for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tmp = tmp + 
                    ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k] * 
                    ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }
            
            ctx.shared.result_matrix.lock(|matrix| {
                matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
            });
        }
        let matrix = ctx.shared.result_matrix;
        let n_tasks = ctx.shared.concurrent_tasks;
        let tim = ctx.shared.tim2;
        (matrix, n_tasks, tim).lock(|matrix, n_tasks, tim2| {
            *n_tasks = *n_tasks - 1;
            if *n_tasks == 0 {
                let end_time = time_us(tim2);
                defmt::info!("Matrix: {}, end_time: {}", matrix, end_time);
            }
        });
    }
}
