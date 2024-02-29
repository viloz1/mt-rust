#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use md407 as _;


const SHARED_SIZE: usize = 4;

const A_MATRIX_ROWS: usize = 3;
const A_MATRIX_COLUMNS: usize = SHARED_SIZE;

const B_MATRIX_ROWS: usize = SHARED_SIZE;
const B_MATRIX_COLUMNS: usize = 7;

const RESULT_MATRIX_ROWS: usize = A_MATRIX_ROWS;
const RESULT_MATRIX_COLUMNS: usize = B_MATRIX_COLUMNS;
// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4],
    peripherals = true
)]
mod app {
    use hal::timer::CounterUs;
    
    use hal::uart::Serial;
    use md407::{hal as hal, setup_usart, time_us_64};

    use hal::pac::{USART1, TIM2};
    use hal::prelude::*;
    use systick_monotonic::*;
    use core::fmt::Write;
    
    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        usart: Serial<USART1>,
        sleep_time: u64,
        timer: CounterUs<TIM2>,
        concurrent_tasks: u8,
        #[lock_free]
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        #[lock_free]
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        #[lock_free]
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS]
    }

    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let cp = ctx.core;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.require_pll48clk().sysclk(168.MHz()).pclk1(8.MHz()).use_hse(25.MHz()).freeze();
        
        let gpioa = dp.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        
        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);
        
        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);

        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        
        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }

        let mut concurrent_tasks = 0;

        for i in 0..crate::RESULT_MATRIX_ROWS {
            concurrent_tasks += 1;
            task_i_row::spawn(i).ok();
        };

        calc_cpu::spawn().ok();

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        (
            Shared {
                timer,
                usart: serial,
                sleep_time: 0,
                concurrent_tasks,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
                a_matrix: [7986.45, 1292.79, 8583.79, 2072.98, 2161.08, 7137.87, 8844.89542, 1241.16699, 9333.09941, 1046.33654, 1766.28663, 227.57371],
                b_matrix: [2089.46, 484.29, 4070.86, 6388.14, 5878.41, 7796.02, 4174.04910, 3779.38097, 1819.78879, 392.12370, 5173.20243, 2525.39435, 4430.77499, 3700.96781, 3312.82425, 2487.50948, 1680.72726, 1257.68497, 5224.09199, 5027.58651, 4620.30426, 7821.20553, 5898.87661, 3104.60453, 4500.93917, 3847.383526, 1115.46655, 1150.36475]


            },
            Local {
                last_timer_value: 0,
            },
            init::Monotonics(mono)

        )
    }

    #[idle(shared = [sleep_time, timer])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
           cortex_m::interrupt::free(|_cs| {
                let start_time = ctx.shared.timer.lock(|timer| {
                    time_us_64(timer)
                });

                rtic::export::wfi();

                let end_time =ctx.shared.timer.lock(|timer| {
                    time_us_64(timer)
                });

                let sleep_time = end_time - start_time;

                 ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }
    
    #[task(shared = [timer, sleep_time, usart], local = [last_timer_value])]
    fn calc_cpu(mut ctx: calc_cpu::Context) {    
        let end_time = ctx.shared.timer.lock(|timer| {
            time_us_64(timer)
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

        calc_cpu::spawn_after((3 as u64).secs()).ok();
        ctx.shared.usart.lock(|usart| {
            writeln!(usart, "CPU usage: {}", cpu_usage * 100 as f64).ok();
        });
    }
    
    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, timer, usart], capacity = 15)]
    fn task_i_row(ctx: task_i_row::Context, i: usize) {
       for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tmp = tmp + 
                    ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k] * 
                    ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }
            
            ctx.shared.result_matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
        }
        let n_tasks = ctx.shared.concurrent_tasks;
        let tim = ctx.shared.timer;
        let usart = ctx.shared.usart;
        (n_tasks, tim, usart).lock(|n_tasks, tim, usart| {
            *n_tasks = *n_tasks - 1;
            if *n_tasks == 0 {
                let end_time = time_us_64(tim);
                writeln!(usart, "End_time: {:?}", end_time as f64).ok();
            }
        });
    }
}
