#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

const SHARED_SIZE: usize = 4;

const A_MATRIX_ROWS: usize = 3;
const A_MATRIX_COLUMNS: usize = SHARED_SIZE;

const B_MATRIX_ROWS: usize = SHARED_SIZE;
const B_MATRIX_COLUMNS: usize = 7;

const RESULT_MATRIX_ROWS: usize = A_MATRIX_ROWS;
const RESULT_MATRIX_COLUMNS: usize = B_MATRIX_COLUMNS;
// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_2],
    peripherals = true
)]
mod app {
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::Clock;
    use rp2040_hal::{
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionUart, PullDown,
        },
        uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
    };
    use rp_pico::pac::UART0;
    use test_app::{setup_clocks, time_us_64, write_to, PointerWrapper, TimerRegs};
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::Pin;
    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        timer_regs: TimerRegs,
        sleep_time: u64,
        concurrent_tasks: u8,
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
    }

    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
        start_time: u64,
        uart: UartPeripheral<
            Enabled,
            UART0,
            (
                Pin<Gpio0, FunctionUart, PullDown>,
                Pin<Gpio1, FunctionUart, PullDown>,
            ),
        >,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let mut pac = ctx.device;

        let timerawh = pac.TIMER.timerawh.as_ptr();
        let timerawl = pac.TIMER.timerawl.as_ptr();

        let clocks = setup_clocks(
            pac.XOSC,
            pac.WATCHDOG,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
        );
        // The single-cycle I/O block controls our GPIO pins
        let sio = rp2040_hal::Sio::new(pac.SIO);

        // Set the pins to their default state
        let pins = rp2040_hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio0.into_function(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio1.into_function(),
        );
        let uart = rp2040_hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let mut led_pin = pins.gpio25.into_push_pull_output();
        led_pin.set_high().unwrap();

        let timer_regs = TimerRegs {
            hi: PointerWrapper(timerawh),
            lo: PointerWrapper(timerawl),
        };

        let mut concurrent_tasks = 0;

        for i in 0..crate::RESULT_MATRIX_ROWS {
            concurrent_tasks += 1;
            task_i_row::spawn(i).ok();
        }

        let sys_clock = clocks.system_clock.freq();
        let mut buf = [0u8; 512];
        let print: &str =
            write_to::show(&mut buf, format_args!("\n\rSys clk: {}\n\r", sys_clock)).unwrap();

        uart.write_full_blocking(print.as_bytes());

        (
            Shared {
                timer_regs,
                sleep_time: 0,
                concurrent_tasks,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
                a_matrix: [
                    7986.45, 1292.79, 8583.79, 2072.98, 2161.08, 7137.87, 8844.89542, 1241.16699,
                    9333.09941, 1046.33654, 1766.28663, 227.57371,
                ],
                b_matrix: [
                    2089.46,
                    484.29,
                    4070.86,
                    6388.14,
                    5878.41,
                    7796.02,
                    4174.04910,
                    3779.38097,
                    1819.78879,
                    392.12370,
                    5173.20243,
                    2525.39435,
                    4430.77499,
                    3700.96781,
                    3312.82425,
                    2487.50948,
                    1680.72726,
                    1257.68497,
                    5224.09199,
                    5027.58651,
                    4620.30426,
                    7821.20553,
                    5898.87661,
                    3104.60453,
                    4500.93917,
                    3847.383526,
                    1115.46655,
                    1150.36475,
                ],
            },
            Local {
                start_time: 0,
                last_timer_value: 0,
                uart,
            },
        )
    }

    #[idle(shared = [sleep_time, timer_regs])]
    fn idle(mut ctx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {
            critical_section::with(|_cs| {
                let start_time = ctx
                    .shared
                    .timer_regs
                    .lock(|timer_regs| time_us_64(timer_regs.hi.0, timer_regs.lo.0));

                rtic::export::wfi();

                let end_time = ctx
                    .shared
                    .timer_regs
                    .lock(|timer_regs| time_us_64(timer_regs.hi.0, timer_regs.lo.0));

                let sleep_time = end_time - start_time;

                ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }

    #[task(shared = [timer_regs, sleep_time], local = [last_timer_value], priority = 1)]
    async fn calc_cpu(mut ctx: calc_cpu::Context) {
        let end_time = ctx
            .shared
            .timer_regs
            .lock(|timer_regs| time_us_64(timer_regs.hi.0, timer_regs.lo.0));

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

    #[task(shared = [result_matrix, &a_matrix, &b_matrix, concurrent_tasks, timer_regs], local=[start_time, uart], priority = 1, capacity = 10)]
    async fn task_i_row(mut ctx: task_i_row::Context, i: usize) {
        if i == 0 {
            ctx.shared.timer_regs.lock(|tim| {
                *ctx.local.start_time = time_us_64(tim.hi.0, tim.lo.0);
            });
        }
        for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tmp = tmp
                    + ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k]
                        * ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }

            ctx.shared
                .result_matrix
                .lock(|matrix| matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp);
        }
        let timer_regs = ctx.shared.timer_regs;
        let concurrent_tasks = ctx.shared.concurrent_tasks;
        let result_matrix = ctx.shared.result_matrix;

        (timer_regs, concurrent_tasks, result_matrix).lock(|tim, tasks, matrix| {
            *tasks = *tasks - 1;

            if *tasks == 0 {
                let end_time = time_us_64(tim.hi.0, tim.lo.0);
                let mut buf = [0u8; 512];
                let print: &str = write_to::show(
                    &mut buf,
                    format_args!(
                        "\n\rEnd_time: {}, diff: {}, Matrix: {:?}\n\r",
                        end_time,
                        end_time - *ctx.local.start_time,
                        matrix
                    ),
                )
                .unwrap();

                ctx.local.uart.write_full_blocking(print.as_bytes());
            }
        });
    }
}
