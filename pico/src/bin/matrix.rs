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
    use core::fmt::Write;
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::{clocks, Clock, Watchdog};
    use rp2040_hal::{
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionUart, PullDown,
        },
        uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
    };
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico::pac::UART0;
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use test_app::{exit, get_stack, setup_clocks, tick, time_us_64, write_to, PointerWrapper, TimerRegs};
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;
    use rp2040_hal::gpio::Pin;
    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        #[lock_free]
        timer_regs: TimerRegs,
        sleep_time: u64,
        #[lock_free]
        concurrent_tasks: u8,
        #[lock_free]
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        #[lock_free]
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        #[lock_free]
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
        #[lock_free]
        uart: UartPeripheral<
            Enabled,
            UART0,
            (
                Pin<Gpio0, FunctionUart, PullDown>,
                Pin<Gpio1, FunctionUart, PullDown>,
            ),
        >,
    }

    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
        start_time: u64,
        start_stack: u32,
        largest_stack: u32
        
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");
        let mut pac = ctx.device;

        let timerawh = pac.TIMER.timerawh.as_ptr();
        let timerawl = pac.TIMER.timerawl.as_ptr();

        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
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

        let timer_regs = TimerRegs {
            hi: PointerWrapper(timerawh),
            lo: PointerWrapper(timerawl),
        };


        for i in 0..crate::RESULT_MATRIX_ROWS {
            task_i_row::spawn(i).ok();
        }
        //task_reference::spawn();
        //
        let start_stack = get_stack() as u32;

        let mono = Rp2040Monotonic::new(pac.TIMER);

        (
            Shared {
                timer_regs,
                sleep_time: 0,
                concurrent_tasks: crate::RESULT_MATRIX_ROWS as u8,
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
                uart

            },
            Local {
                start_time: 0,
                last_timer_value: 0,
                start_stack,
                largest_stack: start_stack
            },
            init::Monotonics(mono),
        )
    }


    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, timer_regs, uart], capacity = 15)]
    fn task_reference(ctx: task_reference::Context) {
        for i in 0..crate::RESULT_MATRIX_ROWS {
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
        let tim = ctx.shared.timer_regs;
        let usart = ctx.shared.uart;
        let end_time = time_us_64(tim.hi.0, tim.lo.0);
        writeln!(usart, "{}", end_time).ok();
    }

    #[task(shared = [result_matrix, a_matrix, b_matrix, concurrent_tasks, timer_regs, uart], local = [start_stack, largest_stack], capacity = 15)]
    fn task_i_row(ctx: task_i_row::Context, i: usize) {
        tick(ctx.local.largest_stack);
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
        let tim = ctx.shared.timer_regs;
        let usart = ctx.shared.uart;

        core::hint::black_box(ctx.shared.result_matrix);

        *n_tasks = *n_tasks - 1;
        tick(ctx.local.largest_stack);

        if *n_tasks == 0 {
            let end_time = time_us_64(tim.hi.0, tim.lo.0);
            writeln!(usart, "{:?}", end_time).ok();
            writeln!(usart, "{:?}", ctx.local.largest_stack).ok();
        }
    }
}
