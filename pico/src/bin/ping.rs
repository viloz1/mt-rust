
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_2, TIMER_IRQ_1],
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
    use rp2040_monotonic::Rp2040Monotonic;
    use rp_pico::pac::UART0;
    use test_app::{setup_clocks, time_us_64, write_to, PointerWrapper, TimerRegs};
    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::Pin;
    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        timer_regs: TimerRegs,
        sleep_time: u64,
        uart:  UartPeripheral<
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
    #[local]
    struct Local {
        start_time: u64,
        start_time_self: u64,
        value: u64,
        value_self: u64,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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
        self_ping::spawn(1).ok();

        let timer_regs = TimerRegs {
            hi: PointerWrapper(timerawh),
            lo: PointerWrapper(timerawl),
        };


        let sys_clock = clocks.system_clock.freq();
        let mut buf = [0u8; 512];
        let print: &str =
            write_to::show(&mut buf, format_args!("\n\rSys clk: {}\n\r", sys_clock)).unwrap();

        uart.write_full_blocking(print.as_bytes());

        let mono = Rp2040Monotonic::new(pac.TIMER);

        let start_time = time_us_64(timer_regs.hi.0, timer_regs.lo.0);

        (
            Shared {
                timer_regs,
                sleep_time: 0,
                uart,
            },
            Local {
                start_time,
                value: 0,
                value_self: 0,
                start_time_self: start_time,
            },            
            init::Monotonics(mono),
        )
    }

    #[task(shared = [timer_regs, sleep_time], priority = 1)]
    fn ping(_ctx: ping::Context) {
        pong::spawn(1).ok();
        ping::spawn().ok();
    }

    #[task(shared = [timer_regs, sleep_time, uart], local = [value, start_time], priority = 2)]
    fn pong(
        mut ctx: pong::Context,
        val: u64,
    ) {
        *ctx.local.value = *ctx.local.value + val;
        if *ctx.local.value == 1_000_000 {
            ctx.shared.timer_regs.lock(|regs| {
                let end_time = time_us_64(regs.hi.0, regs.lo.0);
                let mut buf = [0u8; 512];
                let print: &str = write_to::show(
                    &mut buf,
                    format_args!("\n\rEnd time: {}\n\r", end_time - *ctx.local.start_time),
                )
                .unwrap();

                ctx.shared.uart.lock(|uart| {
                    uart.write_full_blocking(print.as_bytes());
                });
            });
        }
    }

    #[task(shared = [timer_regs, uart, sleep_time], local = [value_self, start_time_self], priority = 2)]
    fn self_ping(
        mut ctx: self_ping::Context,
        val: usize,
    ) {
            *ctx.local.value_self = *ctx.local.value_self + val as u64;
            if *ctx.local.value_self == 1_000_000 {
                ctx.shared.timer_regs.lock(|regs| {
                    let end_time = time_us_64(regs.hi.0, regs.lo.0);
                    let mut buf = [0u8; 512];
                    let print: &str = write_to::show(
                        &mut buf,
                        format_args!("\n\rEnd time: {}\n\r", end_time - *ctx.local.start_time_self),
                    )
                    .unwrap();

                    ctx.shared.uart.lock(|uart| {
                        uart.write_full_blocking(print.as_bytes());
                    });
                });
            }
        self_ping::spawn(1).ok();
    }


}
