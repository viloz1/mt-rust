#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_2, TIMER_IRQ_3, SW0_IRQ],
    peripherals = true

)]
mod app {
    use core::fmt::Write;

    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::gpio::Pin;
    use rp2040_hal::Clock;
    use rp2040_hal::{
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionUart, PullDown,
        },
        uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
    };
    use rp2040_monotonic::{ExtU64, Rp2040Monotonic};
    use rp_pico::pac::UART0;
    use test_app::{setup_clocks, time_us_64, write_to, PointerWrapper, TimerRegs};

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    type NumberType = u32;
    const LIMIT: NumberType = 10_000;

    // Shared resources go here
    #[shared]
    struct Shared {
        shared_num: NumberType,
        timer_regs: TimerRegs,
        done: bool,
        usart: UartPeripheral<
            Enabled,
            UART0,
            (
                Pin<Gpio0, FunctionUart, PullDown>,
                Pin<Gpio1, FunctionUart, PullDown>,
            ),
        >,
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        reference_num: NumberType,
        reference_done: bool,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
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

        let timer_regs = TimerRegs {
            hi: PointerWrapper(timerawh),
            lo: PointerWrapper(timerawl),
        };
        //reference_task::spawn().ok();
        low_priority_task::spawn().ok();
        high_priority_task::spawn().ok();

        let mono = Rp2040Monotonic::new(pac.TIMER);

        (
            Shared {
                shared_num: 1,
                timer_regs,
                done: false,
                usart: uart,
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
    #[task(shared = [shared_num, timer_regs, usart], local = [reference_num, reference_done], priority = 1)]
    fn reference_task(ctx: reference_task::Context) {
        let tim2 = ctx.shared.timer_regs;
        let num = ctx.local.reference_num;
        let done = ctx.local.reference_done;
        let usart = ctx.shared.usart;
        if *num == LIMIT && !*done {
            (tim2, usart).lock(|tim2, usart| {
                let end = time_us_64(tim2.hi.0, tim2.lo.0);
                writeln!(usart, "\n\rEnd time: {}\n\r", end).ok();
            });
            *done = true;
        } else if !*done {
            *num = *num + 1;
            reference_task::spawn().ok();
        }
    }

    #[task(shared = [shared_num, timer_regs, done, usart], priority = 1)]
    fn low_priority_task(ctx: low_priority_task::Context) {
        let tim2 = ctx.shared.timer_regs;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });
        low_priority_task::spawn().ok();
    }

    #[task(shared = [shared_num, timer_regs, done, usart], priority = 2)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        let tim2 = ctx.shared.timer_regs;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        let usart = ctx.shared.usart;

        (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
            increase(num, tim2, done, usart);
        });
        high_priority_task::spawn_after(1.millis()).ok();
    }

    fn increase(
        num: &mut NumberType,
        tim: &mut TimerRegs,
        done: &mut bool,
        usart: &mut UartPeripheral<
            Enabled,
            UART0,
            (
                Pin<Gpio0, FunctionUart, PullDown>,
                Pin<Gpio1, FunctionUart, PullDown>,
            ),
        >,
    ) {
        if *num == LIMIT && !*done {
            let end = time_us_64(tim.hi.0, tim.lo.0);
            writeln!(usart, "\n\rEnd time: {}\n\r", end).ok();
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
}
