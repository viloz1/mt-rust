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
    use fugit::{ExtU64, RateExtU32};
    use rp2040_hal::gpio::Pin;
    use rp2040_hal::Clock;
    use rp2040_hal::{
        gpio::{
            bank0::{Gpio0, Gpio1},
            FunctionUart, PullDown,
        },
        uart::{DataBits, Enabled, StopBits, UartConfig, UartPeripheral},
    };
    use rp_pico::pac::UART0;
    use rtic_monotonics::rp2040::Timer;
    use test_app::{setup_clocks, time_us_64, PointerWrapper, TimerRegs};

    type NumberType = u32;
    const LIMIT: NumberType = 512_000;

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
    fn init(ctx: init::Context) -> (Shared, Local) {
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
                UartConfig::new(115200_u32.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let timer_regs = TimerRegs {
            hi: PointerWrapper(timerawh),
            lo: PointerWrapper(timerawl),
        };

        low_priority_task::spawn().ok();
        high_priority_task::spawn().ok();
        //reference_task::spawn().ok();

        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        Timer::start(pac.TIMER, &mut pac.RESETS, rp2040_timer_token);

        (
            Shared {
                shared_num: 0,
                timer_regs,
                done: false,
                usart: uart,
            },
            Local {
                reference_num: 0,
                reference_done: false,
            },
        )
    }

    // TODO: Add tasks
    //
    //
    #[task(shared = [timer_regs, usart], local = [reference_num, reference_done], priority = 1)]
    async fn reference_task(mut ctx: reference_task::Context) {
        loop {
            let tim2 = &mut ctx.shared.timer_regs;
            let num = &mut *ctx.local.reference_num;
            let usart = &mut ctx.shared.usart;
            let done = &mut *ctx.local.reference_done;

            if *num == LIMIT && !*done {
                (tim2, usart).lock(|tim2, usart| {
                    let end = time_us_64(tim2.hi.0, tim2.lo.0);
                    writeln!(usart, "\n\rEnd time: {}\n\r", end).ok();
                });
                *done = true;
            } else if !*done {
                *num = *num + 1;
            }
        }
    }

    #[task(shared = [shared_num, timer_regs, done, usart], priority = 1)]
    async fn low_priority_task(mut ctx: low_priority_task::Context) {
        loop {
            let tim2 = &mut ctx.shared.timer_regs;
            let shared_num = &mut ctx.shared.shared_num;
            let done = &mut ctx.shared.done;
            let usart = &mut ctx.shared.usart;
            (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
                increase(num, tim2, done, usart);
            });
        }
    }

    #[task(shared = [shared_num, timer_regs, done, usart], priority = 2)]
    async fn high_priority_task(mut ctx: high_priority_task::Context) {
        loop {
            let tim2 = &mut ctx.shared.timer_regs;
            let shared_num = &mut ctx.shared.shared_num;
            let done = &mut ctx.shared.done;
            let usart = &mut ctx.shared.usart;
            (shared_num, tim2, done, usart).lock(|num, tim2, done, usart| {
                increase(num, tim2, done, usart);
            });
            Timer::delay(1_u64.millis()).await;
        }
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