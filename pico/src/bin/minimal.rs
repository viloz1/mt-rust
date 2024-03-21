#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_2],
    peripherals = true

)]
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::{
        clocks,
        gpio::bank0::Gpio25,
        gpio::{FunctionSio, Pin, SioOutput},
        Watchdog,
    };
    use rp2040_monotonic::{ExtU64, Rp2040Monotonic};
    use rp_pico::XOSC_CRYSTAL_FREQ;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        led_state: bool,
        led_pin: Pin<Gpio25, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // Setup the clock. This is required.
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let _clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Set up the led pin
        let sio = rp_pico::hal::Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let mut led_pin = pins.led.into_push_pull_output();
        led_pin.set_low().unwrap();

        // Setup the monotonic timer
        let mono = Rp2040Monotonic::new(ctx.device.TIMER);

        toggle_task::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                led_pin,
                led_state: false,
            },
            init::Monotonics(mono),
        )
    }

    // TODO: Add tasks
    // Toggle the led based on a local state
    #[task(local = [led_state, led_pin])]
    fn toggle_task(ctx: toggle_task::Context) {
        if *ctx.local.led_state {
            defmt::info!("led on");
            ctx.local.led_pin.set_high().unwrap();
            *ctx.local.led_state = false;
        } else {
            defmt::info!("led off");
            ctx.local.led_pin.set_low().unwrap();
            *ctx.local.led_state = true;
        }

        // Re-spawn this task after 1000 milliseconds
        let duration: u64 = 1000;
        toggle_task::spawn_after(duration.millis()).ok();
    }
}
