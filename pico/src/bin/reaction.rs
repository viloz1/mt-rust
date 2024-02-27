#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_2, TIMER_IRQ_1, TIMER_IRQ_3],
    peripherals = true

)]
mod app {

    use rp2040_hal::{clocks, gpio::{bank0::Gpio25, bank0::Gpio26, FunctionSio, Pin, SioOutput}, Watchdog, rosc::{RingOscillator, Enabled}};
    use rp2040_monotonic::{Rp2040Monotonic, ExtU64};
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::Interrupt::LevelHigh;
    use test_app::{get_random_u16, get_random_byte};

    pub struct Time {
        seconds: u32,
        milliseconds: u16
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        in_pin: Pin<Gpio26, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        rosc: RingOscillator<Enabled>,
        timer: Time
    }

    // Local resources go here
    #[local]
    struct Local {
        led_pin: Pin<Gpio25, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        led_state: bool
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");
        let mut pac = ctx.device;

        // Setup the clock. This is required.
        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
        let _clocks = clocks::init_clocks_and_plls(
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

        let rosc = rp2040_hal::rosc::RingOscillator::new(pac.ROSC);
        

        // The single-cycle I/O block controls our GPIO pins
        let sio = rp_pico::hal::Sio::new(pac.SIO);

        // Set the pins to their default state
        let pins = rp_pico::hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // Configure GPIO 25 as an output to drive our LED.
        // we can use reconfigure() instead of into_pull_up_input()
        // since the variable we're pushing it into has that type
        let led = pins.gpio25.reconfigure();

        // Set up the GPIO pin that will be our input
        let in_pin = pins.gpio26.into_push_pull_output();

        // Trigger on the 'falling edge' of the input pin.
        // This will happen as the button is being pressed
        in_pin.set_interrupt_enabled(LevelHigh, true);


        // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
        // will jump to the interrupt function when the interrupt occurs.
        // We do this last so that the interrupt can't go off while
        // it is in the middle of being configured
        unsafe {
            rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::IO_IRQ_BANK0);
        
        }

        let mono = Rp2040Monotonic::new(pac.TIMER);
        


        toggle_interrupt::spawn().ok();
        background_task::spawn(0).ok();
        count_clock::spawn().ok();

        let timer = Time{seconds: 0, milliseconds: 0};

        (
            Shared {
                in_pin,
                timer,
                rosc: rosc.initialize()
            },
            Local {
                led_pin: led,
                led_state: false
            },
            init::Monotonics(mono)

        )
    }

    
    #[idle()]
    fn idle_task(_ctx: idle_task::Context) -> ! {
        loop {
        }
    }
    
    #[task(priority = 3, shared = [timer])]
    fn count_clock(mut ctx: count_clock::Context) {
       ctx.shared.timer.lock(|time| {
            time.milliseconds += 1;
            if time.milliseconds == 1000 {
                time.seconds += 1;
                time.milliseconds = 0;
            }
        });
        let _ = count_clock::spawn_after(1.millis());
    }

    // TODO: Add tasks
// Toggle the led based on a local state
    #[task(shared = [in_pin, rosc], priority = 2)]
    fn toggle_interrupt(mut ctx: toggle_interrupt::Context) {
        let in_pin = ctx.shared.in_pin;
        let rosc = ctx.shared.rosc;
        (in_pin, rosc).lock(|pin, rosc| {
            let _ = pin.set_high();
        });
        // Re-spawn this task after 1000 milliseconds
        let duration: u64 = 1000;
        toggle_interrupt::spawn_after(duration.millis());
    }
    
    #[task(binds = IO_IRQ_BANK0, priority = 2, shared = [in_pin], local = [led_state, led_pin])]
    fn toggle_task(mut ctx: toggle_task::Context) {
         ctx.shared.in_pin.lock(|pin| {
            pin.set_low();
        });
        if *ctx.local.led_state {
            ctx.local.led_pin.set_high().unwrap();
            *ctx.local.led_state = false;
        } else {
            ctx.local.led_pin.set_low().unwrap();
            *ctx.local.led_state = true;
        }

    }

    #[task(priority = 1, shared = [timer, rosc], capacity = 255)]
    fn background_task(mut ctx: background_task::Context, task_id: u16) {
        defmt::info!("New task: {}", task_id);
        let start = ctx.shared.timer.lock(|time| {
            return Time{seconds: time.seconds, milliseconds: time.milliseconds};
        });
        let (sleep_time, spawn_after) = ctx.shared.rosc.lock(|rosc| {
            return (get_random_byte(&rosc) % 10, get_random_byte(&rosc) % 10);
        });
        loop {
            let current_time = ctx.shared.timer.lock(|time| {
                return Time{seconds: time.seconds, milliseconds: time.milliseconds};
            });
            if current_time.seconds - start.seconds > sleep_time.into() {
                break;
            }
        }
        let _ = background_task::spawn(task_id + 1);
        let _ = background_task::spawn_after((spawn_after as u64).secs(), task_id);
    }


}
