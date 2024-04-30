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

    use core::fmt::Write;

    use cortex_m::register::msp;
    use rp2040_hal::fugit::RateExtU32;

    use rp2040_hal::{clocks, gpio::{bank0::{Gpio0, Gpio1, Gpio25, Gpio26}, FunctionSio, FunctionUart, Pin, PullDown, SioOutput}, rosc::{Enabled, RingOscillator}, uart::{DataBits, StopBits, UartConfig, UartPeripheral}, Clock, Watchdog};
    use rp2040_monotonic::{Rp2040Monotonic, ExtU64};
    use rp_pico::{pac::UART0, XOSC_CRYSTAL_FREQ};
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::Interrupt::LevelHigh;
    use test_app::{get_random_byte, tick, time_us_64, PointerWrapper, TimerRegs, CPU_PERIOD};
    
    // Shared resources go here
    #[shared]
    struct Shared {
        in_pin: Pin<Gpio26, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        rosc: RingOscillator<Enabled>,
        timer_regs: TimerRegs,
        sleep_time: u64,
        interrupt_start_time: u64,
        #[lock_free]
        largest_stack: u32
    }

    // Local resources go here
    #[local]
    struct Local {
        number_of_tasks: u16,
        last_timer_value: u64,
        led_pin: Pin<Gpio25, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        led_state: bool,
        uart: UartPeripheral<
            rp2040_hal::uart::Enabled,
            UART0,
            (
                Pin<Gpio0, FunctionUart, PullDown>,
                Pin<Gpio1, FunctionUart, PullDown>,
            ),
        >,

    }

    const BACKGROUND_TASKS: usize = 100;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut pac = ctx.device;

        // Setup the clock. This is required.
        // Set up the watchdog driver - needed by the clock setup code
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

        // Configure the clocks
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
        let mut led_pin = pins.gpio25.into_push_pull_output();
        led_pin.set_low().unwrap();

        // Configure GPIO 25 as an output to drive our LED.
        // we can use reconfigure() instead of into_pull_up_input()
        // since the variable we're pushing it into has that type

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

        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio0.into_function(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio1.into_function(),
        );

        let mut uart = rp2040_hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(
                UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        writeln!(uart, "New run").ok();

        
        let timer_regs = TimerRegs {
            hi: PointerWrapper(pac.TIMER.timerawh.as_ptr()),
            lo: PointerWrapper(pac.TIMER.timerawl.as_ptr())
        };

        let mono = Rp2040Monotonic::new(pac.TIMER);

        for _ in 0..BACKGROUND_TASKS {
            background_task::spawn().ok();
        }

        toggle_interrupt::spawn().ok();

        (
            Shared {
                largest_stack: msp::read(),
                in_pin,
                rosc: rosc.initialize(),
                timer_regs,
                sleep_time: 0,
                interrupt_start_time: 0
            },
            Local {
                number_of_tasks: 0,
                last_timer_value: 0,
                led_pin,
                led_state: false,
                uart
            },
            init::Monotonics(mono)

        )
    }

    #[task(priority = 1, shared = [rosc, timer_regs], capacity = 100)]
    fn background_task(mut ctx: background_task::Context) {
        let start = ctx.shared.timer_regs.lock(|timer_regs| {
            return time_us_64(timer_regs.hi.0, timer_regs.lo.0);
        });
        let (sleep_time, spawn_after) = ctx.shared.rosc.lock(|rosc| {
            return (get_random_byte(&rosc) % 10, get_random_byte(&rosc) % 10);
        });

        loop {
            let current_time = ctx.shared.timer_regs.lock(|timer_regs| {
                return time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            });
            if current_time - start > (sleep_time as u64) * 1_000 {
                break;
            }
        }
        background_task::spawn_after((spawn_after as u64).millis()).ok();
    }


    
    // TODO: Add tasks
// Toggle the led based on a local state
    #[task(shared = [in_pin, rosc, interrupt_start_time, timer_regs, largest_stack], local = [led_pin, led_state], priority = 2)]
    fn toggle_interrupt(ctx: toggle_interrupt::Context) {
        let in_pin = ctx.shared.in_pin;
        
        // Re-spawn this task after 1000 milliseconds
        toggle_interrupt::spawn_after(1.secs()).ok();
        let interrupt_start_time = ctx.shared.interrupt_start_time;
        let timer_regs = ctx.shared.timer_regs;
        (interrupt_start_time, timer_regs, in_pin).lock(|start_time, timer_regs, pin| {
            tick(ctx.shared.largest_stack);
            *start_time = time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            let _ = pin.set_high();
        });
    }
    
    #[task(
        binds = IO_IRQ_BANK0, 
        priority = 2, 
        local = [number_of_tasks, last_timer_value, uart], 
        shared = [in_pin, sleep_time, interrupt_start_time, timer_regs, largest_stack]
    )]
    fn toggle_task(mut ctx: toggle_task::Context) {
        let interrupt_start_time = ctx.shared.interrupt_start_time;
        let (start_time, end_time) = (interrupt_start_time, ctx.shared.timer_regs).lock(|start_time, timer_regs| {
            let end_time = time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            (*start_time, end_time)
        });
        tick(ctx.shared.largest_stack);
        ctx.shared.in_pin.lock(|pin| {
            pin.set_low().ok();
        });

        let interrupt_time = end_time - start_time;

        tick(ctx.shared.largest_stack);

        writeln!(ctx.local.uart, "{:08x}", 
            ctx.shared.largest_stack).ok();
        

    }

    }
