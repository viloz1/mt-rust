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

    use rp2040_hal::{clocks, gpio::{bank0::{Gpio25, Gpio26}, FunctionSio, Pin, SioOutput}, rosc::{Enabled, RingOscillator}, Watchdog};
    use rp_pico::XOSC_CRYSTAL_FREQ;
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::Interrupt::LevelHigh;
    use test_app::{get_random_byte, TimerRegs, PointerWrapper, time_us_64, CPU_PERIOD};
    use rtic_monotonics::rp2040::*;
    
    // Shared resources go here
    #[shared]
    struct Shared {
        in_pin: Pin<Gpio26, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        rosc: RingOscillator<Enabled>,
        timer_regs: TimerRegs,
        sleep_time: u64,
        interrupt_start_time: u64,
        number_of_tasks: u16,
    }

    // Local resources go here
    #[local]
    struct Local {
        spawner: rtic_sync::channel::Sender<'static, u8, 255>, 
        last_timer_value: u64,
        led_pin: Pin<Gpio25, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown>,
        led_state: bool,
        
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
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

        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        Timer::start(pac.TIMER, &mut pac.RESETS, rp2040_timer_token); // default rp2040 clock-rate is 125MHz// default rp2040 clock-rate is 125MHz

        // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
        // will jump to the interrupt function when the interrupt occurs.
        // We do this last so that the interrupt can't go off while
        // it is in the middle of being configured
        unsafe {
            rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::IO_IRQ_BANK0);
        }

        let timer_regs = TimerRegs {
            hi: PointerWrapper(pac.TIMER.timerawh.as_ptr()),
            lo: PointerWrapper(pac.TIMER.timerawl.as_ptr())
        };

        let (mut s, r) = rtic_sync::make_channel!(u8, 255);
        toggle_interrupt::spawn().ok();
        background_task::spawn(r, s.clone());

        (
            Shared {
                in_pin,
                rosc: rosc.initialize(),
                timer_regs,
                sleep_time: 0,
                interrupt_start_time: 0,
                                number_of_tasks: 0,

            },
            Local {
                last_timer_value: 0,
                led_pin,
                led_state: false,
                spawner: s
            }

        )
    }

    
    #[idle(shared = [sleep_time, timer_regs])]
    fn idle(mut ctx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {
           critical_section::with(|_cs| {
                let start_time = ctx.shared.timer_regs.lock(|timer_regs| {
                    time_us_64(timer_regs.hi.0, timer_regs.lo.0)
                });

                rtic::export::wfi();

                let end_time = ctx.shared.timer_regs.lock(|timer_regs| {
                    time_us_64(timer_regs.hi.0, timer_regs.lo.0)
                });

                let sleep_time = end_time - start_time;

                 ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }
    

    // TODO: Add tasks
// Toggle the led based on a local state
    #[task(shared = [in_pin, rosc, interrupt_start_time, timer_regs], local = [led_pin, led_state], priority = 2)]
    async fn toggle_interrupt(mut ctx: toggle_interrupt::Context) {
        loop {
            let in_pin = &mut ctx.shared.in_pin;
            (in_pin).lock(|pin| {
                let _ = pin.set_high();
            });
            // Re-spawn this task after 1000 milliseconds
            let interrupt_start_time = &mut ctx.shared.interrupt_start_time;
            let timer_regs = &mut ctx.shared.timer_regs;
            (interrupt_start_time, timer_regs).lock(|start_time, timer_regs| {
                *start_time = time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            });

            Timer::delay(1000.millis()).await;
        }
        
    }
    
    #[task(
        binds = IO_IRQ_BANK0, 
        priority = 2, 
        local = [last_timer_value, spawner], 
        shared = [number_of_tasks, in_pin, sleep_time, interrupt_start_time, timer_regs]
    )]
    fn toggle_task(mut ctx: toggle_task::Context) {
         ctx.shared.in_pin.lock(|pin| {
            pin.set_low().ok();
        });
        let interrupt_start_time = ctx.shared.interrupt_start_time;
        let (start_time, end_time) = (interrupt_start_time, ctx.shared.timer_regs).lock(|start_time, timer_regs| {
            let end_time = time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            (*start_time, end_time)
        });

        let interrupt_time = end_time - start_time;

        let cpu_usage = (ctx.shared.sleep_time).lock(|sleep_time| {
            let last_timer_value = *ctx.local.last_timer_value;
            let current_timer_value = end_time;
            let elapsed_time = current_timer_value - last_timer_value;

            let cpu_usage = (elapsed_time as f64 - *sleep_time as f64) / (elapsed_time as f64);
            *sleep_time = 0;
            
            *ctx.local.last_timer_value = end_time;
            cpu_usage

        });

        let n_tasks = ctx.shared.number_of_tasks.lock(|tasks|{
            let old = *tasks;
            *tasks = *tasks+1;
            old
        });

        defmt::info!("Interrupt time: {}us, background tasks: {}, CPU usage: {}%", 
            interrupt_time, n_tasks, cpu_usage * 100 as f64);
        create_background_task::spawn(*ctx.local.spawner);

    }
    
    #[task(priority = 2, shared = [rosc, timer_regs])]
    async fn create_background_task(mut _ctx: create_background_task::Context, mut s: rtic_sync::channel::Sender<'static, u8, 255>) {
        s.send(0);
    }

    #[task(priority = 1, shared = [rosc, timer_regs])]
    async fn background_task(mut ctx: background_task::Context, mut r: rtic_sync::channel::Receiver<'static, u8, 255>, mut s: rtic_sync::channel::Sender<'static, u8, 255>,
) {
        loop {
            r.recv().await;
            let start = ctx.shared.timer_regs.lock(|timer_regs| {
                return time_us_64(timer_regs.hi.0, timer_regs.lo.0);
            });
            let (sleep_time, spawn_after) = ctx.shared.rosc.lock(|rosc| {
                return (get_random_byte(&rosc) % 10, get_random_byte(&rosc) % 10);
            });
            defmt::info!("Sleep time: {}", sleep_time);

            loop {
                let current_time = ctx.shared.timer_regs.lock(|timer_regs| {
                    return time_us_64(timer_regs.hi.0, timer_regs.lo.0);
                });
                if current_time - start > (sleep_time as u64) * 1_000_000 {
                    break;
                }
            }
            Timer::delay((spawn_after as u64).secs()).await;
            s.send(0);
        }
    }
}
