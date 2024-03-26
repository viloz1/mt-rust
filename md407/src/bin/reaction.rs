#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use md407 as _;

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3],
    peripherals = true

)]
mod app {
    use hal::rng::Rng;
    use hal::timer::CounterUs;
    
    use hal::uart::Serial;
    use md407::{hal as hal, get_random_byte, setup_usart, time_us_64};

    use stm32f4xx_hal::gpio::Pin;
    use hal::pac::{DAC, TIM2, USART1};
    use hal::prelude::*;
    use systick_monotonic::*;
    use core::fmt::Write;


    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        usart: Serial<USART1>,
        rng: Rng,
        button: Pin<'B', 7>,
        timer: CounterUs<TIM2>,
        sleep_time: u64,
        interrupt_start_time: u64
    }

    // Local resources go here
    #[local]
    struct Local {
        background_tasks: u64,
        last_timer_value: u64,
        dac: hal::dac::C2
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity
    

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let cp = ctx.core;
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.require_pll48clk().sysclk(168.MHz()).pclk1(8.MHz()).use_hse(25.MHz()).freeze();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
       
        // Create a delay abstraction based on SysTick
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;

        let mut button = gpiob.pb7.into_pull_down_input();
        button.make_interrupt_source(&mut syscfg);
        button.trigger_on_edge(&mut exti, hal::gpio::Edge::Rising);
        button.enable_interrupt(&mut exti);
        
        unsafe {
            cortex_m::peripheral::NVIC::unmask(button.interrupt());
        }

        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        
        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);
        
        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        let pa5 = gpioa.pa5.into_analog();
        dp.DAC.cr.modify(|_, w| w.boff2().set_bit());

        //Disable DAC trigger
        dp.DAC.cr.modify(|_, w| w.ten2().clear_bit());

        //Disable wave generation
        dp.DAC.cr.modify(|_, w| w.wave2().disabled());
        
        //Send enable command

        dp.DAC.dhr8r2.modify(|_, w| unsafe{w.bits(0)});
        let mut dac: hal::dac::C2 = dp.DAC.constrain(pa5);
        hal::dac::DacPin::enable(&mut dac);

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }
        
        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        issue_interrupt::spawn_after((5 as u64).secs()).ok();
        generate_sound::spawn();

        let rng = dp.RNG.constrain(&clocks);

        (
            Shared {
                usart: serial,
                rng,
                button,
                timer,
                sleep_time: 0,
                interrupt_start_time: 0
            },
            Local {
                background_tasks: 0,
                last_timer_value: 0,
                dac
            },
            init::Monotonics(mono)

        )
    }

    #[task(local = [dac], shared = [usart])]
    fn generate_sound(mut ctx: generate_sound::Context) {
        
        let dac = ctx.local.dac;
         
        let current = hal::dac::DacOut::get_value(dac);
        ctx.shared.usart.lock(|usart| {
            if current == 0 {
                let dac2 = unsafe { &(*DAC::ptr()) };
                //writeln!(usart, "inc: {:?}", dac2.dhr8r2.as_ptr());

                dac2.dhr8r2.write(|w| unsafe { w.bits(20 as u32) });
                hal::dac::DacOut::set_value(dac, 15);
            } else {
                //writeln!(usart, "dec");
                let dac2 = unsafe { &(*DAC::ptr()) };
                dac2.dhr8r2.write(|w| unsafe { w.bits(0 as u32) });
                hal::dac::DacOut::set_value(dac, 0);
            }
        });
        
        generate_sound::spawn_after((500000 as u64).micros()).unwrap();
             
    }

    #[idle(shared = [sleep_time, timer])]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
           cortex_m::interrupt::free(|_cs| {
                let start_time = ctx.shared.timer.lock(|tim| {
                    time_us_64(tim)
                });

                rtic::export::wfi();

                let end_time = ctx.shared.timer.lock(|tim| {
                    time_us_64(tim)
                });

                let sleep_time = end_time - start_time;

                 ctx.shared.sleep_time.lock(|total_sleep_time| {
                    *total_sleep_time = *total_sleep_time + sleep_time;
                });
            });
        }
    }

    #[task(priority = 1, shared = [rng, timer], capacity = 250)]
    fn background_task(mut ctx: background_task::Context) {
        let mut timer = ctx.shared.timer;

        let start_time = timer.lock(|timer| {
            return time_us_64(timer);
        });

        let (sleep_time, spawn_after) = (ctx.shared.rng).lock(|rng| {
            return (get_random_byte(rng) % 10, get_random_byte(rng) % 10);
        });

        loop {
            let current_time = timer.lock(|timer| {
                time_us_64(timer)
            });
            if current_time - start_time > (sleep_time as u64) * 1_000_000  {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u64).secs());
    }

    #[task(priority = 2, shared = [button, timer, interrupt_start_time])]
    fn issue_interrupt(ctx: issue_interrupt::Context) {
        let button = ctx.shared.button;
        let timer = ctx.shared.timer;
        let start_time = ctx.shared.interrupt_start_time;
        
        issue_interrupt::spawn_after((5 as u64).secs()).ok();
        (button, timer, start_time).lock(|button, timer, start_time| {
            cortex_m::peripheral::NVIC::pend(button.interrupt());
            *start_time = time_us_64(timer);
        });
    }

    #[task(
        priority = 2, 
        binds = EXTI9_5, 
        shared = [timer, button, usart, interrupt_start_time, sleep_time], 
        local = [background_tasks, last_timer_value]
    )]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        let end_time = ctx.shared.timer.lock(|timer| {
            time_us_64(timer)
        });

        let start_time = ctx.shared.interrupt_start_time.lock(|start_time| {
            *start_time
        }); 

        let differnece = end_time - start_time;

        let cpu_usage = (ctx.shared.sleep_time).lock(|sleep_time| {
            let last_timer_value = *ctx.local.last_timer_value;
            let current_timer_value = end_time;
            let elapsed_time = current_timer_value - last_timer_value;

            let cpu_usage = (elapsed_time as f64 - *sleep_time as f64) / (elapsed_time as f64);
            *sleep_time = 0;
            
            *ctx.local.last_timer_value = end_time;
            cpu_usage
        });

        ctx.shared.usart.lock(|usart| {
            writeln!(usart, "Interrupt time: {}us, background tasks: {}, CPU usage: {}%", 
            differnece, ctx.local.background_tasks, cpu_usage * 100 as f64).ok();
        });
        
        ctx.shared.button.lock(|button| {
            button.clear_interrupt_pending_bit();
        });

        *ctx.local.background_tasks = *ctx.local.background_tasks + 1;
        background_task::spawn().ok();
    }
}

