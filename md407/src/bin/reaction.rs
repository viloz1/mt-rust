#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use cortex_m::peripheral::SCB;
use cortex_m_rt::pre_init;
use md407 as _;


#[pre_init]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
}

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3],
    peripherals = true

)]
mod app {
    use hal::rng::Rng;
    use hal::timer::CounterUs;
    
    use hal::uart::{Serial, Event};
    use md407::{hal as hal, get_random_byte};

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::{USART1, TIM5, TIM2};
    use hal::{prelude::*};
    use stm32f4xx_hal::timer::Delay;
    use stm32f4xx_hal::uart::Config;
    use systick_monotonic::*;
    use core::fmt::Write;
    use core::ops::Sub;

    #[derive(Clone)] 
    pub struct Time {
        seconds: u32,
        micros: u32
    }

    impl Sub for Time {
        type Output = Self;

        fn sub(self, other: Self) -> Self::Output {
            Self {
                seconds: self.seconds - other.seconds,
                micros: self.micros - other.micros,
            }
        }
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        delay: Delay<TIM5, 1000000>,
        usart: Serial<USART1>,
        rng: Rng,
        time: Time,
        start_time: Time,
        button: Pin<'B', 7>
    }

    // Local resources go here
    #[local]
    struct Local {
        red_led: Pin<'B', 1, Output>,
        green_led: Pin<'B', 0, Output>,
        timer: CounterUs<TIM2>,
        timer_skip: bool
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
        let red_led = gpiob.pb1.into_push_pull_output();
        let green_led = gpiob.pb0.into_push_pull_output();
        
       
        // Create a delay abstraction based on SysTick
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti = dp.EXTI;
        
        let delay = dp.TIM5.delay_us(&clocks);

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
        
        let mut serial = usart1.serial((tx_pin, rx_pin), Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .stopbits(hal::uart::config::StopBits::STOP1)
                .parity_none(), &clocks).unwrap().with_u8_data();
        serial.listen(Event::RxNotEmpty);
        
        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((1 as u32).micros()).ok();
        timer.listen(hal::timer::Event::Update);
        timer.wait().ok();

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
        }
        
        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        toggle_red_led::spawn().unwrap();
        issue_interrupt::spawn_after((5 as u64).secs()).ok();
        background_task::spawn();

        let rng = dp.RNG.constrain(&clocks);



        (
            Shared {
                usart: serial,
                delay,
                rng,
                time: Time {seconds: 0, micros: 0},
                start_time: Time {seconds: 0, micros: 0},
                button
            },
            Local {
                red_led,
                green_led,
                timer,
                timer_skip: true

            },
            init::Monotonics(mono)

        )
    }

    // TODO: Add tasks
    #[task(binds = USART1, local = [green_led], shared = [delay, usart])]
    fn read_usart(mut ctx: read_usart::Context) {
        ctx.local.green_led.toggle();
        ctx.shared.usart.lock(|usart| {
            while usart.is_rx_not_empty() {
                let input = usart.read();
                writeln!(usart, "Input: {:?}", input).ok();

            }
        }); 
    }




    #[task(binds = TIM2, local=[timer, timer_skip], shared = [usart, time])]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        let current = *ctx.local.timer_skip;

        if current {
            *ctx.local.timer_skip = false;
            return
        } else {
             *ctx.local.timer_skip = true;
        }
        let time = ctx.shared.time;
        let usart = ctx.shared.usart;
        (time, usart).lock(|time, usart| {
            time.micros += 1;
            if time.micros % 1_000_000 == 0 {
                time.seconds += 1;
                time.micros = 0;
            } 
            cortex_m::peripheral::NVIC::unpend(hal::interrupt::TIM2);

        });
        ctx.local.timer.wait().ok();
    }

    #[task(priority = 1, shared = [time, rng], capacity = 250)]
    fn background_task(mut ctx: background_task::Context) {
        let start = ctx.shared.time.lock(|time| {
            return Time{seconds: time.seconds, micros: time.micros};
        });
        let (sleep_time, spawn_after) = ctx.shared.rng.lock(|rng| {
            return (get_random_byte(rng) % 10, get_random_byte(rng) % 10);
        });
        loop {
            let current_time = ctx.shared.time.lock(|time| {
                return Time{seconds: time.seconds, micros: time.micros};
            });
            if current_time.seconds - start.seconds > sleep_time.into() {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u64).secs());
    }

    #[task(priority = 2, shared = [button, time, start_time])]
    fn issue_interrupt(ctx: issue_interrupt::Context) {
        let button = ctx.shared.button;
        let time = ctx.shared.time;
        let start_time = ctx.shared.start_time;
        (button, time, start_time).lock(|button, time, start_time| {
            *start_time = time.clone();
            cortex_m::peripheral::NVIC::pend(button.interrupt());
        });
        issue_interrupt::spawn_after((5 as u64).secs());
    }


    #[task(local = [red_led], shared = [delay, usart])]
    fn toggle_red_led(ctx: toggle_red_led::Context) {
        ctx.local.red_led.toggle();
         
        toggle_red_led::spawn_after((1 as u64).secs()).unwrap();
    }

    #[task(priority = 2, binds = EXTI9_5, shared = [start_time, button, time, usart])]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        let usart = ctx.shared.usart;
        let start_time = ctx.shared.start_time;
        
        let current_time: Time = ctx.shared.time.lock(|time| {
            time.clone()
        });

        (start_time, usart).lock(|start_time_mut, usart| {
            let start_time: Time = start_time_mut.clone();
            let passed_time = current_time - start_time;
            writeln!(usart, "Passed time: {}s {}us", passed_time.seconds, passed_time.micros).ok();
        });

        ctx.shared.button.lock(|button| {
            button.clear_interrupt_pending_bit();
        });
        background_task::spawn();
               
        
    }

}

