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
    use fugit::{NanosDuration, NanosDurationU32, MicrosDurationU32};
    use hal::rng::Rng;
    use hal::timer::CounterUs;
    
    use hal::uart::{Serial, Event};
    use md407::{hal as hal, get_random_byte, setup_usart};

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::{USART1, TIM5, TIM2, TIM4};
    use hal::prelude::*;
    use stm32f4xx_hal::timer::Delay;
    use stm32f4xx_hal::uart::Config;
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
    }

    // Local resources go here
    #[local]
    struct Local {
        red_led: Pin<'B', 1, Output>,
        green_led: Pin<'B', 0, Output>,
        test_timer: CounterUs<TIM5>,
        background_tasks: u64,
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
        timer.start((120 as u32).secs()).ok();
        timer.listen(hal::timer::Event::Update);

        let mut test_timer = dp.TIM5.counter(&clocks);
        test_timer.start((120 as u32).secs()).ok();

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }
        
        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        toggle_red_led::spawn().unwrap();
        issue_interrupt::spawn_after((5 as u64).secs()).ok();
        background_task::spawn().ok();

        let rng = dp.RNG.constrain(&clocks);



        (
            Shared {
                usart: serial,
                rng,
                button,
                timer
            },
            Local {
                red_led,
                green_led,
                test_timer,
                background_tasks: 1
            },
            init::Monotonics(mono)

        )
    }

    // TODO: Add tasks
    #[task(binds = USART1, local = [green_led], shared = [usart])]
    fn read_usart(mut ctx: read_usart::Context) {
        ctx.local.green_led.toggle();
        ctx.shared.usart.lock(|usart| {
            while usart.is_rx_not_empty() {
                let input = usart.read();
                writeln!(usart, "Input: {:?}", input).ok();

            }
        }); 
    }

    #[task(binds = TIM2, shared = [usart, timer])]
    fn timer_interrupt(ctx: timer_interrupt::Context) {
        let usart = ctx.shared.usart;
        let timer = ctx.shared.timer;

        (usart, timer).lock(|usart, timer| {
            writeln!(usart, "Timer timeout!").ok();
            timer.wait().ok();
        });
    }

    #[task(priority = 1, shared = [rng, timer], capacity = 250)]
    fn background_task(mut ctx: background_task::Context) {
        let mut timer = ctx.shared.timer;

        let start_time = timer.lock(|timer| {
            return timer.now().ticks();
        });
        let (sleep_time, spawn_after) = (ctx.shared.rng).lock(|rng| {
            return (get_random_byte(rng) % 10, get_random_byte(rng) % 10);
        });
        let sleep_ticks = MicrosDurationU32::secs(sleep_time.into()).ticks();
        loop {
            let current_time = timer.lock(|timer| {
                return timer.now().ticks();
            });
            if current_time - start_time > sleep_ticks  {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u64).secs());
    }

    #[task(priority = 2, shared = [button, timer])]
    fn issue_interrupt(ctx: issue_interrupt::Context) {
        let button = ctx.shared.button;
        let timer = ctx.shared.timer;

        (button, timer).lock(|button, timer| {
            timer.cancel().ok();
            timer.start((120 as u32).secs()).ok();
            timer.listen(hal::timer::Event::Update);
            cortex_m::peripheral::NVIC::pend(button.interrupt());
        });
        issue_interrupt::spawn_after((5 as u64).secs()).ok();
    }


    #[task(priority = 2, local = [red_led, test_timer], shared = [usart])]
    fn toggle_red_led(mut ctx: toggle_red_led::Context) {
        ctx.local.red_led.toggle();

        let time = MicrosDurationU32::from_ticks(ctx.local.test_timer.now().ticks());
        let sleep_ticks = MicrosDurationU32::secs(4).ticks();

        ctx.shared.usart.lock(|usart| {
            //writeln!(usart,  "Nanos: {}, ticks: {}, Sleep: {}", time, ctx.local.test_timer.now().ticks(), sleep_ticks).ok();
        });
       
        ctx.local.test_timer.cancel().ok();
        ctx.local.test_timer.start((120 as u32).secs()).ok();
         
        toggle_red_led::spawn_after((1 as u64).secs()).unwrap();
    }

    #[task(priority = 2, binds = EXTI9_5, shared = [timer, button, usart], local = [background_tasks])]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        let usart = ctx.shared.usart;
        let timer = ctx.shared.timer;
        

        (timer, usart).lock(|timer, usart| {
            let dur =  MicrosDurationU32::from_ticks(timer.now().ticks());
            writeln!(usart, "Passed time: {} us, Background tasks: {}", dur.to_micros(), ctx.local.background_tasks).ok();
        });

        ctx.shared.button.lock(|button| {
            button.clear_interrupt_pending_bit();
        });
        *ctx.local.background_tasks = *ctx.local.background_tasks + 1;
        background_task::spawn().ok();
               
        
    }

}

