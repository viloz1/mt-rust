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
    use fugit::MicrosDurationU32;
    use hal::rng::Rng;
    use hal::timer::CounterUs;
    
    use hal::uart::Serial;
    use md407::{hal as hal, get_random_byte, setup_usart};

    use stm32f4xx_hal::gpio::{Pin, Output};
    use hal::pac::{USART1, TIM5, TIM2};
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
    }

    // Local resources go here
    #[local]
    struct Local {
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


        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }
        
        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
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
                background_tasks: 1
            },
            init::Monotonics(mono)

        )
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

