#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use test_app as _; // global logger + panicking-behavior + memory layout

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    device = stm32f0xx_hal::pac,
    dispatchers = [USART1, USART2],
    peripherals = true

)]
mod app {
    use core::fmt::Write;

    use stm32f0xx_hal::adc::Adc;
    use stm32f0xx_hal::gpio::gpioa::{PA10, PA9};
    use stm32f0xx_hal::gpio::{Alternate, Analog, AF1};
    use stm32f0xx_hal::pac::{Interrupt, EXTI, TIM2};
    use stm32f0xx_hal::{pac::USART1, serial::Serial};
    use stm32f0xx_hal::prelude::*;
    use systick_monotonic::{
    fugit::ExtU32,
    Systick,
};
    use test_app::{get_random_byte, get_stack, setup_tim2, tick, time_us, SYSTICK_FREQ};
    
    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<100>;

    const BACKGROUND_TASKS: usize = 1;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        button: EXTI,
        tim2: TIM2,
        sleep_time: u64,
        interrupt_start_time: u64,
        #[lock_free]
        largest_stack: u32,
        #[lock_free]
        start_stack: u32
    }


    // Local resources go here
    #[local]
    struct Local {
        last_timer_value: u64,
        adc: Adc,
        an_in: stm32f0xx_hal::gpio::gpioa::PA1<Analog>,
        background_tasks: u8,
        serial: Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let start_stack = get_stack() as u32;
        let mut largest_stack = start_stack;
        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);
        
        let gpioa = p.GPIOA.split(&mut rcc);
        let syscfg = p.SYSCFG;
        let exti = p.EXTI;
        let button = gpioa.pa0;

        

        let mono = Systick::new(cp.SYST, SYSTICK_FREQ);

        let adc = Adc::new(p.ADC, &mut rcc);
        let mut an_in = None;
        
        cortex_m::interrupt::free(|cs| {
            an_in = Some(gpioa.pa1.into_analog(cs));
            button.into_pull_down_input(cs);
            syscfg.exticr1.modify(|_, w| unsafe { w.exti0().bits(1) });

            // Set interrupt request mask for line 1
            exti.imr.modify(|_, w| w.mr0().set_bit());

            // Set interrupt rising trigger for line 1
            exti.rtsr.modify(|_, w| w.tr0().set_bit());
            
            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::EXTI0_1, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0_1);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI0_1);
        });

        let mut serial: Option<Serial<USART1, PA9<Alternate<AF1>>, PA10<Alternate<AF1>>>> = None;

        let _ = cortex_m::interrupt::free(|cs| {
            let tx = gpioa.pa9.into_alternate_af1(cs);
            let rx = gpioa.pa10.into_alternate_af1(cs);

            serial = Some(Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc));
        });

        let mut real_serial = serial.unwrap();

        writeln!(real_serial, "NEW RUN \n\r");


        trigger_interrupt::spawn().ok();
        tick(&mut largest_stack);

        for _ in 0..BACKGROUND_TASKS {
            background_task::spawn().ok();
        }

        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 2.mhz());

        (
            Shared {
                button: exti,
                tim2,
                interrupt_start_time: 0,
                sleep_time: 0,
                start_stack,
                largest_stack
            },
            Local {
                last_timer_value: 0,
                adc,
                an_in: an_in.unwrap(),
                background_tasks: 0,
                serial: real_serial
            },
            init::Monotonics(mono)

        )
    }

    #[task(priority = 2, shared = [button, tim2, interrupt_start_time, largest_stack])]
    fn trigger_interrupt(mut ctx: trigger_interrupt::Context) {
        ctx.shared.button.lock(| button | {
            let tim2 = ctx.shared.tim2;
            let start_time = ctx.shared.interrupt_start_time;
            (tim2, start_time).lock(|tim2, start_time| {
                cortex_m::interrupt::free(|_cs| {
                    //tick(ctx.shared.largest_stack);
                    button.swier.write(|w| w.swier0().set_bit());
                    *start_time = time_us(tim2)
                });
            });
            
        });
        trigger_interrupt::spawn_after(1.secs().into()).ok();
       

    }

    #[task(priority = 1, local = [adc, an_in], shared = [tim2], capacity = 1)]
    fn background_task(mut ctx: background_task::Context) {
        let start = ctx.shared.tim2.lock(|tim| {
            time_us(tim)
        });

        let sleep_time = get_random_byte(ctx.local.adc, ctx.local.an_in) % 10;
        let spawn_after = get_random_byte(ctx.local.adc, ctx.local.an_in) % 10;
        loop {
            let current_time = ctx.shared.tim2.lock(|tim| {
                time_us(tim)
            });
            if current_time - start > (sleep_time as u64) * 1_000 {
                break;
            }
        }
        let _ = background_task::spawn_after((spawn_after as u32).millis().into());
    }


    #[task(binds = EXTI0_1, shared = [largest_stack, start_stack, button, tim2, interrupt_start_time, sleep_time], priority = 2, local = [background_tasks, serial, last_timer_value])]
    fn toggle_task(mut ctx: toggle_task::Context) {    
        let end_time = ctx.shared.tim2.lock(|tim| {
            time_us(tim)
        });
        //tick(ctx.shared.largest_stack);

        ctx.shared.button.lock(| button | {
            cortex_m::interrupt::free(|_cs| { 
                button.pr.write(|w| w.pr0().set_bit()); 
           });
        });

        let start_time = ctx.shared.interrupt_start_time.lock(|start_time| {
            *start_time
        });

        let difference = end_time - start_time;
        //tick(ctx.shared.largest_stack);
        //writeln!(ctx.local.serial, "{:08x}", *ctx.shared.largest_stack);
        writeln!(ctx.local.serial, "{}", difference).ok();
        //tick(ctx.shared.largest_stack);
    }
}
