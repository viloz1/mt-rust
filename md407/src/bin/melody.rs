#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(exposed_provenance)]
#![feature(const_trait_impl)]
#![feature(effects)]
use cortex_m_rt::pre_init;
use md407 as _;
use stm32f4xx_hal::pac::SCB;

const TONE_PERIOD_US: u64 = 2024;
const NEW_TASK_INTERVAL_MS: u64 = 500;
const BACKGROUND_TASK_SLEEP_US: u64 = 1;
const BACKGROUND_PERIOD_US: u64 = 500;

#[pre_init()]
unsafe fn startup() {
    (*SCB::PTR).ccr.modify(|r| r & !(1 << 3));
    //(*RCC::PTR).apb1enr.modify(|_, w| w.dacen().set_bit());
}
#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3, USART3, USART6],
    peripherals = true

)]
mod app {
    use core::fmt::Write;
    use hal::dac::DacPin;
    use hal::pac::{DAC, TIM2, USART1};
    use hal::prelude::*;
    use hal::timer::CounterUs;
    use hal::uart::Serial;
    use md407::{hal, setup_usart, time_us_64};
    use systick_monotonic::*;

    use crate::{BACKGROUND_TASK_SLEEP_US, NEW_TASK_INTERVAL_MS, TONE_PERIOD_US};

    // Shared resources go here
    #[shared]
    struct Shared {
        usart: Serial<USART1>,
        timer: CounterUs<TIM2>,
    }

    // Local resources go here
    #[local]
    struct Local {
        dac: hal::dac::C2,
        peak: bool,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100000>; // 100 000 Hz / 1 us granularity

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let rcc_uninit = dp.RCC;

        rcc_uninit.apb1enr.modify(|_, w| w.dacen().enabled());
        rcc_uninit.ahb1enr.modify(|_, w| w.gpioaen().enabled());

        let rcc: hal::rcc::Rcc = rcc_uninit.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).pclk1(8.MHz()).freeze();

        let gpioa = dp.GPIOA.split();

        let pa5 = gpioa.pa5.into_analog();

        //Enable buffer
        dp.DAC.cr.modify(|_, w| w.boff2().enabled());

        //Disable DAC trigger
        dp.DAC.cr.modify(|_, w| w.ten2().disabled());

        //Disable wave generati on
        dp.DAC.cr.modify(|_, w| w.wave2().disabled());

        //Send enable command
        dp.DAC.cr.modify(|_, w| w.en2().enabled());

        dp.DAC.dhr8r2.modify(|_, w| unsafe { w.bits(0) });
        let mut dac: hal::dac::C2 = dp.DAC.constrain(pa5);
        dac.enable();

        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;

        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);
        background_task::spawn().ok();
        inc_task::spawn().ok();

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        let systick = ctx.core.SYST;
        let mono = Systick::new(systick, 168_000_000);

        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        generate_sound::spawn().unwrap();

        (
            Shared {
                usart: serial,
                timer, // Initialization of shared resources go here
            },
            Local { dac, peak: false },
            init::Monotonics(mono),
        )
    }

    #[task(local = [dac, peak], shared = [usart], priority = 2)]
    fn generate_sound(ctx: generate_sound::Context) {
        generate_sound::spawn_after((2024 as u64).micros()).unwrap();
        let peak = ctx.local.peak;

        if *peak {
            let dac2 = unsafe { &(*DAC::ptr()) };
            dac2.dhr8r2.write(|w| unsafe { w.bits(20) });
            *peak = false;
        } else {
            let dac2 = unsafe { &(*DAC::ptr()) };
            dac2.dhr8r2.write(|w| unsafe { w.bits(0) });
            *peak = true;
        }
    }

    #[task(priority = 1)]
    fn inc_task(_ctx: inc_task::Context) {
        inc_task::spawn_after((NEW_TASK_INTERVAL_MS).millis()).ok();
        background_task::spawn().ok();
    }

    #[task(shared = [timer], capacity = 200, priority = 1)]
    fn background_task(mut ctx: background_task::Context) {
        let period = (500 as u64).micros();
        let end_time = ctx
            .shared
            .timer
            .lock(|tim| time_us_64(tim) + 1);
        loop {
            let current_time = ctx.shared.timer.lock(|tim| time_us_64(tim));
            if current_time > end_time {
                break;
            }
        }
        background_task::spawn_after(period).unwrap();
    }
}
