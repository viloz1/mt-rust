
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(exposed_provenance)]
#![feature(const_trait_impl)]
#![feature(effects)]
use cortex_m_rt::pre_init;
use md407 as _;
use stm32f4xx_hal::pac::SCB;

const PERIODS: [u64; 25] = [
    2024, 1911, 1803, 1702, 1607, 1516, 1431, 1351, 1275, 1203, 1136, 1072, 1012, 955, 901, 851,
    803, 758, 715, 675, 637, 601, 568, 536, 506,
];

const MELODY: [i64; 32] = [
    0, 2, 4, 0, 0, 2, 4, 0, 4, 5, 7, 4, 5, 7, 7, 9, 7, 5, 4, 0, 7, 9, 7, 5, 4, 0, 0, -5, 0, 0, -5,
    0,
];

// a = 2
// b = 4
// c = 1
const NOTE_LENGTH: [u64; 32] = [
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 4, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 4, 2, 2, 4,
];


#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI4, EXTI3, USART3, USART6],
    peripherals = true

)]
mod app {
    use hal::dac::DacPin;
    use hal::pac::{DAC, TIM2, USART1};
    use hal::uart::Serial;
    use md407::{hal, reset, setup_usart, time_us_64};
    use hal::timer::CounterUs;
    use core::fmt::Write;
    use hal::prelude::*;
    use systick_monotonic::*;

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
    
    const BACKGROUND_TASKS: usize = 80;

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

        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::USART1);
        }

        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);

        for _ in 0..BACKGROUND_TASKS {
            background_task::spawn().ok();
        }

        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();

        let systick = ctx.core.SYST;
        let mono = Systick::new(systick, 168_000_000);

        writeln!(serial, "\rwooooow lets gooo\r").unwrap();
        generate_sound::spawn().unwrap();

        (
            Shared {
                usart: serial,
                timer// Initialization of shared resources go here
            },
            Local {
                dac,
                peak: false,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [dac, peak], priority = 1)]
    fn generate_sound(ctx: generate_sound::Context) {
        let period: u64 = 2024;
        generate_sound::spawn_after((period).micros()).unwrap();
        let peak = ctx.local.peak;

        if *peak {
            let dac2 = unsafe { &(*DAC::ptr()) };
            dac2.dhr8r2.write(|w| unsafe { w.bits(1000) });

            *peak = false;
        } else {
            let dac2 = unsafe { &(*DAC::ptr()) };
            dac2.dhr8r2.write(|w| unsafe { w.bits(0) });
            *peak = true;
        }
    }

    #[task(shared = [timer], capacity = 200, priority = 1)]
    fn background_task(mut ctx: background_task::Context) {
        let period = (1000 as u64).micros();
        let sleep = 1;

        let end_time = ctx.shared.timer.lock(|tim| time_us_64(tim) + sleep);
        loop {
            let current_time = ctx.shared.timer.lock(|tim| time_us_64(tim));
            if current_time > end_time {
                break;
            }            
        }
        background_task::spawn_after(period).unwrap();
    }

    #[task(binds=USART1, shared = [usart, timer], priority = 2)]
    fn handle_usart(ctx: handle_usart::Context) {
        let usart = ctx.shared.usart;
        let tim = ctx.shared.timer;
        (usart, tim).lock(|usart, tim| {
            cortex_m::peripheral::NVIC::unpend(hal::interrupt::USART1);
            while usart.is_rx_not_empty() {
                let read = usart.read();

                if let Err(e) = read {
                    writeln!(usart, "error {:?}", e).ok();
                }

                if let Ok(char) = read {
                    if char == b'r' {
                        reset(tim);
                    }
                }
            }
        })
    }
}
