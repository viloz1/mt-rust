#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::pre_init;

use panic_probe as _;

use rp2040_hal::{
    clocks::ClocksManager,
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
    rosc::{Enabled, RingOscillator},
    xosc::setup_xosc_blocking,
    Clock, Watchdog,
};
use rp_pico::{
    hal as _,
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, WATCHDOG, XOSC},
}; // memory layout

use rp2040_hal::clocks::ClockSource;
use rp2040_hal::fugit::RateExtU32;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked

fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);

pub const CPU_PERIOD: u64 = 3;

pub struct TimerRegs {
    pub hi: PointerWrapper,
    pub lo: PointerWrapper,
}

pub struct PointerWrapper(pub *mut u32);

unsafe impl Send for PointerWrapper {}

pub const ADDRESS: *mut u32 = 0x2000_3000 as *mut u32;

#[pre_init]
unsafe fn startup() {
    let test = cortex_m::register::msp::read();
    core::ptr::write_volatile(ADDRESS, test);
}

pub fn get_stack() -> usize {
    let start_stack: usize;
    unsafe {
        let value = core::ptr::read_volatile(ADDRESS);
        start_stack = value as usize;
    }
    start_stack
}

pub fn tick(current: &mut u32) {
    let stack_end = cortex_m::register::msp::read();

    if stack_end < *current {
        *current = stack_end;
    }
}

pub fn get_random_byte(rosc: &RingOscillator<Enabled>) -> u8 {
    let mut num: u8 = 0;
    for i in 0..8 {
        if rosc.get_random_bit() {
            num += 1 << (7 - i)
        }
    }
    num
}

pub fn get_random_u16(rosc: &RingOscillator<Enabled>) -> u16 {
    let mut array: [u8; 2] = [0; 2];
    for i in 0..2 {
        array[i] = get_random_byte(&rosc)
    }
    u16::from_le_bytes(array)
}

pub fn get_random_u32(rosc: &RingOscillator<Enabled>) -> u32 {
    let mut array: [u8; 4] = [0; 4];
    for i in 0..4 {
        array[i] = get_random_byte(&rosc)
    }
    u32::from_le_bytes(array)
}

pub fn get_random_u64(rosc: &RingOscillator<Enabled>) -> u64 {
    let mut array: [u8; 8] = [0; 8];
    for i in 0..8 {
        array[i] = get_random_byte(&rosc)
    }
    u64::from_le_bytes(array)
}

pub fn time_us_64(timerawh: *mut u32, timerawl: *mut u32) -> u64 {
    let mut awh_val = unsafe { core::ptr::read_volatile(timerawh) };
    let mut awl_val: u32;
    loop {
        awl_val = unsafe { core::ptr::read_volatile(timerawl) };
        let next_hi = unsafe { core::ptr::read_volatile(timerawh) };

        if awh_val == next_hi {
            break;
        }
        awh_val = next_hi;
    }
    ((awh_val as u64) << 32) | (awl_val as u64)
}
pub fn setup_clocks(
    xosc: XOSC,
    watchdog: WATCHDOG,
    in_clocks: CLOCKS,
    in_pll_sys: PLL_SYS,
    in_pll_usb: PLL_USB,
    resets: &mut RESETS,
) -> ClocksManager {
    let mut watchdog = Watchdog::new(watchdog);
    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

    // Enable the xosc
    let xosc = setup_xosc_blocking(xosc, XOSC_CRYSTAL_FREQ.Hz()).unwrap();

    // Start tick in watchdog
    watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(in_clocks);

    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    //
    let pll_conf = PLLConfig {
        refdiv: 1,
        vco_freq: 1500.MHz(), //if 1600, 133MHZ. if 1500, 125MHZ
        post_div1: 6, //if 3 and above value is 1600MHz, then 266MHz. if 6 and above value is 1500/1600Mhz, then 125/133MHz
        post_div2: 2,
    };
    let pll_sys = setup_pll_blocking(
        in_pll_sys,
        xosc.operating_frequency().into(),
        pll_conf,
        &mut clocks,
        resets,
    )
    .unwrap();
    let pll_usb = setup_pll_blocking(
        in_pll_usb,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .unwrap();

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clocks
        .reference_clock
        .configure_clock(&xosc, xosc.get_freq())
        .unwrap();

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clocks
        .system_clock
        .configure_clock(&pll_sys, pll_sys.get_freq())
        .unwrap();

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clocks
        .usb_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .unwrap();

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clocks
        .adc_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .unwrap();

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clocks
        .rtc_clock
        .configure_clock(&pll_usb, 46875u32.Hz())
        .unwrap();

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    clocks
}

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}
