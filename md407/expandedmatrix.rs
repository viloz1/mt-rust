#![feature(prelude_import)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;
use md407 as _;
const SHARED_SIZE: usize = 4;
const A_MATRIX_ROWS: usize = 3;
const A_MATRIX_COLUMNS: usize = SHARED_SIZE;
const B_MATRIX_ROWS: usize = SHARED_SIZE;
const B_MATRIX_COLUMNS: usize = 7;
const RESULT_MATRIX_ROWS: usize = A_MATRIX_ROWS;
const RESULT_MATRIX_COLUMNS: usize = B_MATRIX_COLUMNS;
/// The RTIC application module
pub mod app {
    /// Always include the device crate which contains the vector table
    use stm32f4xx_hal::pac as you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    pub use rtic::Monotonic as _;
    /// Holds static methods for each monotonic.
    pub mod monotonics {
        pub use Tonic::now;
        ///This module holds the static implementation for `Tonic::now()`
        #[allow(non_snake_case)]
        pub mod Tonic {
            /// Read the current time from this monotonic
            pub fn now() -> <super::super::Tonic as rtic::Monotonic>::Instant {
                rtic::export::interrupt::free(|_| {
                    use rtic::Monotonic as _;
                    if let Some(m) = unsafe {
                        &mut *super::super::__rtic_internal_MONOTONIC_STORAGE_Tonic
                            .get_mut()
                    } {
                        m.now()
                    } else {
                        <super::super::Tonic as rtic::Monotonic>::zero()
                    }
                })
            }
        }
    }
    use hal::timer::CounterUs;
    use hal::uart::Serial;
    use md407::{hal as hal, setup_usart, time_us_64};
    use hal::pac::{USART1, TIM2};
    use hal::prelude::*;
    use systick_monotonic::*;
    use core::fmt::Write;
    /// User code from within the module
    type Tonic = Systick<100>;
    /// User code end
    /// User provided init function
    #[inline(always)]
    #[allow(non_snake_case)]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = ctx.device;
        let cp = ctx.core;
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .require_pll48clk()
            .sysclk(168.MHz())
            .pclk1(8.MHz())
            .use_hse(25.MHz())
            .freeze();
        let gpioa = dp.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let usart1 = dp.USART1;
        let mut serial = setup_usart(usart1, tx_pin, rx_pin, clocks);
        let systick = cp.SYST;
        let mono = Systick::new(systick, 168_000_000);
        serial.write_fmt(format_args!("\rwooooow lets gooo\r\n")).unwrap();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM2);
            cortex_m::peripheral::NVIC::unmask(hal::interrupt::TIM4);
        }
        let mut concurrent_tasks = 0;
        for i in 0..crate::RESULT_MATRIX_ROWS {
            concurrent_tasks += 1;
            task_i_row::spawn(i).ok();
        }
        calc_cpu::spawn();
        let mut timer = dp.TIM2.counter(&clocks);
        timer.start((300 as u32).secs()).ok();
        (
            Shared {
                timer,
                usart: serial,
                sleep_time: 0,
                concurrent_tasks,
                result_matrix: [0.0; crate::RESULT_MATRIX_ROWS
                    * crate::RESULT_MATRIX_COLUMNS],
                a_matrix: [
                    7986.45,
                    1292.79,
                    8583.79,
                    2072.98,
                    2161.08,
                    7137.87,
                    8844.89542,
                    1241.16699,
                    9333.09941,
                    1046.33654,
                    1766.28663,
                    227.57371,
                ],
                b_matrix: [
                    2089.46,
                    484.29,
                    4070.86,
                    6388.14,
                    5878.41,
                    7796.02,
                    4174.04910,
                    3779.38097,
                    1819.78879,
                    392.12370,
                    5173.20243,
                    2525.39435,
                    4430.77499,
                    3700.96781,
                    3312.82425,
                    2487.50948,
                    1680.72726,
                    1257.68497,
                    5224.09199,
                    5027.58651,
                    4620.30426,
                    7821.20553,
                    5898.87661,
                    3104.60453,
                    4500.93917,
                    3847.383526,
                    1115.46655,
                    1150.36475,
                ],
            },
            Local { last_timer_value: 0 },
            init::Monotonics(mono),
        )
    }
    /// User provided idle function
    #[allow(non_snake_case)]
    fn idle(mut ctx: idle::Context) -> ! {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        loop {
            cortex_m::interrupt::free(|_cs| {
                let start_time = ctx.shared.timer.lock(|timer| { time_us_64(timer) });
                rtic::export::wfi();
                let end_time = ctx.shared.timer.lock(|timer| { time_us_64(timer) });
                let sleep_time = end_time - start_time;
                ctx.shared
                    .sleep_time
                    .lock(|total_sleep_time| {
                        *total_sleep_time = *total_sleep_time + sleep_time;
                    });
            });
        }
    }
    /// User SW task calc_cpu
    #[allow(non_snake_case)]
    fn calc_cpu(mut ctx: calc_cpu::Context) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let end_time = ctx.shared.timer.lock(|timer| { time_us_64(timer) });
        let cpu_usage = (ctx.shared.sleep_time)
            .lock(|sleep_time| {
                let last_timer_value = *ctx.local.last_timer_value;
                let current_timer_value = end_time;
                let elapsed_time = current_timer_value - last_timer_value;
                let cpu_usage = (elapsed_time as f64 - *sleep_time as f64)
                    / (elapsed_time as f64);
                *sleep_time = 0;
                *ctx.local.last_timer_value = end_time;
                cpu_usage
            });
        calc_cpu::spawn_after((3 as u64).secs()).ok();
        ctx.shared
            .usart
            .lock(|usart| {
                usart
                    .write_fmt(format_args!("CPU usage: {0}\n", cpu_usage * 100 as f64))
                    .ok();
            });
    }
    /// User SW task task_i_row
    #[allow(non_snake_case)]
    fn task_i_row(ctx: task_i_row::Context, i: usize) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        for j in 0..crate::RESULT_MATRIX_COLUMNS {
            let mut tmp: f64 = 0.0;
            for k in 0..crate::A_MATRIX_COLUMNS {
                tmp = tmp
                    + ctx.shared.a_matrix[i * crate::A_MATRIX_COLUMNS + k]
                        * ctx.shared.b_matrix[k * crate::B_MATRIX_COLUMNS + j];
            }
            ctx.shared.result_matrix[i * crate::RESULT_MATRIX_COLUMNS + j] = tmp;
        }
        let n_tasks = ctx.shared.concurrent_tasks;
        let tim = ctx.shared.timer;
        let usart = ctx.shared.usart;
        (n_tasks, tim, usart)
            .lock(|n_tasks, tim, usart| {
                *n_tasks = *n_tasks - 1;
                if *n_tasks == 0 {
                    let end_time = time_us_64(tim);
                    usart
                        .write_fmt(
                            format_args!(
                                "End_time: {0:?} Matrix: {1:?}\n",
                                end_time as f64,
                                ctx.shared.result_matrix,
                            ),
                        )
                        .ok();
                }
            });
    }
    /// RTIC shared resource struct
    struct Shared {
        usart: Serial<USART1>,
        sleep_time: u64,
        timer: CounterUs<TIM2>,
        concurrent_tasks: u8,
        a_matrix: [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        b_matrix: [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        result_matrix: [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
    }
    /// RTIC local resource struct
    struct Local {
        last_timer_value: u64,
    }
    /// Monotonics used by the system
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_Monotonics(pub Systick<100>);
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_init_Context<'a> {
        /// Core (Cortex-M) peripherals
        pub core: rtic::export::Peripherals,
        /// Device peripherals
        pub device: stm32f4xx_hal::pac::Peripherals,
        /// Critical section token for init
        pub cs: rtic::export::CriticalSection<'a>,
    }
    impl<'a> __rtic_internal_init_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(core: rtic::export::Peripherals) -> Self {
            __rtic_internal_init_Context {
                device: stm32f4xx_hal::pac::Peripherals::steal(),
                cs: rtic::export::CriticalSection::new(),
                core,
            }
        }
    }
    #[allow(non_snake_case)]
    /// Initialization function
    pub mod init {
        #[doc(inline)]
        pub use super::__rtic_internal_Monotonics as Monotonics;
        #[doc(inline)]
        pub use super::__rtic_internal_init_Context as Context;
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Shared resources `idle` has access to
    pub struct __rtic_internal_idleSharedResources<'a> {
        /// Resource proxy resource `sleep_time`. Use method `.lock()` to gain access
        pub sleep_time: shared_resources::sleep_time_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `timer`. Use method `.lock()` to gain access
        pub timer: shared_resources::timer_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_idle_Context<'a> {
        /// Shared Resources this task has access to
        pub shared: idle::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_idle_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_idle_Context {
                shared: idle::SharedResources::new(priority),
            }
        }
    }
    #[allow(non_snake_case)]
    /// Idle loop
    pub mod idle {
        #[doc(inline)]
        pub use super::__rtic_internal_idleSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_idle_Context as Context;
    }
    mod shared_resources {
        use rtic::export::Priority;
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct usart_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> usart_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                usart_that_needs_to_be_locked {
                    priority,
                }
            }
            #[inline(always)]
            pub unsafe fn priority(&self) -> &Priority {
                self.priority
            }
        }
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct sleep_time_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> sleep_time_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                sleep_time_that_needs_to_be_locked {
                    priority,
                }
            }
            #[inline(always)]
            pub unsafe fn priority(&self) -> &Priority {
                self.priority
            }
        }
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct timer_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> timer_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                timer_that_needs_to_be_locked {
                    priority,
                }
            }
            #[inline(always)]
            pub unsafe fn priority(&self) -> &Priority {
                self.priority
            }
        }
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct concurrent_tasks_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> concurrent_tasks_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                concurrent_tasks_that_needs_to_be_locked {
                    priority,
                }
            }
            #[inline(always)]
            pub unsafe fn priority(&self) -> &Priority {
                self.priority
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Local resources `calc_cpu` has access to
    pub struct __rtic_internal_calc_cpuLocalResources<'a> {
        /// Local resource `last_timer_value`
        pub last_timer_value: &'a mut u64,
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Shared resources `calc_cpu` has access to
    pub struct __rtic_internal_calc_cpuSharedResources<'a> {
        /// Resource proxy resource `timer`. Use method `.lock()` to gain access
        pub timer: shared_resources::timer_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `sleep_time`. Use method `.lock()` to gain access
        pub sleep_time: shared_resources::sleep_time_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `usart`. Use method `.lock()` to gain access
        pub usart: shared_resources::usart_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_calc_cpu_Context<'a> {
        /// Local Resources this task has access to
        pub local: calc_cpu::LocalResources<'a>,
        /// Shared Resources this task has access to
        pub shared: calc_cpu::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_calc_cpu_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_calc_cpu_Context {
                local: calc_cpu::LocalResources::new(),
                shared: calc_cpu::SharedResources::new(priority),
            }
        }
    }
    /// Spawns the task directly
    pub fn __rtic_internal_calc_cpu_spawn() -> Result<(), ()> {
        let input = ();
        unsafe {
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_calc_cpu_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_calc_cpu_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                rtic::export::interrupt::free(|_| {
                    (&mut *__rtic_internal_P1_RQ.get_mut())
                        .enqueue_unchecked((P1_T::calc_cpu, index));
                });
                rtic::pend(stm32f4xx_hal::pac::interrupt::EXTI4);
                Ok(())
            } else {
                Err(input)
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_calc_cpu_Tonic_SpawnHandle {
        #[doc(hidden)]
        marker: u32,
    }
    impl core::fmt::Debug for __rtic_internal_calc_cpu_Tonic_SpawnHandle {
        #[doc(hidden)]
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("Tonic::SpawnHandle").finish()
        }
    }
    impl __rtic_internal_calc_cpu_Tonic_SpawnHandle {
        pub fn cancel(self) -> Result<(), ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                if let Some((_task, index)) = tq.cancel_marker(self.marker) {
                    let msg = (&*__rtic_internal_calc_cpu_INPUTS.get())
                        .get_unchecked(usize::from(index))
                        .as_ptr()
                        .read();
                    (&mut *__rtic_internal_calc_cpu_FQ.get_mut())
                        .split()
                        .0
                        .enqueue_unchecked(index);
                    Ok(msg)
                } else {
                    Err(())
                }
            })
        }
        /// Reschedule after
        #[inline]
        pub fn reschedule_after(
            self,
            duration: <Tonic as rtic::Monotonic>::Duration,
        ) -> Result<Self, ()> {
            self.reschedule_at(monotonics::Tonic::now() + duration)
        }
        /// Reschedule at
        pub fn reschedule_at(
            self,
            instant: <Tonic as rtic::Monotonic>::Instant,
        ) -> Result<Self, ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                __rtic_internal_TIMER_QUEUE_MARKER
                    .get_mut()
                    .write(marker.wrapping_add(1));
                let tq = (&mut *__rtic_internal_TQ_Tonic.get_mut());
                tq.update_marker(
                        self.marker,
                        marker,
                        instant,
                        || rtic::export::SCB::set_pendst(),
                    )
                    .map(|_| calc_cpu::Tonic::SpawnHandle {
                        marker,
                    })
            })
        }
    }
    /// Spawns the task after a set duration relative to the current time
    ///
    /// This will use the time `Instant::new(0)` as baseline if called in `#[init]`,
    /// so if you use a non-resetable timer use `spawn_at` when in `#[init]`
    #[allow(non_snake_case)]
    pub fn __rtic_internal_calc_cpu_Tonic_spawn_after(
        duration: <Tonic as rtic::Monotonic>::Duration,
    ) -> Result<calc_cpu::Tonic::SpawnHandle, ()> {
        let instant = monotonics::Tonic::now();
        __rtic_internal_calc_cpu_Tonic_spawn_at(instant + duration)
    }
    /// Spawns the task at a fixed time instant
    #[allow(non_snake_case)]
    pub fn __rtic_internal_calc_cpu_Tonic_spawn_at(
        instant: <Tonic as rtic::Monotonic>::Instant,
    ) -> Result<calc_cpu::Tonic::SpawnHandle, ()> {
        unsafe {
            let input = ();
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_calc_cpu_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_calc_cpu_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                (&mut *__rtic_internal_calc_cpu_Tonic_INSTANTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(instant);
                rtic::export::interrupt::free(|_| {
                    let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                    let nr = rtic::export::NotReady {
                        instant,
                        index,
                        task: SCHED_T::calc_cpu,
                        marker,
                    };
                    __rtic_internal_TIMER_QUEUE_MARKER
                        .get_mut()
                        .write(
                            __rtic_internal_TIMER_QUEUE_MARKER
                                .get()
                                .read()
                                .wrapping_add(1),
                        );
                    let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                    tq.enqueue_unchecked(
                        nr,
                        || {
                            core::mem::transmute::<_, rtic::export::SYST>(())
                                .enable_interrupt()
                        },
                        || rtic::export::SCB::set_pendst(),
                        (&mut *__rtic_internal_MONOTONIC_STORAGE_Tonic.get_mut())
                            .as_mut(),
                    );
                    Ok(calc_cpu::Tonic::SpawnHandle {
                        marker,
                    })
                })
            } else {
                Err(input)
            }
        }
    }
    #[allow(non_snake_case)]
    /// Software task
    pub mod calc_cpu {
        #[doc(inline)]
        pub use super::__rtic_internal_calc_cpuLocalResources as LocalResources;
        #[doc(inline)]
        pub use super::__rtic_internal_calc_cpuSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_calc_cpu_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_calc_cpu_spawn as spawn;
        pub use Tonic::spawn_after;
        pub use Tonic::spawn_at;
        pub use Tonic::SpawnHandle;
        #[doc(hidden)]
        pub mod Tonic {
            pub use super::super::__rtic_internal_calc_cpu_Tonic_spawn_after as spawn_after;
            pub use super::super::__rtic_internal_calc_cpu_Tonic_spawn_at as spawn_at;
            pub use super::super::__rtic_internal_calc_cpu_Tonic_SpawnHandle as SpawnHandle;
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Shared resources `task_i_row` has access to
    pub struct __rtic_internal_task_i_rowSharedResources<'a> {
        /// Lock free resource `result_matrix`
        pub result_matrix: &'a mut [f64; crate::RESULT_MATRIX_ROWS
            * crate::RESULT_MATRIX_COLUMNS],
        /// Lock free resource `a_matrix`
        pub a_matrix: &'a mut [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
        /// Lock free resource `b_matrix`
        pub b_matrix: &'a mut [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
        /// Resource proxy resource `concurrent_tasks`. Use method `.lock()` to gain access
        pub concurrent_tasks: shared_resources::concurrent_tasks_that_needs_to_be_locked<
            'a,
        >,
        /// Resource proxy resource `timer`. Use method `.lock()` to gain access
        pub timer: shared_resources::timer_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `usart`. Use method `.lock()` to gain access
        pub usart: shared_resources::usart_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_task_i_row_Context<'a> {
        /// Shared Resources this task has access to
        pub shared: task_i_row::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_task_i_row_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_task_i_row_Context {
                shared: task_i_row::SharedResources::new(priority),
            }
        }
    }
    /// Spawns the task directly
    pub fn __rtic_internal_task_i_row_spawn(_0: usize) -> Result<(), usize> {
        let input = _0;
        unsafe {
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_task_i_row_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_task_i_row_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                rtic::export::interrupt::free(|_| {
                    (&mut *__rtic_internal_P1_RQ.get_mut())
                        .enqueue_unchecked((P1_T::task_i_row, index));
                });
                rtic::pend(stm32f4xx_hal::pac::interrupt::EXTI4);
                Ok(())
            } else {
                Err(input)
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_task_i_row_Tonic_SpawnHandle {
        #[doc(hidden)]
        marker: u32,
    }
    impl core::fmt::Debug for __rtic_internal_task_i_row_Tonic_SpawnHandle {
        #[doc(hidden)]
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("Tonic::SpawnHandle").finish()
        }
    }
    impl __rtic_internal_task_i_row_Tonic_SpawnHandle {
        pub fn cancel(self) -> Result<usize, ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                if let Some((_task, index)) = tq.cancel_marker(self.marker) {
                    let msg = (&*__rtic_internal_task_i_row_INPUTS.get())
                        .get_unchecked(usize::from(index))
                        .as_ptr()
                        .read();
                    (&mut *__rtic_internal_task_i_row_FQ.get_mut())
                        .split()
                        .0
                        .enqueue_unchecked(index);
                    Ok(msg)
                } else {
                    Err(())
                }
            })
        }
        /// Reschedule after
        #[inline]
        pub fn reschedule_after(
            self,
            duration: <Tonic as rtic::Monotonic>::Duration,
        ) -> Result<Self, ()> {
            self.reschedule_at(monotonics::Tonic::now() + duration)
        }
        /// Reschedule at
        pub fn reschedule_at(
            self,
            instant: <Tonic as rtic::Monotonic>::Instant,
        ) -> Result<Self, ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                __rtic_internal_TIMER_QUEUE_MARKER
                    .get_mut()
                    .write(marker.wrapping_add(1));
                let tq = (&mut *__rtic_internal_TQ_Tonic.get_mut());
                tq.update_marker(
                        self.marker,
                        marker,
                        instant,
                        || rtic::export::SCB::set_pendst(),
                    )
                    .map(|_| task_i_row::Tonic::SpawnHandle {
                        marker,
                    })
            })
        }
    }
    /// Spawns the task after a set duration relative to the current time
    ///
    /// This will use the time `Instant::new(0)` as baseline if called in `#[init]`,
    /// so if you use a non-resetable timer use `spawn_at` when in `#[init]`
    #[allow(non_snake_case)]
    pub fn __rtic_internal_task_i_row_Tonic_spawn_after(
        duration: <Tonic as rtic::Monotonic>::Duration,
        _0: usize,
    ) -> Result<task_i_row::Tonic::SpawnHandle, usize> {
        let instant = monotonics::Tonic::now();
        __rtic_internal_task_i_row_Tonic_spawn_at(instant + duration, _0)
    }
    /// Spawns the task at a fixed time instant
    #[allow(non_snake_case)]
    pub fn __rtic_internal_task_i_row_Tonic_spawn_at(
        instant: <Tonic as rtic::Monotonic>::Instant,
        _0: usize,
    ) -> Result<task_i_row::Tonic::SpawnHandle, usize> {
        unsafe {
            let input = _0;
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_task_i_row_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_task_i_row_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                (&mut *__rtic_internal_task_i_row_Tonic_INSTANTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(instant);
                rtic::export::interrupt::free(|_| {
                    let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                    let nr = rtic::export::NotReady {
                        instant,
                        index,
                        task: SCHED_T::task_i_row,
                        marker,
                    };
                    __rtic_internal_TIMER_QUEUE_MARKER
                        .get_mut()
                        .write(
                            __rtic_internal_TIMER_QUEUE_MARKER
                                .get()
                                .read()
                                .wrapping_add(1),
                        );
                    let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                    tq.enqueue_unchecked(
                        nr,
                        || {
                            core::mem::transmute::<_, rtic::export::SYST>(())
                                .enable_interrupt()
                        },
                        || rtic::export::SCB::set_pendst(),
                        (&mut *__rtic_internal_MONOTONIC_STORAGE_Tonic.get_mut())
                            .as_mut(),
                    );
                    Ok(task_i_row::Tonic::SpawnHandle {
                        marker,
                    })
                })
            } else {
                Err(input)
            }
        }
    }
    #[allow(non_snake_case)]
    /// Software task
    pub mod task_i_row {
        #[doc(inline)]
        pub use super::__rtic_internal_task_i_rowSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_task_i_row_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_task_i_row_spawn as spawn;
        pub use Tonic::spawn_after;
        pub use Tonic::spawn_at;
        pub use Tonic::SpawnHandle;
        #[doc(hidden)]
        pub mod Tonic {
            pub use super::super::__rtic_internal_task_i_row_Tonic_spawn_after as spawn_after;
            pub use super::super::__rtic_internal_task_i_row_Tonic_spawn_at as spawn_at;
            pub use super::super::__rtic_internal_task_i_row_Tonic_SpawnHandle as SpawnHandle;
        }
    }
    /// App module
    impl<'a> __rtic_internal_idleSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_idleSharedResources {
                #[doc(hidden)]
                sleep_time: shared_resources::sleep_time_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                timer: shared_resources::timer_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic0"]
    static __rtic_internal_shared_resource_usart: rtic::RacyCell<
        core::mem::MaybeUninit<Serial<USART1>>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::usart_that_needs_to_be_locked<'a> {
        type T = Serial<USART1>;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut Serial<USART1>) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 1u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_usart.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f4xx_hal::pac::NVIC_PRIO_BITS,
                    &__rtic_internal_MASKS,
                    f,
                )
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic1"]
    static __rtic_internal_shared_resource_sleep_time: rtic::RacyCell<
        core::mem::MaybeUninit<u64>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::sleep_time_that_needs_to_be_locked<'a> {
        type T = u64;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut u64) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 1u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_sleep_time.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f4xx_hal::pac::NVIC_PRIO_BITS,
                    &__rtic_internal_MASKS,
                    f,
                )
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic2"]
    static __rtic_internal_shared_resource_timer: rtic::RacyCell<
        core::mem::MaybeUninit<CounterUs<TIM2>>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::timer_that_needs_to_be_locked<'a> {
        type T = CounterUs<TIM2>;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut CounterUs<TIM2>) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 1u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_timer.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f4xx_hal::pac::NVIC_PRIO_BITS,
                    &__rtic_internal_MASKS,
                    f,
                )
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic3"]
    static __rtic_internal_shared_resource_concurrent_tasks: rtic::RacyCell<
        core::mem::MaybeUninit<u8>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex
    for shared_resources::concurrent_tasks_that_needs_to_be_locked<'a> {
        type T = u8;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut u8) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 1u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_concurrent_tasks.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f4xx_hal::pac::NVIC_PRIO_BITS,
                    &__rtic_internal_MASKS,
                    f,
                )
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic4"]
    static __rtic_internal_shared_resource_a_matrix: rtic::RacyCell<
        core::mem::MaybeUninit<[f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS]>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic5"]
    static __rtic_internal_shared_resource_b_matrix: rtic::RacyCell<
        core::mem::MaybeUninit<[f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS]>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic6"]
    static __rtic_internal_shared_resource_result_matrix: rtic::RacyCell<
        core::mem::MaybeUninit<
            [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
        >,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    #[doc(hidden)]
    #[allow(non_upper_case_globals)]
    const __rtic_internal_MASK_CHUNKS: usize = rtic::export::compute_mask_chunks([
        stm32f4xx_hal::pac::Interrupt::EXTI4 as u32,
    ]);
    #[doc(hidden)]
    #[allow(non_upper_case_globals)]
    const __rtic_internal_MASKS: [rtic::export::Mask<__rtic_internal_MASK_CHUNKS>; 3] = [
        rtic::export::create_mask([stm32f4xx_hal::pac::Interrupt::EXTI4 as u32]),
        rtic::export::create_mask([]),
        rtic::export::create_mask([]),
    ];
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic7"]
    static __rtic_internal_local_resource_last_timer_value: rtic::RacyCell<
        core::mem::MaybeUninit<u64>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_calc_cpu_FQ: rtic::RacyCell<rtic::export::SCFQ<2>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[link_section = ".uninit.rtic8"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_calc_cpu_Tonic_INSTANTS: rtic::RacyCell<
        [core::mem::MaybeUninit<<Systick<100> as rtic::Monotonic>::Instant>; 1],
    > = rtic::RacyCell::new([core::mem::MaybeUninit::uninit()]);
    #[link_section = ".uninit.rtic9"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_calc_cpu_INPUTS: rtic::RacyCell<
        [core::mem::MaybeUninit<()>; 1],
    > = rtic::RacyCell::new([core::mem::MaybeUninit::uninit()]);
    impl<'a> __rtic_internal_calc_cpuLocalResources<'a> {
        #[inline(always)]
        #[doc(hidden)]
        pub unsafe fn new() -> Self {
            __rtic_internal_calc_cpuLocalResources {
                last_timer_value: &mut *(&mut *__rtic_internal_local_resource_last_timer_value
                    .get_mut())
                    .as_mut_ptr(),
            }
        }
    }
    impl<'a> __rtic_internal_calc_cpuSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_calc_cpuSharedResources {
                #[doc(hidden)]
                timer: shared_resources::timer_that_needs_to_be_locked::new(priority),
                #[doc(hidden)]
                sleep_time: shared_resources::sleep_time_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                usart: shared_resources::usart_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_task_i_row_FQ: rtic::RacyCell<rtic::export::SCFQ<16>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[link_section = ".uninit.rtic10"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_task_i_row_Tonic_INSTANTS: rtic::RacyCell<
        [core::mem::MaybeUninit<<Systick<100> as rtic::Monotonic>::Instant>; 15],
    > = rtic::RacyCell::new([
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
    ]);
    #[link_section = ".uninit.rtic11"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_task_i_row_INPUTS: rtic::RacyCell<
        [core::mem::MaybeUninit<usize>; 15],
    > = rtic::RacyCell::new([
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
        core::mem::MaybeUninit::uninit(),
    ]);
    impl<'a> __rtic_internal_task_i_rowSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_task_i_rowSharedResources {
                /// Exclusive access resource `result_matrix`
                result_matrix: &mut *(&mut *__rtic_internal_shared_resource_result_matrix
                    .get_mut())
                    .as_mut_ptr(),
                /// Exclusive access resource `a_matrix`
                a_matrix: &mut *(&mut *__rtic_internal_shared_resource_a_matrix
                    .get_mut())
                    .as_mut_ptr(),
                /// Exclusive access resource `b_matrix`
                b_matrix: &mut *(&mut *__rtic_internal_shared_resource_b_matrix
                    .get_mut())
                    .as_mut_ptr(),
                #[doc(hidden)]
                concurrent_tasks: shared_resources::concurrent_tasks_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                timer: shared_resources::timer_that_needs_to_be_locked::new(priority),
                #[doc(hidden)]
                usart: shared_resources::usart_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    #[doc(hidden)]
    pub enum P1_T {
        calc_cpu,
        task_i_row,
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::clone::Clone for P1_T {
        #[inline]
        fn clone(&self) -> P1_T {
            *self
        }
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::marker::Copy for P1_T {}
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_P1_RQ: rtic::RacyCell<rtic::export::SCRQ<P1_T, 17>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch tasks at priority 1
    #[no_mangle]
    unsafe fn EXTI4() {
        /// The priority of this interrupt handler
        const PRIORITY: u8 = 1u8;
        rtic::export::run(
            PRIORITY,
            || {
                while let Some((task, index)) = (&mut *__rtic_internal_P1_RQ.get_mut())
                    .split()
                    .1
                    .dequeue()
                {
                    match task {
                        P1_T::calc_cpu => {
                            let () = (&*__rtic_internal_calc_cpu_INPUTS.get())
                                .get_unchecked(usize::from(index))
                                .as_ptr()
                                .read();
                            (&mut *__rtic_internal_calc_cpu_FQ.get_mut())
                                .split()
                                .0
                                .enqueue_unchecked(index);
                            let priority = &rtic::export::Priority::new(PRIORITY);
                            calc_cpu(calc_cpu::Context::new(priority))
                        }
                        P1_T::task_i_row => {
                            let _0 = (&*__rtic_internal_task_i_row_INPUTS.get())
                                .get_unchecked(usize::from(index))
                                .as_ptr()
                                .read();
                            (&mut *__rtic_internal_task_i_row_FQ.get_mut())
                                .split()
                                .0
                                .enqueue_unchecked(index);
                            let priority = &rtic::export::Priority::new(PRIORITY);
                            task_i_row(task_i_row::Context::new(priority), _0)
                        }
                    }
                }
            },
        );
    }
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_TIMER_QUEUE_MARKER: rtic::RacyCell<u32> = rtic::RacyCell::new(
        0,
    );
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    pub enum SCHED_T {
        calc_cpu,
        task_i_row,
    }
    #[automatically_derived]
    #[allow(non_camel_case_types)]
    impl ::core::clone::Clone for SCHED_T {
        #[inline]
        fn clone(&self) -> SCHED_T {
            *self
        }
    }
    #[automatically_derived]
    #[allow(non_camel_case_types)]
    impl ::core::marker::Copy for SCHED_T {}
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_TQ_Tonic: rtic::RacyCell<
        rtic::export::TimerQueue<Systick<100>, SCHED_T, 16>,
    > = rtic::RacyCell::new(
        rtic::export::TimerQueue(rtic::export::SortedLinkedList::new_u16()),
    );
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_MONOTONIC_STORAGE_Tonic: rtic::RacyCell<
        Option<Systick<100>>,
    > = rtic::RacyCell::new(None);
    #[no_mangle]
    #[allow(non_snake_case)]
    unsafe fn SysTick() {
        while let Some((task, index)) = rtic::export::interrupt::free(|_| {
            if let Some(mono) = (&mut *__rtic_internal_MONOTONIC_STORAGE_Tonic.get_mut())
                .as_mut()
            {
                (&mut *__rtic_internal_TQ_Tonic.get_mut())
                    .dequeue(
                        || {
                            core::mem::transmute::<_, rtic::export::SYST>(())
                                .disable_interrupt()
                        },
                        mono,
                    )
            } else {
                core::hint::unreachable_unchecked()
            }
        }) {
            match task {
                SCHED_T::calc_cpu => {
                    rtic::export::interrupt::free(|_| {
                        (&mut *__rtic_internal_P1_RQ.get_mut())
                            .split()
                            .0
                            .enqueue_unchecked((P1_T::calc_cpu, index))
                    });
                    rtic::pend(
                        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EXTI4,
                    );
                }
                SCHED_T::task_i_row => {
                    rtic::export::interrupt::free(|_| {
                        (&mut *__rtic_internal_P1_RQ.get_mut())
                            .split()
                            .0
                            .enqueue_unchecked((P1_T::task_i_row, index))
                    });
                    rtic::pend(
                        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EXTI4,
                    );
                }
            }
        }
        rtic::export::interrupt::free(|_| {
            if let Some(mono) = (&mut *__rtic_internal_MONOTONIC_STORAGE_Tonic.get_mut())
                .as_mut()
            {
                mono.on_interrupt();
            }
        });
    }
    #[doc(hidden)]
    mod rtic_ext {
        use super::*;
        #[no_mangle]
        unsafe extern "C" fn main() -> ! {
            rtic::export::assert_send::<Serial<USART1>>();
            rtic::export::assert_send::<u64>();
            rtic::export::assert_send::<CounterUs<TIM2>>();
            rtic::export::assert_send::<u8>();
            rtic::export::assert_send::<
                [f64; crate::A_MATRIX_ROWS * crate::A_MATRIX_COLUMNS],
            >();
            rtic::export::assert_send::<
                [f64; crate::B_MATRIX_ROWS * crate::B_MATRIX_COLUMNS],
            >();
            rtic::export::assert_send::<
                [f64; crate::RESULT_MATRIX_ROWS * crate::RESULT_MATRIX_COLUMNS],
            >();
            rtic::export::assert_send::<usize>();
            rtic::export::assert_monotonic::<Systick<100>>();
            const _CONST_CHECK: () = { if !rtic::export::have_basepri() {} else {} };
            let _ = _CONST_CHECK;
            rtic::export::interrupt::disable();
            (0..1u8)
                .for_each(|i| {
                    (&mut *__rtic_internal_calc_cpu_FQ.get_mut()).enqueue_unchecked(i)
                });
            (0..15u8)
                .for_each(|i| {
                    (&mut *__rtic_internal_task_i_row_FQ.get_mut()).enqueue_unchecked(i)
                });
            let mut core: rtic::export::Peripherals = rtic::export::Peripherals::steal()
                .into();
            let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EXTI4;
            const _: () = if (1 << stm32f4xx_hal::pac::NVIC_PRIO_BITS) < 1u8 as usize {
                {
                    ::core::panicking::panic_fmt(
                        format_args!(
                            "Maximum priority used by interrupt vector \'EXTI4\' is more than supported by hardware",
                        ),
                    );
                };
            };
            core.NVIC
                .set_priority(
                    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EXTI4,
                    rtic::export::logical2hw(1u8, stm32f4xx_hal::pac::NVIC_PRIO_BITS),
                );
            rtic::export::NVIC::unmask(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::EXTI4,
            );
            const _: () = if (1 << stm32f4xx_hal::pac::NVIC_PRIO_BITS)
                < (1 << stm32f4xx_hal::pac::NVIC_PRIO_BITS) as usize
            {
                {
                    ::core::panicking::panic_fmt(
                        format_args!(
                            "Maximum priority used by monotonic \'Tonic\' is more than supported by hardware",
                        ),
                    );
                };
            };
            core.SCB
                .set_priority(
                    rtic::export::SystemHandler::SysTick,
                    rtic::export::logical2hw(
                        (1 << stm32f4xx_hal::pac::NVIC_PRIO_BITS),
                        stm32f4xx_hal::pac::NVIC_PRIO_BITS,
                    ),
                );
            if !<Systick<100> as rtic::Monotonic>::DISABLE_INTERRUPT_ON_EMPTY_QUEUE {
                core::mem::transmute::<_, rtic::export::SYST>(()).enable_interrupt();
            }
            #[inline(never)]
            fn __rtic_init_resources<F>(f: F)
            where
                F: FnOnce(),
            {
                f();
            }
            __rtic_init_resources(|| {
                let (shared_resources, local_resources, mut monotonics) = init(
                    init::Context::new(core.into()),
                );
                __rtic_internal_shared_resource_usart
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.usart));
                __rtic_internal_shared_resource_sleep_time
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.sleep_time));
                __rtic_internal_shared_resource_timer
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.timer));
                __rtic_internal_shared_resource_concurrent_tasks
                    .get_mut()
                    .write(
                        core::mem::MaybeUninit::new(shared_resources.concurrent_tasks),
                    );
                __rtic_internal_shared_resource_a_matrix
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.a_matrix));
                __rtic_internal_shared_resource_b_matrix
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.b_matrix));
                __rtic_internal_shared_resource_result_matrix
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.result_matrix));
                __rtic_internal_local_resource_last_timer_value
                    .get_mut()
                    .write(
                        core::mem::MaybeUninit::new(local_resources.last_timer_value),
                    );
                monotonics.0.reset();
                __rtic_internal_MONOTONIC_STORAGE_Tonic
                    .get_mut()
                    .write(Some(monotonics.0));
                rtic::export::interrupt::enable();
            });
            idle(idle::Context::new(&rtic::export::Priority::new(0)))
        }
    }
}
