#![feature(prelude_import)]
#![feature(prelude_import)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(rustc_private)]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;
use test_app as _;
/// The RTIC application module
pub mod app {
    /// Always include the device crate which contains the vector table
    use stm32f0xx_hal::pac as you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
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
    use stm32f0xx_hal::pac::TIM2;
    use systick_monotonic::Systick;
    use stm32f0xx_hal::prelude::*;
    use test_app::{setup_tim2, time_us};
    /// User code from within the module
    type Tonic = Systick<100000>;
    fn increase(num: &mut u16, tim: &mut TIM2, done: &mut bool) {
        if *num == u16::MAX && !*done {
            let end = time_us(tim);
            *done = true;
        } else if !*done {
            *num = *num + 1;
        }
    }
    /// User code end
    /// User provided init function
    #[inline(always)]
    #[allow(non_snake_case)]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut p = ctx.device;
        let cp = ctx.core;
        p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        p.RCC.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);
        let tim2 = p.TIM2;
        setup_tim2(&tim2, &rcc.clocks, 1.mhz());
        let mono = Systick::new(cp.SYST, test_app::SYSTICK_FREQ);
        for _ in 0..10 {
            low_priority_task::spawn().ok();
            medium_priority_task::spawn().ok();
            high_priority_task::spawn().ok();
        }
        (
            Shared {
                shared_num: 1,
                tim2,
                done: false,
            },
            Local {},
            init::Monotonics(mono),
        )
    }
    /// User SW task low_priority_task
    #[allow(non_snake_case)]
    fn low_priority_task(ctx: low_priority_task::Context) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        (shared_num, tim2, done)
            .lock(|num, tim2, done| {
                increase(num, tim2, done);
            });
        low_priority_task::spawn_after(systick_monotonic::ExtU64::micros(50)).ok();
    }
    /// User SW task medium_priority_task
    #[allow(non_snake_case)]
    fn medium_priority_task(ctx: medium_priority_task::Context) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        (shared_num, tim2, done)
            .lock(|num, tim2, done| {
                increase(num, tim2, done);
            });
        medium_priority_task::spawn_after(systick_monotonic::ExtU64::micros(100)).ok();
    }
    /// User SW task high_priority_task
    #[allow(non_snake_case)]
    fn high_priority_task(ctx: high_priority_task::Context) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let tim2 = ctx.shared.tim2;
        let shared_num = ctx.shared.shared_num;
        let done = ctx.shared.done;
        (shared_num, tim2, done)
            .lock(|num, tim2, done| {
                increase(num, tim2, done);
            });
        high_priority_task::spawn_after(systick_monotonic::ExtU64::micros(150)).ok();
    }
    /// RTIC shared resource struct
    struct Shared {
        shared_num: u16,
        tim2: TIM2,
        done: bool,
    }
    /// RTIC local resource struct
    struct Local {}
    /// Monotonics used by the system
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_Monotonics(pub Systick<100000>);
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_init_Context<'a> {
        /// Core (Cortex-M) peripherals
        pub core: rtic::export::Peripherals,
        /// Device peripherals
        pub device: stm32f0xx_hal::pac::Peripherals,
        /// Critical section token for init
        pub cs: rtic::export::CriticalSection<'a>,
    }
    impl<'a> __rtic_internal_init_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(core: rtic::export::Peripherals) -> Self {
            __rtic_internal_init_Context {
                device: stm32f0xx_hal::pac::Peripherals::steal(),
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
    mod shared_resources {
        use rtic::export::Priority;
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct shared_num_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> shared_num_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                shared_num_that_needs_to_be_locked {
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
        pub struct tim2_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> tim2_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                tim2_that_needs_to_be_locked {
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
        pub struct done_that_needs_to_be_locked<'a> {
            priority: &'a Priority,
        }
        impl<'a> done_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new(priority: &'a Priority) -> Self {
                done_that_needs_to_be_locked {
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
    /// Shared resources `low_priority_task` has access to
    pub struct __rtic_internal_low_priority_taskSharedResources<'a> {
        /// Resource proxy resource `shared_num`. Use method `.lock()` to gain access
        pub shared_num: shared_resources::shared_num_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `tim2`. Use method `.lock()` to gain access
        pub tim2: shared_resources::tim2_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `done`. Use method `.lock()` to gain access
        pub done: shared_resources::done_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_low_priority_task_Context<'a> {
        /// Shared Resources this task has access to
        pub shared: low_priority_task::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_low_priority_task_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_low_priority_task_Context {
                shared: low_priority_task::SharedResources::new(priority),
            }
        }
    }
    /// Spawns the task directly
    pub fn __rtic_internal_low_priority_task_spawn() -> Result<(), ()> {
        let input = ();
        unsafe {
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_low_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_low_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                rtic::export::interrupt::free(|_| {
                    (&mut *__rtic_internal_P1_RQ.get_mut())
                        .enqueue_unchecked((P1_T::low_priority_task, index));
                });
                rtic::pend(stm32f0xx_hal::pac::interrupt::USART3_4_5_6_7_8);
                Ok(())
            } else {
                Err(input)
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_low_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        marker: u32,
    }
    impl core::fmt::Debug for __rtic_internal_low_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("Tonic::SpawnHandle").finish()
        }
    }
    impl __rtic_internal_low_priority_task_Tonic_SpawnHandle {
        pub fn cancel(self) -> Result<(), ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                if let Some((_task, index)) = tq.cancel_marker(self.marker) {
                    let msg = (&*__rtic_internal_low_priority_task_INPUTS.get())
                        .get_unchecked(usize::from(index))
                        .as_ptr()
                        .read();
                    (&mut *__rtic_internal_low_priority_task_FQ.get_mut())
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
                    .map(|_| low_priority_task::Tonic::SpawnHandle {
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
    pub fn __rtic_internal_low_priority_task_Tonic_spawn_after(
        duration: <Tonic as rtic::Monotonic>::Duration,
    ) -> Result<low_priority_task::Tonic::SpawnHandle, ()> {
        let instant = monotonics::Tonic::now();
        __rtic_internal_low_priority_task_Tonic_spawn_at(instant + duration)
    }
    /// Spawns the task at a fixed time instant
    #[allow(non_snake_case)]
    pub fn __rtic_internal_low_priority_task_Tonic_spawn_at(
        instant: <Tonic as rtic::Monotonic>::Instant,
    ) -> Result<low_priority_task::Tonic::SpawnHandle, ()> {
        unsafe {
            let input = ();
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_low_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_low_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                (&mut *__rtic_internal_low_priority_task_Tonic_INSTANTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(instant);
                rtic::export::interrupt::free(|_| {
                    let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                    let nr = rtic::export::NotReady {
                        instant,
                        index,
                        task: SCHED_T::low_priority_task,
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
                    Ok(low_priority_task::Tonic::SpawnHandle {
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
    pub mod low_priority_task {
        #[doc(inline)]
        pub use super::__rtic_internal_low_priority_taskSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_low_priority_task_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_low_priority_task_spawn as spawn;
        pub use Tonic::spawn_after;
        pub use Tonic::spawn_at;
        pub use Tonic::SpawnHandle;
        #[doc(hidden)]
        pub mod Tonic {
            pub use super::super::__rtic_internal_low_priority_task_Tonic_spawn_after as spawn_after;
            pub use super::super::__rtic_internal_low_priority_task_Tonic_spawn_at as spawn_at;
            pub use super::super::__rtic_internal_low_priority_task_Tonic_SpawnHandle as SpawnHandle;
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Shared resources `medium_priority_task` has access to
    pub struct __rtic_internal_medium_priority_taskSharedResources<'a> {
        /// Resource proxy resource `shared_num`. Use method `.lock()` to gain access
        pub shared_num: shared_resources::shared_num_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `tim2`. Use method `.lock()` to gain access
        pub tim2: shared_resources::tim2_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `done`. Use method `.lock()` to gain access
        pub done: shared_resources::done_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_medium_priority_task_Context<'a> {
        /// Shared Resources this task has access to
        pub shared: medium_priority_task::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_medium_priority_task_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_medium_priority_task_Context {
                shared: medium_priority_task::SharedResources::new(priority),
            }
        }
    }
    /// Spawns the task directly
    pub fn __rtic_internal_medium_priority_task_spawn() -> Result<(), ()> {
        let input = ();
        unsafe {
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_medium_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_medium_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                rtic::export::interrupt::free(|_| {
                    (&mut *__rtic_internal_P2_RQ.get_mut())
                        .enqueue_unchecked((P2_T::medium_priority_task, index));
                });
                rtic::pend(stm32f0xx_hal::pac::interrupt::USART2);
                Ok(())
            } else {
                Err(input)
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_medium_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        marker: u32,
    }
    impl core::fmt::Debug for __rtic_internal_medium_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("Tonic::SpawnHandle").finish()
        }
    }
    impl __rtic_internal_medium_priority_task_Tonic_SpawnHandle {
        pub fn cancel(self) -> Result<(), ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                if let Some((_task, index)) = tq.cancel_marker(self.marker) {
                    let msg = (&*__rtic_internal_medium_priority_task_INPUTS.get())
                        .get_unchecked(usize::from(index))
                        .as_ptr()
                        .read();
                    (&mut *__rtic_internal_medium_priority_task_FQ.get_mut())
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
                    .map(|_| medium_priority_task::Tonic::SpawnHandle {
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
    pub fn __rtic_internal_medium_priority_task_Tonic_spawn_after(
        duration: <Tonic as rtic::Monotonic>::Duration,
    ) -> Result<medium_priority_task::Tonic::SpawnHandle, ()> {
        let instant = monotonics::Tonic::now();
        __rtic_internal_medium_priority_task_Tonic_spawn_at(instant + duration)
    }
    /// Spawns the task at a fixed time instant
    #[allow(non_snake_case)]
    pub fn __rtic_internal_medium_priority_task_Tonic_spawn_at(
        instant: <Tonic as rtic::Monotonic>::Instant,
    ) -> Result<medium_priority_task::Tonic::SpawnHandle, ()> {
        unsafe {
            let input = ();
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_medium_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_medium_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                (&mut *__rtic_internal_medium_priority_task_Tonic_INSTANTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(instant);
                rtic::export::interrupt::free(|_| {
                    let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                    let nr = rtic::export::NotReady {
                        instant,
                        index,
                        task: SCHED_T::medium_priority_task,
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
                    Ok(medium_priority_task::Tonic::SpawnHandle {
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
    pub mod medium_priority_task {
        #[doc(inline)]
        pub use super::__rtic_internal_medium_priority_taskSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_medium_priority_task_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_medium_priority_task_spawn as spawn;
        pub use Tonic::spawn_after;
        pub use Tonic::spawn_at;
        pub use Tonic::SpawnHandle;
        #[doc(hidden)]
        pub mod Tonic {
            pub use super::super::__rtic_internal_medium_priority_task_Tonic_spawn_after as spawn_after;
            pub use super::super::__rtic_internal_medium_priority_task_Tonic_spawn_at as spawn_at;
            pub use super::super::__rtic_internal_medium_priority_task_Tonic_SpawnHandle as SpawnHandle;
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    /// Shared resources `high_priority_task` has access to
    pub struct __rtic_internal_high_priority_taskSharedResources<'a> {
        /// Resource proxy resource `shared_num`. Use method `.lock()` to gain access
        pub shared_num: shared_resources::shared_num_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `tim2`. Use method `.lock()` to gain access
        pub tim2: shared_resources::tim2_that_needs_to_be_locked<'a>,
        /// Resource proxy resource `done`. Use method `.lock()` to gain access
        pub done: shared_resources::done_that_needs_to_be_locked<'a>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_high_priority_task_Context<'a> {
        /// Shared Resources this task has access to
        pub shared: high_priority_task::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_high_priority_task_Context<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_high_priority_task_Context {
                shared: high_priority_task::SharedResources::new(priority),
            }
        }
    }
    /// Spawns the task directly
    pub fn __rtic_internal_high_priority_task_spawn() -> Result<(), ()> {
        let input = ();
        unsafe {
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_high_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_high_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                rtic::export::interrupt::free(|_| {
                    (&mut *__rtic_internal_P3_RQ.get_mut())
                        .enqueue_unchecked((P3_T::high_priority_task, index));
                });
                rtic::pend(stm32f0xx_hal::pac::interrupt::USART1);
                Ok(())
            } else {
                Err(input)
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_high_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        marker: u32,
    }
    impl core::fmt::Debug for __rtic_internal_high_priority_task_Tonic_SpawnHandle {
        #[doc(hidden)]
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("Tonic::SpawnHandle").finish()
        }
    }
    impl __rtic_internal_high_priority_task_Tonic_SpawnHandle {
        pub fn cancel(self) -> Result<(), ()> {
            rtic::export::interrupt::free(|_| unsafe {
                let tq = &mut *__rtic_internal_TQ_Tonic.get_mut();
                if let Some((_task, index)) = tq.cancel_marker(self.marker) {
                    let msg = (&*__rtic_internal_high_priority_task_INPUTS.get())
                        .get_unchecked(usize::from(index))
                        .as_ptr()
                        .read();
                    (&mut *__rtic_internal_high_priority_task_FQ.get_mut())
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
                    .map(|_| high_priority_task::Tonic::SpawnHandle {
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
    pub fn __rtic_internal_high_priority_task_Tonic_spawn_after(
        duration: <Tonic as rtic::Monotonic>::Duration,
    ) -> Result<high_priority_task::Tonic::SpawnHandle, ()> {
        let instant = monotonics::Tonic::now();
        __rtic_internal_high_priority_task_Tonic_spawn_at(instant + duration)
    }
    /// Spawns the task at a fixed time instant
    #[allow(non_snake_case)]
    pub fn __rtic_internal_high_priority_task_Tonic_spawn_at(
        instant: <Tonic as rtic::Monotonic>::Instant,
    ) -> Result<high_priority_task::Tonic::SpawnHandle, ()> {
        unsafe {
            let input = ();
            if let Some(index) = rtic::export::interrupt::free(|_| {
                (&mut *__rtic_internal_high_priority_task_FQ.get_mut()).dequeue()
            }) {
                (&mut *__rtic_internal_high_priority_task_INPUTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(input);
                (&mut *__rtic_internal_high_priority_task_Tonic_INSTANTS.get_mut())
                    .get_unchecked_mut(usize::from(index))
                    .as_mut_ptr()
                    .write(instant);
                rtic::export::interrupt::free(|_| {
                    let marker = __rtic_internal_TIMER_QUEUE_MARKER.get().read();
                    let nr = rtic::export::NotReady {
                        instant,
                        index,
                        task: SCHED_T::high_priority_task,
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
                    Ok(high_priority_task::Tonic::SpawnHandle {
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
    pub mod high_priority_task {
        #[doc(inline)]
        pub use super::__rtic_internal_high_priority_taskSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_high_priority_task_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_high_priority_task_spawn as spawn;
        pub use Tonic::spawn_after;
        pub use Tonic::spawn_at;
        pub use Tonic::SpawnHandle;
        #[doc(hidden)]
        pub mod Tonic {
            pub use super::super::__rtic_internal_high_priority_task_Tonic_spawn_after as spawn_after;
            pub use super::super::__rtic_internal_high_priority_task_Tonic_spawn_at as spawn_at;
            pub use super::super::__rtic_internal_high_priority_task_Tonic_SpawnHandle as SpawnHandle;
        }
    }
    /// App module
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic0"]
    static __rtic_internal_shared_resource_shared_num: rtic::RacyCell<
        core::mem::MaybeUninit<u16>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::shared_num_that_needs_to_be_locked<'a> {
        type T = u16;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut u16) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 3u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_shared_num.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f0xx_hal::pac::NVIC_PRIO_BITS,
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
    static __rtic_internal_shared_resource_tim2: rtic::RacyCell<
        core::mem::MaybeUninit<TIM2>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::tim2_that_needs_to_be_locked<'a> {
        type T = TIM2;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut TIM2) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 3u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_tim2.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f0xx_hal::pac::NVIC_PRIO_BITS,
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
    static __rtic_internal_shared_resource_done: rtic::RacyCell<
        core::mem::MaybeUninit<bool>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::done_that_needs_to_be_locked<'a> {
        type T = bool;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut bool) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 3u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_done.get_mut() as *mut _,
                    self.priority(),
                    CEILING,
                    stm32f0xx_hal::pac::NVIC_PRIO_BITS,
                    &__rtic_internal_MASKS,
                    f,
                )
            }
        }
    }
    #[doc(hidden)]
    #[allow(non_upper_case_globals)]
    const __rtic_internal_MASK_CHUNKS: usize = rtic::export::compute_mask_chunks([
        stm32f0xx_hal::pac::Interrupt::USART3_4_5_6_7_8 as u32,
        stm32f0xx_hal::pac::Interrupt::USART2 as u32,
        stm32f0xx_hal::pac::Interrupt::USART1 as u32,
    ]);
    #[doc(hidden)]
    #[allow(non_upper_case_globals)]
    const __rtic_internal_MASKS: [rtic::export::Mask<__rtic_internal_MASK_CHUNKS>; 3] = [
        rtic::export::create_mask([
            stm32f0xx_hal::pac::Interrupt::USART3_4_5_6_7_8 as u32,
        ]),
        rtic::export::create_mask([stm32f0xx_hal::pac::Interrupt::USART2 as u32]),
        rtic::export::create_mask([stm32f0xx_hal::pac::Interrupt::USART1 as u32]),
    ];
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_low_priority_task_FQ: rtic::RacyCell<
        rtic::export::SCFQ<11>,
    > = rtic::RacyCell::new(rtic::export::Queue::new());
    #[link_section = ".uninit.rtic3"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_low_priority_task_Tonic_INSTANTS: rtic::RacyCell<
        [core::mem::MaybeUninit<<Systick<100000> as rtic::Monotonic>::Instant>; 10],
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
    ]);
    #[link_section = ".uninit.rtic4"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_low_priority_task_INPUTS: rtic::RacyCell<
        [core::mem::MaybeUninit<()>; 10],
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
    ]);
    impl<'a> __rtic_internal_low_priority_taskSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_low_priority_taskSharedResources {
                #[doc(hidden)]
                shared_num: shared_resources::shared_num_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                tim2: shared_resources::tim2_that_needs_to_be_locked::new(priority),
                #[doc(hidden)]
                done: shared_resources::done_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_medium_priority_task_FQ: rtic::RacyCell<
        rtic::export::SCFQ<101>,
    > = rtic::RacyCell::new(rtic::export::Queue::new());
    #[link_section = ".uninit.rtic5"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_medium_priority_task_Tonic_INSTANTS: rtic::RacyCell<
        [core::mem::MaybeUninit<<Systick<100000> as rtic::Monotonic>::Instant>; 100],
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
    #[link_section = ".uninit.rtic6"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_medium_priority_task_INPUTS: rtic::RacyCell<
        [core::mem::MaybeUninit<()>; 100],
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
    impl<'a> __rtic_internal_medium_priority_taskSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_medium_priority_taskSharedResources {
                #[doc(hidden)]
                shared_num: shared_resources::shared_num_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                tim2: shared_resources::tim2_that_needs_to_be_locked::new(priority),
                #[doc(hidden)]
                done: shared_resources::done_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_high_priority_task_FQ: rtic::RacyCell<
        rtic::export::SCFQ<11>,
    > = rtic::RacyCell::new(rtic::export::Queue::new());
    #[link_section = ".uninit.rtic7"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_high_priority_task_Tonic_INSTANTS: rtic::RacyCell<
        [core::mem::MaybeUninit<<Systick<100000> as rtic::Monotonic>::Instant>; 10],
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
    ]);
    #[link_section = ".uninit.rtic8"]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    static __rtic_internal_high_priority_task_INPUTS: rtic::RacyCell<
        [core::mem::MaybeUninit<()>; 10],
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
    ]);
    impl<'a> __rtic_internal_high_priority_taskSharedResources<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            __rtic_internal_high_priority_taskSharedResources {
                #[doc(hidden)]
                shared_num: shared_resources::shared_num_that_needs_to_be_locked::new(
                    priority,
                ),
                #[doc(hidden)]
                tim2: shared_resources::tim2_that_needs_to_be_locked::new(priority),
                #[doc(hidden)]
                done: shared_resources::done_that_needs_to_be_locked::new(priority),
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    #[doc(hidden)]
    pub enum P1_T {
        low_priority_task,
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
    static __rtic_internal_P1_RQ: rtic::RacyCell<rtic::export::SCRQ<P1_T, 11>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch tasks at priority 1
    #[no_mangle]
    unsafe fn USART3_4_5_6_7_8() {
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
                        P1_T::low_priority_task => {
                            let () = (&*__rtic_internal_low_priority_task_INPUTS.get())
                                .get_unchecked(usize::from(index))
                                .as_ptr()
                                .read();
                            (&mut *__rtic_internal_low_priority_task_FQ.get_mut())
                                .split()
                                .0
                                .enqueue_unchecked(index);
                            let priority = &rtic::export::Priority::new(PRIORITY);
                            low_priority_task(low_priority_task::Context::new(priority))
                        }
                    }
                }
            },
        );
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    #[doc(hidden)]
    pub enum P2_T {
        medium_priority_task,
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::clone::Clone for P2_T {
        #[inline]
        fn clone(&self) -> P2_T {
            *self
        }
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::marker::Copy for P2_T {}
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_P2_RQ: rtic::RacyCell<rtic::export::SCRQ<P2_T, 101>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch tasks at priority 2
    #[no_mangle]
    unsafe fn USART2() {
        /// The priority of this interrupt handler
        const PRIORITY: u8 = 2u8;
        rtic::export::run(
            PRIORITY,
            || {
                while let Some((task, index)) = (&mut *__rtic_internal_P2_RQ.get_mut())
                    .split()
                    .1
                    .dequeue()
                {
                    match task {
                        P2_T::medium_priority_task => {
                            let () = (&*__rtic_internal_medium_priority_task_INPUTS
                                .get())
                                .get_unchecked(usize::from(index))
                                .as_ptr()
                                .read();
                            (&mut *__rtic_internal_medium_priority_task_FQ.get_mut())
                                .split()
                                .0
                                .enqueue_unchecked(index);
                            let priority = &rtic::export::Priority::new(PRIORITY);
                            medium_priority_task(
                                medium_priority_task::Context::new(priority),
                            )
                        }
                    }
                }
            },
        );
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    #[doc(hidden)]
    pub enum P3_T {
        high_priority_task,
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::clone::Clone for P3_T {
        #[inline]
        fn clone(&self) -> P3_T {
            *self
        }
    }
    #[automatically_derived]
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    impl ::core::marker::Copy for P3_T {}
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_P3_RQ: rtic::RacyCell<rtic::export::SCRQ<P3_T, 11>> = rtic::RacyCell::new(
        rtic::export::Queue::new(),
    );
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch tasks at priority 3
    #[no_mangle]
    unsafe fn USART1() {
        /// The priority of this interrupt handler
        const PRIORITY: u8 = 3u8;
        rtic::export::run(
            PRIORITY,
            || {
                while let Some((task, index)) = (&mut *__rtic_internal_P3_RQ.get_mut())
                    .split()
                    .1
                    .dequeue()
                {
                    match task {
                        P3_T::high_priority_task => {
                            let () = (&*__rtic_internal_high_priority_task_INPUTS.get())
                                .get_unchecked(usize::from(index))
                                .as_ptr()
                                .read();
                            (&mut *__rtic_internal_high_priority_task_FQ.get_mut())
                                .split()
                                .0
                                .enqueue_unchecked(index);
                            let priority = &rtic::export::Priority::new(PRIORITY);
                            high_priority_task(
                                high_priority_task::Context::new(priority),
                            )
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
        low_priority_task,
        medium_priority_task,
        high_priority_task,
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
        rtic::export::TimerQueue<Systick<100000>, SCHED_T, 120>,
    > = rtic::RacyCell::new(
        rtic::export::TimerQueue(rtic::export::SortedLinkedList::new_u16()),
    );
    #[doc(hidden)]
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    static __rtic_internal_MONOTONIC_STORAGE_Tonic: rtic::RacyCell<
        Option<Systick<100000>>,
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
                SCHED_T::low_priority_task => {
                    rtic::export::interrupt::free(|_| {
                        (&mut *__rtic_internal_P1_RQ.get_mut())
                            .split()
                            .0
                            .enqueue_unchecked((P1_T::low_priority_task, index))
                    });
                    rtic::pend(
                        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART3_4_5_6_7_8,
                    );
                }
                SCHED_T::medium_priority_task => {
                    rtic::export::interrupt::free(|_| {
                        (&mut *__rtic_internal_P2_RQ.get_mut())
                            .split()
                            .0
                            .enqueue_unchecked((P2_T::medium_priority_task, index))
                    });
                    rtic::pend(
                        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART2,
                    );
                }
                SCHED_T::high_priority_task => {
                    rtic::export::interrupt::free(|_| {
                        (&mut *__rtic_internal_P3_RQ.get_mut())
                            .split()
                            .0
                            .enqueue_unchecked((P3_T::high_priority_task, index))
                    });
                    rtic::pend(
                        you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART1,
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
            rtic::export::assert_send::<u16>();
            rtic::export::assert_send::<TIM2>();
            rtic::export::assert_send::<bool>();
            rtic::export::assert_monotonic::<Systick<100000>>();
            const _CONST_CHECK: () = { if !rtic::export::have_basepri() {} else {} };
            let _ = _CONST_CHECK;
            rtic::export::interrupt::disable();
            (0..10u8)
                .for_each(|i| {
                    (&mut *__rtic_internal_low_priority_task_FQ.get_mut())
                        .enqueue_unchecked(i)
                });
            (0..100u8)
                .for_each(|i| {
                    (&mut *__rtic_internal_medium_priority_task_FQ.get_mut())
                        .enqueue_unchecked(i)
                });
            (0..10u8)
                .for_each(|i| {
                    (&mut *__rtic_internal_high_priority_task_FQ.get_mut())
                        .enqueue_unchecked(i)
                });
            let mut core: rtic::export::Peripherals = rtic::export::Peripherals::steal()
                .into();
            let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART1;
            let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART2;
            let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART3_4_5_6_7_8;
            const _: () = if (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS) < 1u8 as usize {
                {
                    ::core::panicking::panic_fmt(
                        format_args!(
                            "Maximum priority used by interrupt vector \'USART3_4_5_6_7_8\' is more than supported by hardware",
                        ),
                    );
                };
            };
            core.NVIC
                .set_priority(
                    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART3_4_5_6_7_8,
                    rtic::export::logical2hw(1u8, stm32f0xx_hal::pac::NVIC_PRIO_BITS),
                );
            rtic::export::NVIC::unmask(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART3_4_5_6_7_8,
            );
            const _: () = if (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS) < 2u8 as usize {
                {
                    ::core::panicking::panic_fmt(
                        format_args!(
                            "Maximum priority used by interrupt vector \'USART2\' is more than supported by hardware",
                        ),
                    );
                };
            };
            core.NVIC
                .set_priority(
                    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART2,
                    rtic::export::logical2hw(2u8, stm32f0xx_hal::pac::NVIC_PRIO_BITS),
                );
            rtic::export::NVIC::unmask(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART2,
            );
            const _: () = if (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS) < 3u8 as usize {
                {
                    ::core::panicking::panic_fmt(
                        format_args!(
                            "Maximum priority used by interrupt vector \'USART1\' is more than supported by hardware",
                        ),
                    );
                };
            };
            core.NVIC
                .set_priority(
                    you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART1,
                    rtic::export::logical2hw(3u8, stm32f0xx_hal::pac::NVIC_PRIO_BITS),
                );
            rtic::export::NVIC::unmask(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::USART1,
            );
            const _: () = if (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS)
                < (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS) as usize
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
                        (1 << stm32f0xx_hal::pac::NVIC_PRIO_BITS),
                        stm32f0xx_hal::pac::NVIC_PRIO_BITS,
                    ),
                );
            if !<Systick<100000> as rtic::Monotonic>::DISABLE_INTERRUPT_ON_EMPTY_QUEUE {
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
                __rtic_internal_shared_resource_shared_num
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.shared_num));
                __rtic_internal_shared_resource_tim2
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.tim2));
                __rtic_internal_shared_resource_done
                    .get_mut()
                    .write(core::mem::MaybeUninit::new(shared_resources.done));
                monotonics.0.reset();
                __rtic_internal_MONOTONIC_STORAGE_Tonic
                    .get_mut()
                    .write(Some(monotonics.0));
                rtic::export::interrupt::enable();
            });
            loop {
                rtic::export::nop()
            }
        }
    }
}
