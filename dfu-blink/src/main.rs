//! # DFU Blink example application
//!
//! Example firmware for DFU bootloader for "Bluepill" board.
//!
//! LED on PC13 blinks 5 times a second. After about 10 seconds
//! allication resets the microcontroller and it stays in
//! DFU mode.
//!
//! First 0x10 bytes of RAM are reserved. In "memory.x" linker script
//! RAM section has 0x10 offset from an actual RAM start. The first
//! 4 bytes of RAM may have a magic value to force the bootloader
//! to enter DFU mode programmatically.
//!

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{CounterHz, Event},
};

use stm32f1xx_hal::gpio;
use stm32f1xx_hal::gpio::{gpioc, Output, PushPull};
use stm32f1xx_hal::pac::{interrupt, TIM2};

use core::{mem::MaybeUninit, ptr::addr_of_mut};

type LedType = gpioc::PC13<Output<PushPull>>;

static mut LED: MaybeUninit<LedType> = MaybeUninit::uninit();
static mut TIM: MaybeUninit<CounterHz<TIM2>> = MaybeUninit::uninit();

/// If this value is found at the address 0x2000_0000 (beginning of RAM),
/// bootloader will enter DFU mode. See memory.x linker script.
const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

/// Configure VTOR register to point to an actual interrupt
/// vector table if the bootloader didn't do this for us.
/// Otherwise bootloader will handle interrupts and TIM2
/// interrupt will not work as expected.
#[inline(never)]
fn configure_vtor_dfu(scb: &cortex_m::peripheral::SCB) {
    extern "C" {
        static __vector_table: u32;
    }

    unsafe {
        let addr: u32 = core::mem::transmute(&__vector_table);
        scb.vtor.write(addr & 0x3ffffe00);
    }
}

/// Initialize hardware, LED GPIO and a timer
fn app_init() {
    let cortex = cortex_m::Peripherals::take().unwrap_or_else(||{panic!()});
    let device = pac::Peripherals::take().unwrap_or_else(||{panic!()});

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();

    rcc.cfgr = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .hclk(72.MHz())
        .pclk1(36.MHz())
        .pclk2(72.MHz());

    configure_vtor_dfu(&cortex.SCB);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    {
        let mut gpioc = device.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, gpio::PinState::High);

        unsafe {
            LED.write(led);
        }
    }

    {
        let mut timer = device.TIM2.counter_hz(&clocks);
        timer.start(10.Hz()).unwrap_or_else(|_|{panic!()});

        timer.listen(Event::Update);

        unsafe {
            TIM.write(timer);
        }
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f1xx_hal::pac::Interrupt::TIM2);
    }
}

#[entry]
fn main() -> ! {
    cortex_m::interrupt::disable();

    app_init();

    cortex_m::asm::dsb();

    unsafe { cortex_m::interrupt::enable() };

    loop {
        cortex_m::asm::wfi();
    }
}

fn magic_mut_ptr() -> *mut u32 {
    extern "C" {
        #[link_name = "_dfu_magic"]
        static mut magic : u32;
    }

    unsafe { addr_of_mut!(magic) }
}

/// Write a magic value to RAM to stay in DFU mode and
/// reset.
fn reset_into_dfu() -> ! {
    cortex_m::interrupt::disable();

    unsafe { magic_mut_ptr().write_volatile(KEY_STAY_IN_BOOT) };

    cortex_m::peripheral::SCB::sys_reset();
}

/// Blink LED, and reset to DFU after some time.
#[interrupt]
fn TIM2() {
    static mut STATUS: u32 = 0;

    // Safety: After initialization these static variables used only here.
    // Also see app_init()
    let led = unsafe { LED.assume_init_mut() };
    let tim = unsafe { TIM.assume_init_mut() };

    let _ = tim.wait();

    led.toggle();
    *STATUS += 1;

    if *STATUS == 100 {
        reset_into_dfu();
    }
}
