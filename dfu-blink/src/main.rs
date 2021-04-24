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
use stm32f1xx_hal::{pac, prelude::*, timer::{Timer, Event, CountDownTimer}};

use stm32f1xx_hal::pac::{interrupt, TIM2};
use stm32f1xx_hal::gpio;
use stm32f1xx_hal::gpio::{Output, PushPull, gpioc};

use core::mem::MaybeUninit;

type LedType = gpioc::PC13<Output<PushPull>>;

static mut LED: MaybeUninit<LedType> = MaybeUninit::uninit();
static mut TIM: MaybeUninit<CountDownTimer<TIM2>> = MaybeUninit::uninit();
static mut LED_STATUS: u32 = 0;

const KEY_STAY_IN_BOOT : u32 = 0xb0d42b89;

/// Configure VTOR register to point to an actual interrupt
/// vector table, otherwise bootloader will handle interrupts
/// and TIM2 interrupt will work as expected.
#[inline(never)]
fn configure_vtor_dfu(scb: &cortex_m::peripheral::SCB) {
    extern "C" {
        static __reset_vector:u32;
    }

    unsafe {
        let addr: u32 = core::mem::transmute(&__reset_vector);
        scb.vtor.write(addr & 0x3ffffe00);
    }
}

/// Initialize hardware, LED GPIO and a timer
fn app_init() {

    let cortex = cortex_m::Peripherals::take().unwrap();
    let device = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = device.FLASH.constrain();
    let mut rcc = device.RCC.constrain();

    rcc.cfgr = rcc.cfgr
                .use_hse(8.mhz())
                .sysclk(72.mhz())
                .hclk(72.mhz())
                .pclk1(36.mhz())
                .pclk2(72.mhz());

    configure_vtor_dfu(&cortex.SCB);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
    let led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, gpio::State::High);

    unsafe {
        LED.as_mut_ptr().write(led);
    }

    let timer = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1);
    let mut ct = timer.start_count_down(10.hz());

    ct.listen(Event::Update);

    unsafe {
        TIM.as_mut_ptr().write(ct);
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

/// Write a magic value to RAM to stay in DFU mode and
/// reset.
fn reset_into_dfu() -> ! {
    cortex_m::interrupt::disable();

    let cortex = unsafe { cortex_m::Peripherals::steal() };

    let p = 0x2000_0000 as *mut u32;
    unsafe { p.write_volatile(KEY_STAY_IN_BOOT) };

    cortex_m::asm::dsb();
    unsafe {
        // System reset request
        cortex.SCB.aircr.modify(|v| 0x05FA_0004 | (v & 0x700) );
    }
    cortex_m::asm::dsb();
    loop {}
}

/// Blink LED, and reset to DFU after some time.
#[interrupt]
fn TIM2() {
    let led = unsafe {&mut *LED.as_mut_ptr() };
    let tim = unsafe {&mut *TIM.as_mut_ptr() };
    let status = unsafe {&mut LED_STATUS };

    tim.clear_update_interrupt_flag();

    led.toggle().ok();
    *status += 1;

    if *status == 100 {
        reset_into_dfu();
    }
}
