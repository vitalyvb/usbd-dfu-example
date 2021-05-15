//! # DFU bootloader
//!
//! Example DFU bootloader for "Bluepill" board.
//!
//! There are two modes of operation: minimal and DFU.
//!
//! After reset, bootloader starts in a minimal mode,
//! it's goal is to determine if bootloader must switch
//! to DFU mode, and if not, try to jump to a main
//! firmware.
//!
//! In minimal mode, following items are checked:
//!
//! > * Magic value in RAM.
//!
//! > * BOOT1 (PB2) state.
//!
//! > * The first few bytes of a firmware (should look like a proper stack pointer).
//!
//! When DFU mode is active, LED on PC13 blinks every 2 seconds.
//! Required peripherals and USB are enabled, host
//! can issue DFU commands.
//!
//! First 0x10 bytes of RAM are reserved. In "memory.x" linker script
//! RAM section has 0x10 offset from an actual RAM start. The first
//! 4 bytes of RAM may have a magic value to force the bootloader
//! to enter DFU mode programmatically. Both DFU and main firmware
//! must agree on used addresses and values for this to work.
//!

#![no_std]
#![no_main]

use core::str;

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

use stm32f1xx_hal::gpio::{gpioc, Output, PushPull};
use stm32f1xx_hal::pac::{interrupt, GPIOB, RCC, TIM2};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{flash, gpio};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_dfu::*;

use core::mem::MaybeUninit;

/// If this value is found at the address 0x2000_0000 (beginning of RAM),
/// bootloader will enter DFU mode.
const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

/// Board flash configuration. MEM_INFO_STRING below must also be changed.
const FLASH_SIZE: flash::FlashSize = flash::FlashSize::Sz128K;
const FLASH_SIZE_BYTES: usize = (FLASH_SIZE as usize) * 1024;
const BOOTLOADER_SIZE_BYTES: u32 = 16 * 1024;
const FW_ADDRESS: u32 = 0x0800_4000;

type LedType = gpioc::PC13<Output<PushPull>>;

static mut LED: MaybeUninit<LedType> = MaybeUninit::uninit();
static mut TIM: MaybeUninit<CountDownTimer<TIM2>> = MaybeUninit::uninit();

static mut FLASH: MaybeUninit<flash::Parts> = MaybeUninit::uninit();
static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBusType>> = MaybeUninit::uninit();
static mut USB_DEVICE: MaybeUninit<UsbDevice<UsbBusType>> = MaybeUninit::uninit();
static mut USB_DFU: MaybeUninit<DFUClass<UsbBusType, STM32Mem>> = MaybeUninit::uninit();

pub struct STM32Mem<'a> {
    writer: flash::FlashWriter<'a>,
    buffer: [u8; 128],
}

impl<'a> STM32Mem<'a> {
    fn new(mut writer: flash::FlashWriter<'a>) -> Self {
        // Disable erase and program verification.
        // It should be enabled, but erase verification
        // does not work.
        // https://github.com/stm32-rs/stm32f1xx-hal/pull/328
        writer.change_verification(false);

        Self {
            writer,
            buffer: [0; 128],
        }
    }
}

impl<'a> DFUMemIO for STM32Mem<'a> {
    const INITIAL_ADDRESS_POINTER: u32 = 0x0800_0000;
    const PAGE_PROGRAM_TIME_MS: u32 = 7; // time it takes to program 128 bytes
    const PAGE_ERASE_TIME_MS: u32 = 50;
    const FULL_ERASE_TIME_MS: u32 = 50 * 112;

    const MEM_INFO_STRING: &'static str = "@Flash/0x08000000/16*1Ka,112*1Kg";
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;

    fn read_block(
        &mut self,
        address: u32,
        length: usize,
    ) -> core::result::Result<&[u8], DFUMemError> {
        let flash_top: u32 = 0x0800_0000 + FLASH_SIZE_BYTES as u32;

        if address < 0x0800_0000 {
            return Err(DFUMemError::Address);
        }
        if address >= flash_top {
            return Ok(&[]);
        }

        let len = length.min((flash_top - address) as usize);

        let mem = unsafe { &*core::ptr::slice_from_raw_parts(address as *const u8, len) };

        Ok(mem)
    }

    fn erase_block(&mut self, address: u32) -> core::result::Result<(), DFUMemError> {
        if address < flash::FLASH_START {
            return Err(DFUMemError::Address);
        }

        if address < flash::FLASH_START + BOOTLOADER_SIZE_BYTES {
            return Err(DFUMemError::Address);
        }

        if address >= flash::FLASH_START + FLASH_SIZE_BYTES as u32 {
            return Err(DFUMemError::Address);
        }

        if address & (1024 - 1) != 0 {
            return Ok(());
        }

        match self.writer.page_erase(address - flash::FLASH_START) {
            Ok(_) => Ok(()),
            Err(flash::Error::EraseError) => Err(DFUMemError::Erase),
            Err(flash::Error::VerifyError) => Err(DFUMemError::CheckErased),
            Err(_) => Err(DFUMemError::Unknown),
        }
    }

    fn erase_all_blocks(&mut self) -> Result<(), DFUMemError> {
        Err(DFUMemError::Unknown)
    }

    fn store_write_buffer(&mut self, src: &[u8]) -> core::result::Result<(), ()> {
        self.buffer[..src.len()].copy_from_slice(src);
        Ok(())
    }

    fn program_block(
        &mut self,
        address: u32,
        length: usize,
    ) -> core::result::Result<(), DFUMemError> {
        if address < flash::FLASH_START {
            return Err(DFUMemError::Address);
        }

        let offset = address - flash::FLASH_START;

        if offset < BOOTLOADER_SIZE_BYTES {
            return Err(DFUMemError::Address);
        }

        if offset as usize >= FLASH_SIZE_BYTES - length {
            return Err(DFUMemError::Address);
        }

        match self.writer.write(offset, &self.buffer[..length]) {
            Ok(_) => Ok(()),
            Err(flash::Error::ProgrammingError) => Err(DFUMemError::Prog),
            Err(flash::Error::LengthNotMultiple2) => Err(DFUMemError::Prog),
            Err(flash::Error::LengthTooLong) => Err(DFUMemError::Prog),
            Err(flash::Error::AddressLargerThanFlash) => Err(DFUMemError::Address),
            Err(flash::Error::AddressMisaligned) => Err(DFUMemError::Address),
            Err(flash::Error::WriteError) => Err(DFUMemError::Write),
            Err(flash::Error::VerifyError) => Err(DFUMemError::Verify),
            Err(_) => Err(DFUMemError::Unknown),
        }
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        controller_reset();
    }
}

/// Return device serial based on U_ID registers.
fn read_serial() -> u32 {
    let u_id0 = 0x1FFF_F7E8 as *const u32;
    let u_id1 = 0x1FFF_F7EC as *const u32;

    unsafe { u_id0.read().wrapping_add(u_id1.read()) }
}

/// Returns device serial number as hex string slice.
fn get_serial_str() -> &'static str {
    static mut SERIAL: [u8; 8] = [b' '; 8];
    let serial = unsafe { SERIAL.as_mut() };

    fn hex(v: u8) -> u8 {
        match v {
            0..=9 => v + b'0',
            0xa..=0xf => v - 0xa + b'a',
            _ => b' ',
        }
    }

    let sn = read_serial();

    for (i, d) in serial.iter_mut().enumerate() {
        *d = hex(((sn >> (i * 4)) & 0xf) as u8)
    }

    unsafe { str::from_utf8_unchecked(serial) }
}

/// Initialize, configure all peripherals, and setup USB DFU.
/// Interrupts must be disabled.
fn dfu_init() {
    // let cortex = cortex_m::Peripherals::take().unwrap();
    let device = unsafe { pac::Peripherals::steal() };

    let flash = unsafe {
        FLASH.as_mut_ptr().write(device.FLASH.constrain());
        &mut *FLASH.as_mut_ptr()
    };

    let mut rcc = device.RCC.constrain();

    rcc.cfgr = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz());

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
    let led = gpioc
        .pc13
        .into_push_pull_output_with_state(&mut gpioc.crh, gpio::State::High);

    unsafe {
        LED.as_mut_ptr().write(led);
    }

    let timer = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1);
    let mut ct = timer.start_count_down(1.hz());
    ct.listen(Event::Update);

    unsafe {
        TIM.as_mut_ptr().write(ct);
    }

    debug_assert!(clocks.usbclk_valid());

    let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);

    usb_dp.set_low().ok();
    cortex_m::asm::delay(1024);

    /* USB Peripheral */

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb_periph = Peripheral {
        usb: device.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    let bus = unsafe {
        USB_BUS.as_mut_ptr().write(UsbBus::new(usb_periph));
        &*USB_BUS.as_ptr()
    };

    /* DFU */

    let fwr = flash.writer(flash::SectorSize::Sz1K, FLASH_SIZE);
    let stm32mem = STM32Mem::new(fwr);

    unsafe {
        USB_DFU.as_mut_ptr().write(DFUClass::new(bus, stm32mem));
    }

    /* USB device */

    let usb_vid_pid_is_for_private_testing_only = ();

    let usb_dev = UsbDeviceBuilder::new(bus, UsbVidPid(0xf055, 0xdf11))
        .manufacturer("Manufacturer")
        .product("Product")
        .serial_number(get_serial_str())
        .device_release(0x0200)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build();

    unsafe {
        USB_DEVICE.as_mut_ptr().write(usb_dev);
    }

    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f1xx_hal::pac::Interrupt::TIM2);
        cortex_m::peripheral::NVIC::unmask(stm32f1xx_hal::pac::Interrupt::USB_LP_CAN_RX0);
    }
}

fn minimal_init() {
    unsafe {
        // enable PWR, AFIO, GPIOB
        (*RCC::ptr()).apb1enr.modify(|_, w| w.pwren().set_bit());
        (*RCC::ptr())
            .apb2enr
            .modify(|_, w| w.afioen().set_bit().iopben().set_bit());
    }

    unsafe {
        // P2 - Input, Floating
        (*GPIOB::ptr())
            .crl
            .modify(|_, w| w.mode2().input().cnf2().open_drain());
    }

    cortex_m::asm::delay(100);
}

/// Check if DFU force external condition.
/// Check BOOT1 jumper position.
fn dfu_enforced() -> bool {
    // check BOOT1, PB2 state
    unsafe { (*GPIOB::ptr()).idr.read().idr2().bit_is_set() }
}

/// Reset registers that were used for a
/// check if DFU mode must be enabled to a
/// default values before starting main firmware.
fn quick_uninit() {
    unsafe {
        (*GPIOB::ptr()).crl.reset();
        (*RCC::ptr()).apb1enr.reset();
        (*RCC::ptr()).apb2enr.reset();
    }
}

/// Initialize stack pointer and jump to a main firmware.
#[inline(never)]
fn jump_to_app() -> ! {
    let vt = FW_ADDRESS as *const u32;
    unsafe {
        cortex_m::asm::bootload(vt);
    }
}

/// Check if FW looks OK and jump to it, or return.
fn try_start_app() {
    let sp = unsafe { (FW_ADDRESS as *const u32).read() };
    if sp & 0xfffe_0000 == 0x2000_0000 {
        quick_uninit();
        jump_to_app();
    }
}

/// Read magic value to determine if
/// device must enter DFU mode.
fn get_uninit_val() -> u32 {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.read_volatile() }
}

/// Erase magic value in RAM so that
/// DFU would be triggered only once.
fn clear_uninit_val() {
    let p = 0x2000_0000 as *mut u32;
    unsafe { p.write_volatile(0) };
}

/// Return true if "uninit" area of RAM has a
/// special value. Used to force DFU mode from
/// a main firmware programmatically.
fn dfu_ram_requested() -> bool {
    let stay = get_uninit_val() == KEY_STAY_IN_BOOT;
    if stay {
        clear_uninit_val();
    }
    stay
}

#[entry]
fn main() -> ! {
    if !dfu_ram_requested() {
        minimal_init();
        if !dfu_enforced() {
            try_start_app();
        }
    }

    cortex_m::interrupt::disable();

    dfu_init();

    cortex_m::asm::dsb();
    unsafe { cortex_m::interrupt::enable() };

    loop {
        cortex_m::asm::wfi();
    }
}

fn controller_reset() -> ! {
    cortex_m::interrupt::disable();

    let cortex = unsafe { cortex_m::Peripherals::steal() };

    cortex_m::asm::dsb();
    unsafe {
        // System reset request
        cortex.SCB.aircr.modify(|v| 0x05FA_0004 | (v & 0x700));
    }
    cortex_m::asm::dsb();

    loop {}
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

#[interrupt]
fn TIM2() {
    let led = unsafe { &mut *LED.as_mut_ptr() };
    let tim = unsafe { &mut *TIM.as_mut_ptr() };

    tim.clear_update_interrupt_flag();

    led.toggle().ok();
}

fn usb_interrupt() {
    let usb_dev = unsafe { &mut *USB_DEVICE.as_mut_ptr() };
    let dfu = unsafe { &mut *USB_DFU.as_mut_ptr() };

    usb_dev.poll(&mut [dfu]);
}
