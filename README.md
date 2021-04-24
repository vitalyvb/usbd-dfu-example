# usbd-dfu-example

Rust STM32 DFU bootloader and runtime firmware example

## DFU bootloader (dfu-bootloader)

Example DFU bootloader for "Bluepill" board equipped with
STM32 with 128KB flash.

There are two modes of operation: minimal and DFU.

After reset, bootloader starts in a minimal mode,
it's goal is to determine if bootloader must switch
to DFU mode, and if not, try to jump to a main
firmware.

In minimal mode, following items are checked:
 * Magic value in RAM.
 * BOOT1 (PB2) state.
 * The first few bytes of a firmware (should look like a proper stack pointer).

Firmware checksum is not verified.

When DFU mode is active, LED on PC13 blinks every 2 seconds.
Required peripherals and USB are enabled, host
can issue DFU commands.

First 0x10 bytes of RAM are reserved. In "memory.x" linker script
RAM section has 0x10 offset from an actual RAM start. The first
4 bytes of RAM may have a magic value to force the bootloader
to enter DFU mode programmatically. Both DFU and main firmware
must agree on used addresses and values for this to work.

## DFU Blink example application (dfu-blink)

Example firmware for DFU bootloader for "Bluepill" board.

LED on PC13 blinks 5 times a second. After about 10 seconds
allication resets the microcontroller and it stays in 
DFU mode.

First 0x10 bytes of RAM are reserved. In "memory.x" linker script
RAM section has 0x10 offset from an actual RAM start. The first
4 bytes of RAM may have a magic value to force the bootloader
to enter DFU mode programmatically.
