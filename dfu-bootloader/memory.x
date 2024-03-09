MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 0x4000
  UNINITDFU : ORIGIN = 0x20000000, LENGTH = 0x10
  RAM : ORIGIN = 0x20000000 + LENGTH(UNINITDFU), LENGTH = 20K - LENGTH(UNINITDFU)
}

SECTIONS {
  .uninit_dfu (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit_dfu = .;
    _dfu_magic = .; . += 4;
    *(.uninit_dfu .uninit_dfu.*)
    . = ALIGN(4);
    __euninit_dfu = .;
  } > UNINITDFU
}
