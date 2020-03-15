MEMORY
{
  FLASH (rx)   : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
  BKPRAM (rw)  : ORIGIN = 0x40024000, LENGTH = 4K
  CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

SECTIONS
{
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.vectors*))
    *(.text*)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  /* exception unwinding information */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  /* index entries for section unwinding */
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  _etext = .;

  .data : AT (_etext)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  . = ALIGN(8);
  heapstart = .;
  . = . + LENGTH(ram) - heapstart + ORIGIN(ram) - __STACK_SIZE;
  heapend = .;
  heapsz = heapend - heapstart;

  ASSERT(heapsz >= __HEAP_SIZE, "Heap does not fit into RAM")

  . = ALIGN(8);
  stackstart = .;
  . = . + __STACK_SIZE;
  StackTop = .;

  ASSERT(StackTop <= ORIGIN(RAM) + LENGTH(RAM), "RAM overflowed by stack and heap")
}
