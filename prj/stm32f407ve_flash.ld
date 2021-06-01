MEMORY
{
  flash (rx)   : ORIGIN = 0x08000000, LENGTH = 512K
  ram (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
  bkpram (rw)  : ORIGIN = 0x40024000, LENGTH = 4K
  ccmram (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
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
  } > flash

  /* exception unwinding information */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > flash

  /* index entries for section unwinding */
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > flash

  _etext = .;

  .data : AT (_etext)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > ram

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > ram

  .ccmram (NOLOAD) :
  {
    *(.ccmram);
  } > ccmram

  .bkpram (NOLOAD) :
  {
    . = ALIGN(8);
    *(.nvram);
  } > bkpram

  .ram_heap :
  {
    . = ALIGN(8);
    heapstart = .;
    . = . + LENGTH(ccmram) - __STACK_SIZE - (heapstart - ORIGIN(ccmram));
    heapend = .;
    heapsz = heapend - heapstart;

    ASSERT(heapsz >= __HEAP_SIZE, "Heap does not fit into RAM");

    . = ALIGN(8);
    stackstart = .;
    . = . + __STACK_SIZE;
    StackTop = .;

    ASSERT(StackTop <= ORIGIN(ccmram) + LENGTH(ccmram), "RAM overflowed by stack and heap");
  } > ccmram
}
