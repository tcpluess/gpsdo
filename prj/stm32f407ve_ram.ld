MEMORY
{
  ram (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
  bkpram (rw)  : ORIGIN = 0x40024000, LENGTH = 4K
  ccmram (rwx) : ORIGIN = 0x10000000, LENGTH = 64k
}

SECTIONS
{
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.vectors*))
    *(.text*)
    KEEP(*(.init))
    KEEP(*(.fini))
    *(SORT(.ctors.*))
    *(.ctors)
    *(SORT(.dtors.*))
    *(.dtors)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > ram

  /* exception unwinding information */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > ram

  /* index entries for section unwinding */
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > ram

  .preinit_array :
  {
      PROVIDE_HIDDEN(__preinit_array_start = .);
      KEEP(*(.preinit_array*))
      PROVIDE_HIDDEN(__preinit_array_end = .);
  } > ram

  .init_array :
  {
      PROVIDE_HIDDEN(__init_array_start = .);
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      PROVIDE_HIDDEN(__init_array_end = .);
  } > ram

  .fini_array :
  {
      PROVIDE_HIDDEN(__fini_array_start = .);
      KEEP(*(SORT(.fini_array.*)))
      KEEP(*(.fini_array*))
      PROVIDE_HIDDEN(__fini_array_end = .);
  } > ram

  _etext = .;

  .data : AT (_etext)
  {
    _sdata = .;
    . = ALIGN(4);
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > ram

  .bss (NOLOAD) :
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

  .stack_heap (COPY) :
  {
    . = ALIGN(8);
    PROVIDE(heapstart = .);
    . = ORIGIN(ccmram) + LENGTH(ccmram) - __STACK_SIZE;
    PROVIDE(heapend = .);
    PROVIDE(heapsz = heapend - heapstart);
    ASSERT(heapsz >= __HEAP_SIZE, "Heap does not fit into RAM");
    PROVIDE(stackstart = .);
    . = . + __STACK_SIZE;
    PROVIDE(StackTop = .);
    ASSERT(StackTop - stackstart <= LENGTH(ccmram), "Stack does not fit into RAM");
  } > ccmram
}
