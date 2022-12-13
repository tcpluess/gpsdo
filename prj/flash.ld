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

  .preinit_array :
  {
      PROVIDE_HIDDEN(__preinit_array_start = .);
      KEEP(*(.preinit_array*))
      PROVIDE_HIDDEN(__preinit_array_end = .);
  } > flash

  .init_array :
  {
      PROVIDE_HIDDEN(__init_array_start = .);
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      PROVIDE_HIDDEN(__init_array_end = .);
  } > flash

  .fini_array :
  {
      PROVIDE_HIDDEN(__fini_array_start = .);
      KEEP(*(SORT(.fini_array.*)))
      KEEP(*(.fini_array*))
      PROVIDE_HIDDEN(__fini_array_end = .);
  } > flash

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
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = .;
  } > ram

  .bkpram (NOLOAD) :
  {
    . = ALIGN(8);
    *(.nvram);
  } > bkpram

  .ccmram (NOLOAD) :
  {
    *(.ccmram);
  } > ccmram

  /* from the gnu ld description:
     DSECT stands for Dummy Section. Space is not allocated in the memory map
     for a DSECT, nor does it appear in the output file. Thus, a DSECT is not
     loaded to the target. Any references to symbols in the DSECT are patched as
     if the section were allocated to the address given for the section. */
  .stack_heap (DSECT) :
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
    PROVIDE(__stack = .);
    ASSERT(StackTop - stackstart <= LENGTH(ccmram), "Stack does not fit into RAM");
  } > ccmram
}
