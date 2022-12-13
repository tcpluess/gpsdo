MEMORY
{
  flash (rx)   : ORIGIN = 0x08000000, LENGTH = 512K
  ram (rw)    : ORIGIN = 0x20000000, LENGTH = 128K
  bkpram (rw)  : ORIGIN = 0x40024000, LENGTH = 4K
  ccmram (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}
