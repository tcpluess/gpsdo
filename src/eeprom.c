/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the 93C66 EEPROM
 *
 * Filename:       eeprom.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  14.04.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "eeprom.h"
#include "stm32f407xx.h"
#include "misc.h"
#include "convert.h"
#include "checksum.h"
#include <string.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/* address bit mask */
#define ADDR_MSK (BIT_09 - 1u)

/* number of bits for the address */
#define ADDR_LEN 9u

/* opcodes for the e2prom */
#define OP_READ (BIT_01 << ADDR_LEN)
#define OP_WRITE (BIT_00 << ADDR_LEN)
#define OP_WEN 0u /* write enable */
#define OP_WDS 0u /* write disable */
#define OP_ERASE ((BIT_01 | BIT_00) << ADDR_LEN)
#define OP_ERAL 0u /* erase all */

#define WEN_ADDR (BIT_08 | BIT_07)
#define WDS_ADDR 0u
#define ERAL_ADDR BIT_08

#define START_BIT BIT_11

#define CHECKSUM_OFFSET (EEP_SZ-2)

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/* macros for easy access to the e2prom */
#define EEP_SS(x) { GPIOD->BSRR = ((x) ? BIT_14 : BIT_30); asm volatile ("nop"); }
#define EEP_MOSI(x) { GPIOD->BSRR = ((x) ? BIT_12 : BIT_28); }
#define EEP_SCK(x) { GPIOD->BSRR = ((x) ? BIT_13 : BIT_29); asm volatile ("nop"); }
#define EEP_MISO() ((GPIOD->IDR & BIT_11) != 0 ? 1u : 0u)

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void load_config(void);
static void addr_cycle(uint32_t addr, uint32_t opcode);
static void write_enable(bool enable);
static bool check_busy(void);
static void erase(uint32_t addr);
static void write_internal(uint32_t addr, uint8_t data);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/* not static because it must be globally accessible */
static config_t cfg;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void eep_init(void)
{
  /* enable gpio port d */
  RCC->AHB1ENR |= BIT_03;

  /* configure pins */
  GPIOD->MODER |= (1u << 24) | (1u << 26) | (1u << 28);

  /* default levels of the signals */
  EEP_MOSI(0);
  EEP_SCK(0);

  /* load the config when the eeprom is initialised */
  load_config();
}


uint8_t eep_read(uint32_t addr)
{
  uint8_t rddata = 0;

  /* select slave */
  EEP_SS(1);

  /* send out the address and opcode */
  addr_cycle(addr, OP_READ);

  for(int i = 0; i < 8; i++)
  {
    /* transfer is msb first */
    rddata = (uint8_t)(rddata << 1);

    EEP_MOSI(0);
    EEP_SCK(1);

    /* save the transferred bit */
    rddata |= EEP_MISO();

    EEP_SCK(0);
  }

  /* de-select slave */
  EEP_SS(0);

  return rddata;
}


void eep_write(uint32_t addr, uint8_t data)
{
  erase(addr);
  write_enable(true);
  write_internal(addr, data);
  while(check_busy());
  write_enable(false);
}


void eep_chip_erase(void)
{
  write_enable(true);

  EEP_SCK(0);
  EEP_SS(1);
  addr_cycle(ERAL_ADDR, OP_ERAL);
  EEP_SS(0);

  while(check_busy());

  write_enable(false);
}


void eep_read_multi(uint32_t addr, uint32_t len, void* buf)
{
  uint8_t* ptr = (uint8_t*)buf;

  /* select slave */
  EEP_SS(1);

  /* send out the address and opcode */
  addr_cycle(addr, OP_READ);

  while(len > 0)
  {
    uint8_t rddata = 0u;
    for(int i = 0; i < 8; i++)
    {
      /* transfer is msb first */
      rddata = (uint8_t)(rddata << 1);

      EEP_MOSI(0);
      EEP_SCK(1);

      /* save the transferred bit */
      rddata |= EEP_MISO();

      EEP_SCK(0);
    }

    *ptr = rddata;
    ptr++;
    len--;
  }
  /* de-select slave */
  EEP_SS(0);
}


void eep_write_multi(uint32_t addr, uint32_t len, void* buf)
{
  uint8_t* ptr = (uint8_t*)buf;
  write_enable(true);
  while(len > 0)
  {
    uint8_t tmp = *ptr;
    write_internal(addr, tmp);
    while(check_busy());
    ptr++;
    addr++;
    len--;
  }

  write_enable(false);
}





void save_config(void)
{
  uint16_t calc_checksum;
  calc_checksum = fletcher16(cfg.bytes, CHECKSUM_OFFSET);
  pack_u16_be(cfg.bytes, CHECKSUM_OFFSET, calc_checksum);
  eep_write_multi(0, EEP_SZ, cfg.bytes);
}


config_t* get_config(void)
{
  return &cfg;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/


/*============================================================================*/
static void load_config(void)
/*------------------------------------------------------------------------------
  Function:
  reads the entire config from the eeprom, verifies the checksum and
  initialises default values if the checksum is wrong
  in:  none
  out: none
==============================================================================*/
{
  (void)memset(cfg.bytes, 0, EEP_SZ);
  eep_read_multi(0, EEP_SZ, &cfg);

  /* the checksum sits at the last 2 bytes in big endian format */
  uint16_t rd_cksum = unpack_u16_be(cfg.bytes, CHECKSUM_OFFSET);

  /* when the checksum is wrong, generate some meaningful initial data
     and populate the eeprom. also ignore the data if the version is wrong */
  uint16_t calc_checksum = fletcher16(cfg.bytes, CHECKSUM_OFFSET);
  if((rd_cksum != calc_checksum) || (cfg.version != CFG_VERSION))
  {
    /* set all data to zero and then explicitly configure the items that are
       nonzero by default */
    (void)memset(cfg.bytes, 0, EEP_SZ);
    cfg.version = CFG_VERSION;
    cfg.last_dacval = 32768u;
    cfg.use_gps = true;
    cfg.use_galileo = true;
    cfg.svin_dur = 86400u; /* 24 hours */
    cfg.accuracy = UINT32_MAX;
    cfg.accuracy_limit = 300; /* 300mm default accuracy for survey-in */
    cfg.tau = 200u; /* 200 seconds default time constant */
    cfg.elevation_mask = 45; /* 45 degree elevation mask */
    cfg.pps_dur = 100u;
  }
}


/*============================================================================*/
static void addr_cycle(uint32_t addr, uint32_t opcode)
/*------------------------------------------------------------------------------
  Function:
  performs an address cycle by sending the start bit, op code and address to the
  device.
  in:  addr -> address
       opcode -> opcode
  out: none
==============================================================================*/
{
  /* clear the top bits of the address, insert the start bit and the opcode */
  addr &= ADDR_MSK;
  addr |= START_BIT;
  addr |= opcode;

  for(int i = 0; i < 12; i++)
  {
    /* clock one bit out */
    EEP_MOSI(addr & BIT_11);
    EEP_SCK(1);
    EEP_SCK(0);

    /* left shift; transfer is msb first */
    addr = (addr << 1);
  }
}


/*============================================================================*/
static void write_enable(bool enable)
/*------------------------------------------------------------------------------
  Function:
  enable or disable write and erase cycles.
  in:  enable -> if true, the device is unlocked for writing; otherwise it is
       locked.
  out: none
==============================================================================*/
{
  /* select slave */
  EEP_MOSI(0);
  EEP_SCK(0);
  EEP_SS(1);

  if(enable)
  {
    addr_cycle(WEN_ADDR, OP_WEN);
  }
  else
  {
    addr_cycle(WDS_ADDR, OP_WDS);
  }

  /* de-select slave */
  EEP_SS(0);
}


/*============================================================================*/
static bool check_busy(void)
/*------------------------------------------------------------------------------
  Function:
  check whether the device is busy or not (i.e. performing an erase or write
  cycle)
  in:  none
  out: returns true if busy, false otherwise
==============================================================================*/
{
  bool ret;

  /* slave select */
  EEP_SS(1);

  /* check the miso line */
  if(EEP_MISO())
  {
    ret = false;
  }
  else
  {
    ret = true;
  }

  /* deselect */
  EEP_SS(0);

  return ret;
}


/*============================================================================*/
static void erase(uint32_t addr)
/*------------------------------------------------------------------------------
  Function:
  erase a particular address
  in:  addr -> address to be erased
  out: none
==============================================================================*/
{
  /* the e2prom must be write-enabled */
  write_enable(true);

  /* slave select */
  EEP_SS(1);

  /* issue the erase command */
  addr_cycle(addr, OP_ERASE);

  EEP_SS(0);

  /* wait until the e2prom is ready */
  while(check_busy());

  /* write disable */
  write_enable(false);
}


/*============================================================================*/
static void write_internal(uint32_t addr, uint8_t data)
/*------------------------------------------------------------------------------
  Function:
  internal write function which does not unprotect/protect the device
  in:  addr -> address to be programmed
       data -> data to be stored at the address
  out: none
==============================================================================*/
{
  /* select slave */
  EEP_MOSI(0);
  EEP_SCK(0);
  EEP_SS(1);

  /* send out the address and opcode */
  addr_cycle(addr, OP_WRITE);

  /* for each bit */
  for(int i = 0; i < 8; i++)
  {
    /* next msb output */
    EEP_MOSI(data & BIT_07);
    data = (uint8_t)(data << 1);

    /* clock */
    EEP_SCK(1);
    EEP_SCK(0);
  }

  /* de-select slave */
  EEP_SS(0);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
