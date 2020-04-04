/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    adt7301 driver
 *
 * Compiler:       ANSI-C
 *
 * Filename:       adt7301.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "ublox.h"
#include "stm32f407.h"
#include "misc.h"
#include "vic.h"

#include <string.h>

#include "timebase.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define IRQ_NUM 39

#define SYNC1 0xb5u
#define SYNC2 0x62u

#define BAUD_INITIAL 9600u
#define BAUD_RECONFIGURE 230400u
#define BAUD_INT(x) ((uint32_t)(10000000u/(16u*x)))
#define BAUD_FRAC(x) ((uint32_t)(10000000u/x-16u*BAUD_INT(x)+0.5f))

#define MAX_UBX_LEN 150u

#define CLASS_CFG 0x06u

#define CFG_PORT 0x00u
#define CFG_PORT_LEN 20u
#define CFG_PORT_ID_UART 1
#define CFG_UART_8N1 BIT_06 | BIT_07 | BIT_11
#define CFG_INOUT_MASK_UBX BIT_00

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/* pseudo functions for unpack and pack 1-byte data */
#define unpack_u1(x, y) x[y]
#define pack_u1(x, y, z) x[y] = z

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

typedef enum
{
  rx_sync1,
  rx_sync2,
  rx_class,
  rx_id,
  rx_len_lo,
  rx_len_hi,
  rx_payload,
  rx_cka,
  rx_ckb
} rxstate_t;

typedef enum
{
  reset,
  wait_ready,
  configure_uart,
  configure_messages,
  normal
} workerstatus_t;

typedef enum
{
  ubx_header1,
  ubx_header2,
  ubx_class,
  ubx_id,
  ubx_len_lsb,
  ubx_len_msb,
  ubx_data,
  ubx_cka,
  ubx_ckb,
} irqstatus_t;

typedef struct
{
  uint8_t msg[MAX_UBX_LEN];
  uint8_t msgclass;
  uint8_t msgid;
  uint16_t len;
  union
  {
    volatile bool valid;
    volatile bool empty;
  };
} ubxbuffer_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static ubxbuffer_t txb;
static ubxbuffer_t rxb;
static float tp;
gpsinfo_t info;

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static void init_uart(void);
void ubx_reconfig(void);
static void enable_txempty_irq(void);
static void disable_txempty_irq(void);
static void uart_configure_baud(uint32_t baud);
static void ubx_configure_baud(uint32_t baudrate);
static void start_transmit(const ubxbuffer_t* tmp);
static void set_ubx_rate(uint8_t msgclass, uint8_t msgid, uint32_t rate);
static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info);
static void ubx_rxhandler(void);
static void ubx_txhandler(void);
static void ubx_irq_handler(void);
static int32_t unpack_i4(const uint8_t* data, uint32_t offset);
static void pack_u4(uint8_t* buffer, uint32_t offset, uint32_t value);
static uint32_t unpack_u4(const uint8_t* data, uint32_t offset);
static uint16_t unpack_u2(const uint8_t* data, uint32_t offset);
static void pack_u2(uint8_t* buffer, uint32_t offset, uint16_t value);


/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void ublox_init(void)
{
  tp = 0.0f;
  txb.empty = true;
  rxb.valid = false;

  init_uart();

  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* configure the reset pin */
  GPIOD_MODER |= (1u << 20);
  GPIOD_OTYPER |= (1u << 10);
  GPIOD_BSRR = BIT_26;
}

void gps_worker(void)
{
  static workerstatus_t status = reset;
  static uint64_t timestamp = 0;

  if(timestamp >= get_uptime_msec())
  {
    return;
  }

  switch(status)
  {
    case reset:
    {
      timestamp = get_uptime_msec() + 1000u;
      status = wait_ready;
      break;
    }

    case wait_ready:
    {
      /* reset pin high */
      GPIOD_BSRR = BIT_10;
      timestamp = get_uptime_msec() + 1000u;
      status = configure_uart;
      break;
    }

    case configure_uart:
    {
      ubx_configure_baud(BAUD_RECONFIGURE);
      uart_configure_baud(BAUD_RECONFIGURE);
      timestamp = get_uptime_msec() + 100u;
      status = configure_messages;
      break;
    }

    case configure_messages:
    {
      USART3_CR1 |= BIT_05; // enable rx not empty interrupt
      set_ubx_rate(0x0d, 0x01, 1);
      set_ubx_rate(0x01, 0x07, 1);
      status = normal;
      break;
    }

    case normal:
    {
      if(rxb.valid)
      {
        if((rxb.msgclass == 0x0d) && (rxb.msgid == 0x01))
        {
          /* get the quantisation error in ps and convert to ns */
          int32_t qerr = unpack_i4(rxb.msg, 8);
          tp = ((float)qerr) / 1000.0f;
        }
        else if((rxb.msgclass == 0x01) && (rxb.msgid == 0x07))
        {
          unpack_pvt(rxb.msg, &info);
        }

        rxb.valid = false;
      }
    }
  }
}

float get_timepulse_error(void)
{
  float z = tp;
  tp = 0;
  return z;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

static void init_uart(void)
{
  /* enable gpio d and configure the uart pins */
  RCC_AHB1ENR |= BIT_03;
  GPIOD_MODER |= (2u << 16) | (2u << 18);
  GPIOD_AFRH |= (7u << 0) | (7u << 4);

  /* enable usart 3 and install the irq handler */
  RCC_APB1ENR |= BIT_18;
  USART3_BRR = (BAUD_FRAC(BAUD_INITIAL) << 0) | (BAUD_INT(BAUD_INITIAL) << 4);
  USART3_CR1 = BIT_13 | BIT_03 | BIT_02;
  vic_enableirq(IRQ_NUM, ubx_irq_handler);
}

static void uart_configure_baud(uint32_t baud)
{
  /* wait until tx done */
  do
  {
    if(USART3_SR & BIT_06)
    {
      break;
    }
  } while(true);

  USART3_BRR = (BAUD_FRAC(baud) << 0) | (BAUD_INT(baud) << 4);
}

static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info)
{
  info->year = unpack_u2(rdata, 4);
  info->month = unpack_u1(rdata, 6);
  info->day = unpack_u1(rdata, 7);
  info->hour = unpack_u1(rdata, 8);
  info->min = unpack_u1(rdata, 9);
  info->sec = unpack_u1(rdata, 10);
  info->valid = unpack_u1(rdata, 11);
  info->tacc = unpack_u4(rdata, 12);
  info->fixtype = unpack_u1(rdata, 20);
  info->flags = unpack_u1(rdata, 21);
  info->xflags = unpack_u1(rdata, 22);
  info->numsv = unpack_u1(rdata, 23);
  info->lon = ((float)unpack_i4(rdata, 24))/1e7f;
  info->lat = ((float)unpack_i4(rdata, 28))/1e7f;
  info->height = unpack_i4(rdata, 32);
  info->hmsl = unpack_i4(rdata, 36);
  info->hacc = unpack_u4(rdata, 40);
  info->vacc = unpack_u4(rdata, 44);
  info->pdop = unpack_u2(rdata, 76);
}

static void start_transmit(const ubxbuffer_t* tmp)
{
  while(!txb.empty);
  txb.empty = false;
  memcpy(txb.msg, tmp->msg, tmp->len);
  txb.len = tmp->len;
  txb.msgclass = tmp->msgclass;
  txb.msgid = tmp->msgid;
  enable_txempty_irq();
}

static void ubx_configure_baud(uint32_t baudrate)
{
  ubxbuffer_t tmp;
  tmp.msgclass = CLASS_CFG;
  tmp.msgid = CFG_PORT;
  tmp.len = CFG_PORT_LEN;

  pack_u1(tmp.msg, 0, CFG_PORT_ID_UART);
  pack_u1(tmp.msg, 1, 0);
  pack_u2(tmp.msg, 2, 0);
  pack_u4(tmp.msg, 4, CFG_UART_8N1);
  pack_u4(tmp.msg, 8, baudrate);
  pack_u2(tmp.msg, 12, CFG_INOUT_MASK_UBX);
  pack_u2(tmp.msg, 14, CFG_INOUT_MASK_UBX);
  pack_u2(tmp.msg, 16, 0);
  pack_u2(tmp.msg, 18, 0);
  start_transmit(&tmp);
}

static void ubx_configure_tmode(uint8_t mode, float lat, float lon, float alt, uint32_t dur)
{
  ubxbuffer_t tmp;
  tmp.msgclass = 0x06;
  tmp.msgid = 0x3d;
  tmp.len = 28;

  pack_u1(tmp.msg, 0, mode);
  pack_u1(tmp.msg, 1, 0);
  pack_u2(tmp.msg, 2, 1);
  pack_u4(tmp.msg, 4, lat*1e7);
  pack_u4(tmp.msg, 8, lon*1e7);
  pack_u4(tmp.msg, 12, alt*100);
  pack_u4(tmp.msg, 16, 50000);
  pack_u4(tmp.msg, 20, dur);
  pack_u4(tmp.msg, 24, 3000);
  start_transmit(&tmp);
}

static void set_ubx_rate(uint8_t msgclass, uint8_t msgid, uint32_t rate)
{
  ubxbuffer_t tmp;
  tmp.msgclass = 0x06;
  tmp.msgid = 0x01;
  tmp.len = 3;
  pack_u1(tmp.msg, 0, msgclass);
  pack_u1(tmp.msg, 1, msgid);
  pack_u1(tmp.msg, 2, rate);
  start_transmit(&tmp);
}

/*============================================================================*/
static void enable_txempty_irq(void)
/*------------------------------------------------------------------------------
  Function:
  switch on the tx empty itnerrupt. as soon as this interrupt is enabled, it is
  triggered all the time and can only be acknowledged by writing to the data
  register
  in:  none
  out: none
==============================================================================*/
{
  USART3_CR1 |= BIT_07;
}

/*============================================================================*/
static void disable_txempty_irq(void)
/*------------------------------------------------------------------------------
  Function:
  switch on the tx empty itnerrupt
  in:  none
  out: none
==============================================================================*/
{
  USART3_CR1 &= ~BIT_07;
}

/*============================================================================*/
static void ubx_rxhandler(void)
/*------------------------------------------------------------------------------
  Function:
  this is the handler for the rx interrupt on the usart. it processes the
  incoming data bytes, assembles the messages and verifies the checksum.
  in:  none
  out: none
==============================================================================*/
{
  static irqstatus_t status = ubx_header1;
  static uint16_t len;
  static uint32_t wrpos;
  static uint8_t cka;
  static uint8_t ckb;

  /* the received data byte */
  uint8_t tmpdata = USART3_DR;

  /* do not overwrite buffer with meaningful data */
  if(rxb.valid)
  {
    return;
  }

  switch(status)
  {
    /* can only proceed if the sync1 and sync2 bytes are received */
    case ubx_header1:
    {
      if(tmpdata == SYNC1)
      {
        status = ubx_header2;
      }

      /* return instead of break skips the checksum computation at the bottom */
      return;
    }

    case ubx_header2:
    {
      if(tmpdata == SYNC2)
      {
        /* if sync1 and then sync2 are received, reset the writing pos
           in the buffer and restart the checksum computation */
        cka = 0;
        ckb = 0;
        status = ubx_class;
      }
      else
      {
        status = ubx_header1;
      }

      /* return instead of break skips the checksum computation at the bottom */
      return;
    }

    /* receive the data in the order as described in the ublox manual */

    case ubx_class:
    {
      rxb.msgclass = tmpdata;
      status = ubx_id;
      break;
    }

    case ubx_id:
    {
      rxb.msgid = tmpdata;
      status = ubx_len_lsb;
      break;
    }

    case ubx_len_lsb:
    {
      len = tmpdata;
      status = ubx_len_msb;
      break;
    }

    case ubx_len_msb:
    {
      /* assemble the 16 bit length (little endian) */
      uint16_t lenmsb = tmpdata;
      lenmsb <<= 8;
      len += lenmsb;
      rxb.len = len;

      /* if the length received will not fit into the receive buffer, then abort */
      if(len > MAX_UBX_LEN)
      {
        status = ubx_header1;
      }
      else
      {
        /* reset the writing position in the receive buffer */
        wrpos = 0;
        status = ubx_data;
      }
      break;
    }

    case ubx_data:
    {
      rxb.msg[wrpos] = tmpdata;
      wrpos++;

      /* count the length of the received data to know when to start looking
         at the checksum bytes */
      len--;
      if(len == 0)
      {
        status = ubx_cka;
      }
      break;
    }

    case ubx_cka:
    {
      /* if the first checksum byte matches, compare the second one;
         if it doesn't match, abort */
      if(tmpdata == cka)
      {
        status = ubx_ckb;
      }
      else
      {
        status = ubx_header1;
      }

      /* return instead of break skips the checksum computation at the bottom */
      return;
    }

    case ubx_ckb:
    {
      /* if the first AND the second checksum byte match, a complete
         message has been received and is ready to be processed in the
         worker thread */
      if(tmpdata == ckb)
      {
        rxb.valid = true;
        status = ubx_header1;
      }
      break;
    }

    /* normally, this wont happen. make sure the rx interrupt starts in the
       header1 state if it does happen due to some error */
    default:
    {
      status = ubx_header1;
      return;
    }
  }

  /* fletcher8 checksum */
  cka = cka + tmpdata;
  ckb = ckb + cka;
}

/*============================================================================*/
static void ubx_txhandler(void)
/*------------------------------------------------------------------------------
  Function:
  the txhandler is the interrupt handler which is called whenever the tx data
  register is empty and thus new data can be transmitted
  in:  none
  out: none
==============================================================================*/
{
  static irqstatus_t status = ubx_header1;
  static uint8_t cka;
  static uint8_t ckb;
  static uint32_t read_pos;
  uint8_t tmpdata;

  /* do nothing if the buffer is empty */
  if(txb.empty == true)
  {
    disable_txempty_irq();
    return;
  }

  switch(status)
  {
    /* send sync1 then sync2; from here on, all the data to be sent is in the tx buffer */
    case ubx_header1:
    {
      USART3_DR = SYNC1;
      status = ubx_header2;

      /* return instead of break skips the checksum calcualtion */
      return;
    }

    case ubx_header2:
    {
      USART3_DR = SYNC2;
      status = ubx_class;

      /* reset the checksum computation and start reading ad position */
      cka = 0;
      ckb = 0;
      read_pos = 0;

      /* return instead of break skips the checksum calcualtion */
      return;
    }

    case ubx_class:
    {
      tmpdata = txb.msgclass;
      USART3_DR = tmpdata;
      status = ubx_id;
      break;
    }

    case ubx_id:
    {
      tmpdata = txb.msgid;
      USART3_DR = tmpdata;
      status = ubx_len_lsb;
      break;
    }

    case ubx_len_lsb:
    {
      tmpdata = txb.len;
      USART3_DR = tmpdata;
      status = ubx_len_msb;
      break;
    }

    case ubx_len_msb:
    {
      tmpdata = txb.len >> 8;
      USART3_DR = tmpdata;
      status = ubx_data;
      break;
    }

    case ubx_data:
    {
      tmpdata = txb.msg[read_pos];
      USART3_DR = tmpdata;
      read_pos++;
      if(read_pos == txb.len)
      {
        status = ubx_cka;
      }
      break;
    }

    case ubx_cka:
    {
      USART3_DR = cka;
      status = ubx_ckb;

      /* return instead of break skips the checksum calcualtion */
      return;
    }

    case ubx_ckb:
    {
      disable_txempty_irq();
      USART3_DR = ckb;
      status = ubx_header1;
      txb.empty = true;

      /* return instead of break skips the checksum calcualtion */
      return;
    }

    default:
    {
      disable_txempty_irq();
      status = ubx_header1;

      /* return instead of break skips the checksum calcualtion */
      return;
    }
  }

  /* fletcher8 checksum */
  cka = cka + tmpdata;
  ckb = ckb + cka;
}

static void ubx_irq_handler(void)
{
  GPIOE_BSRR = BIT_15;
  uint32_t status = USART3_SR;
  uint32_t cr = USART3_CR1;

  /* rx buffer not empty? */
  if(status & BIT_05)
  {
    ubx_rxhandler();
  }

  /* tx buffer empty? */
  if((status & BIT_07) && (cr & BIT_07))
  {
    ubx_txhandler();
  }
 GPIOE_BSRR = BIT_31;
}

static int32_t unpack_i4(const uint8_t* data, uint32_t offset)
{
  int32_t ret = data[offset + 3];
  ret <<= 8;
  ret += data[offset + 2];
  ret <<= 8;
  ret += data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}

static void pack_u4(uint8_t* buffer, uint32_t offset, uint32_t value)
{
  buffer[offset] = value;
  value >>= 8;
  buffer[offset + 1] = value;
  value >>= 8;
  buffer[offset + 2] = value;
  value >>= 8;
  buffer[offset + 3] = value;
}

static uint32_t unpack_u4(const uint8_t* data, uint32_t offset)
{
  uint32_t ret = data[offset + 3];
  ret <<= 8;
  ret += data[offset + 2];
  ret <<= 8;
  ret += data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}

static uint16_t unpack_u2(const uint8_t* data, uint32_t offset)
{
  uint16_t ret = data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}

static void pack_u2(uint8_t* buffer, uint32_t offset, uint16_t value)
{
  buffer[offset] = value;
  value >>= 8;
  buffer[offset + 1] = value;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
