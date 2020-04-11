/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    implementation of the ublox binary protocol
 *
 * Compiler:       ANSI-C
 *
 * Filename:       ublox.c
 *
 * Version:        1.1
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created

   [1.1]    01.04.2020    Tobias Plüss <tpluess@ieee.org>
   - use interrupts
   - configure periodic messages (e.g. UBX_TIM_TP, UBX_NAV_PVT)
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "ublox.h"
#include "stm32f407.h"
#include "misc.h"
#include "vic.h"
#include "timebase.h"

#include <string.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/


/* usart configuration */
#define BAUD_INITIAL 9600u /* initial baudrate */
#define BAUD_RECONFIGURE 115200u /* baudrate after initialisation */

/* macros to set the integer and fractional part of the baudrate generator */
#define BAUD_INT(x) ((uint32_t)(10000000u/(16u*x)))
#define BAUD_FRAC(x) ((uint32_t)(10000000u/x-16u*BAUD_INT(x)+0.5f))

/* usart interrupt vector number */
#define IRQ_NUM 39

/* maximum length of the buffers */
#define MAX_UBX_LEN 150u

/* ubx message classes */
#define UBX_CLASS_NAV 0x01u
#define UBX_CLASS_ACK 0x05u
#define UBX_CLASS_CFG 0x06u
#define UBX_CLASS_TIM 0x0du

#define UBX_ID_NAV_PVT 0x07u /* position, velocity, time */
#define UBX_ID_NAV_SAT 0x35u /* satellite info */
#define UBX_ID_TIM_TP 0x01u /* timepulse info */
#define UBX_ID_TIM_SVIN 0x04u /* survey-in data */
#define UBX_ID_CFG_TMODE2 0x3du /* timing mode configuration */
#define UBX_ID_CFG_NAVMODEL 0x24u /* navigation model configuration */
#define UBX_ID_CFG_RATE 0x01u /* message configuration */
#define UBX_ID_CFG_GNSS 0x3eu /* gnss configuration */
#define UBX_ID_CFG_PORT 0x00u /* i/o port configuration */

#define SYNC1 0xb5u
#define SYNC2 0x62u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/* pseudo functions for unpack and pack: */
/* unsigned 8-bit - works directly on the buffer, but is nicer to have the same
   interface as for the other data types */
#define pack_u8_le(x, y, z) x[y] = z
#define unpack_u8_le(x, y) x[y]

/* signed 32-bit - works the same as for unsigned, but has an embedded cast */
#define pack_i32_le(x, y, z) pack_u32_le(x, y, (uint32_t)(z))
#define unpack_i32_le(x, y) ((int32_t)unpack_u32_le(x, y))

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

typedef enum
{
  tmode_disable = 0u,
  tmode_svin = 1u,
  tmode_fixedpos = 2u
} tmode_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static ubxbuffer_t txb;
static ubxbuffer_t rxb;
static float qerr;
gpsinfo_t info;
svindata_t svinfo;

static ubxbuffer_t buf_pvt;
static ubxbuffer_t buf_svin;
static ubxbuffer_t buf_tp;
static ubxbuffer_t buf_sat;

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

#ifdef DEBUG
uint32_t rxover;
#endif


static void init_uart(void);
static void uart_config_baudrate(uint32_t baud);
static void enable_txempty_irq(void);
static void disable_txempty_irq(void);
static void start_transmit(const ubxbuffer_t* tmp);
static void ubx_rxhandler(void);
static void ubx_txhandler(void);
static void uart_irq_handler(void);
static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info);
static void unpack_svin(const uint8_t* rdata, svindata_t* info);
static void unpack_tp(const uint8_t* rdata, float* ret);
static void ubx_config_baudrate(uint32_t baudrate);
static void ubx_config_gnss(bool gps, bool glonass, bool galileo);
static void ubx_config_msgrate(uint8_t msgclass, uint8_t msgid, uint32_t rate);
static void ubx_config_navmodel(int8_t elev);
static void ubx_config_tmode(tmode_t mode, float lat, float lon, float alt,
  uint32_t dur);
static void pack_u32_le(uint8_t* buffer, uint32_t offset, uint32_t value);
static uint32_t unpack_u32_le(const uint8_t* data, uint32_t offset);
static void pack_u8_le6_le(uint8_t* buffer, uint32_t offset, uint16_t value);
static uint16_t unpack_u8_le6_le(const uint8_t* data, uint32_t offset);

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void ublox_init(void)
{
  qerr = 0.0f;
  txb.empty = true;
  rxb.valid = false;

  buf_tp.valid = false;
  buf_svin.valid = false;
  buf_pvt.valid = false;
  buf_sat.valid = false;

#ifdef DEBUG
  rxover = 0;
#endif

  init_uart();

  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* configure the reset pin */
  GPIOD_MODER |= (1u << 20);
  GPIOD_OTYPER |= (1u << 10);
  GPIOD_BSRR = BIT_26;
}

bool gps_wait_ack(void)
{
  bool ret;
  while(rxb.valid == false);
  /*__disable_irq();
  rxb.valid = false;
  __enable_irq();*/
  if(rxb.msgclass == 0x05)
  {
    if(rxb.msgid == 0x01)
    {
      ret = true;
    }
    else
    {
      ret = false;
    }
  }

  __disable_irq();
  rxb.valid = false;
  __enable_irq();
  return ret;
}

void gps_worker(void)
{
  static workerstatus_t status = reset;
  static uint64_t timestamp = 0;
  static uint32_t numvalidfix = 0;

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
      ubx_config_baudrate(BAUD_RECONFIGURE);
      uart_config_baudrate(BAUD_RECONFIGURE);
      timestamp = get_uptime_msec() + 100u;
      status = configure_messages;
      break;
    }

    case configure_messages:
    {
      USART3_CR1 |= BIT_05; // enable rx not empty interrupt

      do
      {
        /* disable all gnss except gps */
        ubx_config_gnss(true, false, false);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_navmodel(5); //10 degree elevation mask
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(0x0d, 0x01, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(0x01, 0x07, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(0x01, 0x35, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(0x0d, 0x04, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_tmode(tmode_disable, 0, 0, 0, 0);
      } while(gps_wait_ack() == false);

      timestamp = get_uptime_msec() + 1000u;
      status = normal;
      break;
    }

    case normal:
    {

      if(buf_tp.valid)
      {
        unpack_tp(buf_tp.msg, &qerr);
        buf_tp.valid = false;
      }

      if(buf_svin.valid)
      {
        unpack_svin(buf_svin.msg, &svinfo);
        buf_svin.valid = false;
      }

      if(buf_pvt.valid)
      {
        unpack_pvt(buf_pvt.msg, &info);
        buf_pvt.valid = false;

        if(info.fixtype == 3)
        {
          numvalidfix++;
          if(numvalidfix == 10)
            ubx_config_tmode(tmode_svin, 0, 0, 0, 7200);
        }
      }

      if(buf_sat.valid)
      {
        buf_sat.valid = false;
      }

    }
  }
}

float get_timepulse_error(void)
{
#ifdef DEBUG
  float tmp = qerr;
  qerr = 0;
  return tmp;
#else
  return qerr;
#endif
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void init_uart(void)
/*------------------------------------------------------------------------------
  Function:
  initialise the uart; the initial baudrate 9600 must be used to be able to
  communicate to the ublox module after reset
  in:  none
  out: none
==============================================================================*/
{
  /* enable gpio d and configure the uart pins */
  RCC_AHB1ENR |= BIT_03;
  GPIOD_MODER |= (2u << 16) | (2u << 18);
  GPIOD_AFRH |= (7u << 0) | (7u << 4);

  /* enable usart 3 and install the irq handler */
  RCC_APB1ENR |= BIT_18;
  USART3_BRR = (BAUD_FRAC(BAUD_INITIAL) << 0) | (BAUD_INT(BAUD_INITIAL) << 4);
  USART3_CR1 = BIT_13 | BIT_03 | BIT_02;
  vic_enableirq(IRQ_NUM, uart_irq_handler);
}


/*============================================================================*/
static void uart_config_baudrate(uint32_t baud)
/*------------------------------------------------------------------------------
  Function:
  configure the baudrate of the uart
  in:  baud -> new baudrate
  out: none
==============================================================================*/
{
  /* wait until tx done before the baudrate is changed */
  do
  {
    if(USART3_SR & BIT_06)
    {
      break;
    }
  } while(true);

  USART3_BRR = (BAUD_FRAC(baud) << 0) | (BAUD_INT(baud) << 4);
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
static void start_transmit(const ubxbuffer_t* tmp)
/*------------------------------------------------------------------------------
  Function:
  start the transmission of a message
  in:  tmp -> pointer to a message
  out: none
==============================================================================*/
{
  while(!txb.empty);
  disable_txempty_irq();
  txb.empty = false;
  memcpy(txb.msg, tmp->msg, tmp->len);
  txb.len = tmp->len;
  txb.msgclass = tmp->msgclass;
  txb.msgid = tmp->msgid;
  enable_txempty_irq();
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

  static uint8_t msgclass;
  static uint8_t msgid;
  static ubxbuffer_t* rxbuf;

  /* the received data byte */
  uint8_t tmpdata = USART3_DR;

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
      msgclass = tmpdata;
      status = ubx_id;
      break;
    }

    case ubx_id:
    {
      msgid = tmpdata;
      rxbuf = &rxb;

      if(msgclass == UBX_CLASS_NAV)
      {
        if(msgid == UBX_ID_NAV_SAT)
        {
          rxbuf = &buf_sat;
        }
        else if(msgid == UBX_ID_NAV_PVT)
        {
          rxbuf = &buf_pvt;
        }
      }
      else if(msgclass == UBX_CLASS_TIM)
      {
        if(msgid == UBX_ID_TIM_TP)
        {
          rxbuf = &buf_tp;
        }
        else if(msgid == UBX_ID_TIM_SVIN)
        {
          rxbuf = &buf_svin;
        }
      }

      rxbuf->msgclass = msgclass;
      rxbuf->msgid = msgid;

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
      rxbuf->len = len;

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
      rxbuf->msg[wrpos] = tmpdata;
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
        rxbuf->valid = true;
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


/*============================================================================*/
static void uart_irq_handler(void)
/*------------------------------------------------------------------------------
  Function:
  the interrupt handler; calls the rx and/or tx handlers
  in:  none
  out: none
==============================================================================*/
{
    GPIOE_BSRR = BIT_15;

  uint32_t status = USART3_SR;
  uint32_t cr = USART3_CR1;

#ifdef DEBUG
  if(status & BIT_03)
  {
    rxover++;
  }
#endif

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


/*============================================================================*/
static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info)
/*------------------------------------------------------------------------------
  Function:
  unpacks the NAV-PVT (position, velocity, time) message
  in:  rdata -> raw data
       info -> gpsinfo structure
  out: info is populated with position, velocity and time
==============================================================================*/
{
  info->year = unpack_u8_le6_le(rdata, 4);
  info->month = unpack_u8_le(rdata, 6);
  info->day = unpack_u8_le(rdata, 7);
  info->hour = unpack_u8_le(rdata, 8);
  info->min = unpack_u8_le(rdata, 9);
  info->sec = unpack_u8_le(rdata, 10);
  info->valid = unpack_u8_le(rdata, 11);
  info->tacc = unpack_u32_le(rdata, 12);
  info->fixtype = unpack_u8_le(rdata, 20);
  info->flags = unpack_u8_le(rdata, 21);
  info->xflags = unpack_u8_le(rdata, 22);
  info->numsv = unpack_u8_le(rdata, 23);
  info->lon = ((float)unpack_i32_le(rdata, 24))/1e7f;
  info->lat = ((float)unpack_i32_le(rdata, 28))/1e7f;
  info->height = unpack_i32_le(rdata, 32);
  info->hmsl = unpack_i32_le(rdata, 36);
  info->hacc = unpack_u32_le(rdata, 40);
  info->vacc = unpack_u32_le(rdata, 44);
  info->pdop = unpack_u8_le6_le(rdata, 76);
}


/*============================================================================*/
static void unpack_svin(const uint8_t* rdata, svindata_t* info)
/*------------------------------------------------------------------------------
  Function:
  unpacks the TIM-SVIN info message
  in:  rdata -> raw data
       info -> svindata structure
  out: info is populated with the survey-in information
==============================================================================*/
{
  info->dur = unpack_u32_le(rdata, 0);
  info->x = unpack_i32_le(rdata, 4);
  info->y = unpack_i32_le(rdata, 8);
  info->z = unpack_i32_le(rdata, 12);
  info->meanv = ((float)unpack_u32_le(rdata, 16))/1e6f;
  info->obs = unpack_u32_le(rdata, 20);
  if(unpack_u8_le(rdata, 24) > 0)
  {
    info->valid = true;
  }
  else
  {
    info->valid = false;
  }
  if(unpack_u8_le(rdata, 25) > 0)
  {
    info->active = true;
  }
  else
  {
    info->active = false;
  }
}


/*============================================================================*/
static void unpack_tp(const uint8_t* rdata, float* ret)
/*------------------------------------------------------------------------------
  Function:
  unpacks the TIM-TP message
  in:  rdata -> raw data
       ret -> return value
  out: returns the timepulse quantisation error in the ret pointer
==============================================================================*/
{
  int32_t tmp = unpack_i32_le(rdata, 8);
  *ret = ((float)tmp) / 1000.0f;
}


/*============================================================================*/
static void ubx_config_baudrate(uint32_t baudrate)
/*------------------------------------------------------------------------------
  Function:
  configures the baud rate of the ublox module
  in:  baudrate -> new baudrate
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_PORT;
  tmp.len = 20u;

  pack_u8_le(tmp.msg, 0, 1); /* 1 = UART */
  pack_u8_le(tmp.msg, 1, 0);
  pack_u8_le6_le(tmp.msg, 2, 0);
  pack_u32_le(tmp.msg, 4, BIT_06 | BIT_07 | BIT_11); /* 8N1 */
  pack_u32_le(tmp.msg, 8, baudrate);
  pack_u8_le6_le(tmp.msg, 12, BIT_00); /* allow only ubx protocol */
  pack_u8_le6_le(tmp.msg, 14, BIT_00);
  pack_u8_le6_le(tmp.msg, 16, 0);
  pack_u8_le6_le(tmp.msg, 18, 0);
  start_transmit(&tmp);
}


/*============================================================================*/
static void ubx_config_gnss(bool gps, bool glonass, bool galileo)
/*------------------------------------------------------------------------------
  Function:
  configure the gnss system to be used
  in:  gps -> set to true to use gps (qzss needs to be enabled together with gps
       glonass -> set to true to use glonass
       galileo -> set to true to use galileo
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_GNSS;
  tmp.len = 60;

  pack_u8_le(tmp.msg, 0, 0); /* version 0 */
  pack_u8_le(tmp.msg, 1, 0); /* read only */
  pack_u8_le(tmp.msg, 2, 0xff); /* use max. # of tracking channels */
  pack_u8_le(tmp.msg, 3, 7); /* use 7 config blocks */

  pack_u8_le(tmp.msg, 4, 0); /* gps */
  pack_u8_le(tmp.msg, 5, 8); /* use minimum 8 tracking channels */
  pack_u8_le(tmp.msg, 6, 16); /* use at most 16 tracking channels */
  pack_u8_le(tmp.msg, 7, 0); /* reserved */
  pack_u32_le(tmp.msg, 8, 0x01010000 | (gps ? BIT_00 : 0));

  pack_u8_le(tmp.msg, 12, 1); /* sbas - should be disabled for timing */
  pack_u8_le(tmp.msg, 13, 1);
  pack_u8_le(tmp.msg, 14, 3);
  pack_u8_le(tmp.msg, 15, 0);
  pack_u32_le(tmp.msg, 16, 0x01010000);

  pack_u8_le(tmp.msg, 20, 2); /* galileo */
  pack_u8_le(tmp.msg, 21, 4);
  pack_u8_le(tmp.msg, 22, 8);
  pack_u8_le(tmp.msg, 23, 0);
  pack_u32_le(tmp.msg, 24, 0x01010000 | (galileo ? BIT_00 : 0));

  pack_u8_le(tmp.msg, 28, 3); /* beidou */
  pack_u8_le(tmp.msg, 29, 8);
  pack_u8_le(tmp.msg, 30, 16);
  pack_u8_le(tmp.msg, 31, 0);
  pack_u32_le(tmp.msg, 32, 0x01010000);

  pack_u8_le(tmp.msg, 36, 4); /* imes */
  pack_u8_le(tmp.msg, 37, 0);
  pack_u8_le(tmp.msg, 38, 8);
  pack_u8_le(tmp.msg, 39, 0);
  pack_u32_le(tmp.msg, 40, 0x03010000);

  pack_u8_le(tmp.msg, 44, 5); /* qzss - should be enabled together with gps */
  pack_u8_le(tmp.msg, 45, 0);
  pack_u8_le(tmp.msg, 46, 3);
  pack_u8_le(tmp.msg, 47, 0);
  pack_u32_le(tmp.msg, 48, 0x05010000 | (gps ? BIT_00 : 0));

  pack_u8_le(tmp.msg, 52, 6); /* glonass */
  pack_u8_le(tmp.msg, 53, 8);
  pack_u8_le(tmp.msg, 54, 14);
  pack_u8_le(tmp.msg, 55, 0);
  pack_u32_le(tmp.msg, 56, 0x01010000 | (glonass ? BIT_00 : 0));

  start_transmit(&tmp);
}


/*============================================================================*/
static void ubx_config_msgrate(uint8_t msgclass, uint8_t msgid, uint32_t rate)
/*------------------------------------------------------------------------------
  Function:
  configure messages to be periodically sent. the messages of (msgclass, msgid)
  are configured to be sent periodically with the desired repetition rate
  in:  msgclass -> message class
       msgid -> message id
       rate -> message rate
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_RATE;
  tmp.len = 3;
  pack_u8_le(tmp.msg, 0, msgclass);
  pack_u8_le(tmp.msg, 1, msgid);
  pack_u8_le(tmp.msg, 2, rate);
  start_transmit(&tmp);
}


/*============================================================================*/
static void ubx_config_navmodel(int8_t elev)
/*------------------------------------------------------------------------------
  Function:
  configure the navigation model to stationary, set an elevation mask and use
  automatic mode for utc
  in:  msgclass -> message class
       msgid -> message id
       rate -> message rate
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_NAVMODEL;
  tmp.len = 36;

  pack_u8_le6_le(tmp.msg, 0, BIT_00 | BIT_01 | BIT_10);
  pack_u8_le(tmp.msg, 2, 2); /* dynamic model: stationary */
  pack_u8_le(tmp.msg, 3, 0);
  pack_u32_le(tmp.msg, 4, 0);
  pack_u32_le(tmp.msg, 8, 0);
  pack_u8_le(tmp.msg, 12, elev); /* elevation mask */
  pack_u8_le(tmp.msg, 13, 0);
  pack_u8_le6_le(tmp.msg, 14, 0);
  pack_u8_le6_le(tmp.msg, 16, 0);
  pack_u8_le6_le(tmp.msg, 18, 0);
  pack_u8_le6_le(tmp.msg, 20, 0);
  pack_u8_le(tmp.msg, 22, 0);
  pack_u8_le(tmp.msg, 23, 0);
  pack_u8_le(tmp.msg, 24, 0);
  pack_u8_le(tmp.msg, 25, 0);
  pack_u8_le6_le(tmp.msg, 26, 0);
  pack_u8_le6_le(tmp.msg, 28, 0);
  pack_u8_le(tmp.msg, 30, 0); /* utc standard auto */
  pack_u8_le(tmp.msg, 31, 0);
  pack_u8_le(tmp.msg, 32, 0);
  pack_u8_le(tmp.msg, 33, 0);
  pack_u8_le(tmp.msg, 34, 0);
  pack_u8_le(tmp.msg, 35, 0);
  start_transmit(&tmp);
}


/*============================================================================*/
static void ubx_config_tmode(tmode_t mode, float lat, float lon, float alt,
  uint32_t dur)
/*------------------------------------------------------------------------------
  Function:
  configure the timing mode
  in:  mode -> disable timing mode, start survey-in or enable fixed pos mode
       lat -> latitude for fixed position mode
       lon -> longitude for fixed position mode
       alt -> altitude for fixed position mode
       dur -> duration of survey-in
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_TMODE2;
  tmp.len = 28;

  pack_u8_le(tmp.msg, 0, mode);
  pack_u8_le(tmp.msg, 1, 0);
  pack_u8_le6_le(tmp.msg, 2, 1);
  pack_u32_le(tmp.msg, 4, lat*1e7); /* for fixed position mode only */
  pack_u32_le(tmp.msg, 8, lon*1e7);
  pack_u32_le(tmp.msg, 12, alt*100);
  pack_u32_le(tmp.msg, 16, 0); /* fixed position accuracy */
  pack_u32_le(tmp.msg, 20, dur);
  pack_u32_le(tmp.msg, 24, 10000);  /* survey-in accuracy limit */
  start_transmit(&tmp);
}


/*============================================================================*/
static void pack_u32_le(uint8_t* buffer, uint32_t offset, uint32_t value)
/*------------------------------------------------------------------------------
  Function:
  convert a unsigned 32-bit number to 4 bytes and store it in a buffer
  (little endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/
{
  buffer[offset] = value;
  value >>= 8;
  buffer[offset + 1] = value;
  value >>= 8;
  buffer[offset + 2] = value;
  value >>= 8;
  buffer[offset + 3] = value;
}


/*============================================================================*/
static uint32_t unpack_u32_le(const uint8_t* data, uint32_t offset)
/*------------------------------------------------------------------------------
  Function:
  construct a 32-bit unsigned value from raw data bytes in a buffer
  (little endian)
  in:  data -> the raw data bytes
       offset -> offset of the LSB
  out: returns the unpacked value
==============================================================================*/
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


/*============================================================================*/
static void pack_u8_le6_le(uint8_t* buffer, uint32_t offset, uint16_t value)
/*------------------------------------------------------------------------------
  Function:
  convert an unsigned 16-bit number into raw bytes and store them in a buffer
  (little endian)
  in:  buffer -> buffer for the raw data
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/
{
  buffer[offset] = value;
  value >>= 8;
  buffer[offset + 1] = value;
}


/*============================================================================*/
static uint16_t unpack_u8_le6_le(const uint8_t* data, uint32_t offset)
/*------------------------------------------------------------------------------
  Function:
  construct a 16-bit unsigned number from a buffer containing raw data bytes
  (little endian)
  in:  data -> buffer containing the raw data
       offset -> offset into the buffer
  out: returns the unpacked value
==============================================================================*/
{
  uint16_t ret = data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/