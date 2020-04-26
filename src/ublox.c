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
#include "convert.h"
#include "eeprom.h"

#include <math.h>
#include <string.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/


/* usart configuration */
#define BAUD_INITIAL 9600u /* initial baudrate */
#define BAUD_RECONFIGURE 115200u /* baudrate after initialisation */

/* macros to set the integer and fractional part of the baudrate generator */
#define BAUD_INT(x) ((uint32_t)(10000000u/(16u*(x))))
#define BAUD_FRAC(x) ((uint32_t)(10000000u/x-16u*BAUD_INT(x)+0.5f))

/* usart interrupt vector number */
#define IRQ_NUM 39

/* maximum length of the buffers */
#define MAX_UBX_LEN 350u

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
#define UBX_ID_ACK 0x01u
#define UBX_ID_NAK 0x00u

#define SYNC1 0xb5u
#define SYNC2 0x62u

#define RXBUFFER_SIZE 1024u

#define RECEIVE_TIMEOUT 150u /* ms */

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

typedef enum
{
  reset,
  wait_ready,
  configure_uart,
  configure_messages,
  wait_fix,
  fixed_pos,
  survey_in,
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
static bool got_ack;
static bool got_nak;
static uint8_t rxdata_raw[RXBUFFER_SIZE];
static volatile uint32_t num_rxdata;

static float qerr;
gpsinfo_t info;
svindata_t svinfo;
sv_info_t svi;

/* not static because from eeprom */
extern config_t cfg;

static workerstatus_t status;

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static void init_uart(void);
static void uart_config_baudrate(uint32_t baud);
static void enable_txempty_irq(void);
static void disable_txempty_irq(void);
static void start_transmit(const ubxbuffer_t* tmp);
static void ubx_rxhandler(void);
static void ubx_txhandler(void);
static void uart_irq_handler(void);
static bool gps_wait_ack(void);
static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info);
static void unpack_svin(const uint8_t* rdata, svindata_t* info);
static void unpack_tp(const uint8_t* rdata, float* ret);
static void unpack_sv(const uint8_t* rdata, sv_info_t* svi);
static void ubx_config_baudrate(uint32_t baudrate);
static void ubx_config_gnss(bool gps, bool glonass, bool galileo);
static void ubx_config_msgrate(uint8_t msgclass, uint8_t msgid, uint32_t rate);
static void ubx_config_navmodel(int8_t elev);
static void ubx_config_tmode(tmode_t mode, int32_t x, int32_t y, int32_t z,
  uint32_t dur, uint32_t acc, uint32_t acc_lim);
static bool process_messages(void);

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void ublox_init(void)
{
  qerr = 0.0f;
  txb.empty = true;
  got_ack = false;
  got_nak = false;
  num_rxdata = 0;
  status = reset;

  init_uart();

  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* configure the reset pin */
  GPIOD_MODER |= (1u << 20);
  GPIOD_OTYPER |= (1u << 10);
}



void gps_worker(void)
{
  static uint64_t timestamp = 0;
  static uint32_t numvalidfix = 0;
  static uint32_t lasttime = 0;
  uint64_t currenttime = get_uptime_msec();

  /* continue only if the timeout expired */
  if((timestamp != 0) && (timestamp >= currenttime))
  {
    return;
  }

  switch(status)
  {
    case reset:
    {
      /* the uart must use the initial baud rate after reset */
      uart_config_baudrate(BAUD_INITIAL);

      /* reset pin low, continue after 1 sec */
      GPIOD_BSRR = BIT_26;
      timestamp = get_uptime_msec() + 1000u;
      status = wait_ready;
      break;
    }

    case wait_ready:
    {
      /* reset pin high, allow the module 1 sec for startup  */
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
      do
      {
        ubx_config_gnss(cfg.use_gps, cfg.use_glonass, cfg.use_galileo);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_navmodel(cfg.elevation_mask);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_TP, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(UBX_CLASS_NAV, UBX_ID_NAV_SAT, 1);
      } while(gps_wait_ack() == false);

      do
      {
        ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_SVIN, 1);
      } while(gps_wait_ack() == false);


      /*if(cfg.fixpos_valid)
      {
        do
        {
          set_fixpos_mode();
        } while(gps_wait_ack() == false);

        status = fixed_pos;
      }
      else
      {*/
      disable_tmode();

      if(cfg.auto_svin)
      {
        status = wait_fix;
      }
      else
      {
        status = survey_in;
      }

        /*do
        {
          start_svin();
        } while(gps_wait_ack() == false);*/


      //}
      break;
    }

    case wait_fix:
    {
      process_messages();

      /* start survey-in as soon as there is a fix */
      if((info.age_msec == 0) && (info.fixtype == 3))
      {
        start_svin();
        status = survey_in;
      }
    }

    case survey_in:
    {
      if(process_messages())
      {

        /* if we got some info about the survey-in AND the survey-in is valid
           AND the survey-in is not active anymore, we should store the current
           position in the eeprom and switch to fixed-position mode */
        if((svinfo.age_msec == 0) && (svinfo.valid == true) && (svinfo.active == false))
        {
          cfg.x = svinfo.x;
          cfg.y = svinfo.y;
          cfg.z = svinfo.z;
          cfg.accuracy = sqrt(svinfo.meanv);
          cfg.fixpos_valid = true;

//          ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_SVIN, 0);

          status = fixed_pos;
        }
      }
      break;
    }

    case fixed_pos:
    {
      process_messages();
      break;
    }
  }

  uint64_t msec = currenttime - lasttime;
  lasttime = currenttime;
  svinfo.age_msec += msec;
  info.age_msec += msec;
}


float get_timepulse_error(void)
{
  float tmp = qerr;
  qerr = 0;
  return tmp;
}


void start_svin(void)
{
  ubx_config_tmode(tmode_svin, 0, 0, 0, cfg.svin_dur, 0, cfg.accuracy_limit);
}


void set_fixpos_mode(void)
{
    ubx_config_tmode(tmode_fixedpos, cfg.x, cfg.y, cfg.z, 0, cfg.accuracy, 0);
}


void disable_tmode(void)
{
    ubx_config_tmode(tmode_disable, 0, 0, 0, 0, 0, 0);
}


void gps_restart(void)
{
  status = reset;
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
  USART3_CR1 = BIT_13 | BIT_05 | BIT_03 | BIT_02;
  //USART3_CR1 |= BIT_05;
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
  this is the receive data handler for the usart. it does nothing more than
  simply store each received data byte in a circular buffer.
  in:  none
  out: none
==============================================================================*/
{
  /* wrpos always points to a writable position */
  static int wrpos = 0;

  /* read the received data and put it into the circular buffer */
  uint8_t tmp = USART3_DR;
  rxdata_raw[wrpos] = tmp;
  num_rxdata++;

  /* go to the next position in the buffer and take wrap-around into account;
     old data is overwritten, no matter whether it was already processed
     or not! */
  wrpos++;
  if(wrpos == RXBUFFER_SIZE)
  {
    wrpos = 0;
  }
}


/*============================================================================*/
static bool process_messages(void)
/*------------------------------------------------------------------------------
  Function:
  checks whether there are messages to be received. if receive data is pending,
  a timeout is started and if the data is not complete within the timeout, the
  data is dropped.
  in:  none
  out: returns true when one complete message was received
==============================================================================*/
{
  irqstatus_t status = ubx_header1;
  static uint32_t rdpos = 0;

  uint32_t wrpos;
  uint8_t cka;
  uint8_t ckb;
  ubxbuffer_t rxb;
  uint8_t tmpdata;

  uint64_t time = 0;

  while(true)
  {
    if(num_rxdata > 0)
    {
      __disable_irq();
      num_rxdata--;
      tmpdata = rxdata_raw[rdpos];
      __enable_irq();

      rdpos++;
      if(rdpos == RXBUFFER_SIZE)
      {
        rdpos = 0;
      }
    }
    else
    {
      if(time == 0)
      {
        /* never actually started receiving something */
        return false;
      }
      if(get_uptime_msec() - time >= RECEIVE_TIMEOUT)
      {
        /* started receiving something, but didn't get the data on time */
        return false;
      }
      else
      {
        continue;
      }
    }

    switch(status)
    {
      /* can only proceed if the sync1 and sync2 bytes are received */
      case ubx_header1:
      {
        if(tmpdata == SYNC1)
        {
          time = get_uptime_msec();
          status = ubx_header2;
        }

        continue;
      }

      case ubx_header2:
      {
        if(tmpdata == SYNC2)
        {
          status = ubx_class;
        }
        else
        {
          status = ubx_header1;
        }

        continue;
      }

      /* receive the data in the order as described in the ublox manual */

      case ubx_class:
      {
        cka = 0;
        ckb = 0;
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
        rxb.len = tmpdata;
        status = ubx_len_msb;
        break;
      }

      case ubx_len_msb:
      {
        /* assemble the 16 bit length (little endian) */
        uint16_t lenmsb = tmpdata;
        lenmsb <<= 8;
        rxb.len += lenmsb;

        /* if the length received will not fit into the receive buffer, then abort */
        if(rxb.len > MAX_UBX_LEN)
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

        if(wrpos == rxb.len)
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

        continue;
      }

      case ubx_ckb:
      {
        /* if the first AND the second checksum byte match, a complete
           message has been received and is ready to be processed in the
           worker thread */
        if(tmpdata == ckb)
        {
          rxb.valid = true;

          if(rxb.msgclass == UBX_CLASS_NAV)
          {
            if(rxb.msgid == UBX_ID_NAV_SAT)
            {
              unpack_sv(rxb.msg, &svi);
              return true;
            }
            else if(rxb.msgid == UBX_ID_NAV_PVT)
            {
              unpack_pvt(rxb.msg, &info);
              return true;
            }
          }
          else if(rxb.msgclass == UBX_CLASS_TIM)
          {
            if(rxb.msgid == UBX_ID_TIM_TP)
            {
              unpack_tp(rxb.msg, &qerr);
              return true;
            }
            else if(rxb.msgid == UBX_ID_TIM_SVIN)
            {
              unpack_svin(rxb.msg, &svinfo);
              return true;
            }
          }
          else if(rxb.msgclass == UBX_CLASS_ACK)
          {
            if(rxb.msgid == UBX_ID_ACK)
            {
              got_ack = true;
            }
            else if(rxb.msgid == UBX_ID_NAK)
            {
              got_nak = true;
            }
            return true;
          }

        }
      }

    }

    /* fletcher8 checksum */
    cka = cka + tmpdata;
    ckb = ckb + cka;
  }
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
  if(txb.empty)
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
}


/*============================================================================*/
static bool gps_wait_ack(void)
/*------------------------------------------------------------------------------
  Function:
  actively wait until an ack or nak message is received
  in:  none
  out: returns true if ACK received, false in case of NAK
==============================================================================*/
{
  got_ack = false;
  got_nak = false;
  while(true)
  {
    if(process_messages())
    {
      if(got_ack)
      {
        return true;
      }
      else if(got_nak)
      {
        return false;
      }
      else
      {
        continue;
      }
    }
  }
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
  info->year = unpack_u16_le(rdata, 4);
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
  info->pdop = unpack_u16_le(rdata, 76);
  info->age_msec = 0;
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
  info->meanv = unpack_u32_le(rdata, 16);
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
  info->age_msec = 0;
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
static void unpack_sv(const uint8_t* rdata, sv_info_t* svi)
/*------------------------------------------------------------------------------
  Function:
  unpack the NAV-SAT message
  in:  rdata -> raw data
       svi -> satellites in view info
  out: none
==============================================================================*/
{
  svi->numsv = unpack_u8_le(rdata, 5);
  if(svi->numsv > MAX_SV)
  {
    svi->numsv = MAX_SV;
  }
  for(uint8_t n = 0; n < svi->numsv; n++)
  {
    svi->sats[n].gnssid = unpack_u8_le(rdata, 8 + 12*n);
    svi->sats[n].svid = unpack_u8_le(rdata, 9 + 12*n);
    svi->sats[n].cno = unpack_u8_le(rdata, 10 + 12*n);
    svi->sats[n].elev = unpack_i8_le(rdata, 11 + 12*n);
    svi->sats[n].azim = unpack_i16_le(rdata, 12 + 12*n);
  }
  svi->age_msec = 0;
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
  pack_u16_le(tmp.msg, 2, 0);
  pack_u32_le(tmp.msg, 4, BIT_06 | BIT_07 | BIT_11); /* 8N1 */
  pack_u32_le(tmp.msg, 8, baudrate);
  pack_u16_le(tmp.msg, 12, BIT_00); /* allow only ubx protocol */
  pack_u16_le(tmp.msg, 14, BIT_00);
  pack_u16_le(tmp.msg, 16, 0);
  pack_u16_le(tmp.msg, 18, 0);
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

  pack_u16_le(tmp.msg, 0, BIT_00 | BIT_01 | BIT_10);
  pack_u8_le(tmp.msg, 2, 2); /* dynamic model: stationary */
  pack_u8_le(tmp.msg, 3, 0);
  pack_u32_le(tmp.msg, 4, 0);
  pack_u32_le(tmp.msg, 8, 0);
  pack_u8_le(tmp.msg, 12, elev); /* elevation mask */
  pack_u8_le(tmp.msg, 13, 0);
  pack_u16_le(tmp.msg, 14, 0);
  pack_u16_le(tmp.msg, 16, 0);
  pack_u16_le(tmp.msg, 18, 0);
  pack_u16_le(tmp.msg, 20, 0);
  pack_u8_le(tmp.msg, 22, 0);
  pack_u8_le(tmp.msg, 23, 0);
  pack_u8_le(tmp.msg, 24, 0);
  pack_u8_le(tmp.msg, 25, 0);
  pack_u16_le(tmp.msg, 26, 0);
  pack_u16_le(tmp.msg, 28, 0);
  pack_u8_le(tmp.msg, 30, 0); /* utc standard auto */
  pack_u8_le(tmp.msg, 31, 0);
  pack_u8_le(tmp.msg, 32, 0);
  pack_u8_le(tmp.msg, 33, 0);
  pack_u8_le(tmp.msg, 34, 0);
  pack_u8_le(tmp.msg, 35, 0);
  start_transmit(&tmp);
}


/*============================================================================*/
static void ubx_config_tmode(tmode_t mode, int32_t x, int32_t y, int32_t z,
  uint32_t dur, uint32_t acc, uint32_t acc_lim)
/*------------------------------------------------------------------------------
  Function:
  configure the timing mode
  in:  mode -> disable timing mode, start survey-in or enable fixed pos mode
       x, y, z -> ecef coordinates
       dur -> duration of survey-in
       acc -> accuracy of the position data (if fixed position mode)
       acc_lim -> accuracy limit in mm for survey-in
  out: none
==============================================================================*/
{
  ubxbuffer_t tmp;
  tmp.msgclass = UBX_CLASS_CFG;
  tmp.msgid = UBX_ID_CFG_TMODE2;
  tmp.len = 28;

  pack_u8_le(tmp.msg, 0, mode);
  pack_u8_le(tmp.msg, 1, 0);
  pack_u16_le(tmp.msg, 2, 0); /* use ecef coordinates */
  pack_i32_le(tmp.msg, 4, x); /* for fixed position mode only */
  pack_i32_le(tmp.msg, 8, y);
  pack_i32_le(tmp.msg, 12, z);
  pack_u32_le(tmp.msg, 16, acc); /* fixed position accuracy */
  pack_u32_le(tmp.msg, 20, dur); /* survey-in duration */
  pack_u32_le(tmp.msg, 24, acc_lim);  /* survey-in accuracy limit */
  start_transmit(&tmp);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
