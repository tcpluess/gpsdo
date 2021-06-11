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

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "event_groups.h"
#include "task.h"
#include "ublox.h"
#include "stm32f407.h"
#include "misc.h"
#include "nvic.h"
#include "timebase.h"
#include "convert.h"
#include "eeprom.h"

#include <math.h>
#include <stdio.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/* usart configuration */
#define BAUD_INITIAL 9600u /* initial baudrate */
#define BAUD_RECONFIGURE 921600u /* baudrate after initialisation */

/* macros to set the integer and fractional part of the baudrate generator */
#define BAUD_INT(x) ((uint32_t)(40000000.0/(16.0*(x))))
#define BAUD_FRAC(x) ((uint32_t)((40000000.0/x-16.0*BAUD_INT(x))+0.5))

/* usart interrupt vector number */
#define IRQ_NUM 39

/* maximum length of the buffers */
#define MAX_UBX_LEN 500u

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

#define RX_BUFFER_SIZE 200u
#define TX_BUFFER_SIZE 50u
#define RECEIVE_TIMEOUT pdMS_TO_TICKS(10u)
#define ACK_TIMEOUT 100u /* ms */
#define RESET_DELAY pdMS_TO_TICKS(100u)

#define FIX_3D 3u
#define FIX_TIME 5u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

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
  ubx_ckb
} irqstatus_t;

typedef struct
{
  uint8_t msg[MAX_UBX_LEN];
  uint8_t msgclass;
  uint8_t msgid;
  uint16_t len;
} ubxbuffer_t;

typedef enum
{
  tmode_disable = 0u,
  tmode_svin = 1u,
  tmode_fixedpos = 2u
} tmode_t;

typedef struct
{
  float ns;
  bool valid;
  uint8_t flags;
  uint8_t refinfo;
} qerr_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static qerr_t qerr;
gpsinfo_t pvt_info;
svindata_t svin_info;
sv_info_t sat_info;

/* not static because from eeprom */
extern config_t cfg;


/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static void rx_task(void* param);
static void init_uart(void);
static void uart_config_baudrate(uint32_t baud);
static void enable_txempty_irq(void);
static void disable_txempty_irq(void);
static void uart_irq_handler(void);
static bool gps_wait_ack(void);
static void unpack_pvt(const uint8_t* rdata, gpsinfo_t* info);
static void unpack_svin(const uint8_t* rdata, svindata_t* info);
static void unpack_tp(const uint8_t* rdata, qerr_t* ret);
static void unpack_sv(const uint8_t* rdata, sv_info_t* svi);
static void ubx_config_baudrate(uint32_t baudrate);
static void ubx_config_gnss(bool gps, bool glonass, bool galileo);
static void ubx_config_msgrate(uint8_t msgclass, uint8_t msgid, uint8_t rate);
static void ubx_config_navmodel(int8_t elev);
static void ubx_config_tmode(tmode_t mode, int32_t x, int32_t y, int32_t z,
  uint32_t dur, uint32_t acc, uint32_t acc_lim);
static void ubx_receive_packet(ubxbuffer_t* rx);

static void ubx_transmit_packet(const ubxbuffer_t* tx);

static void init_pins(void);
static void gps_reset(bool do_reset);

static StreamBufferHandle_t rxstream;
static StreamBufferHandle_t txstream;
static EventGroupHandle_t ublox_events;


#define EVENT_DO_SVIN BIT_00
#define EVENT_DISABLE_TMODE BIT_01
#define EVENT_SET_FIXPOS_MODE BIT_02

#define EVENT_TP_RECEIVED BIT_04
#define EVENT_PVT_RECEIVED BIT_05
#define EVENT_SAT_RECEIVED BIT_06
#define EVENT_SVIN_RECEIVED BIT_07
#define EVENT_TIMEPULSE_OK BIT_08
#define EVENT_ACK_RECEIVED BIT_09
#define EVENT_NAK_RECEIVED BIT_10

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void gps_task(void* param)
{
  (void)param;

  rxstream = xStreamBufferCreate(RX_BUFFER_SIZE, 10);
  txstream = xStreamBufferCreate(TX_BUFFER_SIZE, 1);
  ublox_events = xEventGroupCreate();
  (void)xTaskCreate(rx_task, "gps rx", 512, NULL, 2, NULL);
  qerr.valid = false;

  /* initialise hardware */
  init_pins();
  init_uart();

  /* the uart must use the initial baud rate after reset */
  uart_config_baudrate(BAUD_INITIAL);

  /* reset the gps module */
  gps_reset(true);
  vTaskDelay(RESET_DELAY);
  gps_reset(false);
  vTaskDelay(RESET_DELAY);

  /* after the module has started up, the uart is reconfigured:
   - high baud rate
   - disable nmea messages */
  ubx_config_baudrate(BAUD_RECONFIGURE);

  /* the gps module is configured:
   - gnss system to use
   - dynamic model and elevation mask
   - periodic reporting of certain messages
   - survey-in, fixed-position or normal mode */
  ubx_config_gnss(cfg.use_gps, cfg.use_glonass, cfg.use_galileo);
  ubx_config_navmodel(cfg.elevation_mask);
  ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_TP, 1);
  ubx_config_msgrate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);
  ubx_config_msgrate(UBX_CLASS_NAV, UBX_ID_NAV_SAT, 1);

  /* timing mode must be disabled before survey-in can be started again */
  ubx_config_tmode(tmode_disable, 0, 0, 0, 0, 0, 0);
  for(;;)
  {
    /* wait until at least one of the following events occurs:
     - data received from the gps module
     - request to disable timing mode
     - request to perform a survey-in
     - request to configure the fixed position mode */
    uint32_t bits = EVENT_DISABLE_TMODE | EVENT_DO_SVIN | EVENT_SET_FIXPOS_MODE | EVENT_SVIN_RECEIVED;
    bits = xEventGroupWaitBits(ublox_events, bits, true, false, portMAX_DELAY);

    if(bits & EVENT_DISABLE_TMODE)
    {
      ubx_config_tmode(tmode_disable, 0, 0, 0, cfg.svin_dur, 0, cfg.accuracy_limit);
      ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_SVIN, 0);
    }

    if(bits & EVENT_DO_SVIN)
    {
      ubx_config_tmode(tmode_disable, 0, 0, 0, 0, 0, 0);
      ubx_config_tmode(tmode_svin, 0, 0, 0, cfg.svin_dur, 0, cfg.accuracy_limit);
      ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_SVIN, 1);
    }

    if(bits & EVENT_SET_FIXPOS_MODE)
    {
      ubx_config_tmode(tmode_fixedpos, cfg.x, cfg.y, cfg.z, 0, cfg.accuracy, 0);
    }

    if(bits & EVENT_SVIN_RECEIVED)
    {
      /* if the svin is finished, disable further svin messages */
      if((svin_info.valid == true) && (svin_info.active == false))
      {
        ubx_config_msgrate(UBX_CLASS_TIM, UBX_ID_TIM_SVIN, 0);

        /* copy the svin data to the configuration */
        cfg.x = svin_info.x;
        cfg.y = svin_info.y;
        cfg.z = svin_info.z;
        cfg.accuracy = (uint32_t)sqrt((double)svin_info.meanv);
        cfg.fixpos_valid = true;
      }
    }
  }
}


bool get_timepulse_error(float* result)
{
  bool valid;
  vTaskSuspendAll();
  valid = qerr.valid;
  *result = qerr.ns;
  qerr.valid = false;
  xTaskResumeAll();

  /* this should never happen */
  if(valid == false)
  {
    (void)printf("# something went wrong: timepulse qerr = 0!\n");
  }
  return valid;
}


void start_svin(void)
{
  (void)xEventGroupSetBits(ublox_events, EVENT_DO_SVIN);
}


void set_fixpos_mode(void)
{
  (void)xEventGroupSetBits(ublox_events, EVENT_SET_FIXPOS_MODE);
}


void disable_tmode(void)
{
  (void)xEventGroupSetBits(ublox_events, EVENT_DISABLE_TMODE);
/*  if(cfg.fixpos_valid)
  {
    set_fixpos_mode();
    status = fixpos_mode;
  }
  else
  {
    status = normal;
  }*/
}


bool gps_waitready(void)
{
  uint32_t bits = EVENT_TP_RECEIVED;
  if(xEventGroupWaitBits(ublox_events, bits, true, true, 0))
  {
    return true;
  }
  else
  {
    return false;
  }
}



/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void rx_task(void* param)
/*------------------------------------------------------------------------------
  Function:
  this task does nothing more than just wait until data from the gps module is
  received. it then processes the received data and notifies any tasks waiting
  for this data.
  in:  param -> not used (freertos)
  out: none
==============================================================================*/
{
  (void)param;
  for(;;)
  {
    ubxbuffer_t rx;
    ubx_receive_packet(&rx);
    switch(rx.msgclass)
    {
      case UBX_CLASS_ACK:
      {
        switch(rx.msgid)
        {
          case UBX_ID_ACK:
          {
            (void)xEventGroupSetBits(ublox_events, EVENT_ACK_RECEIVED);
            break;
          }

          case UBX_ID_NAK:
          {
            (void)xEventGroupSetBits(ublox_events, EVENT_NAK_RECEIVED);
            break;
          }

          default:
          {
            break;
          }
        }
        break;
      }

      case UBX_CLASS_NAV:
      {
        switch(rx.msgid)
        {
          case UBX_ID_NAV_SAT:
          {
            unpack_sv(rx.msg, &sat_info);
            (void)xEventGroupSetBits(ublox_events, EVENT_SAT_RECEIVED);
            break;
          }

          case UBX_ID_NAV_PVT:
          {
            unpack_pvt(rx.msg, &pvt_info);
            (void)xEventGroupSetBits(ublox_events, EVENT_PVT_RECEIVED);
            break;
          }

          default:
          {
            break;
          }
        }
        break;
      }

      case UBX_CLASS_TIM:
      {
        switch(rx.msgid)
        {
          case UBX_ID_TIM_TP:
          {
            unpack_tp(rx.msg, &qerr);
            (void)xEventGroupSetBits(ublox_events, EVENT_TP_RECEIVED);
            break;
          }

          case UBX_ID_TIM_SVIN:
          {
            unpack_svin(rx.msg, &svin_info);
            (void)xEventGroupSetBits(ublox_events, EVENT_SVIN_RECEIVED);
            break;
          }

          default:
          {
            break;
          }
        }
        break;
      }

      default:
      {
        break;
      }
    }
  }
}


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
  /* enable usart 3 and install the irq handler */
  RCC_APB1ENR |= BIT_18;
  USART3_BRR = (BAUD_FRAC(BAUD_INITIAL) << 0) | (BAUD_INT(BAUD_INITIAL) << 4);
  USART3_CR1 = BIT_13 | BIT_05 | BIT_03 | BIT_02;
  //USART3_CR1 |= BIT_05;
  vic_enableirq(IRQ_NUM, uart_irq_handler);
}


/*============================================================================*/
static void init_pins(void)
/*------------------------------------------------------------------------------
  Function:
  initialise the gpio pins for the gps module
  in:  none
  out: none
==============================================================================*/
{
  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* configure the reset pin */
  GPIOD_MODER |= (1u << 20);
  GPIOD_OTYPER |= (1u << 10);

  /* enable gpio d and configure the uart pins */
  GPIOD_MODER |= (2u << 16) | (2u << 18);
  GPIOD_AFRH |= (7u << 0) | (7u << 4);
}


/*============================================================================*/
static void gps_reset(bool do_reset)
/*------------------------------------------------------------------------------
  Function:
  resets the gps module
  in:  do_reset -> activate the reset pin if true, deactivate if false
  out: none
==============================================================================*/
{
  if(do_reset)
  {
    /* reset pin low */
    GPIOD_BSRR = BIT_26;
  }
  else
  {
    /* reset pin high */
    GPIOD_BSRR = BIT_10;
  }
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
static void ubx_receive_packet(ubxbuffer_t* rx)
/*------------------------------------------------------------------------------
  Function:
  checks whether there are messages to be received. if receive data is pending,
  a timeout is started and if the data is not complete within the timeout, the
  data is dropped.
  in:  none
  out: returns true when one complete message was received
==============================================================================*/
{
  irqstatus_t rxstatus = ubx_header1;
  uint32_t wrpos = 0;
  uint8_t cka = 0;
  uint8_t ckb = 0;
  uint32_t delay = portMAX_DELAY;

  for(;;)
  {
    uint8_t tmpdata;

    /* wait until some data is received or a timeout occurs */
    if(xStreamBufferReceive(rxstream, &tmpdata, 1, delay) == 0)
    {
      /* a timeout during reception occured, reset the internal status
         and set the delay to max to wait indefinitely long for the next char */
      rxstatus = ubx_header1;
      delay = portMAX_DELAY;
      continue;
    }

    switch(rxstatus)
    {
      /* can only proceed if the sync1 and sync2 bytes are received */
      case ubx_header1:
      {
        delay = RECEIVE_TIMEOUT;
        if(tmpdata == SYNC1)
        {
          rxstatus = ubx_header2;
        }

        continue;
      }

      case ubx_header2:
      {
        if(tmpdata == SYNC2)
        {
          cka = 0;
          ckb = 0;
          rxstatus = ubx_class;
        }
        else
        {
          rxstatus = ubx_header1;
        }

        continue;
      }

      /* receive the data in the order as described in the ublox manual */

      case ubx_class:
      {
        rx->msgclass = tmpdata;
        rxstatus = ubx_id;
        break;
      }

      case ubx_id:
      {
        rx->msgid = tmpdata;
        rxstatus = ubx_len_lsb;
        break;
      }

      case ubx_len_lsb:
      {
        rx->len = tmpdata;
        rxstatus = ubx_len_msb;
        break;
      }

      case ubx_len_msb:
      {
        /* assemble the 16 bit length (little endian) */
        uint16_t lenmsb = tmpdata;
        lenmsb <<= 8;
        rx->len += lenmsb;

        /* if the length received will not fit into the receive buffer, then abort */
        if(rx->len > MAX_UBX_LEN)
        {
          rxstatus = ubx_header1;
        }
        else
        {
          /* reset the writing position in the receive buffer */
          wrpos = 0;
          rxstatus = ubx_data;
        }
        break;
      }

      case ubx_data:
      {
        rx->msg[wrpos] = tmpdata;
        wrpos++;

        if(wrpos == rx->len)
        {
          rxstatus = ubx_cka;
        }

        break;
      }

      case ubx_cka:
      {
        /* if the first checksum byte matches, compare the second one;
           if it doesn't match, abort */
        if(tmpdata == cka)
        {
          rxstatus = ubx_ckb;
        }
        else
        {
          rxstatus = ubx_header1;
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
          return;
        }
        else
        {
          rxstatus = ubx_header1;
        }
        break;
      }
    }

    /* fletcher8 checksum */
    cka = cka + tmpdata;
    ckb = ckb + cka;
  }
}


/*============================================================================*/
static void ubx_transmit_packet(const ubxbuffer_t* tx)
/*------------------------------------------------------------------------------
  Function:
  the txhandler is the interrupt handler which is called whenever the tx data
  register is empty and thus new data can be transmitted
  in:  none
  out: none
==============================================================================*/
{
  irqstatus_t txstatus = ubx_header1;
  irqstatus_t nextstatus = ubx_header1;
  uint8_t cka = 0;
  uint8_t ckb = 0;
  uint32_t read_pos = 0;
  uint8_t tmpdata = 0;

  for(;;)
  {
    switch(txstatus)
    {
      /* send sync1 then sync2; from here on, all the data to be sent is in the tx buffer */
      case ubx_header1:
      {
        tmpdata = SYNC1;
        nextstatus = ubx_header2;
        break;
      }

      case ubx_header2:
      {
        tmpdata = SYNC2;
        nextstatus = ubx_class;
        break;
      }

      case ubx_class:
      {
        tmpdata = tx->msgclass;
        nextstatus = ubx_id;
        break;
      }

      case ubx_id:
      {
        tmpdata = tx->msgid;
        nextstatus = ubx_len_lsb;
        break;
      }

      case ubx_len_lsb:
      {
        tmpdata = (uint8_t)tx->len;
        nextstatus = ubx_len_msb;
        break;
      }

      case ubx_len_msb:
      {
        tmpdata = tx->len >> 8;
        nextstatus = ubx_data;
        break;
      }

      case ubx_data:
      {
        tmpdata = tx->msg[read_pos];
        read_pos++;
        if(read_pos == tx->len)
        {
          nextstatus = ubx_cka;
        }
        break;
      }

      case ubx_cka:
      {
        tmpdata = cka;
        nextstatus = ubx_ckb;
        break;
      }

      case ubx_ckb:
      {
        tmpdata = ckb;
        break;
      }

      default:
      {
        break;
      }
    }

    /* exclude the sync bytes and the checksum itself from the
       checksum calculation. */
    if((txstatus != ubx_cka) && (txstatus != ubx_ckb) &&
       (txstatus != ubx_header1) && (txstatus != ubx_header2))
    {
      /* fletcher8 checksum */
      cka = cka + tmpdata;
      ckb = ckb + cka;
    }

    (void)xStreamBufferSend(txstream, &tmpdata, 1, portMAX_DELAY);
    enable_txempty_irq();

    if(txstatus == ubx_ckb)
    {
      return;
    }

    txstatus = nextstatus;
  }
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
  uint32_t sr = USART3_SR;
  uint32_t cr = USART3_CR1;

  /* rx buffer not empty? */
  if(sr & BIT_05)
  {
    uint8_t tmp = (uint8_t)USART3_DR;
    (void)xStreamBufferSendFromISR(rxstream, &tmp, 1, NULL);
  }

  /* tx buffer empty AND tx buffer empty interrupt enabled? */
  if((sr & BIT_07) && (cr & BIT_07))
  {
    char tx;
    if(xStreamBufferReceiveFromISR(txstream, &tx, 1, NULL) > 0)
    {
      USART3_DR = tx;
    }
    else
    {
      disable_txempty_irq();
    }
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
  uint32_t bits = EVENT_ACK_RECEIVED | EVENT_NAK_RECEIVED;
  bits = xEventGroupWaitBits(ublox_events, bits, true, false, pdMS_TO_TICKS(ACK_TIMEOUT));
  if(bits & EVENT_ACK_RECEIVED)
  {
    return true;
  }
  else
  {
    return false;
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
  vTaskSuspendAll();
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
  info->time = get_uptime_msec();
  xTaskResumeAll();
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
  vTaskSuspendAll();
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
  info->time = get_uptime_msec();
  xTaskResumeAll();
}


/*============================================================================*/
static void unpack_tp(const uint8_t* rdata, qerr_t* ret)
/*------------------------------------------------------------------------------
  Function:
  unpacks the TIM-TP message
  in:  rdata -> raw data
       ret -> return value
  out: returns the timepulse quantisation error in the ret pointer
==============================================================================*/
{
  vTaskSuspendAll();
  int32_t tmp = unpack_i32_le(rdata, 8);
  ret->ns = ((float)tmp) / 1000.0f;
  ret->flags = unpack_u8_le(rdata, 14);
  ret->refinfo = unpack_u8_le(rdata, 15);
  ret->valid = true;
  xTaskResumeAll();
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
  vTaskSuspendAll();
  svi->numsv = unpack_u8_le(rdata, 5);
  if(svi->numsv > MAX_SV)
  {
    return;
  }
  for(uint8_t n = 0; n < svi->numsv; n++)
  {
    svi->sats[n].gnssid = unpack_u8_le(rdata, 8 + 12*n);
    svi->sats[n].svid = unpack_u8_le(rdata, 9 + 12*n);
    svi->sats[n].cno = unpack_u8_le(rdata, 10 + 12*n);
    svi->sats[n].elev = unpack_i8_le(rdata, 11 + 12*n);
    svi->sats[n].azim = unpack_i16_le(rdata, 12 + 12*n);
  }
  svi->time = get_uptime_msec();
  xTaskResumeAll();
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
  do
  {
    /* if the configuration of the baudrate fails, the module is still using
       the default (initial) baudrate */
    uart_config_baudrate(BAUD_INITIAL);

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
    ubx_transmit_packet(&tmp);
    uart_config_baudrate(baudrate);
  } while(gps_wait_ack() == false);
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
  do
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

    ubx_transmit_packet(&tmp);
  } while(gps_wait_ack() == false);
}


/*============================================================================*/
static void ubx_config_msgrate(uint8_t msgclass, uint8_t msgid, uint8_t rate)
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
  do
  {
    ubxbuffer_t tmp;
    tmp.msgclass = UBX_CLASS_CFG;
    tmp.msgid = UBX_ID_CFG_RATE;
    tmp.len = 3;
    pack_u8_le(tmp.msg, 0, msgclass);
    pack_u8_le(tmp.msg, 1, msgid);
    pack_u8_le(tmp.msg, 2, rate);
    ubx_transmit_packet(&tmp);
  } while(gps_wait_ack() == false);
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
  do
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
    pack_i8_le(tmp.msg, 12, elev); /* elevation mask */
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
    ubx_transmit_packet(&tmp);
  } while(gps_wait_ack() == false);
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
  do
  {
    ubxbuffer_t tmp;
    tmp.msgclass = UBX_CLASS_CFG;
    tmp.msgid = UBX_ID_CFG_TMODE2;
    tmp.len = 28;

    pack_u8_le(tmp.msg, 0, (uint8_t)mode);
    pack_u8_le(tmp.msg, 1, 0);
    pack_u16_le(tmp.msg, 2, 0); /* use ecef coordinates */
    pack_i32_le(tmp.msg, 4, x); /* for fixed position mode only */
    pack_i32_le(tmp.msg, 8, y);
    pack_i32_le(tmp.msg, 12, z);
    pack_u32_le(tmp.msg, 16, acc); /* fixed position accuracy */
    pack_u32_le(tmp.msg, 20, dur); /* survey-in duration */
    pack_u32_le(tmp.msg, 24, acc_lim);  /* survey-in accuracy limit */
    ubx_transmit_packet(&tmp);
  } while(gps_wait_ack() == false);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
