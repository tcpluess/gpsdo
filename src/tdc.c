/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    driver for the TDC7200 time to digital converter
 *
 * Compiler:       ANSI-C
 *
 * Filename:       tdc.c
 *
 * Version:        1.2
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created

   [1.1]    01.04.2020    Tobias Plüss <tpluess@ieee.org>
   - use hardware spi interface

   [1.2]    27.05.2021    Tobias Plüss <tpluess@ieee.org>
   - use external interrupt and semaphores for more efficient access
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "tdc.h"
#include "stm32f407.h"
#include "misc.h"
#include "nvic.h"

#include <stdio.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define CAL_PERIODS 20u
#if CAL_PERIODS == 2u
#define CONFIG2 (0u << 6)
#elif CAL_PERIODS == 10u
#define CONFIG2 (1u << 6)
#elif CAL_PERIODS == 20u
#define CONFIG2 (2u << 6)
#elif CAL_PERIODS == 40u
#define CONFIG2 (3u << 6)
#else
#error "CAL_PERIODS must be one of 2, 10, 20 or 40"
#endif

#define MEASUREMENT_MODE 1u
#if MEASUREMENT_MODE == 1u
#define CONFIG1 0u
#elif MEASUREMENT_MODE == 2u
#define CONFIG1 (1u << 1)
#else
#error "MEASUREMENT_MODE must be one of 1 or 2"
#endif

#define ADDR_CONFIG1 0x00u
#define ADDR_CONFIG2 0x01u
#define ADDR_TIME1 0x10u
#define ADDR_CALIB1 0x1bu
#define ADDR_CALIB2 0x1cu
#define ADDR_INT_STATUS 0x02u
#define ADDR_INT_MASK 0x03u

#define NEW_MEAS_INT (1u << 0)

#define DATA_MSK 0xffffffu

#define TDC_SPI_DIVIDER 2u /* apb2 clock is 80 MHz, 4 amounts to divider 32 */

#define EXTI9_IRQ 23u

#define TDC_READY_TIMEOUT pdMS_TO_TICKS(10) /* timeout to wait for the tdc */

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static uint8_t tdc_read(uint8_t addr);
static uint32_t tdc_read24(uint8_t addr);
static void tdc_write(uint8_t addr, uint8_t data);
static void tdc_hwenable(bool enable);
static void tdc_ss(bool select);
static uint16_t spi_trans16(uint16_t txdata);
static void tdc_config_interrupt(void);
static void tdc_irqhandler(void);
static uint8_t tdc_irq_ack(void);
static bool tdc_waitready(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static EventGroupHandle_t tdc_events;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void setup_tdc(void)
{
  /* enable port a */
  RCC_AHB1ENR |= BIT_00;

  /* ss and enable are gpios; mosi, miso and sck belong to spi1.
     use a internal pullup for the irq pin */
  GPIOA_MODER |= (1u << 6) | (1u << 8) | (2u << 10) | (2u << 12) | (2u << 14);
  GPIOA_PUPDR |= (1u << 18);
  GPIOA_AFRL |= (5u << 20) | (5u << 24) | (5u << 28);

  tdc_events = xEventGroupCreate();
  tdc_config_interrupt();

  /* disble the tdc for now */
  tdc_ss(false);
  tdc_hwenable(false);

  /* enable spi1 */
  RCC_APB2ENR |= BIT_12;

  /* configure spi1 */
  SPI1_CR1 = BIT_14 | BIT_11 | BIT_09 | BIT_08 | (TDC_SPI_DIVIDER << 3) |  BIT_02;
  SPI1_CR2 = 0;

  /* enable spi2 */
  SPI1_CR1 |= BIT_06;

  /* this resets the internal state of the tdc */
  tdc_hwenable(true);

  /* configure the measurement mode, # of average cycles and interrupt when
     measurements are done */
  tdc_write(ADDR_CONFIG1, CONFIG1);
  tdc_write(ADDR_CONFIG2, CONFIG2);
  tdc_write(ADDR_INT_MASK, NEW_MEAS_INT);
}


void enable_tdc(void)
{
  /* sets the enable bit in the config register */
  tdc_write(ADDR_CONFIG1, BIT_00);
}


bool get_tdc(float* result)
{
  if(tdc_waitready())
  {
    float calib1 = tdc_read24(ADDR_CALIB1);
    float calib2 = tdc_read24(ADDR_CALIB2);
    float time1 = tdc_read24(ADDR_TIME1);
    enable_tdc();

    float ns = 100.0f * (time1 * (CAL_PERIODS - 1u))/(calib2 - calib1);

    /* sanity check, allow +/-10% deviation. min. 100ns, max. 200ns */
    if((ns < 90.0f) || (ns > 220.0f))
    {
      (void)printf("# error: wrong tdc value %f\n", ns);
      return false;
    }
    *result = ns;
    return true;
  }
  return false;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static uint8_t tdc_read(uint8_t addr)
/*------------------------------------------------------------------------------
  Function:
  read an 8 bit register from the tdc.
  the data format is as follows: 8 bit register address, followed by one null
  byte (to have a data transfer of 16 bits)
  in:  addr -> register address
  out: 8 bit register data
==============================================================================*/
{
  uint8_t ret;
  uint16_t tmp = addr;
  tmp <<= 8;
  tdc_ss(true);
  ret = (uint8_t)spi_trans16(tmp);
  tdc_ss(false);
  return ret;
}

/*============================================================================*/
static uint32_t tdc_read24(uint8_t addr)
/*------------------------------------------------------------------------------
  Function:
  read a 24 bit register from the tdc.
  the data format is as follows: the 8 bit register address, followed by
  three null bytes. the received data is msb first
  in:  addr -> register address
  out: 24 bit register data
==============================================================================*/
{
  uint32_t ret;
  uint16_t tmp = addr;
  tmp <<= 8;
  tdc_ss(true);
  ret = spi_trans16(tmp);
  ret <<= 16;
  ret |= spi_trans16(0);
  tdc_ss(false);
  ret &= DATA_MSK;
  return ret;
}

/*============================================================================*/
static void tdc_write(uint8_t addr, uint8_t data)
/*------------------------------------------------------------------------------
  Function:
  write to one of the 8 bit registers
  in:  addr -> register address
       data -> data to be written
  out: none
==============================================================================*/
{
  uint16_t tmp = BIT_06 | addr;
  tmp <<= 8;
  tmp |= data;
  tdc_ss(true);
  (void)spi_trans16(tmp);
  tdc_ss(false);
}

/*============================================================================*/
static void tdc_hwenable(bool enable)
/*------------------------------------------------------------------------------
  Function:
  controls the enable pin of the tdc which is used to reset internal circuitry.
  a small delay of 100ms allows internal circuitry to settle.
  in:  enabe -> when true, enables the tdc
  out: none
==============================================================================*/
{
  if(enable)
  {
    GPIOA_BSRR = BIT_03;
  }
  else
  {
    GPIOA_BSRR = BIT_19;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

/*============================================================================*/
static void tdc_ss(bool select)
/*------------------------------------------------------------------------------
  Function:
  controls the serial interface of the tdc
  in:  enabe -> enable the interface for r/w access
  out: none
==============================================================================*/
{
  /* wait until not busy */
  while(SPI1_SR & BIT_07);

  if(select)
  {
    GPIOA_BSRR = BIT_20;
  }
  else
  {
    GPIOA_BSRR = BIT_04;
  }

  /* include a small delay to meet the setup time. */
}

/*============================================================================*/
static uint16_t spi_trans16(uint16_t txdata)
/*------------------------------------------------------------------------------
  Function:
  perform a bidirectional data transfer of 16 bits
  in:  txdata -> data to be sent to the tdc
  out: returns 16 bits of data received from the tdc
==============================================================================*/
{
  /* wait until tx buffer empty */
  do
  {
    if(SPI1_SR & BIT_01)
    {
      break;
    }
  } while(true);

  SPI1_DR = txdata;

  /* wait until rx buffer not empty */
  do
  {
    if(SPI1_SR & BIT_00)
    {
      break;
    }
  } while(true);

  return (uint16_t)SPI1_DR;
}


/*============================================================================*/
static void tdc_config_interrupt(void)
/*------------------------------------------------------------------------------
  Function:
  configure the external interrupt pin. the tdc is connected to pa9, so this
  is the exti9 interrupt
  in:  none
  out: none
==============================================================================*/
{
  vic_enableirq(EXTI9_IRQ, tdc_irqhandler);

  /* configure pa9 as external interrupt */
  SYSCFG_EXTICR3 = (0u << 4);

  /* unmask pa9 interrupt */
  EXTI_IMR = BIT_09;

  /* enable falling edge trigger */
  EXTI_FTSR = BIT_09;
}


/*============================================================================*/
static void tdc_irqhandler(void)
/*------------------------------------------------------------------------------
  Function:
  interrupt handler for the INTB pin. acknowledge the external interrupt and
  release a semaphore such that a waiting task is informed.
  in:  none
  out: none
==============================================================================*/
{
  /* acknowledge the interrupt */
  EXTI_PR = BIT_09;

  /* signal to waiting tasks */
  (void)xEventGroupSetBitsFromISR(tdc_events, BIT_10, NULL);
}


/*============================================================================*/
static uint8_t tdc_irq_ack(void)
/*------------------------------------------------------------------------------
  Function:
  acknowledge the interrupt from the tdc by reading its interrupt status
  register and acknowledging all pending interrupts.
  in:  none
  out: returns the pending interrupts.
==============================================================================*/
{
  /* read the pending interrupts */
  uint8_t irq = tdc_read(ADDR_INT_STATUS);

  /* by writing a 1 to the pending interrupt bits, they are cleared */
  tdc_write(ADDR_INT_STATUS, irq);
  return irq;
}


/*============================================================================*/
static bool tdc_waitready(void)
/*------------------------------------------------------------------------------
  Function:
  wait until the tdc is ready and new measurement data is available.
  in:  none
  out: returns true if the tdc was ready, false if something went wrong
==============================================================================*/
{
  uint32_t bits = BIT_10;
  bits = xEventGroupWaitBits(tdc_events, bits, true, true, TDC_READY_TIMEOUT);
  if(bits & BIT_10)
  {
    if((tdc_irq_ack() & NEW_MEAS_INT) == 0)
    {
      (void)printf("something went wrong!\n");
      return false;
    }
  }
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
