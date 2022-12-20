/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the TDC7200 time to digital converter
 *
 * Filename:       tdc.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "tdc.h"
#include "stm32f407xx.h"
#include "misc.h"
#include "nvic.h"

#ifdef DEBUG
#include <stdio.h>
#endif

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define CAL_PERIODS 40ul
#if CAL_PERIODS == 2ul
#define CONFIG2 (0u << 6)
#elif CAL_PERIODS == 10ul
#define CONFIG2 (1u << 6)
#elif CAL_PERIODS == 20ul
#define CONFIG2 (2u << 6)
#elif CAL_PERIODS == 40ul
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

/* apb2 clock is 80 MHz, 1 amounts to divider 4, TDC7200 max. clk is 20 MHz */
#define TDC_SPI_DIVIDER 1u

/* timeout to wait for the tdc */
#define TDC_READY_TIMEOUT pdMS_TO_TICKS(10u) /* ms */

/* reset default values for config2 and int mask */
#define DEFAULT_CONFIG2 0x40u
#define DEFAULT_INT_MASK 0x07u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void enable_tdc(void);
static void init_hardware(void);
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

static SemaphoreHandle_t tdc_sem;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void tdc_setup(void)
{
  init_hardware();

  tdc_sem = xSemaphoreCreateBinary();

  /* disble the tdc for now */
  tdc_ss(false);
  tdc_hwenable(false);

  /* this resets the internal state of the tdc */
  tdc_hwenable(true);

  /* sanity check if the communication works. */
  uint8_t config2 = tdc_read(ADDR_CONFIG2);
  uint8_t intmask = tdc_read(ADDR_INT_MASK);
  if((config2 == DEFAULT_CONFIG2) && (intmask == DEFAULT_INT_MASK))
  {
    /* configure the measurement mode, # of average cycles and interrupt when
       measurements are done */
    tdc_write(ADDR_CONFIG1, CONFIG1);
    tdc_write(ADDR_CONFIG2, CONFIG2);
    tdc_write(ADDR_INT_MASK, NEW_MEAS_INT);
    enable_tdc();
  }
  else
  {
    #ifdef DEBUG
    (void)printf("something went wrong: tdc not in its default state\n");
    (void)printf("CONFIG2 = %x, INTMASK = %x\n", config2, intmask);
    #endif
  }
}


bool tdc_readtic(float* result)
{
  if(tdc_waitready())
  {
    /* read the measurement data, then prepare for the next measurement
       by setting enable */
    uint32_t calib1 = tdc_read24(ADDR_CALIB1);
    uint32_t calib2 = tdc_read24(ADDR_CALIB2);
    uint32_t time1 = tdc_read24(ADDR_TIME1);
    enable_tdc();

    /*                                  CAL_PERIODS - 1
       the result is: (clock period) *  --------------- * TIME1
                                        calib2 - calib1         */
    uint32_t num = 100 * (time1 * (CAL_PERIODS - 1ul));
    uint32_t den = (calib2 - calib1);
    float ns = ((float)num)/((float)den);

    /* sanity check, allow +/-10% deviation. min. 90ns, max. 220ns */
    if((ns < 90.0f) || (ns > 220.0f))
    {
#ifdef DEBUG
      (void)printf("# error: wrong tdc value %f\n", ns);
#endif
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
static void enable_tdc(void)
/*------------------------------------------------------------------------------
  Function:
  enable the tdc to measure
  in:  none
  out: none
==============================================================================*/
{
  /* sets the enable bit in the config register */
  tdc_write(ADDR_CONFIG1, BIT_00);
}

/*============================================================================*/
static void init_hardware(void)
/*------------------------------------------------------------------------------
  Function:
  initialise the hardware related to the tdc
  in:  none
  out: none
==============================================================================*/
{
  /* enable port a */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* ss and enable are gpios; mosi, miso and sck belong to spi1.
     use a internal pullup for the irq pin */
  GPIOA->MODER |= (1u << 6) | (1u << 8) | (2u << 10) | (2u << 12) | (2u << 14);
  GPIOA->PUPDR |= (1u << 18);
  GPIOA->AFR[0] |= (5u << 20) | (5u << 24) | (5u << 28);

  /* enable spi1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* configure spi1 */
  SPI1->CR1 = BIT_14 | BIT_11 | BIT_09 | BIT_08 | (TDC_SPI_DIVIDER << 3) |  BIT_02;
  SPI1->CR2 = 0;

  /* enable spi2 */
  SPI1->CR1 |= BIT_06;

  /* also configure interrupts */
  tdc_config_interrupt();
}

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
    GPIOA->BSRR = BIT_03;
  }
  else
  {
    GPIOA->BSRR = BIT_19;
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
  while(SPI1->SR & BIT_07);

  if(select)
  {
    GPIOA->BSRR = BIT_20;
  }
  else
  {
    GPIOA->BSRR = BIT_04;
  }
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
    if(SPI1->SR & BIT_01)
    {
      break;
    }
  } while(true);

  SPI1->DR = txdata;

  /* wait until rx buffer not empty */
  do
  {
    if(SPI1->SR & BIT_00)
    {
      break;
    }
  } while(true);

  return (uint16_t)SPI1->DR;
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
  vic_enableirq(EXTI9_5_IRQn, tdc_irqhandler); /*lint !e641 enum conversion */

  /* configure pa9 as external interrupt */
  SYSCFG->EXTICR[2] = (0u << 4);

  /* unmask pa9 interrupt */
  EXTI->IMR = BIT_09;

  /* enable falling edge trigger */
  EXTI->FTSR = BIT_09;
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
  EXTI->PR = BIT_09;

  /* signal to waiting tasks */
  (void)xSemaphoreGiveFromISR(tdc_sem, NULL);
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
  if(xSemaphoreTake(tdc_sem, TDC_READY_TIMEOUT))
  {
    if((tdc_irq_ack() & NEW_MEAS_INT) == 0)
    {
#ifdef DEBUG
      (void)printf("something went wrong!\n");
#endif
      return false;
    }
  }
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
