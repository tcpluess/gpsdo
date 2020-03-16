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

#include "tdc.h"
#include "stm32f407.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define CAL_PERIODS 40u
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

#define PERIOD_PS 100000u

#define NEW_MEAS_INT (1u << 0)

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

#define TDC_CSB(x) GPIOA_BSRR = ((x) != 0) ? BIT_04 : BIT_20
#define TDC_SCK(x) GPIOA_BSRR = ((x) != 0) ? BIT_05 : BIT_21
#define TDC_MOSI(x) GPIOA_BSRR = ((x) != 0) ? BIT_07 : BIT_23
#define TDC_ENA(x) GPIOA_BSRR = ((x) != 0) ? BIT_03 : BIT_19

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static uint8_t tdc_8bit(uint16_t data);
static uint32_t tdc_24bit(uint32_t data);
static uint32_t tdc_read24(uint8_t addr);
static uint8_t tdc_read(uint8_t addr);
static void tdc_write(uint8_t addr, uint8_t data);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void setup_tdc(void)
{
  /* enable port a */
  RCC_AHB1ENR |= BIT_00;

  GPIOA_MODER |= (1u << 6) | (1u << 8) | (1u << 10) | (1u << 14);
  GPIOA_PUPDR |= (1u << 18); // pullup for TDC_IRQ

  TDC_CSB(1);
  TDC_SCK(0);
  TDC_ENA(1);

  tdc_write(ADDR_CONFIG1, CONFIG1);
  tdc_write(ADDR_CONFIG2, CONFIG2);
}

static uint8_t tdc_read(uint8_t addr)
{
  uint8_t ret;
  uint16_t tmp = addr;
  tmp <<= 8;
  ret = tdc_8bit(tmp);
  return ret;
}

static uint32_t tdc_read24(uint8_t addr)
{
  uint32_t ret;
  uint32_t tmp = addr;
  tmp <<= 24;
  ret = tdc_24bit(tmp);
  return ret;
}

static void tdc_write(uint8_t addr, uint8_t data)
{
  uint16_t tmp = BIT_06 | addr;
  tmp <<= 8;
  tmp |= data;
  tdc_8bit(tmp);
}

void enable_tdc(void)
{
  tdc_write(0, BIT_00);
}

float get_tdc(void)
{
  /* the internal calculations need to be done using 64bit numbers */
  float calib1 = tdc_read24(ADDR_CALIB1);
  float calib2 = tdc_read24(ADDR_CALIB2);
  float time1 = tdc_read24(ADDR_TIME1);

  float ps = (time1 * (CAL_PERIODS - 1u))/(calib2 - calib1);
  return ps;
}

bool tdc_check_irq(void)
{
  if((GPIOA_IDR & BIT_09) == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void tdc_int_ack(void)
{
  uint8_t irq = tdc_read(ADDR_INT_STATUS);
  tdc_write(ADDR_INT_STATUS, irq);
  do
  {
    if(GPIOA_IDR & BIT_09)
    {
      break;
    }
  } while(true);
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/


static uint8_t tdc_8bit(uint16_t data)
{
  uint8_t tmp = 0;

  TDC_CSB(0);
  for(int i = 0; i < 16; i++)
  {
    TDC_SCK(0);
    TDC_MOSI(data & BIT_15);
    tmp <<= 1;
    data <<= 1;
    if(GPIOA_IDR & BIT_06)
      tmp |= BIT_00;
    TDC_SCK(1);
  }
  TDC_CSB(1);
  return tmp;
}

static uint32_t tdc_24bit(uint32_t data)
{
  uint32_t tmp = 0;

  TDC_CSB(0);
  for(int i = 0; i < 32; i++)
  {
    TDC_SCK(0);
    TDC_MOSI(data & BIT_31);
    tmp <<= 1;
    data <<= 1;
    if(GPIOA_IDR & BIT_06)
      tmp |= BIT_00;
    TDC_SCK(1);
  }
  TDC_CSB(1);
  tmp &= 0xffffffu;
  return tmp;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
