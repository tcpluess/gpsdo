/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    adt7301 driver
 *
 * Filename:       adt7301.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "temperature.h"
#include "stm32f407xx.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void tmp_init(void)
{
  /* enable gpio port b and configure the spi pins */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER &= ~((3u << 13) | (3u << 12) |(3u << 10));
  GPIOB->MODER |= (1u << 10) | (1u << 14);

  /* ss and clk to inactive state (high) */
  GPIOB->BSRR = BIT_05 | BIT_07;
}

float get_temperature(void)
{
  uint32_t bits = 0;
  float ret;

  /* ss active */
  GPIOB->BSRR = BIT_21;
  for(int i = 0; i < 16; i++)
  {
    /* sclk low */
    GPIOB->BSRR = BIT_23;
    asm volatile ("nop");
    asm volatile ("dsb");
    asm volatile ("isb");

    /* data is clocked in on the falling edge of sclk */
    bits = bits << 1;
    if(GPIOB->IDR & BIT_06)
    {
      bits |= BIT_00;
    }

    /* sclk high */
    GPIOB->BSRR = BIT_07;
    asm volatile ("nop");
    asm volatile ("dsb");
    asm volatile ("isb");
  }

  /* ss inactive */
  GPIOB->BSRR = BIT_05;

  /* convert digital code to temperature. */
  ret = (float)bits;
  ret /= 32.0f;

  return ret;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
