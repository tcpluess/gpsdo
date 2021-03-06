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

#include "temperature.h"
#include "stm32f407.h"
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

static inline void delay(void)
{
  for(int i = 0; i < 1000; i++)
  {
    asm volatile ("nop");
  }
}

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void tmp_init(void)
{
  /* enable gpio port b and configure the spi pins */
  RCC_AHB1ENR |= BIT_01;

  GPIOB_MODER &= ~((3u << 13) | (3u << 12) |(3u << 10));
  GPIOB_MODER |= (1u << 10) | (1u << 14);

  /* ss and clk to inactive state */
  GPIOB_BSRR = BIT_05 | BIT_07;
}

float get_temperature(void)
{
  uint32_t ret = 0;

  /* ss active */
  GPIOB_BSRR = BIT_21;
  delay();
  for(int i = 0; i < 16; i++)
  {
    delay();
    GPIOB_BSRR = BIT_23;
    delay();

    ret = ret << 1;
    if(GPIOB_IDR & BIT_06)
    {
      ret |= BIT_00;
    }
    GPIOB_BSRR = BIT_07;
  }
  delay();
  GPIOB_BSRR = BIT_05;

  return ((float)ret)/32.0f;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
