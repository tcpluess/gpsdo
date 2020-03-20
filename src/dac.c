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

#include "dac.h"
#include "stm32f407.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

#define DAC_SS(x) GPIOB_BSRR = ((x) != 0) ? BIT_12 : BIT_28
#define DAC_SCK(x) GPIOB_BSRR = ((x) != 0) ? BIT_13 : BIT_29
#define DAC_MOSI(x) GPIOB_BSRR = ((x) != 0) ? BIT_15 : BIT_31

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

void dac_setup(void)
{
  /* enable gpio b */
  RCC_AHB1ENR |= BIT_01;

  GPIOB_MODER |= (1u << 24) | (1u << 26) | (1u << 30);
  GPIOB_BSRR = BIT_12|BIT_13|BIT_15;
  set_dac(32768);
}

void set_dac(uint16_t data)
{
  uint32_t tmpdata = data;
  static uint16_t previousdata = 0;

  /* check if this data has already been loaded into the dac
     and bail out if so to keep the dac as quiet as possible */
  if(data == previousdata)
  {
    return;
  }
  else
  {
    previousdata = data;
  }

  /* set ss low */
  DAC_SS(0);

  for(int i = 0; i < 24; i++)
  {
    /* output MSB first */
    DAC_SCK(1);
    DAC_MOSI(tmpdata & BIT_23);
    tmpdata = tmpdata << 1;
    DAC_SCK(0);
  }

  /* set ss high */
  DAC_SS(1);
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
