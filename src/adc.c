/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    driver for the ADC1 of the STM32F407
 *
 * Compiler:       ANSI-C
 *
 * Filename:       adc.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  31.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    31.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "adc.h"
#include "stm32f407.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/* internal reference voltage, VREFINT according to datasheet 1.18V .. 1.24V */
#define VREF 1210.0f /* mV */

/* value of the ocxo shunt resistor */
#define RSHUNT 0.2f /* ohm */

/* gain of the INA193 */
#define GAIN 20.0f

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

void adc_init(void)
{
  /* enable gpio b */
  RCC_AHB1ENR |= BIT_01;

  /* configure analog input for pb1 */
  GPIOB_MODER |= (3u << 2);

  /* enable the adc */
  RCC_APB2ENR |= BIT_08;
  ADC1_CR2 = BIT_00;

  /* enable the internal voltage reference */
  ADC_CCR = BIT_23;

  /* scan the injected channels */
  ADC1_CR1 = BIT_08;

  /* use the slowest sampling rate for both channels */
  ADC1_SMPR1 = (7u << 21);
  ADC1_SMPR2 = (7u << 27);

  /* convert channels 9 and 17; 9 is the ocxo current, 17 is the reference */
  ADC1_JSQR = (1u << 20) | (17u << 15) | (9u << 10);
}

void start_conversion(void)
{
  ADC1_CR2 |= BIT_22;
}

float get_iocxo(void)
{
  do
  {
    /* check if the injected channel sequence is finished */
    uint32_t status = ADC1_SR;
    if(status & BIT_02)
    {
      break;
    }
  } while(true);

  /* these are the internal reference voltage and the ocxo current */
  float ichan = ADC1_JDR1;
  float ref = ADC1_JDR2;
  return VREF/(GAIN*RSHUNT) * ichan / ref;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
