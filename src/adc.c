/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the ADC1 of the STM32F407
 *
 * Filename:       adc.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  31.03.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "semphr.h"
#include "adc.h"
#include "stm32f407xx.h"
#include "misc.h"
#include "nvic.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/* internal reference voltage, VREFINT according to datasheet 1.18V .. 1.24V */
#define VREF 1210.0f /* mV */

/* value of the ocxo shunt resistor */
#define RSHUNT 0.2f /* ohm */

/* gain of the INA193 */
#define GAIN 20.0f

/* maximum time to wait for the ADC */
#define ADC_MAX_WAITDELAY pdMS_TO_TICKS(10u)

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void adc1_interrupt(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static SemaphoreHandle_t adc_ready;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void adc_init(void)
{
  adc_ready = xSemaphoreCreateBinary();

  /* enable gpio b */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /* configure analog input for pb1 */
  GPIOB->MODER |= (3u << 2);

  /* enable the adc */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->CR2 = BIT_00;

  /* enable the internal voltage reference */
  ADC->CCR = BIT_23;

  /* scan the injected channels */
  ADC1->CR1 = BIT_08;

  /* use the slowest sampling rate for both channels */
  ADC1->SMPR1 = (7u << 21);
  ADC1->SMPR2 = (7u << 27);

  /* convert channels 9 and 17; 9 is the ocxo current, 17 is the reference */
  ADC1->JSQR = (1u << 20) | (17u << 15) | (9u << 10);

  /* timer2 trigger output starts the adc conversion */
  ADC1->CR2 |= (3u << 16) | (1u << 20);

  /* trigger an interrupt when the adc has finished the conversion */
  vic_enableirq(ADC_IRQn, adc1_interrupt); /*lint !e641 enum conversion */
  ADC1->CR1 |= BIT_07;
}


float get_iocxo(void)
{
  /* wait maximum 10ms for the ADC if, for some reason, the adc has not been
     properly triggered */
  if(xSemaphoreTake(adc_ready, ADC_MAX_WAITDELAY))
  {
    /* these are the internal reference voltage and the ocxo current */
    float ichan = ADC1->JDR1;
    float ref = ADC1->JDR2;
    return ((VREF/(GAIN*RSHUNT)) * ichan) / ref;
  }
  else
  {
    return 0.0f;
  }
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void adc1_interrupt(void)
/*------------------------------------------------------------------------------
  Function:
  interrupt is triggered when the adc has finished the conversion.
  in:  none
  out: none
==============================================================================*/
{
  /* acknowledge the interrupt and signal the semaphore to wake waiting tasks */
  uint32_t sr = ADC1->SR;
  if(sr & BIT_02)
  {
    sr &= ~BIT_02;
    ADC1->SR = sr;
    (void)xSemaphoreGiveFromISR(adc_ready, NULL);
  }
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
