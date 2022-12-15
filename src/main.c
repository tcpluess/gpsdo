/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    main module
 *
 * Filename:       main.c
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
#include "stm32f407xx.h"
#include "misc.h"

#include "nvic.h"
#include "eeprom.h"
#include "console.h"
#include "cntl.h"
#include "ublox.h"
#include "nmea_output.h"

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

static void led_setup(void);
static void init(void* param);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

int main(void)
{
  vic_init();

  (void)xTaskCreate(init, "init", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);

  vTaskStartScheduler();

  return 0;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void led_setup(void)
/*------------------------------------------------------------------------------
  Function:
  configure the leds ... should be possibly placed somewhere else
  in:  none
  out: none
==============================================================================*/
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  GPIOE->MODER |= (1u << 28) | (1u << 30);
  GPIOE->BSRR = BIT_30 | BIT_31;
}


/*============================================================================*/
static void init(void* param)
/*------------------------------------------------------------------------------
  Function:
  init task that initialises everything else
  in:  param -> not used
  out: none
==============================================================================*/
{
  (void)param;
  led_setup();
  eep_init();

  gnss_init();
  control_init();
  console_init();
  nmea_init();

  /* initialise the watchdog for 2 second timeout */
  DBGMCU->APB1FZ |= BIT_12; /* watchdog stopped during debug */
  IWDG->KR = 0x5555u;
  IWDG->PR = 4u;
  IWDG->KR = 0xccccu;

  /* delete the init task */
  vTaskDelete(NULL);

  for(;;);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
