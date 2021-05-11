/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    main module
 *
 * Compiler:       ANSI-C
 *
 * Filename:       main.c
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

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f407.h"
#include "misc.h"
#include "temperature.h"

#include "rs232.h"
#include "dac.h"
#include "tdc.h"
#include "timebase.h"
#include "vic.h"
#include "adc.h"
#include "ublox.h"
#include "eeprom.h"
#include "console.h"
#include "cntl.h"

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

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void mainfunction(void * pvParameters)
{
  led_setup();
  eep_init();
  load_config();
  timebase_init();
  ublox_init();
  rs232_init();
  dac_setup();
  tmp_init();
  setup_tdc();
  adc_init();

  /* this is for debug only; enable the pps output such that it can be used
     as trigger signal for the scope */
  ppsenable(true);
  timebase_reset();
  enable_tdc();

  for(;;)
  {
    gps_worker();
    cntl_worker();
    console_worker();
    vTaskDelay(1);
  }
}

int main(void)
{
  vic_init();

  xTaskCreate(mainfunction, "task", 500, NULL, 2, NULL);

  vTaskStartScheduler();
  /*lint -unreachable */
  return 0;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

static void led_setup(void)
{
  RCC_AHB1ENR |= BIT_04;
  GPIOE_MODER |= (1u << 28) | (1u << 30);
  GPIOE_BSRR = BIT_30 | BIT_31;
}


void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                               char * pcTaskName )
{
  /* This will get called if a stack overflow is detected during the context
     switch.  Set configCHECKFORSTACKOVERFLOWS to 2 to also check for stack
     problems within nested interrupts, but only do this for debug purposes as
     it will increase the context switch time. /
  (void)pxTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  / Write your code here … */
  for(;;) {}
}

void vApplicationIdleHook(void)
{
  static int i = 0;
  i++;
}


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
