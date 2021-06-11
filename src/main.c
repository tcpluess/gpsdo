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

#include "dac.h"
#include "tdc.h"
#include "timebase.h"
#include "nvic.h"
#include "adc.h"
#include "eeprom.h"
#include "console.h"
#include "cntl.h"
#include "ublox.h"

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




void init(void* param)
{
  (void)param;
  led_setup();
  eep_init();
  load_config();
  timebase_init();
  dac_setup();
  tmp_init();
  setup_tdc();
  adc_init();

  /* this is for debug only; enable the pps output such that it can be used
     as trigger signal for the scope */
  ppsenable(false);
  timebase_reset();


  (void)xTaskCreate(gps_task, "gps", 2000, NULL, 1, NULL);
  (void)xTaskCreate(cntl_task, "control", 1200, NULL, 1, NULL);
  (void)xTaskCreate(console_task, "console", 1000, NULL, 1, NULL);

  for(;;)
  {
    vTaskDelete(NULL);
  }
}

int main(void)
{
  vic_init();

  (void)xTaskCreate(init, "init", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

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


void vApplicationStackOverflowHook(TaskHandle_t task, char* taskname)
{
  (void)task;
  (void)taskname;
  /* This will get called if a stack overflow is detected during the context
     switch.  Set configCHECKFORSTACKOVERFLOWS to 2 to also check for stack
     problems within nested interrupts, but only do this for debug purposes as
     it will increase the context switch time. /
  (void)pxTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  / Write your code here … */
  asm volatile ("bkpt #0");
}

void vApplicationIdleHook(void)
{
  static int i = 0;
  i++;
}


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
