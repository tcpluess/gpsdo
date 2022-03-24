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
#include "stm32f407xx.h"
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
#include "nmea_output.h"

#include <stdio.h>

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

  (void)xTaskCreate(init, "init", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

  vTaskStartScheduler();
  /*lint -unreachable */
  return 0;
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
  (void)printf("# stack overflow!\n");
#ifndef DEBUG
  for(;;);
#else
  asm volatile ("bkpt #0");
#endif
}


void vApplicationIdleHook(void)
{
  static int i = 0;
  i++;

  /* this services the watchdog. if the code hangs for some reason the
     watchdog will time out after approx. 2 sec. */
  IWDG->KR = 0xaaaau;
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
  RCC->AHB1ENR |= BIT_04;
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
  load_config();
  timebase_init();
  dac_setup();
  tmp_init();
  setup_tdc();
  adc_init();
  ppsenable(false);

  (void)xTaskCreate(gps_task, "gps", 2500, NULL, 1, NULL);
  (void)xTaskCreate(cntl_task, "control", 1500, NULL, 1, NULL);
  (void)xTaskCreate(console_task, "console", 1500, NULL, 1, NULL);
  (void)xTaskCreate(nmea_task, "nmea output", 1500, NULL, 1, NULL);

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
