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

#include "stm32f407.h"
#include "misc.h"
#include "adt7301.h"

#include "rs232.h"
#include "dac.h"
#include "tdc.h"
#include "timebase.h"
#include "vic.h"

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



int main(void)
{
  led_setup();
  timebase_init();
  vic_init();
  dac_setup();
  tmp_init();
  rs232_init();
  setup_tdc();

  set_dac(32768);

  ppsenable(true);

  while(1)
  {
    if(pps_elapsed())
    {
      timebase_reset();
      break;
    }
  }

  enable_tdc();

#define KP 2000.0f
#define KI 50.0f
#define AVG 5.0f

  float esum = 32768.0f/KI;
  float e;
  float efc;
  float efcfilt = 32768.0f;
  float tmp;
  float sollphase = -2.0f;

  while(1)
  {
    if(pps_elapsed())
    {
      while(1)
      {
        if(tdc_check_irq())
        {
          tdc_write(2, tdc_read(2));
          if(GPIOA_IDR & BIT_09)
            break;
        }
      }



      float ps = get_tdc_ps();
      get_tic();

      enable_tdc();

      int32_t cap = TIM2_CCR3;
      float phase;
      if(cap > 5000000u)
      {
        phase = 10e6f - cap;
      }
      else
      {
        phase = -cap;
      }
      phase += ps;

      e = phase - sollphase;
      tmp = esum + e;
      if((tmp < (65535.0f/KI)) && (tmp > 0))
        esum = tmp;
      efc = KI*esum + KP*e;
      efcfilt = ((AVG-1)*efcfilt + efc) / AVG;
      if(efcfilt > 65535)
        efcfilt = 65535;
      else if(efcfilt < 0)
        efcfilt = 0;
      uint16_t dacval = efcfilt;
      set_dac(dacval);

      float tmp = get_tmp();
      printf("%f %f %d %f\n", tmp, e, dacval, esum);
    }

  }
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

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
