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

#include "timebase.h"
#include "stm32f407.h"
#include "misc.h"
#include "vic.h"
#include "tdc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define TIM2_VECTOR 28u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void enable_osc(void);
static void enable_mco(void);
static void configure_ppsenable(void);
static void enable_timer(void);
static void capture_irq(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static volatile bool pps;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void timebase_init(void)
{
  pps = false;
  enable_osc();
  enable_mco();
  configure_ppsenable();
  enable_timer();
  ppsenable(false);
}

bool pps_elapsed(void)
{
  if(pps)
  {
    pps = false;
    return true;
  }
  else
  {
    return false;
  }
}

/*============================================================================*/
void ppsenable(bool enable)
/*------------------------------------------------------------------------------
  Function:
  configure the enable signal for the pps signal
  in:  none
  out: none
==============================================================================*/
{
  if(enable)
  {
    GPIOB_BSRR = BIT_25;
  }
  else
  {
    GPIOB_BSRR = BIT_09;
  }
}

void timebase_reset(void)
{
  TIM2_CNT = 0;
}

int64_t get_tic(void)
{
  /* get the interpolator value (in picoseconds) and the capture value (in
     periods) from the tdc and the capture register, respectively */
  uint32_t tic_ps = get_tdc();
  uint32_t tic = TIM2_CCR3;

  /* convert the # of periods to picoseconds */
  uint64_t ti = tic;
  ti = ti * 100000;

  /* to find the exact time interval, the interpolator value must be added */
  ti = ti + tic_ps;

  /* this brings the time interval into the range -0.5sec to +0.5sec */
  if(ti >= 500000000000)
  {
    return 1000000000000 - ti;
  }
  else
  {
    return -ti;
  }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void enable_osc(void)
/*------------------------------------------------------------------------------
  Function:
  enable the external oscillator input
  in:  none
  out: none
==============================================================================*/
{
  /* enable the external oscillator and wait until it is ready */
  RCC_CR |= BIT_16;
  do
  {
    if(RCC_CR & BIT_17)
    {
      break;
    }
  } while(true);

  /* switch the cpu clock to the ext. osc. and wait until it is switched */
  RCC_CFGR |= BIT_00;
  do
  {
    if((RCC_CFGR & (BIT_02 | BIT_03)) == BIT_02)
    {
      break;
    }
  } while(true);

  /* disable internal osc. */
  RCC_CR &= ~BIT_00;
  do
  {
    if((RCC_CR & BIT_01) == 0)
    {
      break;
    }
  } while(true);
}

/*============================================================================*/
static void enable_mco(void)
/*------------------------------------------------------------------------------
  Function:
  enable the mco1 output
  in:  none
  out: none
==============================================================================*/
{
  /* configure the mco1 to output hse clock */
  RCC_CFGR |= BIT_22;

  /* enable gpio port a */
  RCC_AHB1ENR |= BIT_00;
  GPIOA_MODER |= (2u << 16);
}

/*============================================================================*/
static void configure_ppsenable(void)
/*------------------------------------------------------------------------------
  Function:
  configure the enable signal for the pps signal
  in:  none
  out: none
==============================================================================*/
{
  /* enable gpio b */
  RCC_AHB1ENR |= BIT_01;

  /* set ppsen as output and set it to high */
  GPIOB_MODER |= (1u << 18);
  ppsenable(false);
}





/*============================================================================*/
static void enable_timer(void)
/*------------------------------------------------------------------------------
  Function:
  enables the timer 2 which is used as time base
  in:  none
  out: none
==============================================================================*/
{
  vic_enableirq(TIM2_VECTOR, capture_irq);

  /* enable gpio a */
  RCC_AHB1ENR |= BIT_00;
  GPIOA_MODER |= (2u << 2) | (2u << 4);
  GPIOA_AFRL |= (1u << 4) | (1u << 8);

  /* enable timer */
  RCC_APB1ENR |= BIT_00;

  /* no prescaler, timer 2 runs with 10 MHz clock */
  TIM2_PSC = 0;

  /* timer wraps after 1 second */
  TIM2_ARR = 9999999u;

  /* pwm mode for the pps output */
  TIM2_SMCR = 0;
  TIM2_CCER = 0;
  TIM2_CCMR1 = (6u << 12) | BIT_11;
//  TIM2_CCER = BIT_04;
  TIM2_CCR2 = TIM2_ARR/4;


  /* capture mode for ch3 */
  TIM2_CCMR2 = (1u << 0);

  TIM2_CCER = (BIT_04 | BIT_08);
  TIM2_DIER = BIT_03;

  TIM2_CR1 = BIT_00;
}

static void capture_irq(void)
{
  static bool stat = false;
  if(stat)
    GPIOE_BSRR = BIT_15;
  else
    GPIOE_BSRR = BIT_31;
  stat = !stat;
  pps = true;

  TIM2_SR = 0;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
