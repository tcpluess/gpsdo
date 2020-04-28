/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    module for various timing topics for the gpsdo
 *
 * Compiler:       ANSI-C
 *
 * Filename:       timebase.c
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

/* interrupt vector numbers for the systick and the timer 2 */
#define TIM2_VECTOR 28
#define SYSTICK_VECTOR -1

//#define USE_PLL

#ifdef USE_PLL
#define HSECLK 10000000u
#define CPUCLK 160000000u
#define PLLP 0u
#define PLLQ 2u
#define PLLM 5u
#define PLLN 160u
#define PPRE2 7u
#define PPRE1 7u

#define FVCO (HSECLK/PLLM*PLLN)
#define PLL_INCLK (HSECLK/PLLM)

#if (PLL_INCLK < 1000000u) || (PLL_INCLK > 2000000u)
#error "The input clock to the PLL shall be between 1MHz and 2MHz"
#endif

#if (FVCO < 100000000u) || (FVCO > 432000000)
#error "The VCO frequency shall be between 100MHz and 432MHz"
#endif
#endif

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
static void configure_systick(void);
static void systick_handler(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static volatile bool pps;
static volatile bool res;
static volatile uint32_t tic_capture;
static volatile uint64_t uptime_msec;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void timebase_init(void)
{
  pps = false;
  res = false;
  tic_capture = 0;

  configure_systick();

  /* enable the external oscillator and the mco output and start the timer */
  enable_osc();
  enable_mco();
  configure_ppsenable();
  enable_timer();

  /* the 1pps output is disabled by default */
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
  pps = false;
  res = true;
}



float get_tic(void)
{
  /* get the interpolator value (in periods) and the capture value (in
     periods) from the tdc and the capture register, respectively */
  float tdc = get_tdc();
  uint32_t tic = tic_capture;


  float ti = tic;

  /* this brings the time interval into the range -0.5sec to +0.5sec */
  float ret;
  if(ti > 5e6f)
  {
    ret = (10e6f - ti);
  }
  else
  {
    ret = -ti;
  }


  /* to find the exact time interval, the interpolator value must be added */
  ret = ret*100 + tdc;
  return ret;
}

void set_pps_duration(uint32_t ms)
{
  /* convert milliseconds to 100ns */
  ms = ms*10000;
  TIM2_CCR2 = ms;
}

uint64_t get_uptime_msec(void)
{
  return uptime_msec;
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

#ifndef USE_PLL

  /* switch the cpu clock to the ext. osc. and wait until it is switched */
  RCC_CFGR |= BIT_00;
  do
  {
    if((RCC_CFGR & (BIT_02 | BIT_03)) == BIT_02)
    {
      break;
    }
  } while(true);

#else

  /* configure the clock dividers such that the peripheral clocks are 10MHz */
  RCC_CFGR = (PPRE2 << 13) | (PPRE1 << 10);

  /* configure the pll for 160MHz ahb clock */
  RCC_PLLCFGR = (PLLQ << 24) | BIT_22 | (PLLP << 16) | (PLLN << 6) | PLLM;

  /* enable the pll and wait until it is ready */
  RCC_CR |= BIT_24;
  do
  {
    if(RCC_CR & BIT_25)
    {
      break;
    }
  } while(true);

  /* switch to the pll clock */
  RCC_CFGR |= BIT_01;
  do
  {
    if((RCC_CFGR & (BIT_03 | BIT_02)) == BIT_03)
    {
      break;
    }
  } while(true);
#endif

  /* disable internal osc. */
  RCC_CR &= ~BIT_00;
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
  configure the enable for the pps signal
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

#ifdef USE_PLL
  /* prescaler 1 - the timer clock is divided by 2 */
  TIM2_PSC = 1u;
#else
  /* no prescaler, timer 2 runs with 10 MHz clock */
  TIM2_PSC = 0;
#endif

  /* timer wraps after 1 second */
  TIM2_ARR = 9999999u;

  /* pwm mode for the pps output, set pulse duration */
  TIM2_SMCR = 0;
  TIM2_CCER = 0;
  TIM2_CCMR1 = (6u << 12) | BIT_11;
  set_pps_duration(100u);


  /* capture mode for ch3 */
  TIM2_CCMR2 = (1u << 0);

  TIM2_CCER = (BIT_04 | BIT_08);
  TIM2_DIER = BIT_03;

  TIM2_CR1 = BIT_00;
}
extern uint32_t tim;


/*============================================================================*/
static void capture_irq(void)
/*------------------------------------------------------------------------------
  Function:
  the capture interrupt handler is called whenever there is a capture event for
  timer 2 channel 3. further, if timebase_reset() was called previously, the
  timer's counter register is set to zero
  in:  none
  out: none
==============================================================================*/
{
  /* setting the pps flag lets the main program do its job */
  pps = true;

  /* timebase reset requested? */
  if(res)
  {
    /* then this is no valid capture event */
    res = false;
    TIM2_CNT = 60;
    tic_capture = 60;
  }
  else
  {
    /* read out the captured value */
    tic_capture = TIM2_CCR3;
    tim = tic_capture;
  }

  /* acknowledge */
  TIM2_SR = 0;
}


/*============================================================================*/
static void configure_systick(void)
/*------------------------------------------------------------------------------
  Function:
  configures the cortex m systick timer for 10ms repetition frequency
  in:  none
  out: none
==============================================================================*/
{
  uptime_msec = 0;

  /* the systick timer is used to calculate the uptime in msec - the systick
     handler is called 100 times per second */
  #ifndef USE_PLL
  SYSTICKRVR = (((10000000u / 8u) / 100) - 1u);
  #else
  SYSTICKRVR = (((160000000u / 8u) / 100) - 1u);
  #endif
  SYSTICKCSR = (BIT_01 | BIT_00);
  vic_enableirq(SYSTICK_VECTOR, systick_handler);
}


/*============================================================================*/
static void systick_handler(void)
/*------------------------------------------------------------------------------
  Function:
  this is the actual systick handler, called 100 times per second, used to
  keep track of the uptime
  in:  none
  out: none
==============================================================================*/
{
  uptime_msec += 10;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
