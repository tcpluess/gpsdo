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

#define HSECLK 10000000u
#define PLLN 160u /* vco runs at 320 MHz */
#define PLLM 5u
#define PLLQ 2u
#define PLLP 0u /* vco frequency divided by 2 -> 160 MHz pll output */
#define HPRE 0u /* ahb clock = pll clock */
#define PPRE2 4u /* apb2 clock divided by 4 -> apb2 runs at 80 MHz */
#define PPRE1 5u /* apb1 clock divided by 4 -> apb1 runs at 40 MHz */

#define FVCO (HSECLK/PLLM*PLLN)
#define PLL_INCLK (HSECLK/PLLM)
#define CPUCLK (FVCO/((PLLP + 1u) * 2u))

#if (CPUCLK > 168000000)
#error "The CPU clock is too high"
#endif

#if (PLL_INCLK < 1000000u) || (PLL_INCLK > 2000000u)
#error "The input clock to the PLL shall be between 1MHz and 2MHz"
#endif

#if (PLLN < 50u) || (PLLN > 432u)
#error "wrong PLLN value"
#endif

#if (FVCO < 100000000u) || (FVCO > 432000000)
#error "The VCO frequency shall be between 100MHz and 432MHz"
#endif

#define TIMER_DIVISION 8u /* 40 MHz divided by 4 -> 10 MHz */

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
  ret = ret*100.0f + tdc;
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

  /* enable the i and d caches, prefetch and use high latency for the flash */
  FLASH_ACR = BIT_10 | BIT_09 | BIT_08 | 7u;

  /* configure the clock dividers */
  RCC_CFGR = (PPRE2 << 13) | (PPRE1 << 10) | (HPRE << 4);

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

  /* prescaler - the timer runs at twice the apb1 frequency, i.e. at 40 MHz.
     a prescaler of N divides by N+1. the timer shall run at 10 MHz */
  TIM2_PSC = (TIMER_DIVISION - 1u);

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
  if(res)
  {
    /* timebase reset requested */
    TIM2_CNT = 7; // TODO: this is somewhat hackish. better ideas??
    res = false;
  }
  else
  {
    /* setting the pps flag lets the main program do its job */
    pps = true;

    /* read out the captured value */
    tic_capture = TIM2_CCR3;
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
  SYSTICKRVR = (((CPUCLK / 8u) / 100) - 1u);
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
