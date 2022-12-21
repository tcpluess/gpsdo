/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    module for various timing topics for the gpsdo
 *
 * Filename:       timebase.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "event_groups.h"
#include "timebase.h"
#include "stm32f407xx.h"
#include "misc.h"
#include "nvic.h"
#include "tdc.h"
#include "ublox.h"
#include "eeprom.h"
#include "datetime.h"

#include <stdio.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define HSECLK 10000000u
#define PLLN 160u /* vco runs at 320 MHz */
#define PLLM 5u
#define PLLQ 2u
#define PLLP 0u /* vco frequency divided by 2 -> 160 MHz pll output */
#define HPRE 0u /* ahb clock = pll clock */

/* apb2: high speed */
#define PPRE2 4u /* apb2 clock divided by 2 -> apb2 runs at 80 MHz */

/* apb1: low speed */
#define PPRE1 5u /* apb1 clock divided by 4 -> apb1 runs at 40 MHz */

#define FVCO ((HSECLK/PLLM)*PLLN)
#define PLL_INCLK (HSECLK/PLLM)
#define PLLCLK (FVCO/((PLLP + 1u) * 2u))

#if (PLLCLK > 168000000)
#error "The PLL output clock is too high"
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

#if HPRE < 8
#define AHBCLK PLLCLK
#elif HPRE == 8
#define AHBCLK (PLLCLK/2)
#elif HPRE == 9
#define AHBCLK (PLLCLK/4)
#elif HPRE == 10
#define AHBCLK (PLLCLK/8)
#elif HPRE == 11
#define AHBCLK (PLLCLK/16)
#elif HPRE == 12
#define AHBCLK (PLLCLK/64)
#elif HPRE == 13
#define AHBCLK (PLLCLK/128)
#elif HPRE == 14
#define AHBCLK (PLLCLK/256)
#elif HPRE == 15
#define AHBCLK (PLLCLK/512)
#else
#error "Wrong HPRE value"
#endif

#if PPRE1 < 4
#define APB1CLK AHBCLK
#define APB1TIMCLK APB1CLK
#elif PPRE1 < 8
#define APB1CLK (AHBCLK / (1u << (PPRE1 - 3u)))
#define APB1TIMCLK (2u * APB1CLK)
#else
#error "Wrong value for PPRE1"
#endif

#if PPRE2 < 4
#define APB2CLK AHBCLK
#define APB2TIMCLK APB1CLK
#elif PPRE2 < 8
#define APB2CLK (AHBCLK / (1u << (PPRE2 - 3u)))
#define APB2TIMCLK (2u * APB2CLK)
#else
#error "Wrong value for PPRE2"
#endif

#define TIMER_FREQ 10000000u /* the timer shall run at 10MHz */

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
static void datetime_task(void* arg);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static volatile bool res;
static volatile uint32_t tic_capture;
static volatile uint64_t uptime_msec;
static EventGroupHandle_t timepulse_event;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void timebase_init(void)
{
  res = false;
  tic_capture = 0;
  uptime_msec = 0ull;

  timepulse_event = xEventGroupCreate();

  /* enable the external oscillator and the mco output and start the timer */
  enable_osc();
  enable_mco();
  configure_ppsenable();
  enable_timer();

  (void)xTaskCreate(datetime_task,
                    "datetime",
                    DATE_STACK,
                    NULL,
                    DATE_PRIO,
                    NULL);

  /* enable the interpolator */
  tdc_setup();

  /* the 1pps output is disabled by default */
  ppsenable(false);
}

bool pps_elapsed(void)
{
  if(xEventGroupWaitBits(timepulse_event, BIT_00, true, true, pdMS_TO_TICKS(1200)))
  {
    return true;
  }
  else
  {
#ifdef DEBUG
    (void)printf("# no 1PPS!\n");
#endif
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
    /* enable signal */
    GPIOB->BSRR = BIT_25;

    /* lock led */
    GPIOE->BSRR = BIT_14;
  }
  else
  {
    /* enable signal */
    GPIOB->BSRR = BIT_09;

    /* lock led */
    GPIOE->BSRR = BIT_30;
  }
}

void timebase_reset(void)
{
  res = true;
}


float get_tic(void)
{
  /* get the the capture value (in periods) from the and the capture register */
  float ti = (float)tic_capture;
  tic_capture = 0;

  /* this brings the time interval into the range -0.5sec to +0.5sec */
  float ret;
  if(ti > 5e6f)
  {
    ret = (ti - 10e6f);
  }
  else
  {
    ret = ti;
  }

  /* to find the exact time interval, the interpolator value must be added */
  ret = ret*100.0f;
  return ret;
}

void set_pps_duration(uint32_t ms)
{
  /* convert milliseconds to 100ns */
  ms = ms*10000;
  TIM2->CCR2 = ms;
}

uint64_t get_uptime_msec(void)
{
  uint64_t ret;
  vPortEnterCritical();
  ret = uptime_msec;
  vPortExitCritical();
  return ret;
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
  RCC->CR |= RCC_CR_HSEON;
  do
  {
    if(RCC->CR & RCC_CR_HSERDY)
    {
      break;
    }
  } while(true);

  /* enable the i and d caches, prefetch and use high latency for the flash */
  FLASH->ACR = BIT_10 | BIT_09 | BIT_08 | 7u;

  /* configure the clock dividers */
  RCC->CFGR = (PPRE2 << RCC_CFGR_PPRE2_Pos) |
              (PPRE1 << RCC_CFGR_PPRE1_Pos) |
              (HPRE << RCC_CFGR_HPRE_Pos);

  /* configure the pll for 160MHz ahb clock */
  RCC->PLLCFGR = (PLLQ << RCC_PLLCFGR_PLLQ_Pos) |
                 (PLLP << RCC_PLLCFGR_PLLP_Pos) |
                 (PLLN << RCC_PLLCFGR_PLLN_Pos) | PLLM | RCC_PLLCFGR_PLLSRC;

  /* enable the pll and wait until it is ready */
  RCC->CR |= RCC_CR_PLLON;
  do
  {
    if(RCC->CR & RCC_CR_PLLRDY)
    {
      break;
    }
  } while(true);

  /* switch to the pll clock */
  RCC->CFGR |= BIT_01;
  do
  {
    if((RCC->CFGR & (BIT_03 | BIT_02)) == BIT_03)
    {
      break;
    }
  } while(true);

  /* disable internal osc. */
  RCC->CR &= ~RCC_CR_HSION;
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
  RCC->CFGR |= BIT_22;

  /* enable gpio port a */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (2u << 16);
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
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /* set ppsen as output and set it to high */
  GPIOB->MODER |= (1u << 18);
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
  vic_enableirq(TIM2_IRQn, capture_irq); /*lint !e641 enum conversion */

  /* enable gpio a */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (2u << 2) | (2u << 4);
  GPIOA->AFR[0] |= (1u << 4) | (1u << 8);

  /* enable timer */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* prescaler - the timer runs at twice the apb1 frequency, i.e. at 80 MHz.
     a prescaler of N divides by N+1. the timer shall run at 10 MHz */
  TIM2->PSC = ((APB1TIMCLK / TIMER_FREQ) - 1u);

  /* timer wraps after 1 second */
  TIM2->ARR = TIMER_FREQ - 1u;

  /* pwm mode for the pps output (channel 2), set pulse duration */
  TIM2->SMCR = 0;
  TIM2->CCER = 0;
  TIM2->CCMR1 = (6u << 12) | BIT_11;
  set_pps_duration(get_config()->pps_dur);

  /* the 1pps output is also used to trigger the adc */
  TIM2->CR2 = (5u << 4);

  /* capture mode for ch3 (this is the 1pps input from gps) */
  TIM2->CCMR2 = (1u << 0);

  /* enable capture/compare channels 2 and 3 */
  TIM2->CCER = (BIT_08 | BIT_04);
  TIM2->DIER = BIT_03;

  TIM2->CR1 = BIT_00;
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
    TIM2->CNT = 0;
    res = false;
  }
  else
  {
    /* read out the captured value and notify waiting tasks */
    tic_capture = TIM2->CCR3;
    (void)xEventGroupSetBitsFromISR(timepulse_event, BIT_00, NULL);
  }

  /* acknowledge */
  TIM2->SR = 0;
}


/*============================================================================*/
static void datetime_task(void* arg)
/*------------------------------------------------------------------------------
  Function:
  this task handles the date and time. if the gnss module has no signal,
  this task makes sure the date and time are counted.
  as soon as the gnss module receives date and time they will be correctly set.
  in:  arg -> not used
  out: none
==============================================================================*/
{
  (void)arg;
  datetime_init();
  for(;;)
  {
    datetime_tick();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vApplicationTickHook(void)
{
  uptime_msec += 1;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
