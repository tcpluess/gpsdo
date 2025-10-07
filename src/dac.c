/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the DAC8501
 *
 * Filename:       dac.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "dac.h"
#include "stm32f407xx.h"
#include "misc.h"
#include "eeprom.h"
#include "nvic.h"

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

static void spi_ss(bool enable);
static void spi_transmit(uint8_t data);
static void send_dac(uint16_t data);

#ifdef USE_DAC_INTERPOLATION
static void interpolation_interrupt(void);
static void initialise_interpolation_timer(void);
#endif

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/* if hold is active, the dac will not be updated */
static bool hold;

/* if no dac interpolation is used, msb is simply the dac value.
   however, if dac interpolation is used, msb will hold the "integer" part
   and lsb (see below) will hold the "fractional" part. */
static uint16_t msb = 0;

#ifdef USE_DAC_INTERPOLATION
static uint8_t lsb = 0;
#endif

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void dac_setup(void)
{
  /* enable gpio b */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

  /* enable spi2 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  /* ss gpio, mosi and sck: spi2 (alt. func. 5) */
  GPIOB->MODER |= (2u << 30) | (2u << 26) | (1u << 24);
  GPIOB->AFR[1] |= (5u << 20) | (5u << 28);

  /* configure the spi2: cpol=0, cpha=1, msb first, use highest baud rate.
     the 8 bits data format must be used because the dac expects a frame of
     24 bits but the spi can only transmit 8 or 16 bits at a time. so, one
     24 bit transfer will be split up into 3 transfers of 8 bit each */
  SPI2->CR1 = BIT_14 | BIT_09 | BIT_08 | BIT_02 | BIT_00;
  SPI2->CR2 = 0;

  /* enable spi2 */
  SPI2->CR1 |= BIT_06;

  /* default state: chip select not active; set dac to initial value; dac hold
     is disabled */
  spi_ss(false);
  hold = false;
  set_dac(get_config()->last_dacval, true);

#ifdef USE_DAC_INTERPOLATION
  /* if dac interpolation is used, an additional timer for
     256x oversampling is used */
  initialise_interpolation_timer();
#endif
}


void set_dac(float dacval, bool force)
{
  static float last_dacval = 0.0f;

  /* only actually do something if the hold mode is not active and the
     new value is different from the old one. */
  if(((hold == false) && (dacval != last_dacval)) || (force == true))
  {
    last_dacval = dacval;

#ifdef USE_DAC_INTERPOLATION
    /* extract the integer and fractional part into msb and lsb, respectively */
    msb = (uint16_t)dacval;
    lsb = (uint16_t)((dacval - msb) * 256.0f);
#else
    /* if no dac interpolation is used, the dacval is rounded and then saved. */
    msb = (uint16_t)(dacval + 0.5f);

    /* also if no dac interpolation is used, we can directly send the value to
       the dac as no interpolation timer is involved. */
    send_dac(msb);
#endif

    extern volatile float stat_dac;
    stat_dac = dacval;
  }
}


void dac_sethold(bool on)
{
  hold = on;
}


bool dac_gethold(void)
{
  return hold;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void spi_ss(bool enable)
/*------------------------------------------------------------------------------
  Function:
  asserts or deasserts the slave select pin
  in:  enable -> assert ss for the dac if true, and deassert otherwise.
  out: none
==============================================================================*/
{
  /* wait until not busy */
  while(SPI2->SR & BIT_07);

  if(enable)
  {
    GPIOB->BSRR = BIT_28;
  }
  else
  {
    GPIOB->BSRR = BIT_12;
  }
}


/*============================================================================*/
static void spi_transmit(uint8_t data)
/*------------------------------------------------------------------------------
  Function:
  waits unti the tx buffer is empty, then transmits one data byte
  in:  data -> 1 data byte
  out: none
==============================================================================*/
{
  do
  {
    if(SPI2->SR & BIT_01)
    {
      break;
    }
  } while(true);

  SPI2->DR = data;
}


#ifdef USE_DAC_INTERPOLATION
/*============================================================================*/
static void initialise_interpolation_timer(void)
/*------------------------------------------------------------------------------
  Function:
  this initialises a timer that has 200Hz interrupt rate and is then used to
  interpolate the dac output to artificially create additional bits. see the
  timer interrupt for how the interpolation works.
  in:  none
  out: none
==============================================================================*/
{
  /* configure the interrupt vector */
  vic_enableirq(TIM3_IRQn, interpolation_interrupt);

  /* enable timer 3, which runs with 80MHz clock speed */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  /* the timer runs at 1MHz: clock = 80MHz, prescaler is 79 and then the actual
     timer clock is 80MHz / (79 + 1). */
  TIM3->PSC = 79;

  /* timer wraps every 3.906ms = 1s / 256 */
  TIM3->ARR = 3905;
  TIM3->SMCR = 0;
  TIM3->CCER = 0;
  TIM3->DIER = TIM_DIER_UIE;
  TIM3->CR1 = TIM_CR1_CEN;
}


/*============================================================================*/
static void interpolation_interrupt(void)
/*------------------------------------------------------------------------------
  Function:
  this is the interrupt of the interpolation timer. should be called 256 times
  per second. the interpolation works like pulse density modulation. an
  accumulator is increased and when it overflows, the DAC is set one bit higher.
  in:  none
  out: none
==============================================================================*/
{
#ifdef _DEBUG
  static uint32_t intcount = 0;
  intcount++;
#endif

  /* interrupt acknowledge */
  TIM3->SR = 0;

  static uint32_t accu = 0;

  /* increment the accumulator and check if it overflows. */
  accu += lsb;
  if(accu >= 255)
  {
    accu = accu - 256;
    send_dac(msb + 1);
  }
  else
  {
    send_dac(msb);
  }
}
#endif


/*============================================================================*/
static void send_dac(uint16_t data)
/*------------------------------------------------------------------------------
  Function:
  actually send data to the dac.
  in:  data -> 16 bits of data to be loaded into the dac.
  out: none
==============================================================================*/
{
  /* slave select */
  spi_ss(true);

  /* first byte is always zero, then upper byte, then lower byte */
  spi_transmit(0u);
  spi_transmit(data >> 8);
  spi_transmit((uint8_t)data);

  /* unselect */
  spi_ss(false);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
