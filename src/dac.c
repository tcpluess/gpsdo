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

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static bool hold;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void dac_setup(void)
{
  hold = false;

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
  set_dac(get_config()->last_dacval);
}


void set_dac(uint16_t data)
{
  if(hold == false)
  {
    /* check if this data has already been loaded into the dac
       and bail out if so to keep the dac as quiet as possible;
       also bail out if dac hold is enabled */
    static uint16_t previousdata = 0u;
    if(data == previousdata)
    {
      return;
    }
    else
    {
      previousdata = data;
    }

    /* slave select */
    spi_ss(true);

    /* first byte is always zero, then upper byte, then lower byte */
    spi_transmit(0u);
    spi_transmit(data >> 8);
    spi_transmit((uint8_t)data);

    /* unselect */
    spi_ss(false);

    extern volatile uint16_t stat_dac;
    stat_dac = data;
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

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
