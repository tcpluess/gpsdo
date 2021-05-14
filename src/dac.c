/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    driver for the DAC8501
 *
 * Compiler:       ANSI-C
 *
 * Filename:       dac.c
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

#include "dac.h"
#include "stm32f407.h"
#include "misc.h"
#include "eeprom.h"

#include <stdbool.h>

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

/* not static because from eeprom */
extern config_t cfg;

bool dac_hold;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void dac_setup(void)
{
  /* enable gpio b */
  RCC_AHB1ENR |= BIT_01;

  /* enable spi2 */
  RCC_APB1ENR |= BIT_14;

  /* ss gpio, mosi and sck: spi2 (alt. func. 5) */
  GPIOB_MODER |= (2u << 30) | (2u << 26) | (1u << 24);
  GPIOB_AFRH |= (5u << 20) | (5u << 28);

  /* configure the spi2: cpol=0, cpha=1, msb first, use highest baud rate.
     the 8 bits data format must be used because the dac expects a frame of
     24 bits but the spi can only transmit 8 or 16 bits at a time. so, one
     24 bit transfer will be split up into 3 transfers of 8 bit each */
  SPI2_CR1 = BIT_14 | BIT_09 | BIT_08 | BIT_02 | BIT_00;
  SPI2_CR2 = 0;

  /* enable spi2 */
  SPI2_CR1 |= BIT_06;

  /* default state: chip select not active; set dac to initial value; dac hold
     is disabled */
  dac_hold = false;
  spi_ss(false);
  set_dac(cfg.last_dacval);
}

void set_dac(uint16_t data)
{
  /* check if this data has already been loaded into the dac
     and bail out if so to keep the dac as quiet as possible;
     also bail out if dac hold is enabled */
  static uint16_t previousdata = 0u;
  if((data == previousdata) || (dac_hold == true))
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
  while(SPI2_SR & BIT_07);

  if(enable)
  {
    GPIOB_BSRR = BIT_28;
  }
  else
  {
    GPIOB_BSRR = BIT_12;
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
    if(SPI2_SR & BIT_01)
    {
      break;
    }
  } while(true);

  SPI2_DR = data;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
