/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    serial port driver for usage with printf() calls
 *
 * Compiler:       ANSI-C
 *
 * Filename:       rs232.c
 *
 * Version:        1.1
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created

   [1.1]    31.03.2020    Tobias Plüss <tpluess@ieee.org>
   - modify the transmit routine to use the tx empty interrupt
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "rs232.h"
#include "stm32f407.h"
#include "misc.h"
#include "vic.h"
#include "eeprom.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define TXBUFFERSIZE 512
#define BAUD_INT(x) ((uint32_t)(10000000.0/(16.0*(x))))
#define BAUD_FRAC(x) ((uint32_t)((10000000.0/(x)-16.0*BAUD_INT(x))+0.5))

#define RXBUFFERSIZE 10u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void irq_handler(void);
static void enable_txempty_irq(void);
static void disable_txempty_irq(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static char txbuffer[TXBUFFERSIZE];
static volatile uint32_t txlen;

static char rxbuffer[RXBUFFERSIZE];
static volatile uint32_t rxlen;

/* not static because from eeprom */
extern config_t cfg;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void rs232_init(void)
{
  txlen = 0;
  rxlen = 0;

  vic_enableirq(38, irq_handler);

  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* select uart pins for pd5 and pd6 */
  GPIOD_MODER |= (2u << 10) | (2u << 12);
  GPIOD_AFRL |= (7u << 20) | (7u << 24);

  /* enable and configure usart2 */
  RCC_APB1ENR |= BIT_17;
  USART2_BRR = (BAUD_FRAC(cfg.rs232_baudrate) << 0) |
    (BAUD_INT(cfg.rs232_baudrate) << 4);
  USART2_CR1 = BIT_13 | BIT_05 | BIT_03 | BIT_02;
}




int kbhit(void)
{
  static uint32_t rdpos = 0;
  if(rxlen > 0)
  {
    int ret = rxbuffer[rdpos];
    __disable_irq();
    rxlen--;
    __enable_irq();
    rdpos++;
    if(rdpos == RXBUFFERSIZE)
    {
      rdpos = 0;
    }
    return ret;
  }
  return -1;
}


void txchar(char c)
{
  static int write_ind = 0;

  /* this can only proceed if one of the following conditions is met:
     a) the buffer is empty - in this case there is nothing in the buffer
        and it can be used immediately
     b) read index is not the same as write index - if they are equal it means
        the buffer is overflowing */

    while(txlen == TXBUFFERSIZE);

  /* put the character into the buffer */
  txbuffer[write_ind] = c;

  write_ind++;
  if(write_ind == TXBUFFERSIZE)
  {
    write_ind = 0;
  }
  __disable_irq();
  txlen++;
  __enable_irq();


  /* actual transmission is handled only in the interrupt routine */
  enable_txempty_irq();
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void irq_handler(void)
/*------------------------------------------------------------------------------
  Function:
  this is the interrupt handler for the usart.
  in:  none
  out: none
==============================================================================*/
{
  uint32_t sr = USART2_SR;
  uint32_t cr = USART2_CR1;

  if((sr & cr) == BIT_07) //TXE?
  {
    static uint32_t read_ind = 0;

    if(txlen == 0)
    {
      disable_txempty_irq();
      return;
    }

    txlen--;

    /* send the next character and go to the next read position, take care
       of wrapping the read index */
    USART2_DR = txbuffer[read_ind];
    read_ind++;
    if(read_ind == TXBUFFERSIZE)
    {
      read_ind = 0;
    }
  }

  if((sr & cr) == BIT_05)
  {
    /* DR must be read anyways to acknowledge the interrupt */
    uint8_t tmp = USART2_DR;

    static uint32_t wrpos = 0;
    rxbuffer[wrpos] = tmp;
    wrpos++;
    rxlen++;
    if(wrpos == RXBUFFERSIZE)
    {
      wrpos = 0;
    }
  }

  USART2_SR = 0;
}

/*============================================================================*/
static void enable_txempty_irq(void)
/*------------------------------------------------------------------------------
  Function:
  switch on the tx empty itnerrupt. as soon as this interrupt is enabled, it is
  triggered all the time and can only be acknowledged by writing to the data
  register
  in:  none
  out: none
==============================================================================*/
{
  USART2_CR1 |= BIT_07;
}

/*============================================================================*/
static void disable_txempty_irq(void)
/*------------------------------------------------------------------------------
  Function:
  switch on the tx empty itnerrupt
  in:  none
  out: none
==============================================================================*/
{
  USART2_CR1 &= ~BIT_07;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
