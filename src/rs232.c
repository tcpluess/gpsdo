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
 * Version:        2.0
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

   [2.0]    14.05.2021    Tobias Plüss <tpluess@ieee.org>
   - ported to FreeRTOS.
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "rs232.h"
#include "stm32f407.h"
#include "misc.h"
#include "nvic.h"
#include "eeprom.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define BAUD_INT(x) ((uint32_t)(40000000.0/(16.0*(x))))
#define BAUD_FRAC(x) ((uint32_t)((40000000.0/(x)-16.0*BAUD_INT(x))+0.5))
#define RS232_BUFFERSIZE 100u

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

static StreamBufferHandle_t txstream;
static StreamBufferHandle_t rxstream;

/* not static because from eeprom */
extern config_t cfg;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void rs232_init(void)
{
  txstream = xStreamBufferCreate(RS232_BUFFERSIZE, 1);
  rxstream = xStreamBufferCreate(RS232_BUFFERSIZE, 1);

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
  char rx;
  if(xStreamBufferReceive(rxstream, &rx, 1, portMAX_DELAY) > 0)
  {
    return rx;
  }
  else
  {
    return -1;
  }
}

bool canread(void)
{
  if(xStreamBufferIsEmpty(rxstream))
  {
    return false;
  }
  else
  {
    return true;
  }
}


void txchar(char c)
{
  (void)xStreamBufferSend(txstream, &c, 1, portMAX_DELAY);
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
    char c;
    if(xStreamBufferReceiveFromISR(txstream, &c, 1, NULL) > 0)
    {
      USART2_DR = c;
    }
    else
    {
      disable_txempty_irq();
    }
  }

  if((sr & cr) == BIT_05)
  {
    /* DR must be read anyways to acknowledge the interrupt */
    uint8_t tmp = (uint8_t)USART2_DR;
    (void)xStreamBufferSendFromISR(rxstream, &tmp, 1, NULL);
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
