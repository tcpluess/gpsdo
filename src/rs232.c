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

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define TXBUFFERSIZE 512u
#define BAUD 115200u
#define BAUD_INT ((uint32_t)(10000000u/(16u*BAUD)))
#define BAUD_FRAC ((uint32_t)(10000000u/BAUD-16u*BAUD_INT+0.5f))

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
static void buffer_insert(char c);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static char txbuffer[TXBUFFERSIZE];
static volatile uint32_t read_ind;
static uint32_t write_ind;
static volatile bool buffer_empty;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

extern void rs232_init(void)
{
  read_ind = 0;
  write_ind = 0;
  buffer_empty = true;

  vic_enableirq(38, irq_handler);

  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* select uart pins for pd5 and pd6 */
  GPIOD_MODER |= (2u << 10) | (2u << 12);
  GPIOD_AFRL |= (7u << 20) | (7u << 24);

  /* enable and configure usart2 */
  RCC_APB1ENR |= BIT_17;
  USART2_BRR = (BAUD_FRAC << 0) | (BAUD_INT << 4);
  USART2_CR1 = BIT_13 | BIT_03 | BIT_02;
}

_ssize_t _write_r(struct _reent *r, int file, const void *ptr, size_t len)
{
  const unsigned char *p = (const unsigned char*)ptr;
  for(int i = 0; i < len; i++)
  {
    if(*p == '\n')
    {
      buffer_insert('\r');
    }
    buffer_insert(*p++);
  }
  return len;
}

int _isatty(int file)
{
  return 1;
}

_ssize_t _read_r(struct _reent *r, int file, void *ptr, size_t len)
{
#if 0
  char c;
  int  i;
  unsigned char *p;

  p = (unsigned char*)ptr;

  for (i = 0; i < len; i++)
  {
    //c = uart0GetchW();

    *p++ = c;
    //uart0Putch(c);

    if (c == 0x0D && i <= (len - 2))
    {
      *p = 0x0A;
      //uart0Putch(0x0A);
      return i + 2;
    }
  }
  return i;
#endif
  return 0;
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
  /* check if there is something in the buffer. as long as read_ind and
     write_ind are different, there is data to be transmitted */
  if(read_ind != write_ind)
  {
    /* send the next character and go to the next read position, take care
       of wrapping the read index */
    USART2_DR = txbuffer[read_ind];
    read_ind++;
    if(read_ind == TXBUFFERSIZE)
    {
      read_ind = 0;
    }
  }
  else
  {
    /* if the read_ind and write_ind are the same, the buffer is empty.
       the interrupt needs to be disabled because it can only be acknowledged
       by writing to the data register */
    buffer_empty = true;
    disable_txempty_irq();
  }
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

/*============================================================================*/
static void buffer_insert(char c)
/*------------------------------------------------------------------------------
  Function:
  insert a character into the tx buffer
  in:  none
  out: none
==============================================================================*/
{
  /* this can only proceed if one of the following conditions is met:
     a) the buffer is empty - in this case there is nothing in the buffer
        and it can be used immediately
     b) read index is not the same as write index - if they are equal it means
        the buffer is overflowing */
  do
  {
    if(buffer_empty)
    {
      break;
    }
    if(read_ind != write_ind)
    {
      break;
    }
  } while(true);

  /* put the character into the buffer */
  txbuffer[write_ind] = c;
  buffer_empty = false;

  __disable_irq();
  write_ind++;
  if(write_ind == TXBUFFERSIZE)
  {
    write_ind = 0;
  }
  __enable_irq();

  /* actual transmission is handled only in the interrupt routine */
  enable_txempty_irq();
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
