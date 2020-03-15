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

#include "rs232.h"
#include "stm32f407.h"
#include "misc.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define BAUD 57600u
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

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

extern void rs232_init(void)
{
  /* enable port d */
  RCC_AHB1ENR |= BIT_03;

  /* select uart pins for pd5 and pd6 */
  GPIOD_MODER |= (2u << 10) | (2u << 12);
  GPIOD_AFRL |= (7u << 20) | (7u << 24);

  /* enable usart2 */
  RCC_APB1ENR |= BIT_17;


  /* div = 10e6/(16*B) */
  USART2_BRR = (BAUD_FRAC << 0) | (BAUD_INT << 4);

  USART2_CR1 = BIT_13 | BIT_03 | BIT_02;
}

void uart0Putch(char c)
{
  do
  {
    if(USART2_SR & BIT_07)
    {
      break;
    }
  } while(true);
  USART2_DR = c;
}

_ssize_t _write_r (
    struct _reent *r,
    int file,
    const void *ptr,
    size_t len)
{
  int i;
  const unsigned char *p;

  p = (const unsigned char*) ptr;

  for (i = 0; i < len; i++) {
    if (*p == '\n' ) uart0Putch('\r');
   uart0Putch(*p++);
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

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
