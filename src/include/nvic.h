/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    helper functions for the nested vector interrupt controller
 *
 * Compiler:       ANSI-C
 *
 * Filename:       nvic.h
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

#ifndef __VIC_H__
#define __VIC_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void vic_init(void);
/*------------------------------------------------------------------------------
  Function:
  initialise the vic (namely, configure the vector table)
  in:  none
  out: none
==============================================================================*/

/*============================================================================*/
extern void vic_enableirq(int32_t intnum, void* func);
/*------------------------------------------------------------------------------
  Function:
  configure a handler for an interrupt and enable it
  in:  intnum -> interrupt number
       func -> interrupt handler
  out: none
==============================================================================*/

/*============================================================================*/
__attribute__((always_inline)) static inline void __enable_irq(void)
/*------------------------------------------------------------------------------
  Function:
  globally enable interrupts
  in:  none
  out: none
==============================================================================*/
{
  asm volatile ("cpsie i");
}

/*============================================================================*/
__attribute__((always_inline)) static inline void __disable_irq(void)
/*------------------------------------------------------------------------------
  Function:
  globally disable interrupts
  in:  none
  out: none
==============================================================================*/
{
  asm volatile ("cpsid i");
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif
