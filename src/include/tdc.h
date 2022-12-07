/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    driver for the TDC7200 time to digital converter
 *
 * Compiler:       ANSI-C
 *
 * Filename:       tdc.h
 *
 * Version:        1.2
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created

   [1.1]    01.04.2020    Tobias Plüss <tpluess@ieee.org>
   - use hardware spi interface

   [1.2]    27.05.2021    Tobias Plüss <tpluess@ieee.org>
   - use external interrupt and semaphores for more efficient access
 ******************************************************************************/

#ifndef __TDC_H__
#define __TDC_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

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
extern void setup_tdc(void);
/*------------------------------------------------------------------------------
  Function:
  initialise the hardware and configure the tdc (e.g. #of averaging cycles)
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern bool read_tdc(float* result);
/*------------------------------------------------------------------------------
  Function:
  wait until a measurement is finished and return the measured time interval
  in nanoseconds.
  in:  result -> this is where the measurement result goes
  out: returns true if the result is valid, false otherwise
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

