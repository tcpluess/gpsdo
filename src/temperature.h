/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    adt7301 driver
 *
 * Compiler:       ANSI-C
 *
 * Filename:       adt7301.h
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

#ifndef __ADT7301_H__
#define __ADT7301_H__

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
extern void tmp_init(void);
/*------------------------------------------------------------------------------
  Function:
  initialise pin assignment and the interface
  in:  none
  out: none
==============================================================================*/

/*============================================================================*/
extern float get_temperature(void);
/*------------------------------------------------------------------------------
  Function:
  reads out the temperature sensor
  in:  none
  out: none
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif
