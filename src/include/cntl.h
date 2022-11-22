/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    control algorithm for the GNSS frequency standard
 *
 * Compiler:       ANSI-C
 *
 * Filename:       cntl.h
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  28.04.2020
 *******************************************************************************
   Modification History:
   [1.0]    28.04.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

#ifndef __CNTL_H__
#define __CNTL_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

typedef struct
{
  const char* mode;
  float esum;
  float e;

} cntlstatus_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void cntl_task(void* param);
/*------------------------------------------------------------------------------
  Function:
  this is the actual GNSS control program
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void cntl_restart(void);
/*------------------------------------------------------------------------------
  Function:
  restart the control loop
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern const cntlstatus_t* get_cntlstatus(void);
/*------------------------------------------------------------------------------
  Function:
  returns the controller status
  in:  none
  out: none
==============================================================================*/

extern void cntl_set(float sp);

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

