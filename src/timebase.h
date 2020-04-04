/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    module for various timing topics for the gpsdo
 *
 * Compiler:       ANSI-C
 *
 * Filename:       timebase.h
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

#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

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

extern void timebase_init(void);

extern bool pps_elapsed(void);

extern void ppsenable(bool enable);

extern void timebase_reset(void);

extern float get_tic(void);

extern void set_pps_duration(uint32_t ms);

void reset_elapsed_ms(void);

uint32_t get_elapsed_ms(void);

extern uint64_t get_uptime_msec(void);

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

