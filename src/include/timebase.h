/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    module for various timing topics for the gpsdo
 *
 * Filename:       timebase.h
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
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

