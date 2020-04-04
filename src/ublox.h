/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    implementation of the ublox binary protocol
 *
 * Compiler:       ANSI-C
 *
 * Filename:       ublox.h
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

   [1.1]    01.04.2020    Tobias Plüss <tpluess@ieee.org>
   - use interrupts
   - configure periodic messages (e.g. UBX_TIM_TP, UBX_NAV_PVT)
 ******************************************************************************/

#ifndef __UBLOX_H__
#define __UBLOX_H__

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

typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tacc;
  uint8_t fixtype;
  uint8_t flags;
  uint8_t xflags;
  uint8_t numsv;
  float lon;
  float lat;
  int32_t height;
  int32_t hmsl;
  uint32_t hacc;
  uint32_t vacc;
  uint16_t pdop;
} gpsinfo_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

extern void ublox_init(void);
extern void gps_worker(void);
extern float get_timepulse_error(void);

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

