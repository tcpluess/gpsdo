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
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

#define MAX_SV 40u

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
  uint64_t time;
} gpsinfo_t;

typedef struct
{
  uint32_t dur;
  int32_t x, y, z;
  uint32_t meanv;
  uint32_t obs;
  bool valid;
  bool active;
  uint64_t time;
} svindata_t;


typedef struct
{
  uint8_t numsv;
  struct
  {
    uint8_t gnssid;
    uint8_t svid;
    uint8_t cno;
    int8_t elev;
    int16_t azim;
  } sats[MAX_SV];
  uint64_t time;
} sv_info_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void gps_task(void* param);
/*------------------------------------------------------------------------------
  Function:
  the task handling all gnss related stuff
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern bool get_timepulse_error(float* result);
/*------------------------------------------------------------------------------
  Function:
  the gps mdule worker "thread"
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void start_svin(void);
/*------------------------------------------------------------------------------
  Function:
  start the survey-in process; the accuracy limit and duration are taken from
  the config data
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void set_fixpos_mode(void);
/*------------------------------------------------------------------------------
  Function:
  puts the gps module into fixed-position (timing) mode, using the ecef
  xyz coordinates and accuracy from the config data
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void disable_tmode(void);
/*------------------------------------------------------------------------------
  Function:
  disables the timing mode and switches the gps module to normal (positioning)
  mode
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void gps_restart(void);
/*------------------------------------------------------------------------------
  Function:
  initiates a reset and reconfiguration procedure of the gps module
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern bool check_fix(void);
/*------------------------------------------------------------------------------
  Function:
  initiates a reset and reconfiguration procedure of the gps module
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern bool gps_waitready(void);
/*------------------------------------------------------------------------------
  Function:
  wait until all data has been received
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void gps_timepulse_notify(void);
/*------------------------------------------------------------------------------
  Function:
  wait until all data has been received
  in:  none
  out: none
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

