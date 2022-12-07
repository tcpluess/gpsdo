/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    implementation of the ublox binary protocol
 *
 * Filename:       ublox.h
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
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
#define FIX_3D 3u
#define FIX_TIME 5u

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
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hmsl;
  uint32_t hacc;
  uint32_t vacc;
  uint16_t pdop;
  uint64_t time;
} pvtinfo_t;

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
  uint32_t itow;
  uint16_t gdop;
  uint16_t pdop;
  uint16_t tdop;
  uint16_t vdop;
  uint16_t hdop;
  uint16_t ndop;
  uint16_t edop;
  uint64_t time;
} dopinfo_t;

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
  uint8_t best_snr;
  uint64_t time;
} satinfo_t;

typedef struct
{
  const pvtinfo_t* pvt;
  const svindata_t* svi;
  const dopinfo_t* dop;
  const satinfo_t* sat;
} gnssstatus_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void gnss_init(void);
/*------------------------------------------------------------------------------
  Function:
  initialise tne gnss functionality
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
extern void reconfigure_gnss(void);
/*------------------------------------------------------------------------------
  Function:
  reconfigure the gnss systems used
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void manual_svin(void);
/*------------------------------------------------------------------------------
  Function:
  start the survey-in process; the accuracy limit and duration are taken from
  the config data
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void manual_svin_stop(void);
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
extern bool gps_check_health(void);
/*------------------------------------------------------------------------------
  Function:
  check if the satellite snr, fix and time accuracy are acceptable
  in:  none
  out: returns false if the gps reception is unstable/unreliable
==============================================================================*/


/*============================================================================*/
extern const gnssstatus_t* get_gnss_status(void);
/*------------------------------------------------------------------------------
  Function:
  get position, velocity, time, survey etc. status info.
  in:  none
  out: returns a pointer to a gpsstatus_t structure with all data
==============================================================================*/


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

