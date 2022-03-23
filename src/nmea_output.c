/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    nmea sentences output
 *
 * Compiler:       ANSI-C
 *
 * Filename:       nmea_output.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  17.03.2022
 *******************************************************************************
   Modification History:
   [1.0]    17.03.2022    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "misc.h"
#include "ublox.h"
#include "checksum.h"
#include "nmea_output.h"

#include <stdio.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define NMEA_MAX_LEN 90u /* maximum length of nmea sentences should be less */

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

static void decimal2deg(int32_t decimal, int32_t* degrees, float* minutes);

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
void nmea_task(void* param)
/*------------------------------------------------------------------------------
  Function:
  outputs the nmea sentences.
  in:  none
  out: none
==============================================================================*/
{
  /* unused */
  (void)param;
  extern gpsinfo_t pvt_info;

  FILE* nmeaout = fdopen(NMEA_FD, "w");
  (void)setvbuf(nmeaout, NULL, _IOLBF, NMEA_MAX_LEN);

  for(;;)
  {
    /* wait until position, velocity and time are known */
    if(gps_wait_pvt())
    {

      /* if position, velocity and time are known AND valid */
      if((pvt_info.valid & (BIT_00 | BIT_01)) == (BIT_00 | BIT_01))
      {
        char nmea[NMEA_MAX_LEN];
        int32_t lat_degrees, lon_degrees;
        float lat_minutes, lon_minutes;
        uint8_t yr2 = (uint8_t)(pvt_info.year % 1000u);

        decimal2deg(pvt_info.lat, &lat_degrees, &lat_minutes);
        decimal2deg(pvt_info.lon, &lon_degrees, &lon_minutes);

        char sn = 'N';
        if(lat_degrees < 0)
        {
          sn = 'S';
          lat_degrees = -lat_degrees;
        }

        char we = 'E';
        if(lon_degrees < 0)
        {
          we = 'W';
          lon_degrees = -lon_degrees;
        }

        /* construct the nmea message in the buffer. heading, speed and so on
           are constantly set to zero as they are not interesting. */
        int len = snprintf(nmea, NMEA_MAX_LEN,
          "GPRMC,%02d%02d%02d,A,%ld%08.5f,%c,%ld%08.5f,%c,000.0,000.0,%02d%02d%02d,000.0,E",
          pvt_info.hour, pvt_info.min, pvt_info.sec,
          lat_degrees, lat_minutes, sn,
          lon_degrees, lon_minutes, we,
          pvt_info.day, pvt_info.month, yr2);

        /* calculate the checksum for the message */
        uint8_t cksum = nmea0183_checksum(nmea, len);

        /* construct the actual message and print it */
        (void)fprintf(nmeaout, "$%s*%02X\n", nmea, cksum);
      }
    }
  }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void decimal2deg(int32_t decimal, int32_t* degrees, float* minutes)
/*------------------------------------------------------------------------------
  Function:
  convert decimal degrees in the form ddddddd to degrees and minutes, such as
  dd and mm.mmmmmm.
  in:  decimal -> decimal degrees with scaling factor 1e7 (according to ublox)
  out: degrees -> integer part, degrees
       minutes -> minutes and fractions of minutes
==============================================================================*/
{
    *degrees = decimal / 10000000;
    *minutes = (float)((((float)(decimal % 10000000)) * 60.0) / 1e7);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
