/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    control algorithm for the GNSS frequency standard
 *
 * Compiler:       ANSI-C
 *
 * Filename:       cntl.c
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

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cntl.h"
#include "misc.h"
#include "ublox.h"
#include "eeprom.h"
#include "timebase.h"
#include "adc.h"
#include "tdc.h"
#include "dac.h"
#include "temperature.h"

#include "stm32f407.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define OSCGAIN (120.0/32768.0)

/* ocxo current limit when warmed up */
#define OCXO_CURRENT_LIM 210.0f /* approx. 2.5 Watts @ 12 V */

/* define the time limits after which the control loop constants are switched */
#define FASTTRACK_TIME_LIMIT 5u /* min */
#define LOCKED_TIME_LIMIT 15u /* min */

/* allowed phase error to change the control loop constants */
#define MAX_PHASE_ERR 100.0f /* ns */

/* allowed maximum number of outliers. */
#define MAX_ALLOWED_OUTLIERS 10u

/* if no outliers are detected within this time, the outlier counter is
   reset */
#define OUTLIER_DURATION (5u * (60u * 1000u)) /* 5 min in msec */

/* the tic actually has an offset of 300ns because of the synchronisation
   logic internal to the stm32. this offset was determined empirically and
   is chosten such that at a tic value of 0, the 1PPS output and the tic input
   occur at the same time. */
#define TIC_OFFSET 0u

#define TAU_FASTTRACK 10.0
#define TAU_LOCKED 100.0

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

typedef enum
{
  warmup,
  holdover,
  track_lock
} status_t;


typedef enum
{
  fast_track,
  locked,
  stable
} controlstatus_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static float read_tic(void);
static uint16_t pi_control(double KP, double TI, double e);
static void cntl(void);
static void ledon(void);
static void ledoff(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static double esum;
const char* cntl_status = "";

extern config_t cfg;
static status_t gpsdostatus;
static controlstatus_t stat = fast_track;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void cntl_task(void* param)
{
  (void)param;
  gpsdostatus = warmup;
  uint64_t firstfix = 0;
  esum = cfg.last_dacval;

  for(;;)
  {
    start_conversion();

    switch(gpsdostatus)
    {
      /* warmup: wait until the ocxo is ready */
      case warmup:
      {
        ledon();

        /* measure the ocxo current; when the ocxo is warm, the current drops */
        float iocxo = get_iocxo();
        if(iocxo < OCXO_CURRENT_LIM)
        {
          gpsdostatus = holdover;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        ledoff();
        vTaskDelay(pdMS_TO_TICKS(50));

        break;
      }

      /* holdover mode: wait until the position is valid */
      case holdover:
      {
        extern gpsinfo_t pvt_info;
        ledon();

        if(check_fix())
        {
          if(firstfix == 0)
          {
            firstfix = get_uptime_msec();
          }
          else if(pvt_info.tacc < (uint32_t)MAX_PHASE_ERR)
          {
            if(pps_elapsed())
            {
              firstfix = 0;
              gpsdostatus = track_lock;
            }
          }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
        ledoff();
        vTaskDelay(pdMS_TO_TICKS(200));
        break;
      }

      /* track/lock mode: control loop is active */
      case track_lock:
      {

        /* only look at the 1pps signal if the fix is valid; if fix is invalid,
           go to holdover mode */
        if(check_fix())
        {
          /* is it time to run the control loop? */
          if(pps_elapsed())
          {
            ledon();
            cntl();
            vTaskDelay(pdMS_TO_TICKS(50));
            ledoff();
          }
        }
        else
        {
          gpsdostatus = holdover;
        }
      }
    }
  }
}

void cntl_restart(void)
{
  stat = fast_track;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

static void ledon(void)
{
      GPIOE_BSRR = BIT_15;
}


static void ledoff(void)
{
      GPIOE_BSRR = BIT_31;

}


static float read_tic(void)
{
  tdc_waitready();
  float tic = get_tic();
  float qerr = get_timepulse_error();
  enable_tdc();
  return (tic - qerr) + TIC_OFFSET;
}


static double prefilter(double ee, double filt)
{
  static double eold = 0.0;
  double result = ((100.0-filt)*ee + filt*eold)/100.0;
  eold = ee;
  return result;
}


static uint16_t pi_control(double KP, double TI, double ee)
{

  static double eold = 0.0f;
  esum = esum + 0.5*KP*(ee + eold)/TI;
  eold = ee;
  if(esum > 65535.0)
  {
    esum = 65535.0;
  }
  else if(esum < 0.0)
  {
    esum = 0.0;
  }
  float efc = (float)(KP*ee + esum);
  if(efc > 65535.0f)
  {
    return 65535u;
  }
  else if(efc < 0.0f)
  {
    return 0u;
  }
  else
  {
    return (uint16_t)efc;
  }
}

extern volatile float stat_e;

static void cntl(void)
{
  static uint32_t statuscount = 0;
  static uint32_t outlier_count = 0;
  static uint64_t last_outlier_time = 0;
  float e;
  uint16_t dacval = 0u;

  /* determine the time interval (phase) error */
  float tic = read_tic();

  /* TODO: read these values from the eeprom */
  e = tic - (float)cfg.timeoffset;

  /* this is somewhat hack-ish, but avoids using fabs() */
  float abs_err = e > 0 ? e : -e;

  /* if the phase error is less than one period, assume the pll is locked and
     switch on the green lock led */
  if(abs_err < MAX_PHASE_ERR)
  {
    GPIOE_BSRR = BIT_14;
  }
  else
  {
    GPIOE_BSRR = BIT_30;

    if(abs_err > 10000)
    {
      timebase_reset();
      return;
    }
  }

  switch(stat)
  {
    /* fast track: is normally used shortly after the ocxo has just
       warmed up. use small time constant such that the phase settles
       more quickly */
    case fast_track:
    {
      double kp = 1.0/(OSCGAIN * TAU_FASTTRACK);
      double ki = TAU_FASTTRACK;
      dacval = pi_control(kp, ki, (double)e);

      /* if the phase error stays below MAX_PHASE_ERR for the time
         FASTTRACK_TIME_LIMIT, the control loop constants can be changed
         because we assume that the ocxo is now locked */
      if(abs_err < MAX_PHASE_ERR)
      {
        statuscount++;
      }
      else
      {
        statuscount = 0;
      }

      if(statuscount >= (FASTTRACK_TIME_LIMIT * 60u))
      {
        cfg.last_dacval = dacval;
        statuscount = 0;
        stat = locked;
      }

      cntl_status = "fast_track";

      break;
    }

    /* locked: use an intermediate time constant after the ocxo has
       settled for a while. */
    case locked:
    {
      double kp = 1.0/(OSCGAIN * TAU_LOCKED);
      double ki = TAU_LOCKED;
      dacval = pi_control(kp, ki, (double)e);

      /* if the phase error stays below MAX_PHASE_ERR for the time
         LOCKED_TIME_LIMIT, the control loop constants are changed again,
         because the ocxo is stable enough */
      if(abs_err < MAX_PHASE_ERR)
      {
        statuscount++;
      }
      else
      {
        statuscount = 0;
      }

      if(statuscount >= (LOCKED_TIME_LIMIT * 60u))
      {
        cfg.last_dacval = dacval;
        statuscount = 0;
        stat = stable;
      }

      cntl_status = "locked";

      break;
    }

    /* this should be the normal operating state. */
    case stable:
    {
      /* if the phase error increases above the threshold MAX_PHASE_ERR, then
         the ocxo is perhaps not stable enough and the control loop switches
         its constants such that it becomes faster */
      if(abs_err > MAX_PHASE_ERR)
      {
        outlier_count++;
        last_outlier_time = get_uptime_msec();

        if(outlier_count > MAX_ALLOWED_OUTLIERS)
        {
          outlier_count = 0u;
          statuscount = 0u;

          /* if the phase error is small, the slow pll mode is sufficient */
          if(abs_err < 200.0f)
          {
            stat = locked;
          }
          else
          {
            stat = fast_track;
          }
        }
      }
      else
      {
        uint64_t now = get_uptime_msec();
        if(now - last_outlier_time >= OUTLIER_DURATION)
        {
          outlier_count = 0;
        }

        /* kp, ki and prefilter are stored in the eeprom */
        double kp  = 1.0/(OSCGAIN * cfg.tau);
        double ki = cfg.tau;
        double filt = cfg.filt;
        dacval = pi_control(kp, ki, prefilter((double)e, filt));
      }

      cntl_status = "stable";

      break;
    }
  }

  set_dac(dacval);

  stat_e = e;

}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
