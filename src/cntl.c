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
#include <math.h>

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

/* used to add a constant offset to the tic values. */
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

extern volatile float stat_e;
extern volatile float stat_esum;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static bool read_tic(float* result);
static uint16_t pi_control(double KP, double TI, double e);
static bool cntl(void);
static void ledon(void);
static void ledoff(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static double esum;
const char* cntl_status = "";

extern config_t cfg;
static status_t gpsdostatus;
static controlstatus_t cntlstat;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void cntl_task(void* param)
{
  (void)param;
  gpsdostatus = warmup;
  cntlstat = fast_track;

  esum = cfg.last_dacval;
  stat_esum = esum;

  for(;;)
  {
    switch(gpsdostatus)
    {
      /* warmup: wait until the ocxo is ready */
      case warmup:
      {
        cntl_status = "warmup";

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
        cntl_status = "holdover";

        stat_e = 0.0f;
        ledon();

        if(gps_waitready() && pps_elapsed())
        {
          enable_tdc();
          ledoff();
          gpsdostatus = track_lock;
        }

        break;
      }

      /* track/lock mode: control loop is active */
      case track_lock:
      {
        cntl_status = "track_lock";

        /* only look at the 1pps signal if the fix is valid; if fix is invalid,
           go to holdover mode */
        if(pps_elapsed())
        {
            ledon();
            if(cntl() == false)
            {
              gpsdostatus = holdover;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
            ledoff();
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
  cntlstat = fast_track;
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


static bool read_tic(float* result)
{
  float tic = get_tic();
  float tdc;
  float qerr;
  if(get_tdc(&tdc))
  {
    if(get_timepulse_error(&qerr))
    {
      *result = (tic - qerr + tdc) + TIC_OFFSET;
      return true;
    }
  }
  return false;
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


static bool cntl(void)
{
  static uint32_t statuscount = 0;
  static uint32_t outlier_count = 0;
  static uint64_t last_outlier_time = 0;
  float e;
  uint16_t dacval = 0u;

  /* determine the time interval (phase) error */
  float tic;
  if(read_tic(&tic) == false)
  {
    return false;
  }

  e = tic - (float)cfg.timeoffset;
  float abs_err = fabs(e);

  /* if the phase error is less than one period, assume the pll is locked and
     switch on the green lock led */
  if(abs_err < MAX_PHASE_ERR)
  {
    ppsenable(true);
    GPIOE_BSRR = BIT_14;
  }
  else
  {
    ppsenable(false);
    GPIOE_BSRR = BIT_30;

    if(abs_err > 10000)
    {
      timebase_reset();
      return true;
    }
  }

  switch(cntlstat)
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
        cntlstat = locked;
      }

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
        cntlstat = stable;
      }

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
            cntlstat = locked;
          }
          else
          {
            cntlstat = fast_track;
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

      break;
    }
  }

  set_dac(dacval);

  stat_e = e;
  stat_esum = esum;
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
