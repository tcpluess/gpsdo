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
#include "stm32f407xx.h"
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
#define MAX_ALLOWED_OUTLIERS 20u

/* this is the amount of phase error that is tolerated before the timebase
   (pwm output for the 1pps output) is reset */
#define MAX_ALLOWED_PHASE_ERR 1000u /* 1000 ns = 1 us */

/* if no outliers are detected within this time, the outlier counter is
   reset */
#define OUTLIER_DURATION (5u * (60u * 1000u)) /* 5 min in msec */

#define TAU_FASTTRACK 10.0
#define TAU_LOCKED 100.0

#define KP_FASTTRACK (1.0/(OSCGAIN * TAU_FASTTRACK))
#define KI_FASTTRACK (TAU_FASTTRACK)
#define KP_LOCKED (1.0/(OSCGAIN * TAU_LOCKED))
#define KI_LOCKED (TAU_LOCKED)

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
extern volatile double stat_esum;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static bool read_tic(float* result);
static uint16_t pi_control(double KP, double TI, double e);
static bool cntl(void);
static void ledon(void);
static void ledoff(void);
static bool get_phase_err(float* ret);
static status_t warmup_handler(void);
static status_t holdover_handler(void);
static status_t track_lock_handler(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static double esum;
const char* cntl_status = "";

extern config_t cfg;
static controlstatus_t cntlstat;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void cntl_task(void* param)
{
  (void)param;
  status_t gpsdostatus = warmup;
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
        gpsdostatus = warmup_handler();
        break;
      }

      /* holdover mode: wait until the position is valid for at least one
         minute before switchint back to track/lock mode */
      case holdover:
      {
        gpsdostatus = holdover_handler();
        break;
      }

      /* track/lock mode: control loop is active */
      case track_lock:
      {
        gpsdostatus = track_lock_handler();
        break;
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

static status_t warmup_handler(void)
{
  cntl_status = "warmup";

  for(;;)
  {
    /* measure the ocxo current; when the ocxo is warm, the current drops */
    float iocxo = get_iocxo();
    if(iocxo < OCXO_CURRENT_LIM)
    {
      return holdover;
    }
    ledon();
    vTaskDelay(pdMS_TO_TICKS(50));
    ledoff();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


static status_t holdover_handler(void)
{
  uint32_t holdover_tic_count = 0u;
  cntl_status = "holdover";
  ledon();

  for(;;)
  {
    if(gps_check_health())
    {
      float e;
      if(pps_elapsed() && get_phase_err(&e))
      {
        holdover_tic_count++;

        /* reset the time base (1pps pwm output) if the phase error
           is too large. otherwise it will take too long until the pll
           locks! */
        if(fabs(e) > MAX_ALLOWED_PHASE_ERR)
        {
          timebase_reset();
        }
      }
      else
      {
        holdover_tic_count = 0u;
      }

      /* 1pps pulses were okay for one minute?
         then go to track_lock mode */
      if(holdover_tic_count > 60u)
      {
        ledoff();
        /* fast recovery after holdover! */
        cntlstat = fast_track;
        holdover_tic_count = 0u;

        /*lint -e438 last value assigned to holdover_tic_count not used */
        return track_lock;
      }
    }
    else
    {
      holdover_tic_count = 0u;
      vTaskDelay(pdMS_TO_TICKS(100u));
    }
  }
}


static status_t track_lock_handler(void)
{


  for(;;)
  {
    /* only look at the 1pps signal if the fix is valid; if fix is invalid,
       go to holdover mode */
    if(pps_elapsed() && gps_check_health())
    {
        ledon();
        if(cntl() == false)
        {
          return holdover;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
        ledoff();
    }
    else
    {
      stat_e = 0.0f;
      return holdover;
    }
  }
}


static bool get_phase_err(float* ret)
{
  /* determine the time interval (phase) error */
  float tic;
  if(read_tic(&tic) == false)
  {
    return false;
  }

  float e = (float)cfg.timeoffset - tic;
  if(ret != NULL)
  {
    *ret = e;
  }
  stat_e = e;

  return true;
}

static void ledon(void)
{
  GPIOE->BSRR = BIT_15;
}


static void ledoff(void)
{
  GPIOE->BSRR = BIT_31;
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
      /*lint -e834 operator order is ok */
      *result = (tic + qerr - tdc);
      return true;
    }
  }
  return false;
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
  static uint32_t lock_counter = 0u;
  static uint32_t outlier_count = 0;
  static uint64_t last_outlier_time = 0;
  float e;
  uint16_t dacval = 0u;

  /* phase error cannot be determined properly? then bail out */
  if(get_phase_err(&e) == false)
  {
    return false;
  }
  else
  {
    float abs_err = fabsf(e);

    /* if the phase error is less than one period, assume the pll is locked and
       switch on the green lock led */
    if(abs_err < MAX_PHASE_ERR)
    {
      ppsenable(true);
      GPIOE->BSRR = BIT_14;
    }
    else
    {
      ppsenable(false);
      GPIOE->BSRR = BIT_30;
    }

    switch(cntlstat)
    {
      /* fast track: is normally used shortly after the ocxo has just
         warmed up. use small time constant such that the phase settles
         more quickly */
      case fast_track:
      {
        cntl_status = "fast_track";
        dacval = pi_control(KP_FASTTRACK, KI_FASTTRACK, (double)e);

        /* if the phase error stays below MAX_PHASE_ERR for the time
           FASTTRACK_TIME_LIMIT, the control loop constants can be changed
           because we assume that the ocxo is now locked */
        if(abs_err < MAX_PHASE_ERR)
        {
          lock_counter++;
        }
        else
        {
          lock_counter = 0;
        }

        /* if locked successfully for a certain time, go to the next
           slower control mode */
        if(lock_counter >= (FASTTRACK_TIME_LIMIT * 60u))
        {
          cfg.last_dacval = dacval;
          lock_counter = 0;
          cntlstat = locked;
        }

        break;
      }

      /* locked: use an intermediate time constant after the ocxo has
         settled for a while. */
      case locked:
      {
        cntl_status = "locked";
        dacval = pi_control(KP_LOCKED, KI_LOCKED, (double)e);

        /* if the phase error stays below MAX_PHASE_ERR for the time
           LOCKED_TIME_LIMIT, the control loop constants are changed again,
           because the ocxo is stable enough. if not stable enough, go back
           to the fast_track mode */
        if(abs_err < MAX_PHASE_ERR)
        {
          lock_counter++;
        }
        else
        {
          lock_counter = 0;
          cntlstat = fast_track;
        }

        /* if locked successfully for a certain time, go to the slowest
           (adjustable) control mode */
        if(lock_counter >= (LOCKED_TIME_LIMIT * 60u))
        {
          cfg.last_dacval = dacval;
          lock_counter = 0;
          cntlstat = stable;
        }

        break;
      }

      /* this should be the normal operating state. */
      case stable:
      {
        cntl_status = "stable";
        /* if the phase error increases above the threshold MAX_PHASE_ERR, then
           the ocxo is perhaps not stable enough and the control loop switches
           its constants such that it becomes faster */
        if(abs_err > MAX_PHASE_ERR)
        {
          /* count the outliers */
          outlier_count++;

          /* remember the time when the last outlier occured */
          last_outlier_time = get_uptime_msec();

          /* if the number of outliers becomes too large, switch to a faster
             control mode */
          if(outlier_count > MAX_ALLOWED_OUTLIERS)
          {
            outlier_count = 0u;
            lock_counter = 0u;

            /* go to the fast_track mode */
            cntlstat = fast_track;
          }
        }
        else
        {
          /* if no outliers for more than 5 minutes, reset the outlier counter */
          uint64_t now = get_uptime_msec();
          if(now - last_outlier_time >= OUTLIER_DURATION)
          {
            outlier_count = 0;
          }

          /* kp, ki and prefilter are stored in the eeprom */
          double kp = (1.0/(OSCGAIN * (double)cfg.tau));
          double ki = (double)cfg.tau;
          dacval = pi_control(kp, ki, e);
        }

        break;
      }
    }

    set_dac(dacval);

    stat_esum = esum;
    return true;
  }
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
