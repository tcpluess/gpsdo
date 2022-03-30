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

/* minimum time the 1pps pulses must be present to switch out from holdover */
#define HOLDOVER_PPS_LIMIT 60u

/* this is the amount of phase error that is tolerated before the timebase
   (pwm output for the 1pps output) is reset */
#define MAX_ALLOWED_PHASE_ERR 1000u /* 1000 ns = 1 us */

#define TAU_FASTTRACK 5.0
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

static uint16_t pi_control(double KP, double TI, double e);
static bool cntl(void);
static inline void ledon(void) { GPIOE->BSRR = BIT_15; }
static inline void ledoff(void) { GPIOE->BSRR = BIT_31; }
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

/*============================================================================*/
static status_t warmup_handler(void)
/*------------------------------------------------------------------------------
  Function:
  this is called while the ocxo needs to warm up to stabilise.
  in:  none
  out: returns the next status of the controller.
==============================================================================*/
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


/*============================================================================*/
static status_t holdover_handler(void)
/*------------------------------------------------------------------------------
  Function:
  checks the gpsdo status during holdover and determines when to switch
  on the pll.
  in:  none
  out: returns the next status of the controller.
==============================================================================*/
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
        if(fabsf(e) > MAX_ALLOWED_PHASE_ERR)
        {
          timebase_reset();
        }

        /* if 1pps pulses were properly arriving for a certain time,
           switch on the pll */
        if(holdover_tic_count > HOLDOVER_PPS_LIMIT)
        {
          /* fast recovery after holdover! */
          cntlstat = fast_track;
          ledoff();
          return track_lock;
        }
        /* go back, check again the gps status, 1pps pulses and phase error */
        continue;
      }
    }

    holdover_tic_count = 0u;
    vTaskDelay(pdMS_TO_TICKS(100u));
  }
}


/*============================================================================*/
static status_t track_lock_handler(void)
/*------------------------------------------------------------------------------
  Function:
  this is called when the gpsdo enters the tracking state. checks the gpsdo
  status and, depending on the outcome, calls the control loop or switches to
  the holdover mode.
  in:  none
  out: returns the next status of the controller.
==============================================================================*/
{
  for(;;)
  {
    /* only look at the 1pps signal if the fix is valid; if fix is invalid,
       go to holdover mode */
    if(pps_elapsed() && gps_check_health())
    {
      ledon();

      /* if errors occur in the control loop, switch to holdover mode */
      if(cntl() == false)
      {
        return holdover;
      }
      vTaskDelay(pdMS_TO_TICKS(25));
      ledoff();
    }
    else
    {
      /* no 1pps pulses or gps fix is invalid, switch to holdover */
      stat_e = 0.0f;
      return holdover;
    }
  }
}


/*============================================================================*/
static bool get_phase_err(float* ret)
/*------------------------------------------------------------------------------
  Function:
  determines the phase error in nanoseconds.
  in:  ret -> this is where the phase error will be stored, is not NULL checked
       because this seems not to be necessary here
  out: returns true if phase error is valid, false otherwise
==============================================================================*/
{
  /* determine the time interval (phase) error from the input capture */
  float tic = get_tic();

  /* obtain the interpolator value */
  float tdc;
  if(get_tdc(&tdc) == false)
  {
    return false;
  }

  /* obtain the sawtooth correction value from the gps module */
  float qerr;
  if(get_timepulse_error(&qerr) == false)
  {
    return false;
  }

  /* calculate the actual phase error */
  float result = (float)cfg.timeoffset - (tic + qerr - tdc);

  *ret = result;
  stat_e = result;
  return true;
}


/*============================================================================*/
static uint16_t pi_control(double KP, double TI, double ee)
/*------------------------------------------------------------------------------
  Function:
  this is the actual pi controller.
  in:  KP -> proportional gain
       TI -> integration time
       ee -> error signal
  out: returns the controller output signal
==============================================================================*/
{
  static double eold = 0.0f;

  /* this is actually a bilinear transform: the integral part is
          1                                     2  z-1
     I = ----   with the bilinear transform s = -  --- this becomes:
         Ti*s                                   T  z+1

          T    z+1
     I = ----  --- . note the sampling time T is 1 (second), and by converting
         2*Ti  z-1
                                       1   1+z^-1
     all z to z^-1, this becomes: I = ---- ------
                                      2*Ti 1-z^-1
     now this is the z-transfer function of the integrator which is then
     converted to the below difference equation. */
  esum = esum + 0.5*(ee + eold)/TI;
  eold = ee;
  stat_esum = esum;

  /* limit the integrator ("anti windup") */
  if(esum > 65535.0)
  {
    esum = 65535.0;
  }
  else if(esum < 0.0)
  {
    esum = 0.0;
  }

  /* calculate the output signal and limit it ("output saturation") */
  float efc = (float)(KP*(ee + esum));
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


/*============================================================================*/
static bool cntl(void)
/*------------------------------------------------------------------------------
  Function:
  this calls the pi controller and steers the ocxo.
  in:  none
  out: returns true if all good, false if the phase error could not be
       determined or if another error occured.
==============================================================================*/
{
  /* phase error cannot be determined properly? then bail out */
  float e;
  if(get_phase_err(&e) == false)
  {
    return false;
  }

  /* if the phase error is less than one period, assume the pll is locked and
     switch on the green lock led */
  float abs_err = fabsf(e);
  if(abs_err < MAX_PHASE_ERR)
  {
    ppsenable(true);
  }
  else
  {
    ppsenable(false);
  }

  static uint32_t lock_counter = 0u;
  uint16_t dacval = 0u;
  switch(cntlstat)
  {
    /* fast track: is normally used shortly after the ocxo has just
       warmed up. use small time constant such that the phase settles
       more quickly */
    case fast_track:
    {
      cntl_status = "fast_track";
      dacval = pi_control(KP_FASTTRACK, KI_FASTTRACK, (double)e);

      if(abs_err < MAX_PHASE_ERR)
      {
        lock_counter++;
      }
      else
      {
        lock_counter = 0u;
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

      if(abs_err < MAX_PHASE_ERR)
      {
        lock_counter++;
      }
      else
      {
        lock_counter = 0u;
      }

      /* if locked successfully for a certain time, go to the slowest
         (adjustable) control mode */
      if(lock_counter >= (LOCKED_TIME_LIMIT * 60u))
      {
        cfg.last_dacval = dacval;
        lock_counter = 0;
        cntlstat = stable;
      }
      else if(lock_counter == 0u)
      {
        cntlstat = fast_track;
      }

      break;
    }

    /* this should be the normal operating state. */
    case stable:
    {
      cntl_status = "stable";

      /* kp, ki and prefilter are stored in the eeprom */
      double kp = (1.0/(OSCGAIN * (double)cfg.tau));
      double ki = (double)cfg.tau;
      dacval = pi_control(kp, ki, e);

      /* if the phase error increases above the threshold MAX_PHASE_ERR, then
         the ocxo is perhaps not stable enough and the control loop switches
         its constants such that it becomes faster */
      if(abs_err > 2*MAX_PHASE_ERR)
      {
        cntlstat = fast_track;
      }
      else if(abs_err > MAX_PHASE_ERR)
      {
        cntlstat = locked;
      }

      break;
    }
  }

  set_dac(dacval);
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
