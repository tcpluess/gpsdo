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
#define HOLDOVER_PPS_LIMIT 10u

/* this is the amount of phase error that is tolerated before the timebase
   (pwm output for the 1pps output) is reset */
#define MAX_ALLOWED_PHASE_ERR 1000u /* 1000 ns = 1 us */

#define TAU_FASTTRACK 10.0
#define TAU_LOCKED 100.0

#define KP_FASTTRACK (1.0/(OSCGAIN * TAU_FASTTRACK))
#define KI_FASTTRACK (1.0*TAU_FASTTRACK)
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

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static uint16_t pi_control(double KP, double TI, float u);
static bool cntl(void);
static inline void ledon(void) { GPIOE->BSRR = BIT_15; }
static inline void ledoff(void) { GPIOE->BSRR = BIT_31; }
static bool get_phase_err(void);
static status_t warmup_handler(void);
static status_t holdover_handler(void);
static status_t track_lock_handler(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

bool dac_hold;
static controlstatus_t cntlstat;

static cntlstatus_t ctl;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void cntl_task(void* param)
{
  (void)param;
  status_t gpsdostatus = warmup;
  cntlstat = fast_track;

  ctl.esum = get_config()->last_dacval;
  ctl.mode = "";
  dac_hold = false;

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


const cntlstatus_t* get_cntlstatus(void)
{
  return &ctl;
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
  ctl.mode = "warmup";

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
  ctl.mode = "holdover";
  ledon();

  for(;;)
  {
    if(gps_check_health())
    {
      if(pps_elapsed() && get_phase_err())
      {
        holdover_tic_count++;

        /* reset the time base (1pps pwm output) if the phase error
           is too large. otherwise it will take too long until the pll
           locks! */
        if(fabsf(ctl.e) > MAX_ALLOWED_PHASE_ERR)
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
      return holdover;
    }
  }
}


/*============================================================================*/
static bool get_phase_err(void)
/*------------------------------------------------------------------------------
  Function:
  determines the phase error in nanoseconds and stores it in the variable e.
  this is necessary such that the phase error can be accessed by other modules
  as well (via access function).
  in:  none
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
  ctl.e = (float)get_config()->timeoffset - (tic + qerr - tdc);

  return true;
}


/*============================================================================*/
static uint16_t pi_control(double KP, double TI, float u)
/*------------------------------------------------------------------------------
  Function:
  this is the actual pi controller.
  in:  KP -> proportional gain
       TI -> integration time
       u -> error signal (input to the PI controller)
  out: returns the controller output signal
==============================================================================*/
{
  static float uold = 0.0f;

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
  ctl.esum = fminf(fmaxf(0.0, ctl.esum + KP*(u + uold)/(2.0*TI)), 65535.0);
  uold = u;

  /* calculate the output signal and limit it ("output saturation") */
  float efc = fminf(fmaxf(0.0, KP*u + ctl.esum), 65535.0);
  return (uint16_t)efc;
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
  if(get_phase_err() == false)
  {
    return false;
  }

  /* if the phase error is less than one period, assume the pll is locked and
     switch on the green lock led */
  float abs_err = fabsf(ctl.e);
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
      ctl.mode = "fast_track";
      dacval = pi_control(KP_FASTTRACK, KI_FASTTRACK, ctl.e);

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
        get_config()->last_dacval = dacval;
        lock_counter = 0;
        cntlstat = locked;
      }

      break;
    }

    /* locked: use an intermediate time constant after the ocxo has
       settled for a while. */
    case locked:
    {
      ctl.mode = "locked";
      dacval = pi_control(KP_LOCKED, KI_LOCKED, ctl.e);

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
        get_config()->last_dacval = dacval;
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
      ctl.mode = "stable";

      /* kp, ki and prefilter are stored in the eeprom */
      double ki = (double)get_config()->tau;
      double kp = (1.0/(OSCGAIN * ki));
      dacval = pi_control(kp, ki, ctl.e);

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

  if(dac_hold == false)
  {
    set_dac(dacval);
  }
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
