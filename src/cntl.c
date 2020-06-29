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

#include "cntl.h"
#include "misc.h"
#include "ublox.h"
#include "eeprom.h"
#include "timebase.h"
#include "adc.h"
#include "tdc.h"
#include "dac.h"


#include "stm32f407.h"

#include <stdio.h>
#include <math.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define OSCGAIN (120.0/32768.0)

/* ocxo current limit when warmed up */
#define OCXO_CURRENT_LIM 210.0f /* approx. 2.5 Watts @ 12 V */

/* define the time limits after which the control loop constants are switched */
#define FASTTRACK_TIME_LIMIT 5u /* min */
#define LOCKED_TIME_LIMIT 30u /* min */

/* allowed phase error to change the control loop constants */
#define MAX_PHASE_ERR 100.0f /* ns */

/* the tic actually has an offset of 300ns because of the synchronisation
   logic internal to the stm32. this offset was determined empirically and
   is chosten such that at a tic value of 0, the 1PPS output and the tic input
   occur at the same time. */
#define TIC_OFFSET 300u

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
  stable,

  /*init,
  lock1,
  lock2,
  lock3*/
} controlstatus_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static float read_tic(void);
static uint16_t pi_control(double KP, double KI, double damp, float e);
static void cntl(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

float soll = 0.0f;

float e = 0.0f;
uint16_t dacval = 0u;

extern svindata_t svin_info;
extern config_t cfg;
static status_t gpsdostatus = warmup;
static controlstatus_t stat = fast_track;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

static void blink(uint32_t ontime, uint32_t offtime)
{
  uint64_t ms = get_uptime_msec();
  static uint64_t next = 0;
  static bool on = false;

  if(ms >= next)
  {
    if(on)
    {
      on = false;
      GPIOE_BSRR = BIT_31;
      next = ms + offtime;
    }
    else
    {
      on = true;
      GPIOE_BSRR = BIT_15;
      next = ms + ontime;
    }
  }
}


static void blink_nervous(void)
{
  blink(50u, 50u);
}

static void blink_fast(void)
{
  blink(100u, 100u);
}

static void blink_slow(bool on)
{
  uint64_t ms = get_uptime_msec();
  static uint64_t next = 0;
  if(on)
  {
    GPIOE_BSRR = BIT_15;
    next = ms + 100ull;
  }
  else
  {
    if(ms >= next)
    {
      GPIOE_BSRR = BIT_31;
    }
  }
}


void cntl_worker(void)
{
  switch(gpsdostatus)
  {
    /* warmup: wait until the ocxo is ready */
    case warmup:
    {
      blink_nervous();

      /* measure the ocxo current; when the ocxo is warm, the current drops */
      start_conversion();
      float iocxo = get_iocxo();
      if(iocxo < OCXO_CURRENT_LIM)
      {
        ppsenable(true);

        /* if gnss is available, go to track/lock mode, otherwise holdover */
        if(check_fix())
        {
          gpsdostatus = track_lock;
        }
        else
        {
          gpsdostatus = holdover;
        }
      }
      break;
    }

    /* holdover mode: wait until the position is valid */
    case holdover:
    {
      blink_fast();

      if(check_fix())
      {
        gpsdostatus = track_lock;
      }
      break;
    }

    /* track/lock mode: control loop is active */
    case track_lock:
    {
      blink_slow(false);

      /* only look at the 1pps signal if the fix is valid; if fix is invalid,
         go to holdover mode */
      if(check_fix())
      {
        /* is it time to run the control loop? */
        if(pps_elapsed())
        {
          blink_slow(true);
          cntl();
        }
      }
      else
      {
        gpsdostatus = holdover;
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

static float read_tic(void)
{
  tdc_waitready();
  float tic = get_tic();
  float qerr = get_timepulse_error();
  enable_tdc();
  return tic - qerr + TIC_OFFSET;
}


static uint16_t pi_control(double KP, double KI, double damp, float e)
{
  static double esum = 32768.0;
  double P = e*KP;
  double I = P/(KI * damp);
  esum = esum + I;
  if(esum > 65535)
  {
    esum = 65535.0;
  }
  else if(esum < 0)
  {
    esum = 0.0;
  }
  float efc = P + esum;
  if(efc > 65535)
  {
    return 65535;
  }
  else if(efc < 0)
  {
    return 0;
  }
  else
  {
    return (uint16_t)efc;
  }
}


static void cntl(void)
{
  static uint32_t statuscount = 0;

  /* determine the time interval (phase) error */
  float tic = read_tic();

  /* TODO: read these values from the eeprom */
  e = tic - soll;

  /* this is somewhat hack-ish, but avoids using fabs() */
  float abs_err = e > 0 ? e : -e;

  /* this is required if the ocxo current consumption is to be measured */
  start_conversion();

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
        /* if the phase error is too large it takes too much time until the
           controller would settle, so we reset the timers. however this will
           give a sharp 'bump' in the frequency and phase. */
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
      double kp = 1.0/(OSCGAIN * 10);
      double ki = 10;
      double damp = 1.0;
      dacval = pi_control(kp, ki, damp, e);

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

      break;
    }

    /* locked: use an intermediate time constant after the ocxo has
       settled for a while. */
    case locked:
    {
      double kp = 1.0/(OSCGAIN * 100);
      double ki = 100;
      double damp = 1.0;
      dacval = pi_control(kp, ki, damp, e);

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

      break;
    }

    /* this should be the normal operating state. */
    case stable:
    {
      /* TODO: read these constants from the eeprom */
      double kp  = 1.0/(OSCGAIN * 250);
      double ki = 250;
      double damp = 2.0;
      dacval = pi_control(kp, ki, damp, e);

      /* if the phase error increases above the threshold MAX_PHASE_ERR, then
         the ocxo is perhaps not stable enough and the control loop switches
         its constants such that it becomes faster */
      if(abs_err > MAX_PHASE_ERR)
      {
        statuscount = 0;
        stat = locked;
      }

      break;
    }
  }

  set_dac(dacval);
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
