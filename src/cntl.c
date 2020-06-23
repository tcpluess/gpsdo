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
#include "adt7301.h"

#include "stm32f407.h"

#include <stdio.h>
#include <math.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define OSCGAIN (120.0/32768.0)

/* ocxo current limit when warmed up */
#define OCXO_CURRENT_LIM 210.0f /* approx. 2.5 Watts @ 12 V */

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

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/


uint64_t last_pps = 0;
uint32_t numpps = 0;
uint64_t ms;
double KP;
double KI;
float AVG;

double esum = 32768.0;
float e=0;
float efc;
float efcfilt = 32768.0f;

float f = 0.0f;

float ticfilt = 0.0f;
float ticfiltlast = 0.0f;


//controlstatus_t mstatus = init;
int loopcount = 0;
uint16_t dacval = 32768;

float soll = 0.0f;
uint64_t event = 0;



float tdc;
uint32_t tim;

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


static uint16_t pi_control(double KP, double KI, float e)
{
  static double esum = 32768.0;
  double P = e*KP;
  double I = P/KI;
  esum = esum + I;
  if(esum > 65535)
  {
    esum = 65535.0;
  }
  else if(esum < 0)
  {
    esum = 0.0;
  }
  efc = P + esum;
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
  float e = tic + 300 - soll;
  double abs_err = fabs(e);

  double kp, ki;
  uint16_t dacval;
  const char* status;

  /* if the phase error is too large it takes too much time until
     the controller would settle, so we reset the timers. however
     this will give a sharp 'bump' in the frequency and phase. */
  if(abs_err > 10000)
  {
    timebase_reset();
  }

  switch(stat)
  {
    /* fast track: is normally used shortly after the ocxo has just
       warmed up. use small time constant such that the phase settles
       more quickly */
    case fast_track:
    {
      kp  = 1.0/(OSCGAIN * 10);
      ki = 10;

      /* count for how many seconds the error stays small */
      if(abs_err < 100.0f)
      {
        statuscount++;
      }
      else
      {
        statuscount = 0;
      }

      /* if the error says small for the time specified, the
         controller time constants can be increased */
      if(statuscount >= 600)
      {
        statuscount = 0;
        stat = locked;
      }
      status = "fasttrack";
      break;
    }

    /* locked: use an intermediate time constant after the ocxo has
       settled for a while. */
    case locked:
    {
      kp  = 1.0/(OSCGAIN * 100);
      ki = 100;

      if(abs_err < 100.0f)
      {
        statuscount++;
      }
      else
      {
        statuscount = 0;
      }

      if(statuscount >= 1800)
      {
        cfg.last_dacval = dacval;
        statuscount = 0;
        stat = stable;
      }
      status = "locked";
      break;
    }

    /* this should be the normal operating state. */
    case stable:
    {
      /* TODO: read these constants from the eeprom */
      kp  = 1.0/(OSCGAIN * 400);
      ki = 400;

      if(abs_err > 100.0f)
      {
        statuscount = 0;
        stat = locked;
      }
      status = "stable";
      break;
    }
  }

  /* calculate the control value and set the dac accordingly */
  dacval = pi_control(kp, ki, e);
  set_dac(dacval);

  /* if auto display is enabled, print the status information */
  extern bool auto_disp;
  if(auto_disp)
  {
    float i = get_iocxo();
    float t = get_temperature();
    (void)printf("e=%.3f dac=%d iocxo=%.1f temp=%.1f status=%s\n", e, dacval, i, t, status);
  }
}



#if 1

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

#else


void cntl_worker(void)
{
  extern gpsinfo_t pvt_info;



  ms = get_uptime_msec();

    if(ms > event)
    {
      GPIOE_BSRR = BIT_31;
    }

    /* do nothing until a 1pps pulse has been registered */
    if(pps_elapsed())
    {
      last_pps = get_uptime_msec();
      GPIOE_BSRR = BIT_15;
      if((ms - event > 1200u) && (event != 0))
      {
        //printf("missed pps");
      }
      event = ms + 100u;
      start_conversion();


      loopcount++;



        switch(mstatus)
        {
          case init:
          {
            KP = 1.0/(OSCGAIN * 10);
            KI = 10;
            AVG = 1.0f;
            if(fabs(e) > 100.0)
            {
              loopcount=0;
            }
            else if(loopcount > 600)
            {
              cfg.last_dacval = dacval;
              mstatus = lock1;
              loopcount = 0;
            }
          }
          break;

          case lock1:
          {
            KP = 1.0/(OSCGAIN * 100);
            KI = 100;
            AVG = 1.0f;

            if(fabs(e) > 30.0)
            {
              loopcount = 0;
            }
            else if(loopcount > 3600)
            {
              cfg.last_dacval = dacval;
              mstatus = lock2;
              loopcount = 0;

             /* cfg.last_dacval = esum;
              save_config();*/
            }
          }
          break;

          case lock2:
          {
            KP = 1.0/(OSCGAIN*200);
            KI = 200;
            AVG = 1.0f;
            if(fabs(e)>10)
            {
              cfg.last_dacval = dacval;
              loopcount=0;
            }
            /*else if(loopcount > 1800)
            {
              mstatus = lock3;
              loopcount = 0;
            }*/
          }

          case lock3:
          {
            KP = 1.0/(OSCGAIN*2000);
            KI = 2000;
            AVG = 0.02f;
          }
          break;
        }







      /* obtain the phase error measured */
      float tic = mtic();



      /*if(fabs(tic - ticfilt) < 10.0f)
      {*/
       // ticfilt += (tic - ticfilt) * AVG;
      ticfilt = tic;
     // ticfilt = tic;
      //}

      /* enable the interpolator for the next measurement */


    if(fabs(tic) > 20000)
      {
        timebase_reset();
        return;
      }

      // THIS WOULD BE A QUICK AND DIRTY PI-CONTROLLER
      // 300 ns offset due to input synchronization logic!!!
      e = ticfilt + 300 - soll; //+ (ticfilt-ticfiltlast);
      ticfiltlast = ticfilt;

      double P = e*KP;
      double I = P/KI;
      esum = esum + I;
      if(esum > 65535)
        esum = 65535;
      else if(esum < 0)
        esum = 0;
      efc = P + esum;
      /*if(efcfilt > 65535)
        efcfilt = 65535;
      else if(efcfilt < 0)
        efcfilt = 0;*/
      if(efc > 65535)
        efc=65535;
      else if(efc < 0)
        efc = 0;
      //efcfilt = AVG*efc + (1.0f-AVG)*efcfilt;
      dacval = efc;
      set_dac(dacval);


      // do nothing but print the phase
      float i = get_iocxo();
      float tmp = get_temperature();

      if(fabs(e) < 20)
      {
        GPIOE_BSRR = BIT_14;
      }
      else
      {
        GPIOE_BSRR = BIT_30;
      }

      extern bool auto_disp;
      extern uint32_t tim;
      if(auto_disp)
      {
        //printf("%f %f %f\n", soll, ticfilt+300, e);
        printf("%-12.3f %-7d %-d %-d %d tic=%-f tdc=%-f iocxo=%f lat=%f lon=%f %f numsv=%d svinobs=%lu, act=%d mv=%lf valid=%d pdop=%d tacc=%lu\n", e, dacval,  mstatus, loopcount,tim,tic,tdc,i,pvt_info.lat,pvt_info.lon, tmp, pvt_info.numsv, svin_info.obs, svin_info.active, sqrt(svin_info.meanv)/1000, svin_info.valid, pvt_info.pdop, pvt_info.tacc);
      }
      //printf("%f %f %d %d %f\n", ticfilt+300, soll, loopcount, dacval, f);
      //printf("%f %f %f %f\n",mytdc,mtic,mret, e);

      //ubx_pvt(&info);




    }



   /* if((get_elapsed_ms() > 20) && qry)
    {
      GPIOE_BSRR = BIT_14;
      qry = false;

      GPIOE_BSRR = BIT_30;
    }*/
}

#endif

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

static float read_tic(void)
{
  tdc_waitready();
  float tic = get_tic();
  float qerr = get_timepulse_error();
  enable_tdc();
  return tic - qerr;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
