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

#define OSCGAIN 120.0/32768.0

/* ocxo current limit when warmed up */
#define OCXO_CURRENT_LIM 250.0f

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
  normal
} status_t;


typedef enum
{
  fast_track,
  locked,
  stable,

  init,
  lock1,
  lock2,
  lock3
} controlstatus_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

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


controlstatus_t mstatus = init;
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


static bool check_fix(void)
{
  uint64_t ms = get_uptime_msec();

  /* position, velocity and time info */
  extern gpsinfo_t pvt_info;
  uint8_t fix = pvt_info.fixtype;

  /* fix is only valid if it is a timing or 3d fix */
  if((fix == 3) || (fix == 5))
  {
    return true;
  }

  return false;
}


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

static float mtic(void)
{
  tdc_waitready();
  float tic = get_tic();
  float qerr = get_timepulse_error();
  enable_tdc();
  return tic - qerr;
}


static void cntl(void)
{

  static uint32_t statuscount = 0;


  float tic = mtic();
  float e = tic + 300 - soll;
  double KP;
  double KI;


  switch(stat)
  {
    case fast_track:
    {
      KP  = 1.0/(OSCGAIN * 10);
      KI = 10;

      if(fabs(e) < 100.0f)
      {
        statuscount++;
      }
      else
      {
        statuscount = 0;
      }

      if(statuscount >= 600)
      {
        statuscount = 0;
        stat = locked;
      }
      break;
    }

    case locked:
    {
      KP  = 1.0/(OSCGAIN * 100);
      KI = 100;

      if(fabs(e) < 20.0f)
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
      break;
    }

    case stable:
    {
      KP  = 1.0/(OSCGAIN * 200);
      KI = 200;

      if(fabs(e) > 10.0f)
      {
        statuscount = 0;
        stat = locked;
      }
      break;
    }
  }

  double P = e*KP;
  double I = P/KI;
  esum = esum + I;
  if(esum > 65535)
    esum = 65535;
  else if(esum < 0)
    esum = 0;
  efc = P + esum;
  if(efc > 65535)
    efc=65535;
  else if(efc < 0)
    efc = 0;
  set_dac(efc);
}



#if 0
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
        gpsdostatus = holdover;
      }
      break;
    }

    /* holdover mode: wait until the position is valid */
    case holdover:
    {
      blink_fast();

      if(check_fix())
      {
        gpsdostatus = normal;
      }
      break;
    }

    /* normal mode: control loop is active */
    case normal:
    {
      blink_slow(false);

      /* only look at the 1pps signal if the fix is valid */
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
#endif


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

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
