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
typedef enum
{
  init,
  lock1,
  lock2,
  lock3
} status_t;

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


status_t mstatus = init;
int loopcount = 0;
uint16_t dacval = 32768;

float soll = 0.0f;
uint64_t event = 0;



float tdc;
uint32_t tim;

extern svindata_t svinfo;

extern config_t cfg;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void cntl_worker(void)
{
  extern gpsinfo_t info;



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

                    /*do
      {*/
      do
      {
        if(tdc_check_irq())
        {
          break;
        }
      } while(1);
          tdc_int_ack();


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






      f = get_timepulse_error();
      /* obtain the phase error measured */
      float tic = get_tic()-f;



      /*if(fabs(tic - ticfilt) < 10.0f)
      {*/
       // ticfilt += (tic - ticfilt) * AVG;
      ticfilt = tic;
     // ticfilt = tic;
      //}

      /* enable the interpolator for the next measurement */
      enable_tdc();

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
      if(auto_disp)
        printf("%-12.3f %-7d %-d %-d %-f tic=%-f tdc=%-f iocxo=%f lat=%f lon=%f %f numsv=%d svinobs=%lu, act=%d mv=%lf valid=%d pdop=%d tacc=%lu\n", e, dacval,  mstatus, loopcount,f,tic,tdc,i,info.lat,info.lon, tmp, info.numsv, svinfo.obs, svinfo.active, sqrt(svinfo.meanv)/1000, svinfo.valid, info.pdop, info.tacc);
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
