/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         STM32F407VE
 *
 * Type:           module
 *
 * Description:    main module
 *
 * Compiler:       ANSI-C
 *
 * Filename:       main.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "stm32f407.h"
#include "misc.h"
#include "adt7301.h"

#include "rs232.h"
#include "dac.h"
#include "tdc.h"
#include "timebase.h"
#include "vic.h"
#include "adc.h"
#include "ublox.h"

#include <math.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void led_setup(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/


#define OSCGAIN 120.0/32768.0

uint32_t over;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

typedef enum
{
  init,
  wait_ready,
  lock1,
  lock2,
  lock3
} status_t;

extern void lea_init(void);

float tdc;
uint32_t tim;

extern svindata_t svinfo;

int main(void)
{
  vic_init();
  led_setup();
  timebase_init();
  ublox_init();
  rs232_init();
  dac_setup();
  tmp_init();
  setup_tdc();
  adc_init();

 /* GPIOD_BSRR = BIT_10;
  while(1)
  {
    if(USART2_SR & BIT_05)
      USART3_DR = USART2_DR;
    if(USART3_SR & BIT_05)
      USART2_DR = USART3_DR;
  }*/
over=0;

  /* this is for debug only; enable the pps output such that it can be used
     as trigger signal for the scope */
  ppsenable(true);
timebase_reset();
  /* enable the interpolator */
  enable_tdc();

double KP;
double KI;
float AVG;

  double esum = 32767.0f;
  float e;
  float efc;
  //float efcfilt = 32768.0f;

  float f = 0.0f;

  float ticfilt = 0.0f;
  float ticfiltlast = 0.0f;


  status_t mstatus = init;
  int loopcount = 0;
  uint16_t dacval = 32768;

  float soll = 1000.0f;



extern gpsinfo_t info;

  while(1)
  {

    gps_worker();
    /* do nothing until a 1pps pulse has been registered */
    if(pps_elapsed())
    {
      start_conversion();



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
            }
          }
          break;

          case lock2:
          {
            KP = 1.0/(OSCGAIN*1000);
            KI = 1000;
            AVG = 1.0f;
            /*if(fabs(e)>10)
            {
              loopcount=0;
            }
            else if(loopcount > 1800)
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
      /*efcfilt = AVG*efc + (1.0f-AVG)*efcfilt;
      if(efcfilt > 65535)
        efcfilt = 65535;
      else if(efcfilt < 0)
        efcfilt = 0;*/
      if(efc > 65535)
        efc=65535;
      else if(efc < 0)
        efc = 0;
      dacval = efc;
      set_dac(dacval);


      // do nothing but print the phase
      float i = get_iocxo();
      float tmp = get_temperature();

      printf("%-12.3f %-7d %-d %-d %-f tic=%-f tdc=%-f iocxo=%f lat=%f lon=%f %f numsv=%d svinobs=%lu, act=%d mv=%f valid=%d over=%lu\n", e, dacval,  mstatus, loopcount,f,tic,tdc,i,info.lat,info.lon, tmp, info.numsv, svinfo.obs, svinfo.active, svinfo.meanv, svinfo.valid, over);
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
  return 0;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

static void led_setup(void)
{
  RCC_AHB1ENR |= BIT_04;
  GPIOE_MODER |= (1u << 28) | (1u << 30);
  GPIOE_BSRR = BIT_30 | BIT_31;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
