/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the ADC1 of the STM32F407
 *
 * Filename:       datetime.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  21.12.2022
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "datetime.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

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

static bool isleapyear(void);
static void calc_dow(void);
static void resume(void);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static datetime_t tm;

const uint8_t monthdays[] =
{
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static const char* weekdays[] =
{
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

static const char* months[] =
{
  "Jan", "Feb", "Mar", "Apr",
  "May", "Jun", "Jul", "Aug",
  "Sep", "Oct", "Nov", "Dec"
};

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void datetime_init(void)
{
  vTaskSuspendAll();
  (void)memset(&tm, 0, sizeof(tm));
  tm.day = 1;
  tm.month = 1;
  tm.year = 2023;
  calc_dow();
  resume();
}


void datetime_tick(void)
{
  bool newday = false;

  vTaskSuspendAll();
  tm.sec++;
  if(tm.sec == 60)
  {
    tm.sec = 0;
    tm.min++;
  }
  if(tm.min == 60)
  {
    tm.min = 0;
    tm.hour++;
  }
  if(tm.hour == 24)
  {
    tm.hour = 0;
    tm.day++;
    newday = true;
  }

  uint8_t numdays = monthdays[tm.month - 1];
  if(isleapyear())
  {
    if(tm.month == 2)
    {
      numdays = 29;
    }
  }
  if(tm.day > numdays)
  {
    tm.day = 1;
    tm.month++;
  }
  if(tm.month == 13)
  {
    tm.month = 1;
    tm.year++;
  }

  if(newday)
  {
    calc_dow();
  }

  resume();
}


void datetime_set(uint8_t sec, uint8_t min, uint8_t hour,
                  uint8_t day, uint8_t month, uint16_t year)
{
  vTaskSuspendAll();
  tm.day = day;
  tm.month = month;
  tm.year = year;
  tm.sec= sec;
  tm.min = min;
  tm.hour = hour;
  calc_dow();
  resume();
}


void datetime_print(char* ret)
{
  datetime_t tmp;
  datetime_get(&tmp);

  const char* dow = weekdays[tmp.dow];
  const char* month = months[tmp.month - 1];
  (void)snprintf(ret, 30, "%s %s %02d %04d %02d:%02d:%02d UTC\n",
                 dow, month, tmp.day, tmp.year, tmp.hour, tmp.min, tmp.sec);
}


void datetime_get(datetime_t* ret)
{
  if(ret != NULL)
  {
    vTaskSuspendAll();
    (void)memcpy(ret, &tm, sizeof(tm));
    resume();
  }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/


/*============================================================================*/
static bool isleapyear(void)
/*------------------------------------------------------------------------------
  Function:
  returns true if the current year is a leap year.
  in:  none
  out: true if leap year, false otherwise.
==============================================================================*/
{
  if(tm.year % 4 != 0)
  {
    return false;
  }

  if(tm.year % 100 != 0)
  {
    return true;
  }

  if(tm.year % 400 != 0)
  {
    return false;
  }

  return true;
}


/*============================================================================*/
static void calc_dow(void)
/*------------------------------------------------------------------------------
  Function:
  calcualtes the day of week, 0 = sunday. algorithm is from
  Michael Keith and Tom Craver, 1990
  in:  none
  out: tm is updated with the current day of week
==============================================================================*/
{
  int d = tm.day;
  int m = tm.month;
  int y = tm.year;
  tm.dow = (d += m < 3 ? y-- : y - 2, 23*m/9 +
            d + 4 + y/4- y/100 + y/400)%7; /*lint !e834 !e732 */
}

/*============================================================================*/
static void resume(void)
/*------------------------------------------------------------------------------
  Function:
  resume blocked tasks
  in:  none
  out: none
==============================================================================*/
{
  if(xTaskResumeAll())
  {
    taskYIELD(); /*lint !e10 !e40 !e522 */
  }
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
