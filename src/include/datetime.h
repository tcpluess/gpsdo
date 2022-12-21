/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    driver for the ADC1 of the STM32F407
 *
 * Filename:       datetime.h
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  21.12.2022
 ******************************************************************************/

#ifndef __DATETIME_H__
#define __DATETIME_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t dow; /* day of week, 0 = sunday */
  uint8_t day; /* day of month */
  uint8_t month;
  uint16_t year;
} datetime_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void datetime_init(void);
/*------------------------------------------------------------------------------
  Function:
  initialises the date and time to Jan 1, 2023, 00:00:00.
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void datetime_tick(void);
/*------------------------------------------------------------------------------
  Function:
  advances the current date and time by 1 second. updates the date and day of
  week accordingly.
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void datetime_set(uint8_t sec, uint8_t min, uint8_t hour,
                         uint8_t day, uint8_t month, uint16_t year);
/*------------------------------------------------------------------------------
  Function:
  sets the current date and time.
  in:  sec, min, hour, day, month, year -> self explanatory.
  out: none. internally sets the date and time.
==============================================================================*/


/*============================================================================*/
extern void datetime_print(char* ret);
/*------------------------------------------------------------------------------
  Function:
  prints the current date and time in the following format:
  Sun Jan 01 2023 00:00:10 UTC\n\0.
  in:  ret -> buffer to store the result string. must have at least
       30 characters capacity.
  out: none. the buffer pointed to by ret is populated with the current date
       and time in human readable format.
==============================================================================*/


/*============================================================================*/
extern void datetime_get(datetime_t* ret);
/*------------------------------------------------------------------------------
  Function:
  get a datetime struct with the current date and time.
  in:  ret -> pointer to a datetime struct.
  out: none. the struct pointed to by ret is populated with the current date
       and time.
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

