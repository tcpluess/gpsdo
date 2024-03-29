/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    checksum functions
 *
 * Filename:       checksum.h
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  20.04.2020
 ******************************************************************************/

#ifndef __CHECKSUM_H__
#define __CHECKSUM_H__

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

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
uint16_t fletcher16(const void* data, uint32_t len);
/*------------------------------------------------------------------------------
  Function:
  calculate the fletcher16 checksum over some data
  in:  data -> data of which the checksum is calculated
       len -> number of bytes
  out: fletcher16 checksum
==============================================================================*/


/*============================================================================*/
uint16_t crc16(const void* data, uint32_t len);
/*------------------------------------------------------------------------------
  Function:
  calculate the crc16 (polynomial 0xa001)
  in:  data -> data over which the checksum is calculated
       len -> number of bytes
  out: crc ccitt
==============================================================================*/


/*============================================================================*/
uint32_t crc32(const void* data, uint32_t len);
/*------------------------------------------------------------------------------
  Function:
  calculate the crc16 (polynomial 0xedb88320)
  in:  data -> data over which the checksum is calculated
       len -> number of bytes
  out: crc ccitt
==============================================================================*/

/*============================================================================*/
uint8_t nmea0183_checksum(const char* data, int len);
/*------------------------------------------------------------------------------
  Function:
  calculate the nmea0183 checksum
  in:  data -> data over which the checksum is calculated
       len -> number of bytes
  out: nmea0183 checksum
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif
