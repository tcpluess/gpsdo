/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    checksum functions
 *
 * Compiler:       ANSI-C
 *
 * Filename:       checksum.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  20.04.2020
 *******************************************************************************
   Modification History:
   [1.0]    20.04.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "checksum.h"

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define POLY_CRC16 0xA001u
#define POLY_CRC32 0xEDB88320

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static inline uint16_t calc_crc16(uint32_t crc, uint8_t data, uint16_t poly);
static inline uint32_t calc_crc32(uint32_t crc, uint8_t data, uint32_t poly);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

uint16_t fletcher16(const void* data, uint32_t len)
{
  uint16_t lo = 0;
  uint16_t hi = 0;

  const uint8_t* tmp = (const uint8_t*)data;

  for(uint32_t i = 0; i < len; i++)
  {
    lo = (lo + tmp[i]) % 255;
    hi = (hi + lo) % 255;
  }

  uint16_t result = hi;
  result <<= 8;
  result += lo;
  return result;
}


uint16_t crc16(const void* data, uint32_t len)
{
  const uint8_t* tmp = (const uint8_t*)data;
  uint16_t crc = 0xffffu;
  for(uint32_t i = 0; i < len; i++)
  {
    crc = calc_crc16(crc, tmp[i], POLY_CRC16);
  }
  crc ^= 0xffffu;
  return crc;
}


uint32_t crc32(const void* data, uint32_t len)
{
  const uint8_t* tmp = (const uint8_t*)data;
  uint32_t crc = 0xffffffffu;
  for(uint32_t i = 0; i < len; i++)
  {
    crc = calc_crc32(crc, tmp[i], POLY_CRC32);
  }
  crc ^= 0xffffffffu;
  return crc;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static inline uint16_t calc_crc16(uint32_t crc, uint8_t data, uint16_t poly)
/*------------------------------------------------------------------------------
  Function:
  calculate the crc16 of one byte
  in:  crc -> previous crc value
       data -> a byte
       poly -> crc polynomial to be used
  out: new crc value
==============================================================================*/
{
  for(int i = 0; i < 8; i++)
  {
    if((crc ^ data) & 1)
    {
      crc = (crc >> 1) ^ poly;
    }
    else
    {
      crc = crc >> 1;
    }

    data = data >> 1;
  }
  return (uint16_t)crc;
}

/*============================================================================*/
static inline uint32_t calc_crc32(uint32_t crc, uint8_t data, uint32_t poly)
/*------------------------------------------------------------------------------
  Function:
  calculate the crc32 of one byte
  in:  crc -> previous crc value
       data -> a byte
       poly -> crc polynomial to be used
  out: new crc value
==============================================================================*/
{
  for(int i = 0; i < 8; i++)
  {
    if((crc ^ data) & 1)
    {
      crc = (crc >> 1) ^ poly;
    }
    else
    {
      crc = crc >> 1;
    }

    data = data >> 1;
  }
  return crc;
}


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
