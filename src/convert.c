/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    conversion functions for raw byte arrays <-> number formats
 *
 * Compiler:       ANSI-C
 *
 * Filename:       convert.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  11.04.2020
 *******************************************************************************
   Modification History:
   [1.0]    11.04.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "convert.h"

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

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void pack_u32_le(uint8_t* buffer, uint32_t offset, uint32_t value)
{
  buffer[offset] = (uint8_t)value;
  value >>= 8;
  buffer[offset + 1] = (uint8_t)value;
  value >>= 8;
  buffer[offset + 2] = (uint8_t)value;
  value >>= 8;
  buffer[offset + 3] = (uint8_t)value;
}


uint32_t unpack_u32_le(const uint8_t* data, uint32_t offset)
{
  uint32_t ret = data[offset + 3];
  ret <<= 8;
  ret += data[offset + 2];
  ret <<= 8;
  ret += data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}


void pack_u16_le(uint8_t* buffer, uint32_t offset, uint16_t value)
{
  buffer[offset] = (uint8_t)value;
  value >>= 8;
  buffer[offset + 1] = (uint8_t)value;
}


uint16_t unpack_u16_le(const uint8_t* data, uint32_t offset)
{
  uint16_t ret = data[offset + 1];
  ret <<= 8;
  ret += data[offset];
  return ret;
}


void pack_u16_be(uint8_t* buffer, uint32_t offset, uint16_t value)
{
  buffer[offset] = value >> 8;
  buffer[offset + 1] = (uint8_t)value;
}


uint16_t unpack_u16_be(const uint8_t* data, uint32_t offset)
{
  uint16_t ret = data[offset];
  ret <<= 8;
  ret += data[offset + 1];
  return ret;
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
