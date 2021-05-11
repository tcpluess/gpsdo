/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    conversion functions for raw byte arrays <-> number formats
 *
 * Compiler:       ANSI-C
 *
 * Filename:       convert.h
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

#ifndef __CONVERT_H__
#define __CONVERT_H__

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

/* pseudo functions for unpack and pack: */
/* unsigned 8-bit - works directly on the buffer, but is nicer to have the same
   interface as for the other data types */
#define pack_u8_le(x, y, z) { x[y] = z; }
#define unpack_u8_le(x, y) ((uint8_t)x[y])

#define pack_i8_le(x, y, z) { x[y] = (uint8_t)z; }
#define unpack_i8_le(x, y) ((int8_t)x[y])

/* signed 16-bit - works the same as for unsigned, but has an embedded cast */
#define pack_i16_le(x, y, z) pack_u16_le(x, y, (uint16_t)(z))
#define unpack_i16_le(x, y) ((int16_t)unpack_u16_le(x, y))

/* signed 32-bit - works the same as for unsigned, but has an embedded cast */
#define pack_i32_le(x, y, z) pack_u32_le(x, y, (uint32_t)(z))
#define unpack_i32_le(x, y) ((int32_t)unpack_u32_le(x, y))

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void pack_u32_le(uint8_t* buffer, uint32_t offset, uint32_t value);
/*------------------------------------------------------------------------------
  Function:
  convert a unsigned 32-bit number to 4 bytes and store it in a buffer
  (little endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/


/*============================================================================*/
extern uint32_t unpack_u32_le(const uint8_t* data, uint32_t offset);
/*------------------------------------------------------------------------------
  Function:
  retrieve a unsigned 32 bit number from raw data (little endian)
  in:  data -> raw data
       offset -> offset into the buffer
  out: returns the number
==============================================================================*/


/*============================================================================*/
extern void pack_u16_le(uint8_t* buffer, uint32_t offset, uint16_t value);
/*------------------------------------------------------------------------------
  Function:
  convert a unsigned 16-bit number to 2 bytes and store it in a buffer
  (little endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/


/*============================================================================*/
extern uint16_t unpack_u16_le(const uint8_t* data, uint32_t offset);
/*------------------------------------------------------------------------------
  Function:
  retrieve a unsigned 16 bit number from raw data (little endian)
  in:  data -> raw data
       offset -> offset into the buffer
  out: returns the number
==============================================================================*/


/*============================================================================*/
extern void pack_u16_be(uint8_t* buffer, uint32_t offset, uint16_t value);
/*------------------------------------------------------------------------------
  Function:
  convert a unsigned 16-bit number to 2 bytes and store it in a buffer
  (big endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/


/*============================================================================*/
extern uint16_t unpack_u16_be(const uint8_t* data, uint32_t offset);
/*------------------------------------------------------------------------------
  Function:
  retrieve a unsigned 16 bit number from raw data (big endian)
  in:  data -> raw data
       offset -> offset into the buffer
  out: returns the number
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

