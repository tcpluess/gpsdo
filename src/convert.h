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
#define pack_u8_le(x, y, z) x[y] = z
#define unpack_u8_le(x, y) x[y]

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
  convert a unsigned 32-bit number to 4 bytes and store it in a buffer
  (little endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/


/*============================================================================*/
extern void pack_u8_le6_le(uint8_t* buffer, uint32_t offset, uint16_t value);
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
extern uint16_t unpack_u8_le6_le(const uint8_t* data, uint32_t offset);
/*------------------------------------------------------------------------------
  Function:
  convert a unsigned 32-bit number to 4 bytes and store it in a buffer
  (little endian)
  in:  buffer -> buffer to put the bytes into
       offset -> offset into the buffer
       value -> value to be packed
  out: none
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

