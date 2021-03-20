/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           header file
 *
 * Description:    driver for the 93C66 EEPROM
 *
 * Compiler:       ANSI-C
 *
 * Filename:       eeprom.h
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  14.04.2020
 *******************************************************************************
   Modification History:
   [1.0]    14.04.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

#ifndef __EEPROM_H__
#define __EEPROM_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

#define EEP_SZ 512u
#define CFG_VERSION 34u

/*******************************************************************************
 * MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

typedef union
{
  struct
  {
    uint8_t version;
    uint16_t last_dacval;
    bool use_gps;
    bool use_glonass;
    bool use_galileo;
    uint32_t rs232_baudrate;

    /* fixed positon data */
    bool fixpos_valid;
    uint32_t svin_dur;
    int32_t x; /* ecef coordinates */
    int32_t y;
    int32_t z;
    uint32_t accuracy;
    uint32_t accuracy_limit;

    bool auto_svin; /* automatically start survey in at boot time */

    /* navigation model */
    int8_t elevation_mask;

    /* control loop time constants */
    uint16_t tau;
    uint8_t filt;

    /* time offset to pps */
    int32_t timeoffset;
  };
  uint8_t bytes[EEP_SZ];
} config_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void eep_init(void);
/*------------------------------------------------------------------------------
  Function:
  initialise pin assignment
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern uint8_t eep_read(uint32_t addr);
/*------------------------------------------------------------------------------
  Function:
  read a byte from the e2prom at the specified address using a software emulated
  microwire protocol
  in:  addr -> address to read from
  out: byte read from the address
==============================================================================*/


/*============================================================================*/
extern void eep_write(uint32_t addr, uint8_t data);
/*------------------------------------------------------------------------------
  Function:
  write a byte to the e2prom
  in:  addr -> address to write
       data -> data to be written
  out: none
==============================================================================*/


/*============================================================================*/
extern void eep_chip_erase(void);
/*------------------------------------------------------------------------------
  Function:
  erase the whole e2prom
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void eep_read_multi(uint32_t addr, uint32_t len, void* buf);
/*------------------------------------------------------------------------------
  Function:
  read multiple bytes
  in:  addr -> start address
       len -> number of bytes to be read
       buf -> target buffer
  out: none
==============================================================================*/


/*============================================================================*/
extern void eep_write_multi(uint32_t addr, uint32_t len, void* buf);
/*------------------------------------------------------------------------------
  Function:
  write multiple bytes
  in:  addr -> start address
       len -> number of bytes
       buf -> buffer for the write data
  out: none
==============================================================================*/


/*============================================================================*/
extern void load_config(void);
/*------------------------------------------------------------------------------
  Function:
  reads the entire config from the eeprom, verifies the checksum and
  initialises default values if the checksum is wrong
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern void save_config(void);
/*------------------------------------------------------------------------------
  Function:
  calculates the checksum of the new configuration and stores it on the eeprom
  in:  none
  out: none
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif

