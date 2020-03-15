/*******************************************************************************
 * Company:
 *
 * Project:
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    some necessary functions for C standard library calls (
 *                 e.g. malloc).
 *
 * Compiler:       ANSI-C
 *
 * Filename:       syscalls.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  04.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    04.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created. initial version only with sbrk
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <errno.h>
#include <sys/stat.h>

#include "stm32f407.h"
#include "misc.h"

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

caddr_t _sbrk_r(struct _reent *r, int incr)
{
  /* heap start and end come from the linker script */
  extern char heapstart asm("heapstart");
  extern char heapend asm("heapend");

  static char* heap_top = &heapstart;
  char* prev_heap_top = heap_top;
  char* new_top = heap_top + incr;

  if(new_top > &heapend)
  {
    /* Some of the libstdc++-v3 tests rely upon detecting
      out of memory errors, so do not abort here.  */
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_top = new_top;

  return (caddr_t)prev_heap_top;
}



int _close_r(
    struct _reent *r,
    int file)
{
  return 0;
}

_off_t _lseek_r(
    struct _reent *r,
    int file,
    _off_t ptr,
    int dir)
{
  return (_off_t)0; /*  Always indicate we are at file beginning. */
}

int _fstat_r(
    struct _reent *r,
    int file,
    struct stat *st)
{
  /*  Always set as character device.       */
  st->st_mode = S_IFCHR;
    /* assigned to strong type with implicit  */
    /* signed/unsigned conversion.  Required by   */
    /* newlib.          */

  return 0;
}




/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
