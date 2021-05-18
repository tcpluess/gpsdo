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
#include <sys/times.h>
#include <sys/unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f407.h"
#include "misc.h"
#include "rs232.h"

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

//caddr_t _sbrk_r(struct _reent *r, int incr)
void* _sbrk_r(struct _reent *r, ptrdiff_t incr)
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
    r->_errno = ENOMEM;
    return (void*) -1;
  }

  heap_top = new_top;

  return (void*)prev_heap_top;
}

_ssize_t _write_r(struct _reent *r, int file, const void *ptr, size_t len)
{
  switch(file)
  {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    {
      const unsigned char *p = (const unsigned char*)ptr;
      for(int i = 0; i < len; i++)
      {
        if(*p == '\n')
        {
          txchar('\r');
        }
        txchar(*p++);
      }
      return len;
    }

    default:
    {
      r->_errno = EBADF;
      return -1;
    }
  }
}

int
_isatty(int file)
{
  switch (file)
  {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
      return 1;

    default:
      // errno = ENOTTY;
      errno = EBADF;
      return -1;
  }
}


_ssize_t _read_r(struct _reent *r, int file, void *ptr, size_t len)
{
#if 0
  char c;
  int  i;
  unsigned char *p;

  p = (unsigned char*)ptr;

  for (i = 0; i < len; i++)
  {
    //c = uart0GetchW();

    *p++ = c;
    //uart0Putch(c);

    if (c == 0x0D && i <= (len - 2))
    {
      *p = 0x0A;
      //uart0Putch(0x0A);
      return i + 2;
    }
  }
  return i;
#endif
  return 0;
}

int _close_r(struct _reent *r, int file)
{
  return 0;
}

int
_open_r(struct _reent *ptr, const char *file, int flags, int mode)
{
  ptr->_errno = ENODEV;
  return -1;
}

_off_t _lseek_r(
    struct _reent *r,
    int file,
    _off_t ptr,
    int dir)
{
  return (_off_t)0; /*  Always indicate we are at file beginning. */
}


//int _stat_r(
//    struct _reent *r,
//    int file,
//    struct stat *st)
int _stat_r(struct _reent *ptr, const char *file, struct stat *st)
{
  /*  Always set as character device.       */
  st->st_mode = S_IFCHR;
    /* assigned to strong type with implicit  */
    /* signed/unsigned conversion.  Required by   */
    /* newlib.          */

  return 0;
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


int
_getpid(void)
{
  TaskHandle_t current = xTaskGetCurrentTaskHandle();
  return uxTaskGetTaskNumber(current);
}


int _kill(int pid, int sig)
{
  errno = ENOTSUP;
  return -1;
}

void
_exit(int status)
{
  while (1) {
    ;
  }
}


void __malloc_lock(struct _reent *p)
{
  vTaskSuspendAll();
}

void __malloc_unlock(struct _reent *p)
{
  (void)xTaskResumeAll();
}

void __env_lock(void)
{
  vTaskSuspendAll();
}

void __env_unlock(void)
{
  (void)xTaskResumeAll();
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
