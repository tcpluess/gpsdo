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
#include "misc.h"
#include "rs232.h"
#include "nmea_output.h"

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


_ssize_t _write_r(struct _reent *r, int file, const void *ptr, size_t len)
{
  switch(file)
  {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case NMEA_FD:
    {
      const unsigned char *p = (const unsigned char*)ptr;
      for(size_t i = 0; i < len; i++)
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


int _isatty(int file)
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
  switch(file)
  {
    case STDIN_FILENO:
    {
      extern int kbhit(void);
      size_t numread;
      char* dest = (char*)ptr;

      for(numread = 0; numread < len; numread++)
      {
        int rxchar = kbhit();
        if(rxchar >= 0)
        {
          dest[numread] = (char)rxchar;
        }
        else
        {
          r->_errno = EIO;
        }
        break;
      }
      return numread;
    }

    case NMEA_FD:
    {
      return 0;
    }

    default:
    {
      r->_errno = EBADF;
      return -1;
    }
  }
  return 0;
}


int _close_r(struct _reent *r, int file)
{
  (void)r;
  (void)file;
  return 0;
}


int _open_r(struct _reent *ptr, const char *file, int flags, int mode)
{
  (void)file;
  (void)flags;
  (void)mode;
  ptr->_errno = ENODEV;
  return -1;
}

_off_t _lseek_r(struct _reent *r, int file, _off_t ptr, int dir)
{
  (void)r;
  (void)file;
  (void)ptr;
  (void)dir;
  /* always indicate we are at file beginning */
  return (_off_t)0;
}


//int _stat_r(
//    struct _reent *r,
//    int file,
//    struct stat *st)
int _stat_r(struct _reent *r, const char *file, struct stat *st)
{
  (void)r;
  (void)file;
  /* always set as character device */
  st->st_mode = S_IFCHR;
  return 0;
}


int _fstat_r(struct _reent *r, int file, struct stat *st)
{
  (void)r;
  (void)file;
  /* always set as character device */
  st->st_mode = S_IFCHR;
  return 0;
}


__attribute__((used))
int _getpid(void)
{
  TaskHandle_t current = xTaskGetCurrentTaskHandle();
  return uxTaskGetTaskNumber(current);
}


__attribute__((used))
int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = ENOTSUP;
  return -1;
}

void _exit(int status)
{
  (void)status;
  for(;;);
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
