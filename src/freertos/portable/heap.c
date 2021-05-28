/*******************************************************************************
 * Company:
 *
 * Project:
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    implementation of the newlib syscalls such that freertos uses
 *                 the ordinary malloc.
 *
 * Compiler:       ANSI-C
 *
 * Filename:       syscalls.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  28.05.2021
 *******************************************************************************
   Modification History:
   [1.0]    28.05.2021    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdlib.h>
#include <malloc.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>


#include "FreeRTOS.h"
#if !defined(configUSE_NEWLIB_REENTRANT) ||  (configUSE_NEWLIB_REENTRANT != 1)
/* remove if really sure freertos newlib reentrancy support is not needed */
#error "#define configUSE_NEWLIB_REENTRANT 1 required for thread-safety \
of newlib sprintf, dtoa, strtok, etc..."
#endif
#include "task.h"

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

extern char heapsz; /* from the linker script */
static int heapBytesRemaining = (int)&heapsz;
static int totalBytesProvidedBySBRK = 0;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/


void __env_lock(void)
{
  vTaskSuspendAll();
}


void __env_unlock(void)
{
  (void)xTaskResumeAll();
}


void __malloc_lock(struct _reent* r)
{
  bool insideAnISR = xPortIsInsideInterrupt();

  /* no malloc inside isr! */
  configASSERT(!insideAnISR);
  vTaskSuspendAll();
}


void __malloc_unlock(struct _reent* r)
{
  (void)xTaskResumeAll();
}


void *pvPortMalloc(size_t size)
{
  void *p = malloc(size);
  return p;
}


void vPortFree(void* pv)
{
  free(pv);
}


size_t xPortGetFreeHeapSize(void)
{
  struct mallinfo mi = mallinfo();
  return mi.fordblks + heapBytesRemaining;
}


void* _sbrk_r(struct _reent* r, int incr)
{
  /* symbols from the linker script */
  extern char heapstart, heapend;
  static char *currentHeapEnd = &heapstart;
  char* limit = &heapend;

  vTaskSuspendAll();
  if(currentHeapEnd + incr > limit)
  {
    /* no more memory available */
#if(configUSE_MALLOC_FAILED_HOOK == 1)
    {
      extern void vApplicationMallocFailedHook(void);
      xTaskResumeAll();
      vApplicationMallocFailedHook();
    }
#elif defined(configHARD_STOP_ON_MALLOC_FAILURE)
    for(;;)
    {
      asm volatile ("bkpt #0");
    };
#else
    r->_errno = ENOMEM;
    xTaskResumeAll();
#endif
    return (void*)-1;
  }

  /* 'incr' of memory is available: update accounting and return it. */
  char *previousHeapEnd = currentHeapEnd;
  currentHeapEnd += incr;
  heapBytesRemaining -= incr;
  totalBytesProvidedBySBRK += incr;
  xTaskResumeAll();
  return (void*)previousHeapEnd;
}


/* non-reentrant sbrk uses is actually reentrant by using current context
   because the current _reent structure is pointed to by global _impure_ptr */
char* sbrk(int incr)
{
  return _sbrk_r(_impure_ptr, incr);
}


/* _sbrk is a synonym for sbrk */
char* _sbrk(int incr)
{
  return sbrk(incr);
}


/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
