#include <stdlib.h>
#include <malloc.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>


#include "FreeRTOS.h"
#if !defined(configUSE_NEWLIB_REENTRANT) ||  (configUSE_NEWLIB_REENTRANT!=1)
  #warning "#define configUSE_NEWLIB_REENTRANT 1 // Required for thread-safety of newlib sprintf, dtoa, strtok, etc..."
  // If you're *REALLY* sure you don't need FreeRTOS's newlib reentrancy support, comment out the above warning...
#endif
#include "task.h"


extern char heapsz; /* from the linker script */
static int heapBytesRemaining = (int)&heapsz;



void __env_lock(void)
{
  vTaskSuspendAll();
}

void __env_unlock(void)
{
  (void)xTaskResumeAll();
}


void __malloc_lock(struct _reent *r)
{
  bool insideAnISR = xPortIsInsideInterrupt();
  configASSERT(!insideAnISR); // Make damn sure no more mallocs inside ISRs!!
  vTaskSuspendAll();
}

void __malloc_unlock(struct _reent *r)
{
  (void)xTaskResumeAll();
}


void *pvPortMalloc(size_t xSize)
{
  void *p = malloc(xSize);
  return p;
}



void vPortFree(void *pv)
{
  free(pv);
}

static int totalBytesProvidedBySBRK = 0;

size_t xPortGetFreeHeapSize(void)
{
    struct mallinfo mi = mallinfo(); // available space now managed by newlib
    return mi.fordblks + heapBytesRemaining; // plus space not yet handed to newlib by sbrk
}




extern char heapstart, heapend;  // symbols from linker LD command file

void* _sbrk_r(struct _reent *pReent, int incr)
{
    static char *currentHeapEnd = &heapstart;
    char* limit = &heapend;
    vTaskSuspendAll();
    if(currentHeapEnd + incr > limit)
    {
        // Ooops, no more memory available...
        #if( configUSE_MALLOC_FAILED_HOOK == 1 )
          {
            extern void vApplicationMallocFailedHook( void );
            xTaskResumeAll();
            vApplicationMallocFailedHook();
          }
        #elif defined(configHARD_STOP_ON_MALLOC_FAILURE)
            // If you want to alert debugger or halt...
            // WARNING: brkpt instruction may prevent watchdog operation...
            while(1) { __asm("bkpt #0"); }; // Stop in GUI as if at a breakpoint (if debugging, otherwise loop forever)
        #else
            // Default, if you prefer to believe your application will gracefully trap out-of-memory...
            pReent->_errno = ENOMEM; // newlib's thread-specific errno
            xTaskResumeAll();
        #endif
        return (char *)-1; // the malloc-family routine that called sbrk will return 0
    }
    // 'incr' of memory is available: update accounting and return it.
    char *previousHeapEnd = currentHeapEnd;
    currentHeapEnd += incr;
    heapBytesRemaining -= incr;
    #ifndef NDEBUG
        totalBytesProvidedBySBRK += incr;
    #endif
    xTaskResumeAll();
    return (char *) previousHeapEnd;
}

//! non-reentrant sbrk uses is actually reentrant by using current context
// ... because the current _reent structure is pointed to by global _impure_ptr
char* sbrk(int incr)
{
  return _sbrk_r(_impure_ptr, incr);
}

//! _sbrk is a synonym for sbrk.
char* _sbrk(int incr)
{
  return sbrk(incr);
}

