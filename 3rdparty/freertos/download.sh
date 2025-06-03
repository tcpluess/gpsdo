#/bin/bash
VERSION=V11.2.0

get () {
  curl --create-dirs https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/$VERSION/$1 -o $1
}

get "event_groups.c"
get "LICENSE.md"
get "list.c"
get "queue.c"
get "stream_buffer.c"
get "tasks.c"
get "timers.c"
get "portable/GCC/ARM_CM4F/port.c"
get "portable/GCC/ARM_CM4F/portmacro.h"
get "include/atomic.h"
get "include/deprecated_definitions.h"
get "include/event_groups.h"
get "include/FreeRTOS.h"
get "include/list.h"
get "include/message_buffer.h"
get "include/mpu_prototypes.h"
get "include/mpu_wrappers.h"
get "include/newlib-freertos.h"
get "include/picolibc-freertos.h"
get "include/portable.h"
get "include/projdefs.h"
get "include/queue.h"
get "include/semphr.h"
get "include/stack_macros.h"
get "include/stream_buffer.h"
get "include/task.h"
get "include/timers.h"
