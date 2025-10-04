#/bin/bash
VERSION=6.2.0

get () {
  curl --create-dirs https://raw.githubusercontent.com/ARM-software/CMSIS_6/refs/tags/v$VERSION/$1 -o $1
}

get "LICENSE.txt"
get "CMSIS/Core/Include/cmsis_compiler.h"
get "CMSIS/Core/Include/cmsis_gcc.h"
get "CMSIS/Core/Include/cmsis_version.h"
get "CMSIS/Core/Include/core_cm4.h"
get "CMSIS/Core/Include/m-profile/cmsis_gcc_m.h"
get "CMSIS/Core/Include/m-profile/armv7m_mpu.h"
get "CMSIS/Core/Include/m-profile/armv7m_cachel1.h"

