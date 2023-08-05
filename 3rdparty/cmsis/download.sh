#/bin/bash
VERSION=5.9.0

get () {
  curl --create-dirs https://raw.githubusercontent.com/ARM-software/CMSIS_5/$VERSION/$1 -o $1
}

get "LICENSE.txt"
get "CMSIS/Core/Include/cmsis_compiler.h"
get "CMSIS/Core/Include/cmsis_gcc.h"
get "CMSIS/Core/Include/cmsis_version.h"
get "CMSIS/Core/Include/core_cm4.h"
get "CMSIS/Core/Include/mpu_armv7.h"
