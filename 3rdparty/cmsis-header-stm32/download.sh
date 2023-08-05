#/bin/bash

get () {
  curl --create-dirs https://raw.githubusercontent.com/modm-io/cmsis-header-stm32/master/$1 -o $1
}

get "License.md"
get "stm32f4xx/Include/system_stm32f4xx.h"
get "stm32f4xx/Include/stm32f407xx.h"
