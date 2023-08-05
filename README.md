Required software
=================

* GCC: `arm-none-eabi-gcc`
* Debugger: `Segger Ozone`

Build: `make` generates the output files in the `bin` directory, in the
`elf`, `bin`, `hex` and `s19` formats.

Change `RUN_FROM_FLASH` in the makefile to switch between a ROM and a RAM
build. This also automatically changes the optimisation level.

Recommended packages for Sublime Text
=====================================

Highly recommended:
* `EditorConfig` to ensure that line endings, tabs and so on are unified

Fancy:
* `GCC Assembly Listing` for syntax highlighting of the listing file
* `Linker Script Syntax` for syntax highlighting of linker scripts
* `Intel HEX` for nicer display of the `.hex` file
* `SREC` for nicer display of the `.s19` file
* `MAPListing` for nicer display of the map file (`https://github.com/abcminiuser/sublimetext-gnu-map`)

Header Files
============

* CMSIS Header files from `https://github.com/ARM-software/CMSIS_5.git`
* STM32 Device Header files from `https://github.com/modm-io/cmsis-header-stm32.git`

Push to Github
==============

Normally, my own Git server is used (`https://hb9fsx.ch/git`) but to push the
repo also to Github, use:
`git push git@github.com:tcpluess/gpsdo.git --force`
