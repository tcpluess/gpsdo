Required software
=================

* GCC: `arm-none-eabi-gcc`
* Debugger: `Segger Ozone`

Build: `make` generates the output files in the `bin` directory, in the
`elf`, `bin`, `hex` and `s19` formats.

Change `RUN_FROM_FLASH` in the makefile to switch between a ROM and a RAM
build. This also automatically changes the optimisation level. Note that
a `make clean`/`make` sequence is required if any changes have been
made to the makefile.

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
