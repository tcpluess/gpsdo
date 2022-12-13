/*******************************************************************************
 * Company:
 *
 * Project:
 *
 * Description:    C startup code for Cortex M4F, not suitable for C++
 *
 * Filename:       crt0.s
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 ******************************************************************************/

.syntax unified
.arch armv7-m
.thumb

/* this is only a very basic vector table; if more vectors are used, create a
   new vector table in the C code and use the VTOR register */
.section .vectors
.long   StackTop        /* initial sp */
.long   ResetHandler    /* reset entry point */

.text

.thumb_func
ResetHandler:

/* copy initialised data from flash to ram.
   not necessary if we run from ram anyways. */
#ifdef RUN_FROM_FLASH
    ldr    r0, =_sdata
    ldr    r1, =_etext
    ldr    r2, =_edata
    sub    r2, r0
    blx    memcpy
#endif


/* this fills the stack initially with some pattern; this allows one to find out
   the max. stack usage */
    ldr    r3, =heapend
    mov    sp, r3
    ldr    r0, =stackstart
    mov    r1, #0xdc
    ldr    r2, =StackTop
    sub    r2, r0
    blx    memset
    ldr    sp, =StackTop


/* fill the heap with some pattern. */
    ldr    r0, =heapstart
    mov    r1, #0x55
    ldr    r2, =heapend
    subs   r2, r2, r0
    blx    memset


/* this enables the FPU if necessary */
#if defined(__VFP_FP__) && !defined(__SOFTFP__)
    ldr    r0, =0xe000ed88
    ldr    r1, [r0]
    orr    r1, r1, #(0xf << 20)
    str    r1, [r0]
    dsb
    isb
#endif


/* to see how this works, look at newlib/libgloss/arm/crt0.S
   newlib does bss initialisation by itself, but not the copying from flash to
   ram. calling _start makes sure the c library is initialised correctly. */
    bl     _start
    bl     exit

    .end
