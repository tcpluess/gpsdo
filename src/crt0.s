/*******************************************************************************
 * Company:
 *
 * Project:
 *
 * Target:         Cortex M4F
 *
 * Type:           C startup code
 *
 * Description:    C startup code for Cortex M4F, not suitable for C++
 *
 * Compiler:       GNU AS
 *
 * Filename:       crt0.s
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  03.03.2020
 *******************************************************************************
   Modification History:
   [1.0]    03.03.2020    Tobias Plüss <tpluess@ieee.org>
   - created
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

/* copy initialised data from flash to ram */
    ldr    r1, =_etext
    ldr    r2, =_sdata
    ldr    r3, =_edata
    subs   r3, r2
    ble    1f
1:  subs   r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    1b
1:

/* this fills the stack initially with some pattern; this allows one to find out
   the max. stack usage */
    ldr    r0,=stackstart
    mov    r1,#0xdcdcdcdc
1:  cmp    sp,r0
    beq    1f
    str    r1,[r0]
    add    r0,#4
    b      1b
1:

/* fill the heap with some pattern. */
    ldr    r0,=heapstart
    ldr    r1,=heapend
    mov    r2,#0xaaaaaaaa
1:  cmp    r0, r1
    beq    1f
    str    r2, [r0]
    add    r0, #4
    b      1b
1:

/* this enables the FPU if necessary */
#ifdef __FPU_USED
    ldr    r0, =0xe000ed88
    ldr    r1, [r0]
    orr    r1, r1, #(0xf << 20)
    str    r1, [r0]
    dsb
    isb
#endif

/* clear the bss region */
    mov    r0, #0
    ldr    r1, =_sbss
    ldr    r2, =_ebss
1:  cmp    r1, r2
    beq    1f
    str    r0, [r1], #4
    b      1b
1:

    bl     main
    b      .

    .end
