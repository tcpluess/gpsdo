/*******************************************************************************
 * Company:
 *
 * Project:        -
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    some generic macros
 *
 * Compiler:       ANSI-C
 *
 * Filename:       misc.h
 *
 * Version:        1.0
 *
 * Author:         Tobias Pluess
 *
 * Creation-Date:  19.08.2005
 *******************************************************************************
   Modification History:
   [1.0]    19.08.2005    Tobias Pluess
   - first release
 ******************************************************************************/

#ifndef __MISC_H__
#define __MISC_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

#define BIT_00 (1u << 0)
#define BIT_01 (1u << 1)
#define BIT_02 (1u << 2)
#define BIT_03 (1u << 3)
#define BIT_04 (1u << 4)
#define BIT_05 (1u << 5)
#define BIT_06 (1u << 6)
#define BIT_07 (1u << 7)
#define BIT_08 (1ul << 8)
#define BIT_09 (1ul << 9)
#define BIT_10 (1ul << 10)
#define BIT_11 (1ul << 11)
#define BIT_12 (1ul << 12)
#define BIT_13 (1ul << 13)
#define BIT_14 (1ul << 14)
#define BIT_15 (1ul << 15)
#define BIT_16 (1ul << 16)
#define BIT_17 (1ul << 17)
#define BIT_18 (1ul << 18)
#define BIT_19 (1ul << 19)
#define BIT_20 (1ul << 20)
#define BIT_21 (1ul << 21)
#define BIT_22 (1ul << 22)
#define BIT_23 (1ul << 23)
#define BIT_24 (1ul << 24)
#define BIT_25 (1ul << 25)
#define BIT_26 (1ul << 26)
#define BIT_27 (1ul << 27)
#define BIT_28 (1ul << 28)
#define BIT_29 (1ul << 29)
#define BIT_30 (1ul << 30)
#define BIT_31 (1ul << 31)
#define BIT_32 (1ull << 32)
#define BIT_33 (1ull << 33)
#define BIT_34 (1ull << 34)
#define BIT_35 (1ull << 35)
#define BIT_36 (1ull << 36)
#define BIT_37 (1ull << 37)
#define BIT_38 (1ull << 38)
#define BIT_39 (1ull << 39)
#define BIT_40 (1ull << 40)
#define BIT_41 (1ull << 41)
#define BIT_42 (1ull << 42)
#define BIT_43 (1ull << 43)
#define BIT_44 (1ull << 44)
#define BIT_45 (1ull << 45)
#define BIT_46 (1ull << 46)
#define BIT_47 (1ull << 47)
#define BIT_48 (1ull << 48)
#define BIT_49 (1ull << 49)
#define BIT_50 (1ull << 50)
#define BIT_51 (1ull << 51)
#define BIT_52 (1ull << 52)
#define BIT_53 (1ull << 53)
#define BIT_54 (1ull << 54)
#define BIT_55 (1ull << 55)
#define BIT_56 (1ull << 56)
#define BIT_57 (1ull << 57)
#define BIT_58 (1ull << 58)
#define BIT_59 (1ull << 59)
#define BIT_60 (1ull << 60)
#define BIT_61 (1ull << 61)
#define BIT_62 (1ull << 62)
#define BIT_63 (1ull << 63)

#ifndef NULL
#define NULL 0
#endif

#define PI 3.14159265f

/*******************************************************************************
 * OTHER DEFINITIONS (MACROS ETC)
 ******************************************************************************/

/* macro boundaries */
#define MACRO_START do {
#define MACRO_END } while(false)

/* determine the lower of two values */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* determine the higher of two values */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/* determine the absolute value of a variable or constant */
#define ABS(a) (((a) >= 0) ? (a) : (0 - (a)))

/* call a function and ignore its return value */
#define call (void)

/* unused parameters in functions */
#define unused(a) (void)a

/* number of elements */
#define numof_elements(a) (sizeof(a) / sizeof(a[0]))

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif
