/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    VT100 terminal interface functions with line editor
 *
 * Filename:       vt100.h
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  24.11.2022
 ******************************************************************************/

#ifndef __VT100_H__
#define __VT100_H__

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include <stdio.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT DEFINITIONS
 ******************************************************************************/

#ifndef VT100_MAX_LINELEN
#define VT100_MAX_LINELEN 80
#endif

#ifndef VT100_MAX_TOKENS
#define VT100_MAX_TOKENS 10
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

typedef int (*interpreter_func)(int argc, const char* const argv[]);

typedef struct
{
  char linebuffer[VT100_MAX_LINELEN];
  size_t cursor;
  size_t linelen;
  bool escape;
  char esc_code;
  bool insert;
  interpreter_func interpreter;
  FILE* out;
  FILE* in;
} vt100_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
extern void vt100_init(vt100_t* term, interpreter_func intr, FILE* out, FILE* in);
/*------------------------------------------------------------------------------
  Function:
  this is the worker "thread" for the console
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern int vt100_lineeditor(vt100_t* term);
/*------------------------------------------------------------------------------
  Function:
  this is the worker "thread" for the console
  in:  none
  out: none
==============================================================================*/


/*============================================================================*/
extern char* vt100_line(vt100_t* term);
/*------------------------------------------------------------------------------
  Function:
  this is the worker "thread" for the console
  in:  none
  out: none
==============================================================================*/

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/

#endif
