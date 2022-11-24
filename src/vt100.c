/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Target:         any
 *
 * Type:           module
 *
 * Description:    simple terminal based console
 *
 * Compiler:       ANSI-C
 *
 * Filename:       console.c
 *
 * Version:        1.0
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  20.04.2020
 *******************************************************************************
   Modification History:
   [1.0]    20.04.2020    Tobias Plüss <tpluess@ieee.org>
   - created
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "vt100.h"

#include <string.h>
#include <ctype.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define ANSI_NUL 0x00
#define ANSI_SOH 0x01
#define ANSI_STX 0x02
#define ANSI_ETX 0x03
#define ANSI_EOT 0x04
#define ANSI_ENQ 0x05
#define ANSI_ACK 0x06
#define ANSI_BEL 0x07
#define ANSI_BS 0x08
#define ANSI_HT 0x09
#define ANSI_LF 0x0A
#define ANSI_VT 0x0B
#define ANSI_FF 0x0C
#define ANSI_CR 0x0D
#define ANSI_SO 0x0E
#define ANSI_SI 0x0F
#define ANSI_DLE 0x10
#define ANSI_DC1 0x11
#define ANSI_DC2 0x12
#define ANSI_DC3 0x13
#define ANSI_DC4 0x14
#define ANSI_NAK 0x15
#define ANSI_SYN 0x16
#define ANSI_ETB 0x17
#define ANSI_CAN 0x18
#define ANSI_EM 0x19
#define ANSI_SUB 0x1A
#define ANSI_ESC 0x1B
#define ANSI_FS 0x1C
#define ANSI_GS 0x1D
#define ANSI_RS 0x1E
#define ANSI_US 0x1F
#define ANSI_DEL 0x7F

#define ESC_BRACKET '['
#define ESC_O 'O'
#define ESC_HOME 'H'
#define ESC_END 'E'
#define ESC_DEL 'D'
#define ESC_INS 'I'
#define ESC_PGUP '5'
#define ESC_PGDOWN '6'

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void insertchar(vt100_t* term, char c);
static void cntlchar(vt100_t* term, char c);
static void backspace(vt100_t* term, uint32_t num);
static void delete(vt100_t* term, uint32_t num);
static void term_cursorleft(vt100_t* term, uint32_t num);
static void term_cursorright(vt100_t* term, uint32_t num);
static bool handle_esc(vt100_t* term, char c);
static void key_end(vt100_t* term);
static void key_del(vt100_t* term);
static void key_home(vt100_t* term);
static void key_left(vt100_t* term);
static void key_right(vt100_t* term);
static void key_bksp(vt100_t* term);
static void key_ctrl_u(vt100_t* term);
static void key_ctrl_k(vt100_t* term);
static void key_enter(vt100_t* term);
static int find_tokens(char* line, const char** toks, int maxtoks);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

/*============================================================================*/
void vt100_init(vt100_t* term, interpreter_func interpreter, FILE* out, FILE* in)
/*------------------------------------------------------------------------------
  Function:
  the line editor works similar to gnu's libreadline.
  in:  noen
  out: none
==============================================================================*/
{
  memset(term, 0, sizeof(vt100_t));
  term->interpreter = interpreter;
  term->out = out;
  term->in = in;
  (void)setvbuf(out, NULL, _IONBF, 0);
  (void)setvbuf(in, NULL, _IONBF, 0);
}


/*============================================================================*/
int vt100_lineeditor(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  the line editor works similar to gnu's libreadline.
  in:  noen
  out: none
==============================================================================*/
{
  for(;;)
  {
    int rx = fgetc(term->in);

    /* wrong character */
    if(rx < 0)
    {
      return -1;
    }

    /* if the escape character is received, process escape sequence accordingly */
    if(term->escape)
    {
      if(handle_esc(term, rx))
      {
        term->escape = false;
      }
      continue;
    }

    /* if enter is pressed, terminate the entered string; also the line editor
       is complete */
    if((rx == ANSI_CR) || (rx == ANSI_LF))
    {
      key_enter(term);

      const char* toks[MAX_TOKENS];
      int ntoks = find_tokens(term->linebuffer, &toks[0], MAX_TOKENS);
      int ret = term->interpreter(ntoks, toks);
      //TODO: use E2BIG in find_tokens
      return ret;
    }

    if(iscntrl(rx))
    {
      cntlchar(term, rx);
    }
    else
    {
      insertchar(term, rx);
    }
  }
}


/*============================================================================*/
char* vt100_line(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  the line editor works similar to gnu's libreadline.
  in:  noen
  out: none
==============================================================================*/
{
  return term->linebuffer;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void backspace(vt100_t* term, uint32_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if(term->cursor >= num)
  {
    memmove(term->linebuffer + term->cursor - num,
            term->linebuffer + term->cursor,
            term->linelen - term->cursor + num);
    term->cursor -= num;
    term->linelen -= num;
    term->linebuffer[term->linelen] = '\0';
  }
}


/*============================================================================*/
static void delete(vt100_t* term, uint32_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if((term->linelen != 0) && (term->cursor < term->linelen))
  {
    memmove(term->linebuffer + term->cursor,
            term->linebuffer + term->cursor + num,
            term->linelen - term->cursor + num);
    term->linelen -= num;
    term->linebuffer[term->linelen] = '\0';
  }
}


/*============================================================================*/
static void term_cursorleft(vt100_t* term, uint32_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  (void)term;
  if(num != 0)
  {
    (void)fprintf(term->out, "\033[%ldD", num);
  }
}


/*============================================================================*/
static void term_cursorright(vt100_t* term, uint32_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  (void)term;
  if(num != 0)
  {
    (void)fprintf(term->out, "\033[%ldC", num);
  }
}


/*============================================================================*/
static bool handle_esc(vt100_t* term, char c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  /* esc codes start with the bracket or with O */
  if(c == '[')
  {
    term->esc_code = ESC_BRACKET;
    return false;
  }
  else if(c == 'O')
  {
    term->esc_code = ESC_O;
    return false;
  }

  if(c == ANSI_DEL)
  {
    /* TODO */
    return true;
  }

  if(term->esc_code == ESC_BRACKET)
  {
    /* handle esc codes starting with the bracket */
    switch(c)
    {
      /* arrow up: [A */
      case 'A':
      {
        return true;
      }

      /* arrow down [B */
      case 'B':
      {
        return true;
      }

      /* arrow right [C */
      case 'C':
      {
        key_right(term);
        return true;
      }

      /* arrow left [D */
      case 'D':
      {
        key_left(term);
        return true;
      }

      /* end key alternative [E */
      case 'F':
      {
        key_end(term);
        return true;
      }

      /* home key alternative [H */
      case 'H':
      {
        key_home(term);
        return true;
      }

      /* home key [1~ */
      case '1':
      {
        term->esc_code = ESC_HOME;
        return false;
      }

      /* insert key [2~ */
      case '2':
      {
        term->esc_code = ESC_INS;
        return false;
      }

      /* delete key [3~ */
      case '3':
      {
        term->esc_code = ESC_DEL;
        return false;
      }

      /* page up [5~ */
      case '5':
      {
        term->esc_code = ESC_PGUP;
        return false;
      }

      /* page down [6~ */
      case '6':
      {
        term->esc_code = ESC_PGDOWN;
        return false;
      }

      /* unknown esc code */
      default:
      {
        return true;
      }
    }
  }
  else if(term->esc_code == ESC_O)
  {
    /* handle esc codes starting with O */
    switch(c)
    {
      /* end key OF */
      case 'F':
      {
        key_end(term);
        return true;
      }

      case 'o': /* forward slash with numlock off Oo */
      case 'j': /* asterisk with numlock off Oj */
      case 'm': /* minus with numlock off Om */
      case 'k': /* plus with numlock off Ok */
      case 'E': /* 5 with numlock off Oe */
      case 'P': /* F1 OP */
      case 'Q': /* F2 OQ */
      case 'R': /* F3 OR */
      case 'S': /* F4 OS */
      case 'M': /* enter with numlock off OM */
      {
        return true;
      }
    }

  }

  /* some of the esc codes are terminated with the tilde */
  if(c == '~')
  {
    switch(term->esc_code)
    {
      case ESC_HOME:
      {
        key_home(term);
        return true;
      }

      case ESC_END:
      {
        key_end(term);
        return true;
      }

      case ESC_DEL:
      {
        key_del(term);
        return true;
      }

      case ESC_INS:
      {
        term->insert = !term->insert;
        return true;
      }

      case ESC_PGUP:
      {
        return true;
      }

      case ESC_PGDOWN:
      {
        return true;
      }
    }
  }

  return true;
}


/*============================================================================*/
static void cntlchar(vt100_t* term, char c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  switch(c)
  {
    /* escape character */
    case ANSI_ESC:
    {
      term->escape = true;
      break;
    }

    /* single backspace */
    case ANSI_BS:
    {
      key_bksp(term);
      break;
    }

    /* delete to the beginning of the line (ctrl+u) */
    case ANSI_NAK:
    {
      key_ctrl_u(term);
      break;
    }

    /* delete to the end of the line (ctrl+k) */
    case ANSI_VT:
    {
      key_ctrl_k(term);
      break;
    }

    /* jump to end of line (ctrl+e) */
    case ANSI_ENQ:
    {
      key_end(term);
      break;
    }

    /* jump to begin of line (ctrl+a) */
    case ANSI_SOH:
    {
      key_home(term);
      break;
    }

    /* move cursor right (ctrl+f) */
    case ANSI_ACK:
    {
      key_right(term);
      break;
    }

    /* move cursor left (ctrl+b) */
    case ANSI_STX:
    {
      key_left(term);
      break;
    }

    /* delete character (ctrl+d) */
    case ANSI_EOT:
    {
      key_del(term);
      break;
    }
  }
}


/*============================================================================*/
static void insertchar(vt100_t* term, char c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if((term->insert) && (term->cursor < term->linelen))
  {
    term->linebuffer[term->cursor] = c;
    fputc(c, stdout);
    term->cursor++;
    return;
  }
  if(term->cursor < MAX_LINELEN - 2)
  {
    memmove(term->linebuffer + term->cursor + 1,
            term->linebuffer + term->cursor,
            MAX_LINELEN - term->linelen - 1);
    term->linebuffer[term->cursor] = c;

    if(term->cursor == term->linelen)
    {
      fputc(c, stdout);
    }
    else
    {
    (void)fprintf(term->out, term->linebuffer + term->cursor);
    term_cursorleft(term, term->linelen - term->cursor);

    }
    term->cursor++;
    term->linelen++;
    term->linebuffer[term->linelen] = '\0';
  }
}

/*============================================================================*/
static void key_end(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  term_cursorright(term, term->linelen - term->cursor);
  term->cursor = term->linelen;
}


/*============================================================================*/
static void key_del(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  delete(term, 1);
  (void)fprintf(term->out, "\033[K%s", term->linebuffer + term->cursor);
  term_cursorleft(term, term->linelen - term->cursor);
}


/*============================================================================*/
static void key_home(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  term_cursorleft(term, term->cursor);
  term->cursor = 0;
}


/*============================================================================*/
static void key_left(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if(term->cursor > 0)
  {
    term->cursor--;
    term_cursorleft(term, 1);
  }
}


/*============================================================================*/
static void key_right(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if(term->cursor < term->linelen)
  {
    term->cursor++;
    term_cursorright(term, 1);
  }
}


/*============================================================================*/
static void key_bksp(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if(term->cursor > 0)
  {
    backspace(term, 1);
    if(term->cursor == term->linelen)
    {
      (void)fprintf(term->out, "\033[D \033[D");
    }
    else
    {
      /* go back one char, delete until the end of the line and print the
         modified string */
      (void)fprintf(term->out, "\033[D\033[K%s", term->linebuffer + term->cursor);
      term_cursorleft(term, term->linelen - term->cursor);
    }
  }
}


/*============================================================================*/
static void key_ctrl_u(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  term_cursorleft(term, term->cursor);
  backspace(term, term->cursor);
  (void)fprintf(term->out, "\033[K%s", term->linebuffer + term->cursor);
  term_cursorleft(term, term->linelen);
}


/*============================================================================*/
static void key_ctrl_k(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  (void)fprintf(term->out, "\033[K");
  term->linelen = term->cursor;
  term->linebuffer[term->linelen] = '\0';
}


/*============================================================================*/
static void key_enter(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  (void)fprintf(term->out, "\n\r");
  term->linebuffer[term->linelen] = '\0';
  term->cursor = 0;
  term->linelen = 0;
  term->insert = false;
  term->escape = false;
  term->esc_code = '\0';
}


/*============================================================================*/
static int find_tokens(char* line, const char** toks, int maxtoks)
/*------------------------------------------------------------------------------
  Function:
  finds all tokens separated by at least one space and returns the pointers to
  them as well as their number
  in:  line -> string containing some space-separated tokens
       toks -> pointer to an array where the tokens will be stored
       maxtoks -> maximum number of tokens to find
  out: the array toks is populated with the pointers to the individual tokens
==============================================================================*/
{
  char* ptr = strtok(line, " ");

  for(int toknum = 0; toknum < maxtoks; toknum++)
  {
    if(ptr != NULL)
    {
      toks[toknum] = ptr;
      ptr = strtok(NULL, " ");
    }
    else
    {
      return toknum;
    }
  }
  return 0;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
