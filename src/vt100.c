/*******************************************************************************
 * Company:
 *
 * Project:        GPS frequency standard
 *
 * Description:    simple terminal based console
 *
 * Filename:       console.c
 *
 * Author:         Tobias Plüss <tpluess@ieee.org>
 *
 * Creation-Date:  20.04.2020
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE FILES
 ******************************************************************************/

#include "vt100.h"

#include <string.h>
#include <ctype.h>
#include <errno.h>

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

static void insertchar(vt100_t* term, int c);
static void cntlchar(vt100_t* term, int c);
static void backspace(vt100_t* term, size_t num);
static void delete(vt100_t* term, size_t num);
static void term_cursorleft(vt100_t* term, size_t num);
static void term_cursorright(vt100_t* term, size_t num);
static bool handle_esc(vt100_t* term, int c);
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
void vt100_init(vt100_t* term, interpreter_func intr, FILE* out, FILE* in)
/*------------------------------------------------------------------------------
  Function:
  the line editor works similar to gnu's libreadline.
  in:  none
  out: none
==============================================================================*/
{
  (void)memset(term, 0, sizeof(vt100_t));
  term->interpreter = intr;
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
  in:  none
  out: none
==============================================================================*/
{
  for(;;)
  {
    int c = fgetc(term->in);

    /* wrong character */
    if(c < 0)
    {
      return -1;
    }

    /* if the escape character is received, process escape sequence accordingly */
    if(term->escape)
    {
      if(handle_esc(term, c))
      {
        term->escape = false;
      }
      continue;
    }

    /* if enter is pressed, terminate the entered string; also the line editor
       is complete */
    if((c == ANSI_CR) || (c == ANSI_LF))
    {
      key_enter(term);

      const char* toks[MAX_TOKENS];
      int ntoks = find_tokens(term->linebuffer, &toks[0], MAX_TOKENS);
      if(ntoks > MAX_TOKENS)
      {
        errno = E2BIG;
        return -1;
      }

      int ret = term->interpreter(ntoks, toks);
      return ret;
    }

    if(iscntrl(c))
    {
      cntlchar(term, c);
    }
    else
    {
      insertchar(term, c);
    }
  }
}


/*============================================================================*/
char* vt100_line(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  the line editor works similar to gnu's libreadline.
  in:  none
  out: none
==============================================================================*/
{
  return term->linebuffer;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void backspace(vt100_t* term, size_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  if(term->cursor >= num)
  {
    (void)memmove(term->linebuffer + term->cursor - num,
                  term->linebuffer + term->cursor,
                  (term->linelen - term->cursor) + num);
    term->cursor -= num;
    term->linelen -= num;
    term->linebuffer[term->linelen] = '\0';
  }
}


/*============================================================================*/
static void delete(vt100_t* term, size_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  if((term->linelen != 0) && (term->cursor < term->linelen))
  {
    (void)memmove(term->linebuffer + term->cursor,
                  term->linebuffer + term->cursor + num,
                  (term->linelen - term->cursor) + num);
    term->linelen -= num;
    term->linebuffer[term->linelen] = '\0';
  }
}


/*============================================================================*/
static void term_cursorleft(vt100_t* term, size_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  (void)term;
  if(num != 0)
  {
    (void)fprintf(term->out, "\033[%uD", num);
  }
}


/*============================================================================*/
static void term_cursorright(vt100_t* term, size_t num)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  (void)term;
  if(num != 0)
  {
    (void)fprintf(term->out, "\033[%zuC", num);
  }
}


/*============================================================================*/
static bool handle_esc(vt100_t* term, int c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
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
      default:
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
      case ESC_PGDOWN:
      default:
      {
        return true;
      }
    }
  }

  return true;
}


/*============================================================================*/
static void cntlchar(vt100_t* term, int c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
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

    default:
    {
      break;
    }
  }
}


/*============================================================================*/
static void insertchar(vt100_t* term, int c)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  if((term->insert) && (term->cursor < term->linelen))
  {
    term->linebuffer[term->cursor] = (char)c;
    (void)fputc(c, stdout);
    term->cursor++;
    return;
  }
  if(term->cursor < MAX_LINELEN - 2)
  {
    (void)memmove(term->linebuffer + term->cursor + 1,
                  term->linebuffer + term->cursor,
                  (MAX_LINELEN - term->linelen) - 1u);
    term->linebuffer[term->cursor] = (char)c;

    if(term->cursor == term->linelen)
    {
      (void)fputc(c, stdout);
    }
    else
    {
      (void)fputs(term->linebuffer + term->cursor, term->out);
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
  in:  none
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
  in:  none
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
  in:  none
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
  in:  none
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
  in:  none
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
  in:  none
  out: none
==============================================================================*/
{
  if(term->cursor > 0)
  {
    backspace(term, 1);
    if(term->cursor == term->linelen)
    {
      (void)fputs("\033[D \033[D", term->out);
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
  in:  none
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
  in:  none
  out: none
==============================================================================*/
{
  (void)fputs("\033[K", term->out);
  term->linelen = term->cursor;
  term->linebuffer[term->linelen] = '\0';
}


/*============================================================================*/
static void key_enter(vt100_t* term)
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  (void)fputs("\n\r", term->out);
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
  them as well as their number. maximum maxtoks tokens are returned; if there
  are more tokens, this is indicated by the return value.
  in:  line -> string containing some space-separated tokens
       toks -> pointer to an array where the tokens will be stored
       maxtoks -> maximum number of tokens to find
  out: the array toks is populated with the pointers to the individual tokens
       and the number of tokens present is returned
==============================================================================*/
{
  char* ptr = strtok(line, " ");
  int toknum = 0;
  do
  {
    if(toknum < maxtoks)
    {
        toks[toknum] = ptr;
    }
    ptr = strtok(NULL, " ");
    toknum++;
  } while(ptr != NULL);
  return toknum;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
