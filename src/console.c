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

#include "console.h"
#include "rs232.h"
#include "eeprom.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

#define MAX_LINELEN 256u
#define MAX_TOKENS 10u

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

typedef enum
{
  startup,
  prompt,
  input,
  evaluate,
} consolestatus_t;

typedef struct
{
  void(*func)(int argc, char* argv[]);
  const char* name;
} command_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static uint32_t find_tokens(char* line, char** toks, uint32_t maxtoks);

static void interpreter(int argc, char* argv[]);

static void help(int argc, char* argv[]);
static void conf_gnss(int argc, char* argv[]);
static void conf_elev_mask(int argc, char* argv[]);
static void savecfg(int argc, char* argv[]);
static void showcfg(int argc, char* argv[]);
static void enable_disp(int argc, char* argv[]);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static char linebuffer[MAX_LINELEN];
static uint32_t wrpos = 0;
static consolestatus_t status = startup;

static command_t cmds[] =
{
  {help, "help"},
  {conf_gnss, "gnss"},
  {savecfg, "save_cfg"},
  {conf_elev_mask, "elev_mask"},
  {showcfg, "show_cfg"},
  {enable_disp, "disp"},
};

/* not static because it must be globally accessible */
extern config_t cfg;

bool auto_disp;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void console_worker(void)
{
  int nesbit;

  switch(status)
  {
    case startup:
    {
      printf("\033[2J\rHB9FSX GPSDO\n");
      printf("HW=v2.0, SW=v0.1\n");
      printf("use \"help\" to see the available commands\n\n");
      status = prompt;
      auto_disp = false;
    }
    case prompt:
    {
      sendchar('#');
      sendchar(' ');
      status = input;
      wrpos = 0;
      break;
    }

    case input:
    {
      nesbit = gc();

      switch(nesbit)
      {
        case -1:
        {
          return;
        }

        case '\b':
          if(wrpos > 0)
          {
            sendchar('\b');
            sendchar(' ');
            sendchar('\b');
            wrpos--;
          }
          break;

        case '\r':
        case '\n':
          auto_disp = false;
          sendchar('\r');
          sendchar('\n');
          linebuffer[wrpos] = '\0';
          if(strlen(linebuffer) > 0)
          {
            status = evaluate;
          }
          else
          {
            status = prompt;
          }
          break;

        default:
          if(wrpos < MAX_LINELEN-2)
          {
            sendchar(nesbit);
            linebuffer[wrpos] = nesbit;
            wrpos++;
          }
          break;
      }
      break;
    }

    case evaluate:
    {
      char* toks[MAX_TOKENS];
      int ntoks = find_tokens(linebuffer, &toks[0], MAX_TOKENS);
      interpreter(ntoks, toks);
      status = prompt;
      break;
    }
  }
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static uint32_t find_tokens(char* line, char** toks, uint32_t maxtoks)
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

static void interpreter(int argc, char* argv[])
{
  if(argc >= 1)
  {
    /* the actual command is always first */
    char* command = argv[0];

    int numcmds = sizeof(cmds) / sizeof(cmds[0]);

    for(int i = 0; i < numcmds; i++)
    {
      if(!strcasecmp(command, cmds[i].name))
      {
        cmds[i].func(argc-1, &argv[1]);
      }
    }

  }
  else
  {
    printf("error\n");
  }
}

static void help(int argc, char* argv[])
{
  printf("gnss [gps|glonass|galileo] - enables the selected GNSS\n");
  printf("baud <baudrate> - changes the RS-232 baud rate\n");
  printf("svin_limit <x> - sets the accuracy limit (mm) for survey-in\n");
  printf("elev_mask <x> - configures an elevation mask\n");
  printf("save_cfg - save the configuration to EEPROM\n");
  printf("show_cfg - display all configuration items\n");
  printf("disp - auto display status (for logging); leave with <enter>\n");
}


static void conf_gnss(int argc, char* argv[])
{
  bool use_gps = false;
  bool use_glonass = false;
  bool use_galileo = false;
  for(int i = 0; i < argc; i++)
  {
    if(!strcasecmp(argv[i], "gps"))
    {
      use_gps = true;
    }
    else if(!strcasecmp(argv[i], "galileo"))
    {
      use_galileo = true;
    }
    else if(!strcasecmp(argv[i], "glonass"))
    {
      use_glonass = true;
    }
    else
    {
      printf("unknown GNSS <%s> - skipped\n", argv[i]);
    }
  }

  cfg.use_gps = use_gps;
  cfg.use_galileo = use_galileo;
  cfg.use_glonass = use_glonass;
}


static void conf_elev_mask(int argc, char* argv[])
{
  if(argc != 1)
  {
    printf("unknown syntax\n");
  }
  else
  {
    int8_t elevmask = atoi(argv[0]);
    if((elevmask < 0) || (elevmask > 90))
    {
      printf("elevation mask %d deg out of range!\n", elevmask);
    }
    else
    {
      cfg.elevation_mask = elevmask;
      printf("set the elevation mask to %d deg\n", elevmask);
    }
  }
}

static void savecfg(int argc, char* argv[])
{
  save_config();
}


static void showcfg(int argc, char* argv[])
{
  printf("last DAC value: %d\n", cfg.last_dacval);
  printf("use GPS: %s\n", (cfg.use_gps ? "yes" : "no"));
  printf("use GLONASS: %s\n", (cfg.use_glonass ? "yes" : "no"));
  printf("use GALILEO: %s\n", (cfg.use_galileo ? "yes" : "no"));
  printf("RS-232 baudrate: %lu\n", cfg.rs232_baudrate);
  printf("fixed position valid: %s\n", (cfg.fixpos_valid ? "yes" : "no"));
  printf("ECEF X position: %ld cm\n", cfg.x);
  printf("ECEF Y position: %ld cm\n", cfg.y);
  printf("ECEF Z position: %ld cm\n", cfg.z);
  printf("position accuracy: %lu mm\n", cfg.accuracy);
  printf("survey-in duration: %lu sec\n", cfg.svin_dur);
  printf("survey-in accuracy limit: %lu mm\n", cfg.accuracy_limit);
}


static void enable_disp(int argc, char* argv[])
{
  auto_disp = true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
