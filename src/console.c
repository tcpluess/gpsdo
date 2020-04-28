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
#include "ublox.h"
#include "timebase.h"

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
  void(*func)(int argc, const char* argv[]);
  const char* name;
  const char* helpstring;
} command_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static uint32_t find_tokens(char* line, const char** toks, uint32_t maxtoks);
static void interpreter(int argc, const char* argv[]);
static void help(int argc, const char* argv[]);
static void conf_gnss(int argc, const char* argv[]);
static void conf_elev_mask(int argc, const char* argv[]);
static void savecfg(int argc, const char* argv[]);
static void showcfg(int argc, const char* argv[]);
static void enable_disp(int argc, const char* argv[]);
static void svin(int argc, const char* argv[]);
static void sat(int argc, const char* argv[]);
static void restart(int argc, const char* argv[]);
static void auto_svin(int argc, const char* argv[]);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static char linebuffer[MAX_LINELEN];
static uint32_t wrpos = 0;

static command_t cmds[] =
{
  {help,            "help",       "display this screen"},
  {showcfg,         "show_cfg",   "display all configuration items"},
  {savecfg,         "save_cfg",   "save the configuration to EEPROM"},
  {conf_gnss,       "gnss",       "[gps|glonass|galileo] - enables the selected GNSS"},
  {conf_elev_mask,  "elev_mask",  "configures an elevation mask"},
  {enable_disp,     "disp",       "auto display status (for logging); leave with <enter>"},
  {svin,            "svin",       "[<time> <accuracy> | stop] - perform survey in for <time> seconds with accuracy <accuracy> or stop running survey-in"},
  {sat,             "sat",        "display satellite info"},
  {restart,         "restart",    "restart the gps module"},
  {auto_svin,       "auto_svin",  "[on|off] configure auto-svin"},
};

/* not static because it must be globally accessible */
extern config_t cfg;

bool auto_disp;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void console_worker(void)
{
  static consolestatus_t status = startup;
  int rx;

  switch(status)
  {
    /* clears the terminal and prints the info screen */
    case startup:
    {
      printf("\033[2J\rHB9FSX GNSS Frequency Standard\n");
      printf("use \"help\" to see the available commands\n\n");
      status = prompt;
      auto_disp = false;
    }

    /* prints the console prompt character */
    case prompt:
    {
      txchar('#');
      txchar(' ');
      status = input;
      wrpos = 0;
      break;
    }

    /* processes the received characters and echoes them back accordingly */
    case input:
    {
      rx = kbhit();

      switch(rx)
      {
        /* no character received, nothing to do */
        case -1:
        {
          return;
        }

        /* backspace entered: only need to do something if the line buffer is not
           empty; erase the last character in the terminal by going back one,
           sending a space and again going back one; also decrease the
           writing pos into the line buffer */
        case '\b':
        {
          if(wrpos > 0)
          {
            txchar('\b');
            txchar(' ');
            txchar('\b');
            wrpos--;
          }
          break;
        }

        /* enter is pressed, terminate the line buffer and evaluate it
           or do nothing if the line buffer is empty */
        case '\r':
        case '\n':
        {
          auto_disp = false;
          txchar('\r');
          txchar('\n');
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
        }

        /* a normal character is received, so echo back and add it into
           the line buffer */
        default:
        {
          /* FIXME: need to check this condition */
          if(wrpos < MAX_LINELEN-2)
          {
            txchar(rx);
            linebuffer[wrpos] = rx;
            wrpos++;
          }
          break;
        }
      }
      break;
    }

    /* if a command line has been entered, it is processed in this state */
    case evaluate:
    {
      const char* toks[MAX_TOKENS];
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
static uint32_t find_tokens(char* line, const char** toks, uint32_t maxtoks)
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


/*============================================================================*/
static void interpreter(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  the interpreter gets the number of arguments and the individual arguments,
  searches the command to be executed and passes the arguments to it
  in:  argc -> number of arguments, including the command itself
       argv -> array of strings
  out: none
==============================================================================*/
{
  if(argc >= 1)
  {
    /* the actual command is always first */
    const char* command = argv[0];

    int numcmds = sizeof(cmds) / sizeof(cmds[0]);

    for(int i = 0; i < numcmds; i++)
    {
      if(!strcmp(command, cmds[i].name))
      {
        cmds[i].func(argc-1, &argv[1]);
        return;
      }
    }
  }
  printf("error\n");
}


/*============================================================================*/
static void help(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  the interpreter gets the number of arguments and the individual arguments,
  searches the command to be executed and passes the arguments to it
  in:  argc -> number of arguments, including the command itself
       argv -> array of strings
  out: none
==============================================================================*/
{
  int numcmds = sizeof(cmds) / sizeof(cmds[0]);
  for(int i = 0; i < numcmds; i++)
  {
    /* shorthand */
    command_t* cmd = &cmds[i];
    if(cmd->helpstring != NULL)
    {
      printf("%s : %s\n", cmd->name, cmd->helpstring);
    }
  }
}


/*============================================================================*/
static void conf_gnss(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  configure the gnss systems to be used
  in:  argc -> number of arguments, see below
       argv -> array of strings, any combination of "gps", "glonass" and
       "galileo"
  out: none
==============================================================================*/
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


/*============================================================================*/
static void conf_elev_mask(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  configure the elevation mask
  in:  argc -> number of arguments, see below
       argv -> array of strings; usually only one element which is an integer
       which is then used as the elevation mask in degrees
  out: none
==============================================================================*/
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


/*============================================================================*/
static void savecfg(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  save the configuration to the eeprom
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  if(argc == 0)
  {
    save_config();
  }
  else
  {
    printf("error");
  }
}


/*============================================================================*/
static void showcfg(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  displays the currently used configuration data (not necessarily stored in
  the eeprom!)
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  if(argc == 0)
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
    printf("elevation mask: %d\n", cfg.elevation_mask);
    printf("auto-svin: %s\n", cfg.auto_svin ? "on" : "off");
  }
  else
  {
    printf("error");
  }
}


/*============================================================================*/
static void enable_disp(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  enables the auto-display -- currently implemented as an UGLY HACK
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  if(argc == 0)
  {
    auto_disp = true;
  }
}


/*============================================================================*/
static void svin(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  starts a survey-in usng a given duration and accuracy limit
  in:  argc -> number of arguments, see below
       argv -> array of strings; first one is an integer representing the
       required duration in seconds, second one is an integer representing
       the accuracy limit in mm
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    if(!strcasecmp(argv[0], "stop"))
    {
      disable_tmode();
    }
  }
  else if(argc == 2)
  {
    uint32_t time = atoi(argv[0]);
    uint32_t accuracy = atoi(argv[1]);
    if((time != 0) && (accuracy != 0))
    {
      disable_tmode();
      cfg.svin_dur = time;
      cfg.accuracy_limit = accuracy;
      start_svin();
    }
  }
  else
  {
    printf("unknown syntax\n");
    return;
  }
}


/*============================================================================*/
static void sat(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  displays satellite info
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  extern sv_info_t svi;
  for(int i = 0; i < svi.numsv; i++)
  {
    const char* gnss;
    switch(svi.sats[i].gnssid)
    {
      case 0:
      {
        gnss = "GPS    ";
        break;
      }

      case 2:
      {
        gnss = "GALILEO";
        break;
      }

      case 6:
      {
        gnss = "GLONASS";
        break;
      }

      default:
      {
        gnss = "?      ";
        break;
      }
    }
    printf("%s ID: %2d; C/N0: %2d dB; Azimuth: %3d deg; Elevation: %3d deg\n",
      gnss, svi.sats[i].svid, svi.sats[i].cno, svi.sats[i].azim, svi.sats[i].elev);
  }
  uint64_t age = get_uptime_msec() - svi.time;
  printf("%d sats; last update: %llu ms ago\n\n", svi.numsv, age);
}


/*============================================================================*/
static void restart(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  displays satellite info
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  if(argc == 0)
  {
    gps_restart();
  }
}


/*============================================================================*/
static void auto_svin(int argc, const char* argv[])
/*------------------------------------------------------------------------------
  Function:
  configure auto survey-in
  in:  argc -> number of arguments, see below
       argv -> array of strings; one required ("on" enables auto-svin, "off"
       disabels it)
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    if(!strcmp(argv[0], "on"))
    {
      cfg.auto_svin = true;
    }
    else if(!strcmp(argv[0], "off"))
    {
      cfg.auto_svin = false;
    }
  }
}


/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
