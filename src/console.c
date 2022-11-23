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

#include "FreeRTOS.h"
#include "task.h"
#include "console.h"
#include "rs232.h"
#include "eeprom.h"
#include "ublox.h"
#include "timebase.h"
#include "temperature.h"
#include "adc.h"
#include "dac.h"
#include "cntl.h"
#include "ublox.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

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
  evaluate
} consolestatus_t;

typedef struct
{
  void(*func)(int argc, const char* const argv[]);
  const char* name;
  const char* helpstring;
} command_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static int find_tokens(char* line, const char** toks, int maxtoks);
static void interpreter(int argc, const char* const argv[]);
static void help(int argc, const char* const argv[]);
static void conf_gnss(int argc, const char* const argv[]);
static void conf_elev_mask(int argc, const char* const argv[]);
static void savecfg(int argc, const char* const argv[]);
static void showcfg(int argc, const char* const argv[]);
static void enable_disp(int argc, const char* const argv[]);
static void svin(int argc, const char* const argv[]);
static void sat(int argc, const char* const argv[]);
static void auto_svin(int argc, const char* const argv[]);
static void conf_timeconst(int argc, const char* const argv[]);
static void man_hold(int argc, const char* const argv[]);
static void uptime(int argc, const char* const argv[]);
static void offset(int argc, const char* const argv[]);
static void info(int argc, const char* const argv[]);
static void set_pps_dur(int argc, const char* const argv[]);
static bool str2num(const char* str, int32_t* value, int32_t max, int32_t min);

/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static char linebuffer[MAX_LINELEN];
static uint32_t wrpos = 0;

static command_t cmds[] =
{
  {help,            "help",       "display this screen"},
  {showcfg,         "conf",       "display all configuration items"},
  {savecfg,         "save",       "save the configuration to EEPROM"},
  {conf_gnss,       "gnss",       "[gps|glonass|galileo] - select GNSS to use"},
  {conf_elev_mask,  "elev_mask",  "configures an elevation mask, must be between 0deg and 90deg"},
  {enable_disp,     "disp",       "auto display status (for logging); leave with <enter>"},
  {svin,            "svin",       "[<time> <accuracy> | stop] - perform survey in for <time> seconds with accuracy <accuracy> or stop running survey-in"},
  {sat,             "sat",        "display satellite info"},
  {auto_svin,       "auto_svin",  "[on|off] configure auto-svin"},
  {conf_timeconst,  "timeconst",  "<tau> - sets the time constant (sec) between 10s and 7200s"},
  {man_hold,        "hold",       "<number> | off - holds the DAC value to <number>, disabling the control loop, or sets the DAC automatically"},
  {uptime,          "uptime",     "shows the current uptime"},
  {offset,          "offset",     "sets the offset of the pps output in ns, must be between -0.5s to 0.5s"},
  {info,            "info",       "show version information"},
  {set_pps_dur,     "pps_dur",    "<ms> configures the duration of the PPS pulse"},
};

volatile uint16_t stat_dac = 0u;

extern volatile svindata_t svin_info;

/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void console_task(void* param)
{
  (void)param;
  rs232_init();
  static consolestatus_t status = startup;
  int rx;
  for(;;)
  {
    switch(status)
    {
      /* clears the terminal and prints the info screen */
      case startup:
      {
        info(0, NULL);
        status = prompt;
        break;
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
            break;
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
            if((wrpos < MAX_LINELEN-2) && (rx >= ' '))
            {
              txchar((char)rx);
              linebuffer[wrpos] = (char)rx;
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
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

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


/*============================================================================*/
static void interpreter(int argc, const char* const argv[])
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
  (void)printf("error\n");
}


/*============================================================================*/
static void help(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  the interpreter gets the number of arguments and the individual arguments,
  searches the command to be executed and passes the arguments to it
  in:  argc -> number of arguments, including the command itself
       argv -> array of strings
  out: none
==============================================================================*/
{
  /* unused */
  (void)argc;
  (void)argv;

  int numcmds = sizeof(cmds) / sizeof(cmds[0]);
  for(int i = 0; i < numcmds; i++)
  {
    /* shorthand */
    command_t* cmd = &cmds[i];
    if(cmd->helpstring != NULL)
    {
      (void)printf("%s : %s\n", cmd->name, cmd->helpstring);
    }
  }
}


/*============================================================================*/
static void conf_gnss(int argc, const char* const argv[])
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
    if(!strcmp(argv[i], "gps"))
    {
      use_gps = true;
    }
    else if(!strcmp(argv[i], "galileo"))
    {
      use_galileo = true;
    }
    else if(!strcmp(argv[i], "glonass"))
    {
      use_glonass = true;
    }
    else
    {
      (void)printf("unknown GNSS <%s> - skipped\n", argv[i]);
    }
  }

  config_t* cfg = get_config();
  cfg->use_gps = use_gps;
  cfg->use_galileo = use_galileo;
  cfg->use_glonass = use_glonass;

  /* inform the gps module about the changed configuration */
  reconfigure_gnss();
}


/*============================================================================*/
static void conf_elev_mask(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  configure the elevation mask
  in:  argc -> number of arguments, see below
       argv -> array of strings; usually only one element which is an integer
       which is then used as the elevation mask in degrees
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    int32_t elevmask;
    if(str2num(argv[0], &elevmask, 0, 90))
    {
      get_config()->elevation_mask = (int8_t)elevmask;
    }
  }
}


/*============================================================================*/
static void savecfg(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  save the configuration to the eeprom
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  /* unused */
  (void)argv;

  if(argc == 0)
  {
    save_config();
  }
  else
  {
    (void)printf("error");
  }
}


/*============================================================================*/
static void showcfg(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  displays the currently used configuration data (not necessarily stored in
  the eeprom!)
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  /* unused */
  (void)argv;

  if(argc == 0)
  {
    config_t* cfg = get_config();
    (void)printf("last DAC value: %d\n", cfg->last_dacval);
    (void)printf("use GPS: %s\n", (cfg->use_gps ? "yes" : "no"));
    (void)printf("use GLONASS: %s\n", (cfg->use_glonass ? "yes" : "no"));
    (void)printf("use GALILEO: %s\n", (cfg->use_galileo ? "yes" : "no"));
    (void)printf("RS-232 baudrate: %lu\n", cfg->rs232_baudrate);
    (void)printf("fixed position valid: %s\n", (cfg->fixpos_valid ? "yes" : "no"));
    (void)printf("ECEF X position: %ld cm\n", cfg->x);
    (void)printf("ECEF Y position: %ld cm\n", cfg->y);
    (void)printf("ECEF Z position: %ld cm\n", cfg->z);
    (void)printf("position accuracy: %lu mm\n", cfg->accuracy);
    (void)printf("survey-in duration: %lu sec\n", cfg->svin_dur);
    (void)printf("survey-in accuracy limit: %lu mm\n", cfg->accuracy_limit);
    (void)printf("elevation mask: %d\n", cfg->elevation_mask);
    (void)printf("auto-svin: %s\n", cfg->auto_svin ? "on" : "off");
    (void)printf("tau: %d sec\n", cfg->tau);
    (void)printf("time offset: %ld ns\n", cfg->timeoffset);
    (void)printf("pps duration: %lu ms\n", cfg->pps_dur);
  }
  else
  {
    (void)printf("error");
  }
}


/*============================================================================*/
static void enable_disp(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  enables the auto-display
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  /* unused */
  (void)argv;

  if(argc == 0)
  {
    extern bool canread(void);
    for(;;)
    {
      /* wait until the 1pps pulse arrives, this precisely synchronises the
         console output with the controller task */
      (void)pps_elapsed();

      /* collect all data */
      uint64_t now = get_uptime_msec();
      float i = get_iocxo();
      float t = get_temperature();
      const gnssstatus_t* gnss = get_gnss_status();
      const cntlstatus_t* ctl = get_cntlstatus();

      uint32_t meanv = (uint32_t)sqrt((double)gnss->svi->meanv);


      (void)printf("%-10llu e=%-7.2f eI=%-9.3f D=%-5d I=%.1f T=%.1f sat=%-2d " \
                   "lat=%ld lon=%ld obs=%-5lu mv=%-5lu tacc=%-3lu " \
                   "status=%s\n",
                   now, ctl->e, ctl->esum, stat_dac, i, t, gnss->sat->numsv,
                   gnss->pvt->lat, gnss->pvt->lon, gnss->svi->obs, meanv,
                   gnss->pvt->tacc, ctl->mode);
      if(canread())
      {
        return;
      }
    }
  }
}


/*============================================================================*/
static void svin(int argc, const char* const argv[])
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
  /* unused */
  (void)argv;

  switch(argc)
  {
    /* svin stop */
    case 1:
    {
      if(!strcmp(argv[0], "stop"))
      {
        disable_tmode();
      }
      break;
    }

    /* svin <duration> <accuracy> */
    case 2:
    {
      int32_t tm;
      int32_t accuracy;
      if(str2num(argv[0], &tm, 1, INT32_MAX) &&
         str2num(argv[1], &accuracy, 0, INT32_MAX))
      {
        config_t* cfg = get_config();
        cfg->svin_dur = (uint32_t)tm;
        cfg->accuracy_limit = (uint32_t)accuracy;
        start_svin();
      }
      break;
    }

    default:
    {
      (void)printf("unknown syntax\n");
    }
  }
  return;
}


/*============================================================================*/
static void sat(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  displays satellite info
  in:  argc -> number of arguments, see below
       argv -> array of strings; none required
  out: none
==============================================================================*/
{
  /* unused */
  (void)argv;

  if(argc == 0)
  {
    const gnssstatus_t* gnss = get_gnss_status();

    for(int i = 0; i < gnss->sat->numsv; i++)
    {
      const char* syst;
      switch(gnss->sat->sats[i].gnssid)
      {
        case 0:
        {
          syst = "GPS    ";
          break;
        }

        case 2:
        {
          syst = "GALILEO";
          break;
        }

        case 6:
        {
          syst = "GLONASS";
          break;
        }

        default:
        {
          syst = "?      ";
          break;
        }
      }
      (void)printf("%s ID: %2d; C/N0: %2d dB; Az: %3d deg; El: %3d deg\n",
        syst, gnss->sat->sats[i].svid, gnss->sat->sats[i].cno,
        gnss->sat->sats[i].azim, gnss->sat->sats[i].elev);
    }

    const char* fixtype;
    switch(gnss->pvt->fixtype)
    {
      case 3:
      {
        fixtype = "3D fix";
        break;
      }

      case 5:
      {
        fixtype = "timing";
        break;
      }

      default:
      {
        fixtype = "none";
        break;
      }
    }
    (void)printf("fix status: %s\n", fixtype);

    uint64_t age = get_uptime_msec() - gnss->sat->time;
    (void)printf("%d sats; last update: %llu ms ago\n\n", gnss->sat->numsv, age);
  }
}


/*============================================================================*/
static void auto_svin(int argc, const char* const argv[])
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
      get_config()->auto_svin = true;
    }
    else if(!strcmp(argv[0], "off"))
    {
      get_config()->auto_svin = false;
    }
  }
}


/*============================================================================*/
static void conf_timeconst(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  configure time constant and prefilter
  in:  argc -> number of arguments, see below
       argv -> array of strings; two required
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    int32_t tau;
    if(str2num(argv[0], &tau, 10, 7200))
    {
      get_config()->tau = (uint16_t)tau;
    }
  }
}


/*============================================================================*/
static void man_hold(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  enable/disable DAC hold
  in:  argc -> number of arguments, see below
       argv -> array of strings; one required ("on" or "off")
  out: none
==============================================================================*/
{
  extern bool dac_hold;
  if(argc == 1)
  {
    /* command given is "hold off" ? */
    if(!strcmp(argv[0], "off"))
    {
      dac_hold = false;
    }
    else
    {
      int32_t dac;
      if(str2num(argv[0], &dac, 0, UINT16_MAX))
      {
        dac_hold = true;
        set_dac((uint16_t)dac);
      }
    }
  }
}


/*============================================================================*/
static void uptime(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  shows the uptime. does not need arguments
  in:  noen
  out: none
==============================================================================*/
{
  (void)argv;
  if(argc == 0)
  {
    uint64_t tm = get_uptime_msec();
    tm = tm / 1000;
    uint32_t sec = tm % 60;
    tm = tm / 60;
    uint32_t min = tm % 60;
    tm = tm / 60;
    uint32_t hour = tm % 24;
    uint32_t day = (uint32_t)(tm / 24ull);

    (void)printf("uptime: %lu days, %lu hours, %lu min, %lu sec\n", day, hour, min, sec);
  }
}


/*============================================================================*/
static void offset(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  sets the offset of the pps output
  in:  noen
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    /* when the setpoint is changed, then restart the control loop.
       otherwise, it takes too long until the pll locks with the new phase. */
    int32_t off;
    if(str2num(argv[0], &off, -500000000, 500000000))
    {
      get_config()->timeoffset = off;
    }
  }
}


/*============================================================================*/
static void info(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  show version info
  in:  noen
  out: none
==============================================================================*/
{
  (void)argc;
  (void)argv;
  (void)printf("\033[2J\rHB9FSX GNSS Frequency Standard\n");
  (void)printf("compiled " __DATE__ " " __TIME__ "\n");
  (void)printf("with FreeRTOS " tskKERNEL_VERSION_NUMBER "\n");
  (void)printf("use \"help\" to see the available commands\n\n");
}


/*============================================================================*/
static void set_pps_dur(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  noen
  out: none
==============================================================================*/
{
  if(argc == 1)
  {
    int32_t dur;
    if(str2num(argv[0], &dur, 1, 999))
    {
      get_config()->pps_dur = (uint32_t)dur;
      set_pps_duration(dur);
    }
  }
}


/*============================================================================*/
static bool str2num(const char* str, int32_t* value, int32_t min, int32_t max)
/*------------------------------------------------------------------------------
  Function:
  convert string to number, but more safely than atoi().
  in:  str -> string
       value -> pointer to the result
       min -> minimum (inclusive)
       max -> maximum (inclusive)
  out: returns true if the conversion succeeds, otherwise sets errno, prints an
       error message and returns false.
==============================================================================*/
{
  char* end = NULL;
  errno = 0;
  int32_t num = strtol(str, &end, 10);
  if((end == str) || (*end != '\0'))
  {
    errno = EINVAL;
    goto error;
  }
  if(errno == ERANGE)
  {
    goto error;
  }
  if((num < min) || (num > max))
  {
    errno = ERANGE;
    goto error;
  }
  *value = num;
  return true;

error:
  (void)printf("invalid value or value out of range: %s\n", str);
  return false;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
