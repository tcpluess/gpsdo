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
  {auto_svin,       "auto_svin",  "[on|off] configure auto-svin"},
  {conf_timeconst,  "timeconst",  "<tau> <prefilter> - sets the time constant (sec) and the prefilter (%)"},
  {man_hold,        "hold",       "on|off - holds the DAC value"},
  {uptime,          "uptime",     "shows the current uptime"},
  {offset,          "offset",     "sets the offset of the pps output"},
  {info,            "info",       "show version information"},
};

/* not static because it must be globally accessible */
extern config_t cfg;

volatile float stat_e;
volatile float stat_esum;
volatile uint16_t stat_dac;

extern volatile gpsinfo_t pvt_info;
extern volatile svindata_t svin_info;
extern volatile sv_info_t sat_info;

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
            if(wrpos < MAX_LINELEN-2)
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

  cfg.use_gps = use_gps;
  cfg.use_galileo = use_galileo;
  cfg.use_glonass = use_glonass;
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
  if(argc != 1)
  {
    (void)printf("unknown syntax\n");
  }
  else
  {
    int elevmask = atoi(argv[0]);
    if((elevmask < 0) || (elevmask > 90))
    {
      (void)printf("elevation mask %d deg out of range!\n", elevmask);
    }
    else
    {
      cfg.elevation_mask = (int8_t)elevmask;
      (void)printf("set the elevation mask to %d deg\n", elevmask);
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
    (void)printf("last DAC value: %d\n", cfg.last_dacval);
    (void)printf("use GPS: %s\n", (cfg.use_gps ? "yes" : "no"));
    (void)printf("use GLONASS: %s\n", (cfg.use_glonass ? "yes" : "no"));
    (void)printf("use GALILEO: %s\n", (cfg.use_galileo ? "yes" : "no"));
    (void)printf("RS-232 baudrate: %lu\n", cfg.rs232_baudrate);
    (void)printf("fixed position valid: %s\n", (cfg.fixpos_valid ? "yes" : "no"));
    (void)printf("ECEF X position: %ld cm\n", cfg.x);
    (void)printf("ECEF Y position: %ld cm\n", cfg.y);
    (void)printf("ECEF Z position: %ld cm\n", cfg.z);
    (void)printf("position accuracy: %lu mm\n", cfg.accuracy);
    (void)printf("survey-in duration: %lu sec\n", cfg.svin_dur);
    (void)printf("survey-in accuracy limit: %lu mm\n", cfg.accuracy_limit);
    (void)printf("elevation mask: %d\n", cfg.elevation_mask);
    (void)printf("auto-svin: %s\n", cfg.auto_svin ? "on" : "off");
    (void)printf("tau: %d sec\n", cfg.tau);
    (void)printf("prefilter: %d%%\n", cfg.filt);
    (void)printf("time offset: %ld ns\n", cfg.timeoffset);
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
      uint64_t now = get_uptime_msec();

      float i = get_iocxo();
      float t = get_temperature();

      extern const char* cntl_status;
      uint32_t meanv = (uint32_t)sqrt((double)svin_info.meanv);

      (void)printf("%-9llu e=%-7.2f eI=%-9.3f D=%-5d I=%.1f T=%.1f sat=%-2d " \
                   "lat=%f lon=%f obs=%-5lu mv=%-5lu tacc=%-3lu status=%s\n",
                   now, stat_e, stat_esum, stat_dac, i, t, sat_info.numsv,
                   pvt_info.lat, pvt_info.lon, svin_info.obs, meanv,
                   pvt_info.tacc, cntl_status);
      vTaskDelay(1000);
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
      uint32_t time = (uint32_t)atoi(argv[0]);
      uint32_t accuracy = (uint32_t)atoi(argv[1]);
      if((time != 0) && (accuracy != 0))
      {
        cfg.svin_dur = time;
        cfg.accuracy_limit = accuracy;
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
    for(int i = 0; i < sat_info.numsv; i++)
    {
      const char* gnss;
      switch(sat_info.sats[i].gnssid)
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
      (void)printf("%s ID: %2d; C/N0: %2d dB; Azimuth: %3d deg; Elevation: %3d deg\n",
        gnss, sat_info.sats[i].svid, sat_info.sats[i].cno,
        sat_info.sats[i].azim, sat_info.sats[i].elev);
    }

    const char* fixtype;
    switch(pvt_info.fixtype)
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

    uint64_t age = get_uptime_msec() - sat_info.time;
    (void)printf("%d sats; last update: %llu ms ago\n\n", sat_info.numsv, age);
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
      cfg.auto_svin = true;
    }
    else if(!strcmp(argv[0], "off"))
    {
      cfg.auto_svin = false;
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
  if(argc == 2)
  {
    uint16_t tau = (uint16_t)atoi(argv[0]);
    uint8_t filt = (uint8_t)atoi(argv[1]);

    if((tau > 10) && (tau < 3600))
    {
      cfg.tau = tau;
    }
    else
    {
      (void)printf("value for tau out of range: 10 < tau < 3600\n");
    }

    if((filt >= 0) && (filt < 100))
    {
      cfg.filt = filt;
    }
    else
    {
      (void)printf("value for the prefilter out of range: prefilter must be less than 100%%\n");
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
    if(!strcmp(argv[0], "on"))
    {
      dac_hold = true;
    }
    else if(!strcmp(argv[0], "off"))
    {
      dac_hold = false;
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
    cfg.timeoffset = atoi(argv[0]);
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

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
