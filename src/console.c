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
#include "vt100.h"
#include "stm32f407xx.h"
#include "datetime.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*******************************************************************************
 * PRIVATE CONSTANT DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE MACRO DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/

typedef struct
{
  int(*func)(int argc, const char* const argv[]);
  const char* name;
  const char* helpstring;
} command_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES (STATIC)
 ******************************************************************************/

static void console_task(void* param);
static int interpreter(int argc, const char* const argv[]);
static int help(int argc, const char* const argv[]);
static int gnss(int argc, const char* const argv[]);
static int elev_mask(int argc, const char* const argv[]);
static int conf(int argc, const char* const argv[]);
static int disp(int argc, const char* const argv[]);
static int svin(int argc, const char* const argv[]);
static int sat(int argc, const char* const argv[]);
static int timeconst(int argc, const char* const argv[]);
static int hold(int argc, const char* const argv[]);
static int uptime(int argc, const char* const argv[]);
static int info(int argc, const char* const argv[]);
static int pps(int argc, const char* const argv[]);
static int reboot(int argc, const char* const argv[]);
static int date(int argc, const char* const argv[]);
static bool str2num(const char* str, int32_t* value, int32_t max, int32_t min);


/*******************************************************************************
 * PRIVATE VARIABLES (STATIC)
 ******************************************************************************/

static vt100_t term;

static command_t cmds[] =
{
  {help,            "help",       "display this screen"},
  {conf,            "conf",       "<show> | <save> - display all configuration items"},
  {gnss,            "gnss",       "[gps|glonass|galileo] - select GNSS to use"},
  {elev_mask,       "elev_mask",  "configures an elevation mask, must be between 0deg and 90deg"},
  {disp,            "disp",       "auto display status (for logging); leave with <enter>"},
  {svin,            "svin",       "[<time> <accuracy> | stop | {auto|off} ] - perform survey in for <time> seconds with accuracy <accuracy>, stop running survey-in or auto survey-in"},
  {sat,             "sat",        "display satellite info"},
  {timeconst,       "timeconst",  "<tau> - sets or shows the time constant (sec) between 10s and 7200s"},
  {hold,            "hold",       "<number> | on | off - holds the DAC value to <number>, disabling the control loop, holds the dac at current value or sets the DAC automatically"},
  {uptime,          "uptime",     "shows the current uptime"},
  {info,            "info",       "show version information"},
  {reboot,          "reboot",     "reboot the system"},
  {pps,             "pps",        "offset <ns> | dur <ms> - configures the duration of the PPS pulse in ms and the offset in ns"},
  {date,            "date",       "show date and time"},
};

volatile uint16_t stat_dac = 0u;


/*******************************************************************************
 * MODULE FUNCTIONS (PUBLIC)
 ******************************************************************************/

void console_init(void)
{
  (void)xTaskCreate(console_task, "console", CONSOLE_STACK, NULL, CONSOLE_PRIO, NULL);
}

/*******************************************************************************
 * PRIVATE FUNCTIONS (STATIC)
 ******************************************************************************/

/*============================================================================*/
static void console_task(void* param)
/*------------------------------------------------------------------------------
  Function:
  this is the actual task that handles the console i/o. at startup, the version
  information is printed, then the line editor is called which waits for user
  input.
  in:  param -> not used
  out: none
==============================================================================*/
{
  (void)param;
  rs232_init();
  vt100_init(&term, interpreter, stdout, stdin);
  (void)info(0, NULL);
  for(;;)
  {
    /* the line editor receives characters and then invokes the interpreter
       callback function with the input tokens */
    if(vt100_lineeditor(&term) != 0)
    {
      const char* err = strerror(errno);
      (void)printf("%s\n", err);
      vTaskDelay(100);
    }
  }
}


/*============================================================================*/
static int interpreter(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  the interpreter gets the number of arguments and the individual arguments,
  searches the command to be executed and passes the arguments to it.
  in:  argc -> number of arguments, including the command itself
       argv -> array of strings
  out: if the command is found, the command's exit code is returned. for empty
       commands, the return code 0 is used (success) and if the command cannot
       be found, -1 is returned (error). commands shall use the same behaviour
       (return 0 on success, -1 on error.)
==============================================================================*/
{
  if(argc >= 1)
  {
    /* the actual command is always first */
    const char* command = argv[0];

    /* do nothing if command is empty */
    if(strlen(command) > 0)
    {
      int numcmds = sizeof(cmds) / sizeof(cmds[0]);
      for(int i = 0; i < numcmds; i++)
      {
        if(!strcmp(command, cmds[i].name))
        {
          return cmds[i].func(argc-1, &argv[1]);
        }
      }
    }
    else
    {
      return 0;
    }
  }

  /* command not found gives error message "not implemented" */
  errno = ENOSYS;
  return -1;
}


/*============================================================================*/
static int help(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  the help command displays all available commands together with their help
  strings.
  in:  argc -> not used
       argv -> not used
  out: always succeeds (returns 0)
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

  return 0;
}


/*============================================================================*/
static int gnss(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  configure the gnss systems to be used. if called without arguments, it
  displays which gnss systems are enabled. if called with arguments, the
  specified gnss systems are enabled and the others are disabled.
  in:  argc -> number of arguments, see below
       argv -> array of strings, any combination of "gps", "glonass" and
       "galileo"
  out: returns 0 on success, -1 in case of errors.
==============================================================================*/
{
  bool use_gps = false;
  bool use_glonass = false;
  bool use_galileo = false;
  config_t* cfg = get_config();
  if(argc == 0)
  {
    (void)printf("use GPS: %s\n", (cfg->use_gps ? "yes" : "no"));
    (void)printf("use GLONASS: %s\n", (cfg->use_glonass ? "yes" : "no"));
    (void)printf("use GALILEO: %s\n", (cfg->use_galileo ? "yes" : "no"));
    return 0;
  }
  else if(argc < 4)
  {
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
        (void)printf("unknown GNSS <%s> - abort\n", argv[i]);
        errno = EINVAL;
        return -1;
      }
    }

    cfg->use_gps = use_gps;
    cfg->use_galileo = use_galileo;
    cfg->use_glonass = use_glonass;

    /* inform the gps module about the changed configuration */
    reconfigure_gnss();
    return 0;
  }
  else
  {
    errno = E2BIG;
    return -1;
  }
}


/*============================================================================*/
static int elev_mask(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  configure the elevation mask. if called without arguments, it will return the
  currently configured elevation mask; if called with one numeric argument, a
  new elevation mask will be configured.
  in:  argc -> number of arguments, see below
       argv -> a number between 0 and 90
  out: returns 0 on success, -1 in case of errors.
==============================================================================*/
{
  if(argc == 0)
  {
    (void)printf("elevation mask: %d\n", get_config()->elevation_mask);
    return 0;
  }
  else if(argc == 1)
  {
    int32_t elevmask;
    if(str2num(argv[0], &elevmask, 0, 90))
    {
      get_config()->elevation_mask = (int8_t)elevmask;
    }

    return 0;
  }
  else
  {
    errno = E2BIG;
    return -1;
  }
}


/*============================================================================*/
static int conf(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  displays or saves the currently used configuration data. if called without
  arguments, the configuration is displayed. if called as "conf save", the
  current configuration is saved to the eeprom.
  in:  argc -> number of arguments, see below
       argv -> "save" to save the configuration
  out: returns 0 on success, -1 in case of errors.
==============================================================================*/
{
  if(argc == 0)
  {
    errno = EINVAL;
    return -1;
  }
  else if(argc == 1)
  {
    if(!strcmp(argv[0], "show"))
    {
      config_t* cfg = get_config();
      (void)printf("last DAC value: %d\n", cfg->last_dacval);
      (void)printf("fixed position valid: %s\n", (cfg->fixpos_valid ? "yes" : "no"));
      (void)printf("ECEF X position: %ld cm\n", cfg->x);
      (void)printf("ECEF Y position: %ld cm\n", cfg->y);
      (void)printf("ECEF Z position: %ld cm\n", cfg->z);
      (void)printf("position accuracy: %lu mm\n", cfg->accuracy);

      (void)gnss(0, NULL);
      (void)elev_mask(0, NULL);
      (void)svin(0, NULL);
      (void)timeconst(0, NULL);
      (void)pps(0, NULL);
      return 0;
    }
    else if(!strcmp(argv[0], "save"))
    {
      save_config();
      (void)printf("config saved!\n");
      return 0;
    }

  }

  errno = E2BIG;
  return -1;
}


/*============================================================================*/
static int disp(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  enables the auto-display
  in:  argc -> not used
       argv -> not used
  out:
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
      const gnssstatus_t* mygnss = get_gnss_status();
      const cntlstatus_t* ctl = get_cntlstatus();

      uint32_t meanv = (uint32_t)sqrtf((float)mygnss->svi->meanv);


      (void)printf("%-10llu e=%-7.2f eI=%-10.4f D=%-5d I=%.1f T=%.1f sat=%-2d " \
                   "lat=%ld lon=%ld obs=%-5lu mv=%-5lu tacc=%-3lu " \
                   "status=%s\n",
                   now, ctl->e, ctl->esum, stat_dac, i, t, mygnss->sat->numsv,
                   mygnss->pvt->lat, mygnss->pvt->lon, mygnss->svi->obs, meanv,
                   mygnss->pvt->tacc, ctl->mode);
      if(canread())
      {
        return 0;
      }
    }
  }
  else
  {
    errno = EINVAL;
    return -1;
  }
}


/*============================================================================*/
static int svin(int argc, const char* const argv[])
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
    /* just svin */
    case 0:
    {
      config_t* cfg = get_config();

      (void)printf("survey-in duration: %lu sec\n", cfg->svin_dur);
      (void)printf("survey-in accuracy limit: %lu mm\n", cfg->accuracy_limit);
      return 0;
    }

    /* svin stop */
    case 1:
    {
      if(!strcmp(argv[0], "stop"))
      {
        manual_svin_stop();
        return 0;
      }

      errno = EINVAL;
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
        manual_svin();
        return 0;
      }
      break;
    }

    default:
    {
      errno = E2BIG;
      break;
    }
  }

  return -1;
}


/*============================================================================*/
static int sat(int argc, const char* const argv[])
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
    const gnssstatus_t* mygnss = get_gnss_status();

    for(int i = 0; i < mygnss->sat->numsv; i++)
    {
      const char* syst;
      switch(mygnss->sat->sats[i].gnssid)
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
        syst, mygnss->sat->sats[i].svid, mygnss->sat->sats[i].cno,
        mygnss->sat->sats[i].azim, mygnss->sat->sats[i].elev);
    }

    const char* fixtype;
    switch(mygnss->pvt->fixtype)
    {
      case FIX_3D:
      {
        fixtype = "3D fix";
        break;
      }

      case FIX_TIME:
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

    uint64_t age = get_uptime_msec() - mygnss->sat->time;
    (void)printf("%d sats; last update: %llu ms ago\n\n", mygnss->sat->numsv, age);
    return 0;
  }

  errno = E2BIG;
  return -1;
}


/*============================================================================*/
static int timeconst(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  configure time constant and prefilter
  in:  argc -> number of arguments, see below
       argv -> array of strings; two required
  out: none
==============================================================================*/
{
  if(argc == 0)
  {
    (void)printf("time constant: %d sec\n", get_config()->tau);
    return 0;
  }
  else if(argc == 1)
  {
    int32_t tau;
    if(str2num(argv[0], &tau, 10, 7200))
    {
      get_config()->tau = (uint16_t)tau;
      return 0;
    }

    return -1;
  }

  errno = E2BIG;
  return -1;
}


/*============================================================================*/
static int hold(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  enable/disable DAC hold
  in:  argc -> number of arguments, see below
       argv -> array of strings; one required ("on" or "off")
  out: none
==============================================================================*/
{
  if(argc == 0)
  {
    (void)printf("manual hold: %s\n", dac_gethold() ? "on" : "off");
    return 0;
  }
  else if(argc == 1)
  {
    /* command given is "hold off" ? */
    if(!strcmp(argv[0], "off"))
    {
      dac_sethold(false);
      return 0;
    }
    else if(!strcmp(argv[0], "on"))
    {
      dac_sethold(true);
      return 0;
    }
    else
    {
      /* command is given hold <number> ? */
      int32_t dac;
      if(str2num(argv[0], &dac, 0, UINT16_MAX))
      {
        dac_sethold(true);
        set_dac((uint16_t)dac);
        return 0;
      }
      else
      {
        return -1;
      }
    }
  }

  errno = E2BIG;
  return -1;
}


/*============================================================================*/
static int uptime(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  shows the uptime. does not need arguments
  in:  none
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
    return 0;
  }
  errno = E2BIG;
  return -1;
}


/*============================================================================*/
static int info(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  show version info
  in:  none
  out: none
==============================================================================*/
{
  (void)argc;
  (void)argv;
  (void)printf("\033[2J\rHB9FSX GNSS Frequency Standard\n");
  (void)printf("compiled " __DATE__ " " __TIME__ "\n");
  (void)printf("with FreeRTOS " tskKERNEL_VERSION_NUMBER "\n");
  (void)printf("use \"help\" to see the available commands\n\n");
  return 0;
}


/*============================================================================*/
static int pps(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  set the duration of the pps pulse
  in:  none
  out: none
==============================================================================*/
{
  /* shorthand */
  config_t* cfg = get_config();

  if(argc == 0)
  {
    (void)printf("pps offset: %ld ns\n", cfg->timeoffset);
    (void)printf("pps duration: %lu ms\n", cfg->pps_dur);
    return 0;
  }
  else if(argc == 2)
  {
    if(!strcmp(argv[0], "offset"))
    {
      int32_t off;
      if(str2num(argv[1], &off, -500000000, 500000000))
      {
        cfg->timeoffset = off;
        return 0;
      }
      else
      {
        return -1;
      }
    }
    else if(!strcmp(argv[0], "dur"))
    {
      int32_t dur;
      if(str2num(argv[1], &dur, 1, 999))
      {
        cfg->pps_dur = (uint32_t)dur;
        set_pps_duration((uint32_t)dur);
        return 0;
      }
      else
      {
        return -1;
      }
    }
  }

  errno = EINVAL;
  return -1;
}


/*============================================================================*/
static int reboot(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  tight loop to force a watchdog reset
  in:  none
  out: none
==============================================================================*/
{
  (void)argc;
  (void)argv;
  __NVIC_SystemReset();
  return 0; /*lint !e527 unreachable */
}


/*============================================================================*/
static int date(int argc, const char* const argv[])
/*------------------------------------------------------------------------------
  Function:
  print date and time
  in:  none
  out: none
==============================================================================*/
{
  (void)argc;
  (void)argv;
  char buffer[30];
  datetime_print(buffer);
  (void)puts(buffer);
  return 0;
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
    return false;
  }
  if(errno == ERANGE)
  {
    return false;
  }
  if((num < min) || (num > max))
  {
    errno = ERANGE;
    return false;
  }
  *value = num;
  return true;
}

/*******************************************************************************
 * END OF CODE
 ******************************************************************************/
