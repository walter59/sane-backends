/* sane - Scanner Access Now Easy. */

/** @file pie.c

   Copyright (C) 2000 Simon Munton, based on the umax backend by Oliver Rauch

   Copyright (C) 2012 Michael Rickmann <mrickma@gwdg.de>
                 for initial SCSI over USB additions

   This file is part of the SANE package.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston,
   MA 02111-1307, USA.

   As a special exception, the authors of SANE give permission for
   additional uses of the libraries contained in this release of SANE.

   The exception is that, if you link a SANE library with other files
   to produce an executable, this does not by itself cause the
   resulting executable to be covered by the GNU General Public
   License.  Your use of that executable is in no way restricted on
   account of linking the SANE library code into it.

   This exception does not, however, invalidate any other reasons why
   the executable file might be covered by the GNU General Public
   License.

   If you submit changes to SANE to the maintainers to be included in
   a subsequent release, you agree by submitting the changes that
   those changes may be distributed with this exception intact.

   If you write modifications of your own for SANE, it is your choice
   whether to permit this exception to apply to your modifications.
   If you do not wish that, delete this exception notice.
 * <hr>
 * History
 *
 * 19-8-2012 add PIE "SF Scanner" USB film scanner support
 *
 * 19-8-2012 the names of all internal functions start with pie_ now
 *
 * 22-2-2003 set devlist to NULL in sane_exit()
 *           set first_dev to NULL in sane_exit()
 *           eliminated num_devices
 *
 * 23-7-2002 added TL_X > BR_X, TL_Y > BR_Y check in sane_start
 *
 * 17-9-2001 changed ADLIB to AdLib as the comparison is case sensitive and
 * 	     the scanner returns AdLib
 *
 * 7-5-2001 removed removal of '\n' after sanei_config_read()
 *	    free devlist allocated in sane_get_devices() on sane_exit()
 *
 * 2-3-2001 improved the reordering of RGB data in pie_reader_process()
 *
 * 11-11-2000 eliminated some warnings about signed/unsigned comparisons
 *            removed #undef NDEBUG and C++ style comments
 *
 * 1-10-2000 force gamma table to one to one mappping if lineart or halftone selected
 *
 * 30-9-2000 added ADLIB devices to scanner_str[]
 *
 * 29-9-2000 wasn't setting 'background is halftone bit' (BGHT) in halftone mode
 *
 * 27-9-2000 went public with build 4
 *
 *
 * <hr>
 * The source code is divided in sections which you can find by
 * searching for the tag "@@"
 *
 * - Definitions  and static allocation
 * - Utility functions
 * - SCSI over USB functions
 * - Initialization for SCSI and USB scanners
 * - Mid level USB functions
 * - USB calibration functions
 * - USB image reading and processing
 * - pie_usb_sane_xy operations
 * - SCSI functions
 * - sane_xy operations
 */


#include "../include/sane/config.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>

#include "../include/byteorder.h"	/* TODO portable ? */
#include "../include/sane/sane.h"
#include "../include/sane/sanei.h"
#include "../include/sane/saneopts.h"
#include "../include/sane/sanei_scsi.h"
#include "../include/sane/sanei_usb.h"
#include "../include/sane/sanei_debug.h"

#define BACKEND_NAME	pie

#include "../include/sane/sanei_backend.h"
#include "../include/sane/sanei_config.h"

#include "../include/sane/sanei_thread.h"
#include "../include/sane/sanei_ir.h"
#include "../include/sane/sanei_magic.h"

/*
 * @@ Definitions  and static allocation
 */

#include "pie-scsidef.h"
#include "pie_usb.h"

#define DBG_error0  0
#define DBG_error   1
#define DBG_sense   2
#define DBG_warning 3
#define DBG_inquiry 4

#define DBG_info    5
#define DBG_info2   6
#define DBG_proc    7
#define DBG_read    8
#define DBG_sane_init   10
#define DBG_sane_proc   11
#define DBG_sane_info   12
#define DBG_sane_option 13
#define DBG_dump	14
#define DBG_image	15
#define DBG_poke        16

#define BUILD 10

#define PIE_CONFIG_FILE "pie.conf"

/* Option string defines */

#define LINEART_STR             SANE_VALUE_SCAN_MODE_LINEART
#define HALFTONE_STR		SANE_VALUE_SCAN_MODE_HALFTONE
#define GRAY_STR		SANE_VALUE_SCAN_MODE_GRAY
#define COLOR_STR		SANE_VALUE_SCAN_MODE_COLOR
#define COLOR_IR_STR		"RGBI"

#define IR_NAME_STR             "swired"
#define IR_TITLE_STR            "Infrared processing"
#define IR_DESC_STR		"What to do with infrared plane";
#define IR_SPECT_STR		"Reduce red overlap"
#define IR_CLEAN_STR		"Remove dirt"

#define THE_NONE_STR             "None"

#define CROP_NAME_STR             "swcrop"
#define CROP_TITLE_STR            "Cropping"
#define CROP_DESC_STR             "How to crop the image";
#define CROP_OUTER_STR            "Outside"
#define CROP_INNER_STR            "Inside"

/* Color modes the scanner is operated in */

#define LINEART				1
#define HALFTONE			2
#define GRAYSCALE			3
#define RGB				4
/* USB film scanners: infrared modus */
#define RGBI				8

/* USB film scanners: post scan processing */

#define POST_SW_COLORS          (1 << 0)	/* gain, negatives, ..., can be done at any time */
#define POST_SW_IRED		(1 << 1)	/* remove spectral overlap, needs complete scan */
#define POST_SW_DIRT		(1 << 2)	/* our digital lavabo, needs complete scan */
#define POST_SW_GRAIN		(1 << 3)	/* smoothen a bit */
#define POST_SW_CROP		(1 << 4)	/* trim whole image in sane_start
                                                   before sane_get_parameters() is answered */
#define POST_SW_IRED_MASK       (POST_SW_IRED | POST_SW_DIRT)
#define POST_SW_ACCUM_MASK	(POST_SW_IRED_MASK | POST_SW_GRAIN | POST_SW_CROP)

/* SCSI scanners: calibration modes */

#define CAL_MODE_PREVIEW        (INQ_CAP_FAST_PREVIEW)
#define CAL_MODE_FLATBED        0x00
#define CAL_MODE_ADF            (INQ_OPT_DEV_ADF)
#define CAL_MODE_TRANPSARENCY   (INQ_OPT_DEV_TP)
#define CAL_MODE_TRANPSARENCY1  (INQ_OPT_DEV_TP1)

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

/* names of scanners that are supported because */
/* the inquiry_return_block is ok and driver is tested,
 * for USB film scanners a first rough identification
 */
static char *scanner_str[] = {
  "DEVCOM", "9636PRO",
  "DEVCOM", "9636S",
  "DEVCOM", "9630S",
  "PIE", "ScanAce 1236S",
  "PIE", "ScanAce 1230S",
  "PIE", "ScanAce II",
  "PIE", "ScanAce III",
  "PIE", "ScanAce Plus",
  "PIE", "ScanAce II Plus",
  "PIE", "ScanAce III Plus",
  "PIE", "ScanAce V",
  "PIE", "ScanMedia",
  "PIE", "ScanMedia II",
  "PIE", "ScanAce 630S",
  "PIE", "ScanAce 636S",
  "AdLib", "JetScan 630",
  "AdLib", "JetScan 636PRO",
  /* USB film scanners */
  "PIE", "SF Scanner",
  "PIE", "MS Scanner",
  "END_OF_LIST"
};

/* times (in us) to delay after certain commands. Scanner seems to lock up if it returns busy
 * status and commands are repeatedly reissued (by kernel error handler) */

#define DOWNLOAD_GAMMA_WAIT_TIME	(1000000)
#define SCAN_WAIT_TIME			(1000000)
#define SCAN_WARMUP_WAIT_TIME		(500000)
#define TUR_WAIT_TIME			(500000)

/* otherwise lots of warnings when compiled with pthread enabled */
#if defined USE_PTHREAD
#define NO_PID (pthread_t) -1
#else
#define NO_PID -1
#endif

/**
 * This option list has to contain all options for all SCSI and USB scanners
 * supported by this driver. If a certain scanner cannot handle a certain
 * option, there's the possibility to say so, later.
 */
enum Pie_Option
{
  OPT_NUM_OPTS = 0,

  /* ------------------------------------------- */
  OPT_MODE_GROUP,
  OPT_MODE,
  OPT_BIT_DEPTH,
  OPT_RESOLUTION,


  /* ------------------------------------------- */

  OPT_GEOMETRY_GROUP,
  OPT_TL_X,			/* top-left x */
  OPT_TL_Y,			/* top-left y */
  OPT_BR_X,			/* bottom-right x */
  OPT_BR_Y,			/* bottom-right y */

  /* ------------------------------------------- */

  OPT_ENHANCEMENT_GROUP,

  OPT_HALFTONE_PATTERN,         /* SCSI only */
  OPT_SPEED,
  OPT_THRESHOLD,                /* SCSI only */

  OPT_SW_CROP,                  /* USB only */
  OPT_SW_IRED,                  /* USB only */
  OPT_SW_GRAIN,                 /* USB only */
  OPT_SW_SRGB,                  /* USB only */
  OPT_SW_NEGA,                  /* USB only */

  OPT_GAMMA_VECTOR,             /* SCSI only */
  OPT_GAMMA_VECTOR_R,           /* SCSI only */
  OPT_GAMMA_VECTOR_G,           /* SCSI only */
  OPT_GAMMA_VECTOR_B,           /* SCSI only */

  /* ------------------------------------------- */

  OPT_ADVANCED_GROUP,
  OPT_PREVIEW,

  /* must come last: */
  NUM_OPTIONS
};


/**
 * @brief This defines the information needed during calibration
 * of SCSI scanners
 */
struct Pie_cal_info
{
  int cal_type;
  int receive_bits;
  int send_bits;
  int num_lines;
  int pixels_per_line;
};

/**
 * @brief This structure holds the information about a physical scanner
 */
typedef struct Pie_Device
{
  struct Pie_Device *next;

  char *devicename;		/**< name of the scanner device */

  char *vendor;			/**< will be xxxxx */
  char *product;		/**< e.g. "SuperVista_S12" or so */
  char *version;		/**< e.g. V1.3 */

  PIE_USB_Model *model;		/**< USB scanner model, NULL for SCSI scanners */
  SANE_Device sane;
  SANE_Range dpi_range;
  SANE_Range x_range;
  SANE_Range y_range;

  SANE_Range exposure_range;
  SANE_Range shadow_range;
  SANE_Range highlight_range;

  int inquiry_len;		/**< length of inquiry return block */

  int inquiry_x_res;		/**< maximum x-resolution */
  int inquiry_y_res;		/**< maximum y-resolution */
  int inquiry_pixel_resolution;
  double inquiry_fb_width;	/**< flatbed width in inches */
  double inquiry_fb_length;	/**< flatbed length in inches */

  int inquiry_trans_top_left_x;
  int inquiry_trans_top_left_y;
  double inquiry_trans_width;	/**< transparency width in inches */
  double inquiry_trans_length;	/**< transparency length in inches */

  int inquiry_halftones;	/**< number of halftones supported */
  int inquiry_filters;		/**< available colour filters */
  int inquiry_color_depths;	/**< available colour depths */
  int inquiry_color_format;	/**< colour format from scanner */
  int inquiry_image_format;	/**< image data format */
  int inquiry_scan_capability;	/**< additional scanner features, number of speeds */
  int inquiry_optional_devices;	/**< optional devices */
  int inquiry_enhancements;	/**< enhancements */
  int inquiry_gamma_bits;	/**< no of bits used for gamma table */
  int inquiry_fast_preview_res;	/**< fast preview resolution */
  int inquiry_min_highlight;	/**< min highlight % that can be used */
  int inquiry_max_shadow;	/**< max shadow % that can be used */
  int inquiry_cal_eqn;		/**< which calibration equation to use */
  int inquiry_min_exp;		/**< min exposure % */
  int inquiry_max_exp;		/**< max exposure % */

  SANE_String scan_mode_list[7];	/**< holds names of types of scan (color, ...) */
  SANE_String ir_sw_list[4];		/**< holds names for infrared processing */
  SANE_String crop_sw_list[4];          /**< holds names for cropping */

  SANE_Word bpp_list[4];	/**< USB film scanners: 8, 16 */
  SANE_Word grain_sw_list[6];	/**< USB film scanners: 0, 1, 2, 3, 4 grain removal */

  SANE_String halftone_list[17];	/**< holds the names of the halftone patterns from the scanner */

  SANE_String speed_list[9];	/**< holds the names of available speeds */

  int cal_info_count;		/**< number of calibration info sets */
  struct Pie_cal_info *cal_info;	/**< points to the actual calibration information */
  /** sanei_scsi_cmd or pie_usb_scsi_wrapper */
  SANE_Status (*scsi_cmd) (int fd, const void *src, size_t src_size,
                           void *dst, size_t * dst_size);
}
Pie_Device;

/**
 * @brief This structure holds information about an instance of an 'opened' scanner
 */
typedef struct Pie_Scanner
{
  struct Pie_Scanner *next;
  Pie_Device *device;		/**< pointer to physical scanner */

  int sfd;			/**< scanner file desc. */
  int bufsize;			/**< max scsi buffer size */

  SANE_Option_Descriptor opt[NUM_OPTIONS];	/**< option descriptions for this instance */
  Option_Value val[NUM_OPTIONS];	/**< option settings for this instance */
  SANE_Int *gamma_table[4];	/**< gamma tables for this instance */
  SANE_Range gamma_range;
  int gamma_length;		/**< size of gamma table */

  uint16_t *gamma_lut8;		/**< USB scanners gamma lookup tables */
  uint16_t *gamma_lut16;
  double *ln_lut;              /**< USB scanners logarithm lookup */

  int scanning;			/**< true if actually doing a scan */
  SANE_Parameters params;

  SANE_Pid parking_pid;         /**< USB scanners may use a thread to watch parking */
  SANE_Pid reader_pid;
  int pipe;
  int reader_fds;
  
  int colormode;		/**< whether RGBI, RGB, GRAY, LINEART, HALFTONE */
  int processing;		/**< USB scanners may process scan data */
  int resolution;
  int cal_mode;			/**< set to value to compare cal_info mode to */

  int cal_filter;		/**< set to indicate which filters will provide data for cal */
  PIE_USB_Calibration *cal_data;	/**< USB only, is updated during calibration */

  int filter_offset1;		/**< offsets between colors in indexed scan mode */
  int filter_offset2;

  int bytes_per_line;		/**< number of bytes per line */

  SANEI_IR_bufptr img_buffer;  /**< USB: store a whole image in RGB(I) format */
  int total_bytes_stored;
  int total_bytes_read;         /**< what has been read from the stored image */
}
Pie_Scanner;

/* forward declarations */

static SANE_Status pie_usb_read_status (int dn, unsigned char *buf);
SANE_Status
pie_usb_scsi_wrapper (int fd, const void *src, size_t src_size, void *dst,
                      size_t * dst_size);
static SANE_Status
pie_sense_handler (int scsi_fd, unsigned char *result, void *arg);
static SANE_Status pie_wait_scanner (Pie_Scanner * scanner);
static SANE_Status pie_send_exposure (Pie_Scanner * scanner);
static SANE_Status pie_send_highlight_shadow (Pie_Scanner * scanner);
static SANE_Status pie_power_save (Pie_Scanner * scanner, int time);
static SANE_Status pie_attach_one (const char *name);


/* USB scanners can not be "killed" by a cancel
 * but have to complete their current USB transaction,
 *  used to gracefully cancel the child thread / process */
static volatile sig_atomic_t cancel_requ;

static const SANE_Range percentage_range_100 = {
  0 << SANE_FIXED_SCALE_SHIFT,	/* minimum */
  100 << SANE_FIXED_SCALE_SHIFT,	/* maximum */
  0 << SANE_FIXED_SCALE_SHIFT	/* quantization */
};

static Pie_Device *first_dev = NULL;
static Pie_Scanner *first_handle = NULL;
static const SANE_Device **devlist = NULL;

#define DBG_DUMP(level, buf, n)	{ if (DBG_LEVEL >= (level)) pie_dump_buffer(level,buf,n); }


/* USB film scanners we support
 */
static PIE_USB_Model crystalscan_7200_model = {
  "PIE/Reflecta",               /* Device vendor string */
  "CrystalScan 7200",           /* Device model name */
  0x30,                         /* Model ID */
  PIE_USB_FLAG_MIRROR_IMAGE |   /* flags */
    PIE_USB_FLAG_XRES_FAKE,
  /* operation mode values for preview, skip calibration and quality */
  {0x00, 0x08, 0x0a},
  75000,                        /* default brightness */
  /* R, G, B gain constant */
  {4.19682524E-04, 3.92060196E-04, 3.89647803E-04},
  /* R, G, B normal mode offset factor */
  {1.05, 1.05, 1.05},
  16,                           /* minimal gain */
  42,                           /* maximal gain in quality mode */
  0x0f00,                       /* normal mode maximal exposure time */
  10,                           /* gain calibration part */
  {{0x16e6, 0x0ff2, 0x0ff2},    /* RGB texp and gain, first calibration if quality mode */
   0x16e6, {0x21, 0x21, 0x19}},
  {{0x0be2, 0x0bcf, 0x0b88},    /* RGB texp and gain, first calibration if normal mode */
   0x0be2, {0x35, 0x2c, 0x27}}
};

static PIE_USB_Model proscan_7200_model = {
  "PIE/Reflecta",               /* Device vendor string */
  "ProScan 7200",               /* Device model name */
  0x36,                         /* Model ID */
  PIE_USB_FLAG_MIRROR_IMAGE,    /* flags */
  /* operation mode values for preview, skip calibration and quality */
  {0x00, 0x08, 0x02},
  75000,                        /* default brightness */
  /* R, G, B gain constant */
  {4.19682524E-04, 3.92060196E-04, 3.89647803E-04},
  /* R, G, B normal mode offset factor */
  {1.2229896394, 1.0447735936, 0.9805181615},
  16,                           /* minimal gain */
  42,                           /* maximal gain in quality mode */
  0x0f00,                       /* normal mode maximal exposure time */
  10,                           /* gain calibration part */
  {{0x2c89, 0x1eb7, 0x17ca},    /* RGB texp and gain, first calibration if quality mode */
   0x2c89, {0x25, 0x25, 0x28}},
  {{0x0e79, 0x0bff, 0x0c6c},    /* RGB texp and gain, first calibration if normal mode */
   0x0e79, {0x3f, 0x3d, 0x39}}
};

static PIE_USB_Model powerslide_3600_model = {
  "PIE/Reflecta",               /* Device vendor string */
  "Powerslide 3600/DigitDia 6000",               /* Device model name */
  0x3a,                         /* Model ID */
  PIE_USB_FLAG_MIRROR_IMAGE,    /* flags */
  /* operation mode values for preview, skip calibration and quality */
  {0x00, 0x08, 0x02},
  75000,                        /* default brightness */
  /* R, G, B gain constant */
  {4.19682524E-04, 3.92060196E-04, 3.89647803E-04},
  /* R, G, B normal mode offset factor */
  {1.2229896394, 1.0447735936, 0.9805181615},
  16,                           /* minimal gain */
  42,                           /* maximal gain in quality mode */
  0x0f00,                       /* normal mode maximal exposure time */
  10,                           /* gain calibration part */
  {{0x2c89, 0x1eb7, 0x17ca},    /* RGB texp and gain, first calibration if quality mode */
   0x2c89, {0x25, 0x25, 0x28}},
  {{0x0e79, 0x0bff, 0x0c6c},    /* RGB texp and gain, first calibration if normal mode */
   0x0e79, {0x3f, 0x3d, 0x39}}
};

/* list of USB descriptors, do not mind different models with the same USB id
 */
static PIE_USB_Device_Entry pie_usb_device_list[] = {
  {0x05e3, 0x0142, &powerslide_3600_model},     /* PIE Powerslide 3600 / Reflecta DigitDia 5000,6000 */
  {0x05e3, 0x0145, &crystalscan_7200_model},    /* Reflecta CrystalScan 7200, id 0x30 */
  {0x05e3, 0x0145, &proscan_7200_model},        /* Reflecta ProScan 7200, id 0x36 */
  {0, 0, 0}
};

/*
 * @@ Utility functions mostly for USB code
 */

/* ---------------------------------- PIE DUMP_BUFFER ---------------------------------- */
/**
 * @brief Debug output dumping hexadecimal bytes
 *
 * @param[in] level debug level at which to dump
 * @param[in] buf contains what to dump
 * @param[in] n number of bytes to dump
 */
static void
pie_dump_buffer (int level, unsigned char *buf, int n)
{
  char s[80], *p = s;
  int a = 0;

  while (n--)
    {
      if ((a % 16) == 0)
        p += sprintf (p, "  %04X  ", a);

      p += sprintf (p, "%02X ", *buf++);

      if ((n == 0) || (a % 16) == 15)
        {
          DBG (level, "%s\n", s);
          p = s;
        }
      a++;
    }
}

/* --------------------------------- PIE_USB_POKE_INTS --------------------------------- */
/**
 * @brief Read white space separated integers from a text file
 *
 * @param[in] filename name of file to read from
 * @param[out] nums Pointer to buffer to store the data
 * @param[in] len number of integers to read
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_INVAL - if the file could not be read
 *
 * Use this function to override and test calibration.
 */

static SANE_Status
pie_usb_poke_ints (char *filename, int *nums, int *len)
{
  FILE *afile;
  char buffer[1024];
  char *fres, *nptr;
  char *eptr = NULL;
  long int lnum;
  int i;

  afile = fopen (filename, "r");
  if (afile == NULL)
    {
      DBG (DBG_error, "pie_usb_poke_ints: file %s does not exist\n",
           filename);
      return SANE_STATUS_INVAL;
    }

  i = 0;
  do
    {
      fres = fgets (buffer, sizeof (buffer), afile);
      if (fres != NULL)
        {
          nptr = buffer;
          errno = 0;
          lnum = strtol (nptr, &eptr, 10);
          while ((errno == 0) && (nptr != eptr) && (i < *len))
            {
              nums[i] = lnum;
              i++;
              nptr = eptr;

              errno = 0;
              lnum = strtol (nptr, &eptr, 10);
            }
        }
    }
  while (fres != NULL);
  *len = i;

  fclose (afile);

  return SANE_STATUS_GOOD;
}

/* -------------------------------- PIE_USB_POKE_BYTES --------------------------------- */
/**
 * @brief Read white space separated bytes from a text file
 *
 * @param[in] filename name of file to read from
 * @param[out] nums Pointer to buffer to store the data
 * @param[in] len number of integers to read
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_INVAL - if the file could not be read
 *
 * Use this function to override and test calibration.
 */
static SANE_Status
pie_usb_poke_bytes (char *filename, unsigned char *nums, int *len)
{
  FILE *afile;
  char buffer[1024];
  char *fres, *nptr;
  char *eptr = NULL;
  long int lnum;
  int i;

  afile = fopen (filename, "r");
  if (afile == NULL)
    {
      DBG (DBG_error, "pie_usb_poke_bytes: file %s does not exist\n",
           filename);
      return SANE_STATUS_INVAL;
    }

  i = 0;
  do
    {
      fres = fgets (buffer, sizeof (buffer), afile);
      if (fres != NULL)
        {
          nptr = buffer;
          errno = 0;
          lnum = strtol (nptr, &eptr, 16);
          while ((errno == 0) && (nptr != eptr) && (i < *len))
            {
              nums[i] = lnum & 0xff;
              i++;
              nptr = eptr;

              errno = 0;
              lnum = strtol (nptr, &eptr, 16);
            }
        }
    }
  while (fres != NULL);
  *len = i;

  fclose (afile);

  return SANE_STATUS_GOOD;
}


/* ------------------------------ PIE_USB_WRITE_PNM_FILE ------------------------------- */
/**
 * @brief Write RGB or grey scale image to a pnm file , with big endian byte order.
 *
 * @param[in] filename Name of file
 * @param[in] data Pointer to image data
 * @param[in] channels Number of interleaved color planes
 * @param[in] pixels_per_line Dimension in x
 * @param[in] lines Dimension in y
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_INVAL - file could not be written
 */
static SANE_Status
pie_usb_write_pnm_file (char *filename, uint8_t * data, int depth,
                        int channels, int pixels_per_line, int lines)
{
  FILE *out;
  int count;

  DBG (DBG_proc,
       "pie_usb_write_pnm_file: depth=%d, channels=%d, ppl=%d, lines=%d\n",
       depth, channels, pixels_per_line, lines);

  out = fopen (filename, "w");
  if (!out)
    {
      DBG (DBG_error,
           "pie_usb_write_pnm_file: could nor open %s for writing: %s\n",
           filename, strerror (errno));
      return SANE_STATUS_INVAL;
    }
  if (depth == 1)
    {
      fprintf (out, "P4\n%d\n%d\n", pixels_per_line, lines);
    }
  else
    {
      fprintf (out, "P%c\n%d\n%d\n%d\n", channels == 1 ? '5' : '6',
               pixels_per_line, lines, (int) pow (2, depth) - 1);
    }
  if (channels == 3)
    {
      for (count = 0; count < (pixels_per_line * lines * 3); count++)
        {
          if (depth == 16)
            fputc (*(data + 1), out);
          fputc (*(data++), out);
          if (depth == 16)
            data++;
        }
    }
  else
    {
      if (depth == 1)
        {
          pixels_per_line /= 8;
        }
      for (count = 0; count < (pixels_per_line * lines); count++)
        {
          switch (depth)
            {
            case 8:
              fputc (*(data + count), out);
              break;
            case 16:
              fputc (*(data + 1), out);
              fputc (*(data), out);
              data += 2;
              break;
            default:
              fputc (data[count], out);
              break;
            }
        }
    }
  fclose (out);

  DBG (DBG_info, "pie_usb_write_pnm_file: finished\n");
  return SANE_STATUS_GOOD;
}

/* ------------------------------ PIE_USB_SHADES_TO_PNM -------------------------------- */
/**
 * @brief Write a set of gray scale pnm files from shading data.
 *
 * @param[in] scanner points to structure holding the data
 * @param[in] name first part of file name
 * @param[in] lines height of image
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_NO_MEM - image buffer not allocated
 * - SANE_STATUS_INVAL - file could not be written
 */
static SANE_Status
pie_usb_shades_to_pnm (Pie_Scanner * scanner, char *name, int lines)
{
  char plane[4][12] = { "-red.pnm", "-green.pnm", "-blue.pnm", "-ired.pnm" };
  char filename[256];
  unsigned char *buffer, *dest;
  int *shade[4];
  int pixels_per_line;
  int snd_length, bits;
  int i, j, k, val;
  SANE_Int status;

  DBG (DBG_proc, "pie_usb_shades_to_pnm\n");

  pixels_per_line = scanner->device->cal_info[0].pixels_per_line;
  snd_length = pixels_per_line;
  bits = scanner->device->cal_info[0].receive_bits;
  if (bits > 8)
    snd_length *= 2;

  buffer = malloc (snd_length * lines);
  if (!buffer)
    return SANE_STATUS_NO_MEM;
  for (k = 0; k < 4; k++)
    {
      shade[k] = scanner->cal_data->shades + k * pixels_per_line;
    }

  for (j = 0; j < 4; j++)
    {
      dest = buffer;
      if (bits > 8)
        for (i = 0; i < pixels_per_line; i++)
          {
            val = shade[j][i];
            *dest++ = val & 0xff;
            *dest++ = (val >> 8) & 0xff;
          }
      else
        for (i = 0; i < pixels_per_line; i++)
          *dest++ = shade[j][i] & 0xff;
      for (i = 1; i < lines; i++)
        {
          memcpy (dest, buffer, snd_length);
          dest += snd_length;
        }

      strncpy (filename, name, 240);
      strncat (filename, plane[j], 255);
      status = pie_usb_write_pnm_file (filename, buffer, bits,
                                       1, pixels_per_line, lines);
      if (status != SANE_STATUS_GOOD)
        return status;
    }

  free (buffer);
  return SANE_STATUS_GOOD;
}

/*
 * @@ SCSI over USB and related functions
 */

/* ----------------------- PIE_USB_WRITE_CONTROL_SEQUENCE ------------------------ */
/**
 * @brief Write a control sequence of value-data pairs to the scanner
 *
 * @param[in] dn USB device number
 * @param[in] sequ list of value-data pairs terminated by a 0-0 pair
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_IO_ERROR - if an error occurred during the read
 * - SANE_STATUS_INVAL - on every other error
 */
static SANE_Status
pie_usb_write_control_sequence (SANE_Int dn, PIE_USB_Value_Data sequ[])
{
  SANE_Int status = SANE_STATUS_GOOD;
  int i;

  DBG (DBG_proc, "pie_usb_write_control_sequence writing\n");

  for (i = 0; (status == SANE_STATUS_GOOD) && (sequ[i].bValue != 0); i++)
    {
      status =
        sanei_usb_control_msg (dn, REQUEST_TYPE_OUT, REQUEST_REGISTER,
                               sequ[i].bValue, INDEX, 1, &(sequ[i].bData));
      if (status != SANE_STATUS_GOOD)
        {
          DBG (DBG_error, "pie_usb_write_control_sequence failed\n");
          return status;
        }
    }
  return SANE_STATUS_GOOD;
}

/* ---------------------------- PIE_USB_READY_STATE ------------------------------ */
/**
 * @brief Query the scanners state after the last USB transaction.
 *
 * @param[in] dn Device number
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 *
 * Actually, there are two answers, a simple one where 1 byte with 0 or 1 is ok,
 * and a more elaborate one in two bytes starting with 3. The second byte signals
 * some condition, 0 is ready.
 */
static SANE_Status
pie_usb_ready_state (SANE_Int dn)
{
  SANE_Status status;
  SANE_Byte val;

  status =
    sanei_usb_control_msg (dn, REQUEST_TYPE_IN, REQUEST_REGISTER,
                           VALUE_READ_REGISTER, INDEX, 1, &val);
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "pie_usb_ready_state failed at 1st read\n");
      return status;
    }
  DBG (DBG_info, "pie_usb_ready_state got 0x%02x at 1st read\n", val);

  if (val <= 1)
    return SANE_STATUS_GOOD;
  if (val != 3)
    {
      DBG (DBG_error, "pie_usb_ready_state failed\n");
      return SANE_STATUS_INVAL;
    }

  status =
    sanei_usb_control_msg (dn, REQUEST_TYPE_IN, REQUEST_REGISTER,
                           VALUE_READ_REGISTER, INDEX, 1, &val);
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "pie_usb_ready_state failed at 2nd read\n");
      return status;
    }
  DBG (DBG_info, "pie_usb_ready_state got 0x%02x at 2nd read\n", val);

  if (val == 0)
    return SANE_STATUS_GOOD;
  else if (val == 8)
    return SANE_STATUS_DEVICE_BUSY;
  else if (val == 2)
    return SANE_STATUS_IO_ERROR;
  else
    return SANE_STATUS_INVAL;
}

/* --------------------------- PIE_USB_WRITE_SCSI_CMD ---------------------------- */
/**
 * @brief Send 6 byte SCSI command to scanner
 *
 * @param[in] dn USB device number
 * @param[in] cmnd array holding the six bytes
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_IO_ERROR - if an error occurred during the read
 * - SANE_STATUS_INVAL - on every other error
 */
static SANE_Status
pie_usb_write_scsi_cmd (SANE_Int dn, const SANE_Byte cmnd[6])
{
  SANE_Status status;
  SANE_Byte mnd;
  int i;

  DBG (DBG_proc, "pie_usb_write_scsi_cmd writing 6 bytes\n");

  for (i = 0; i < 6; i++)
    {
      mnd = cmnd[i];
      status =
        sanei_usb_control_msg (dn, REQUEST_TYPE_OUT, REQUEST_REGISTER,
                               VALUE_WRITE_REGISTER, INDEX, 1, &mnd);
      if (status != SANE_STATUS_GOOD)
        {
          DBG (DBG_error, "pie_usb_write_scsi_cmd failed at byte %d\n", i);
          return status;
        }
    }
  return SANE_STATUS_GOOD;
}

/* ----------------------------- PIE_USB_BULK_READ ------------------------------- */
/**
 * @brief Read a lot of data
 *
 * @param[in] dn USB device number
 * @param[out] data points to buffer receiving data
 * @param len Requested / received number of bytes
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_INVAL - on every other error
 */
static SANE_Status
pie_usb_bulk_read (SANE_Int dn, SANE_Byte * data, size_t len)
{
  SANE_Status status;
  size_t size;
  SANE_Byte outdata[8];

  DBG (DBG_proc, "pie_usb_bulk_read requesting %lu bytes\n", (u_long) len);

  if (len == 0)

    return SANE_STATUS_GOOD;

  memset (outdata, '\0', sizeof (outdata));

  while (len)
    {
      if (len > BULKIN_MAXSIZE)
        size = BULKIN_MAXSIZE;
      else
        size = len;

      outdata[4] = (size & 0xff);
      outdata[5] = ((size >> 8) & 0xff);
      outdata[6] = ((size >> 16) & 0xff);
      outdata[7] = ((size >> 24) & 0xff);

      status =
        sanei_usb_control_msg (dn, REQUEST_TYPE_OUT, REQUEST_BUFFER,
                               VALUE_BUFFER, INDEX, sizeof (outdata),
                               outdata);
      if (status != SANE_STATUS_GOOD)
        {
          DBG (DBG_error,
               "pie_usb_bulk_read failed while writing command: %s\n",
               sane_strstatus (status));
          return status;
        }

      DBG (DBG_info,
           "pie_usb_bulk_read trying to read %lu bytes of data\n",
           (u_long) size);
      status = sanei_usb_read_bulk (dn, data, &size);
      if (status != SANE_STATUS_GOOD)
        {
          DBG (DBG_error,
               "pie_usb_bulk_read failed while reading bulk data: %s\n",
               sane_strstatus (status));
          return status;
        }

      DBG (DBG_info,
           "pie_usb_bulk_read read %lu bytes, %lu remaining\n",
           (u_long) size, (u_long) (len - size));
      len -= size;
      data += size;
    }

  DBG (DBG_info, "pie_usb_bulk_read completed\n");
  return SANE_STATUS_GOOD;
}

/* ------------------------------- PIE_USB_READ ---------------------------------- */
/**
 * @brief Do a SCSI read transaction over USB
 *
 * @param[in] dn USB device number
 * @param[in] cmnd SCSI command used for transaction
 * @param[out] buf points to buffer receiving
 * @param[in] buf_len requested number of bytes
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @note
 * The transfer length in a 6-byte READ command can have two meanings,
 * Normally it means length in bytes, however, when an image is read it amounts to
 * (lines * color-planes).
 */
static SANE_Status
pie_usb_read (int dn, const SANE_Byte * cmnd, void *buf, size_t * buf_len)
{
  SANE_Status status;
  size_t length = *buf_len;

  DBG (DBG_proc, "pie_usb_read\n");

  status = pie_usb_write_scsi_cmd (dn, cmnd);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_usb_ready_state (dn);
  if (status != SANE_STATUS_GOOD)
    return status;

  status = pie_usb_bulk_read (dn, buf, length);
  if (status != SANE_STATUS_GOOD)
    return status;

  return pie_usb_ready_state (dn);
}

/* ------------------------------- PIE_USB_WRITE --------------------------------- */
/**
 * @brief Do a SCSI write transaction over USB
 *
 * @param[in] dn USB device number
 * @param[in] cmnd SCSI command appended with the data to be written
 * @param[in] length total length, i.e. command + data
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @note
 * Data bytes are sent only one at a time, for SF scanners
 * there seems to be no bulk write.
 */
static SANE_Status
pie_usb_write (int dn, const SANE_Byte * cmnd, size_t length)
{
  SANE_Status status;
  SANE_Byte mnd;
  size_t i;

  DBG (DBG_proc, "pie_usb_write\n");
  if (length <= 6)
    return SANE_STATUS_GOOD;

  status = pie_usb_write_scsi_cmd (dn, cmnd);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_usb_ready_state (dn);
  if (status != SANE_STATUS_GOOD)
    return status;

  DBG (DBG_info, "pie_usb_write: now writing %lu bytes\n",
       (u_long) length - 6);
  for (i = 6; i < length; i++)
    {
      mnd = cmnd[i];
      status =
        sanei_usb_control_msg (dn, REQUEST_TYPE_OUT, REQUEST_REGISTER,
                               VALUE_WRITE_REGISTER, INDEX, 1, &mnd);
      if (status != SANE_STATUS_GOOD)
        {
          DBG (DBG_error, "pie_usb_write failed at byte %lu\n",
               (unsigned long) length);
          return status;
        }
    }

  return pie_usb_ready_state (dn);
}

/* ------------------------------ PIE_USB_COMMAND -------------------------------- */
/**
 * @brief Send a simple SCSI command without data transfer
 *
 * @param[in] dn USB device number
 * @param[in] cmnd points to SCSI command bytes
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_INVAL - on every other error
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 */
static SANE_Status
pie_usb_command (int dn, const SANE_Byte * cmnd)
{
  SANE_Status status;

  DBG (DBG_proc, "pie_usb_command\n");

  status = pie_usb_write_scsi_cmd (dn, cmnd);
  if (status != SANE_STATUS_GOOD)
    return status;

  return pie_usb_ready_state (dn);
}

/* ---------------------------- PIE_USB_SCSI_WRAPPER ----------------------------- */
/**
 * @brief The one and only entry for SCSI over USB command wrapping.
 *
 * @param[in] fd file descriptor holds USB device number
 * @param[in] src points to the SCSI command and appended write data (if any)
 * @param[in] src_size length of the command and data
 * @param[out] dst points to buffer receiving data, NULL if no data is returned
 * @param     dst_size requested / received number of bytes
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * Currently all communication with the scanners is done via this function.
 * Arguments are the same as for sanei_scsi_cmd.
 */
SANE_Status
pie_usb_scsi_wrapper (int fd, const void *src, size_t src_size, void *dst,
                      size_t * dst_size)
{
  /* values for some stereotype USB control write sequences */
  static PIE_USB_Value_Data PIE_USB_Init_Sequence_1[] = {
    {VALUE_INIT_1, 0x04},
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0xaa}, {VALUE_INIT_2, 0x55},
    {VALUE_INIT_2, 0x00},
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0x87}, {VALUE_INIT_2, 0x78},
    {VALUE_INIT_2, 0x30},
    {VALUE_INIT_1, 0x05}, {VALUE_INIT_1, 0x04},
    {VALUE_INIT_2, 0xff},
    {0x0, 0x0}
  };
  static PIE_USB_Value_Data PIE_USB_Init_Sequence_2[] = {
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0xaa}, {VALUE_INIT_2, 0x55},
    {VALUE_INIT_2, 0x00},
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0x87}, {VALUE_INIT_2, 0x78},
    {VALUE_INIT_2, 0x00},
    {VALUE_INIT_1, 0x05}, {VALUE_INIT_1, 0x04},
    {VALUE_INIT_2, 0xff},
    {0x0, 0x0}
  };
  static PIE_USB_Value_Data PIE_USB_Setup_SCSI_Sequence[] = {
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0xaa}, {VALUE_INIT_2, 0x55},
    {VALUE_INIT_2, 0x00},
    {VALUE_INIT_2, 0xff}, {VALUE_INIT_2, 0x87}, {VALUE_INIT_2, 0x78},
    {VALUE_INIT_2, 0xe0},
    {VALUE_INIT_1, 0x05}, {VALUE_INIT_1, 0x04},
    {VALUE_INIT_2, 0xff},
    {0x0, 0x0}
  };

  SANE_Status status = SANE_STATUS_GOOD;
  const SANE_Byte *cmnd = src;

  if (cmnd[0] == INQUIRY)
    {
      status = pie_usb_write_control_sequence (fd, PIE_USB_Init_Sequence_1);
      if (status != SANE_STATUS_GOOD)
        return status;
      status = pie_usb_write_control_sequence (fd, PIE_USB_Init_Sequence_2);
      if (status != SANE_STATUS_GOOD)
        return status;
    }
  status = pie_usb_write_control_sequence (fd, PIE_USB_Setup_SCSI_Sequence);
  if (status != SANE_STATUS_GOOD)
    return status;

  switch (cmnd[0])
    {
    case TEST_UNIT_READY:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing TEST_UNIT_READY\n");
      status = pie_usb_command (fd, cmnd);
      break;
    case REQUEST_SENSE:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing REQUEST_SENSE\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case READ:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing READ\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case WRITE:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing WRITE\n");
      status = pie_usb_write (fd, cmnd, src_size);
      break;
    case INQUIRY:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing INQUIRY\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case PARAM:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PARAM\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case MODE:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing MODE\n");
      status = pie_usb_write (fd, cmnd, src_size);
      break;
    case RESERVE_UNIT:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing RESERVE_UNIT\n");
      status = pie_usb_command (fd, cmnd);
      break;
    case RELEASE_UNIT:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing RELEASE_UNIT\n");
      status = pie_usb_command (fd, cmnd);
      break;
    case PIE_COPY:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PIE_COPY\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case SCAN:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing SCAN\n");
      status = pie_usb_command (fd, cmnd);
      break;
    case PIE_RELEASE_SCANNER:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PIE_RELEASE_SCANNER\n");
      status = pie_usb_write (fd, cmnd, src_size);
      break;
    case PIE_READ_CALIBRATION:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PIE_READ_CALIBRATION\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    case PIE_WRITE_CALIBRATION:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PIE_WRITE_CALIBRATION\n");
      status = pie_usb_write (fd, cmnd, src_size);
      break;
    case PIE_READ_STATUS:
      DBG (DBG_proc, "pie_usb_scsi_wrapper doing PIE_READ_STATUS\n");
      status = pie_usb_read (fd, cmnd, dst, dst_size);
      break;
    default:
      DBG (DBG_proc, "pie_usb_scsi_wrapper failed for command 0x%02x\n",
           cmnd[0]);
      status = SANE_STATUS_INVAL;
      break;
    }

  return status;
}

/* ---------------------------- PIE_USB_REQUEST_SENSE ---------------------------- */
/**
 *  @brief Send a Request Sense SCSI command to the scanner.
 *
 *  @param[in] dn USB device number
 *  @param[out] kascq amalgam single Integer returning result
 *
 *  @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @note
 *  There are a few cases where a request sense may help
 *  the USB scanner to recover from an "error", e.g.
 *  "Not Ready - Warming Up", "calibration disable not granted".
 */
static SANE_Status
pie_usb_request_sense (int dn, uint32_t * kascq)
{
  unsigned char buffer[16];     /* for PIE's sense */
  size_t size;
  SANE_Status status;

  DBG (DBG_proc, "pie_usb_request_sense\n");

  size = 14;
  set_RS_allocation_length (request_senseC, size);

  status =
    pie_usb_scsi_wrapper (dn, request_senseC, sizeof (request_senseC), buffer,
                          &size);
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "pie_usb_request_sense failed\n");
      return status;
    }
  *kascq = ((int) get_RS_sense_key (buffer) << 16) +
    ((int) get_RS_ASC (buffer) << 8) + (int) get_RS_ASCQ (buffer);
  pie_sense_handler (dn, buffer, NULL);

  return SANE_STATUS_GOOD;
}


/* -------------------------- PIE_SENSE_HANDLER ---------------------------- */
/**
 * @brief Called by sanei_scsi_cmd or pie_usb_request_sense
 *
 * @param[in] result points to data returned by a request sense command
 *
 * @return
 * - SANE_STATUS_DEVICE_BUSY - sense key invalid
 * - SANE_STATUS_IO_ERROR - no sense, hardware error, illegal request,
 *                              unit attention or vendor specific
 *
 * Except for debug output and the return value no major action.
 */
static SANE_Status
pie_sense_handler (__sane_unused__ int scsi_fd, unsigned char *result,
    __sane_unused__ void *arg)
{
  unsigned char asc, ascq, sensekey;
  int asc_ascq, len;
  /* Pie_Device *dev = arg; */

  DBG (DBG_proc, "check condition sense handler\n");

  sensekey = get_RS_sense_key (result);
  asc = get_RS_ASC (result);
  ascq = get_RS_ASCQ (result);
  asc_ascq = (int) (256 * asc + ascq);
  len = 7 + get_RS_additional_length (result);

  if (get_RS_error_code (result) != 0x70)
    {
      DBG (DBG_proc, "invalid sense key => handled as DEVICE BUSY!\n");
      return SANE_STATUS_DEVICE_BUSY;	/* sense key invalid */
    }

  DBG (DBG_sense, "check condition sense: %s\n", sense_str[sensekey]);

  if (get_RS_ILI (result) != 0)
    {
      DBG (DBG_sense,
	   "-> ILI-ERROR: requested data length is larger than actual length\n");
    }

  switch (sensekey)
    {
    case 0x00:			/* no sense, could have been busy */
      return SANE_STATUS_IO_ERROR;
      break;

    case 0x02:
      if (asc_ascq == 0x0401)
	DBG (DBG_sense, "-> Not Ready - Warming Up\n");
      else if (asc_ascq == 0x0483)
	DBG (DBG_sense, "-> Not Ready - Need manual service\n");
      else if (asc_ascq == 0x0881)
	DBG (DBG_sense, "-> Not Ready - Communication time out\n");
      else
	DBG (DBG_sense, "-> unknown medium error: asc=%d, ascq=%d\n", asc,
	     ascq);
      break;

    case 0x03:			/* medium error */
      if (asc_ascq == 0x5300)
	DBG (DBG_sense, "-> Media load or eject failure\n");
      else if (asc_ascq == 0x3a00)
	DBG (DBG_sense, "-> Media not present\n");
      else if (asc_ascq == 0x3b05)
	DBG (DBG_sense, "-> Paper jam\n");
      else if (asc_ascq == 0x3a80)
	DBG (DBG_sense, "-> ADF paper out\n");
      else
	DBG (DBG_sense, "-> unknown medium error: asc=%d, ascq=%d\n", asc,
	     ascq);
      break;


    case 0x04:			/* hardware error */
      if (asc_ascq == 0x4081)
	DBG (DBG_sense, "-> CPU RAM failure\n");
      else if (asc_ascq == 0x4082)
	DBG (DBG_sense, "-> Scanning system RAM failure\n");
      else if (asc_ascq == 0x4083)
	DBG (DBG_sense, "-> Image buffer failure\n");
      else if (asc_ascq == 0x0403)
	DBG (DBG_sense, "-> Manual intervention required\n");
      else if (asc_ascq == 0x6200)
	DBG (DBG_sense, "-> Scan head position error\n");
      else if (asc_ascq == 0x6000)
	DBG (DBG_sense, "-> Lamp or CCD failure\n");
      else if (asc_ascq == 0x6081)
	DBG (DBG_sense, "-> Transparency lamp failure\n");
      else if (asc_ascq == 0x8180)
	DBG (DBG_sense, "-> DC offset or black level calibration failure\n");
      else if (asc_ascq == 0x8181)
	DBG (DBG_sense,
	     "-> Integration time adjustment failure (too light)\n");
      else if (asc_ascq == 0x8182)
	DBG (DBG_sense,
	     "-> Integration time adjustment failure (too dark)\n");
      else if (asc_ascq == 0x8183)
	DBG (DBG_sense, "-> Shading curve adjustment failure\n");
      else if (asc_ascq == 0x8184)
	DBG (DBG_sense, "-> Gain adjustment failure\n");
      else if (asc_ascq == 0x8185)
	DBG (DBG_sense, "-> Optical alignment failure\n");
      else if (asc_ascq == 0x8186)
	DBG (DBG_sense, "-> Optical locating failure\n");
      else if (asc_ascq == 0x8187)
	DBG (DBG_sense, "-> Scan pixel map less than 5100 pixels!\n");
      else if (asc_ascq == 0x4700)
	DBG (DBG_sense, "-> Parity error on SCSI bus\n");
      else if (asc_ascq == 0x4b00)
	DBG (DBG_sense, "-> Data phase error\n");
      else
	DBG (DBG_sense, "-> unknown hardware error: asc=%d, ascq=%d\n", asc,
	     ascq);
      return SANE_STATUS_IO_ERROR;
      break;


    case 0x05:			/* illegal request */
      if (asc_ascq == 0x1a00)
	DBG (DBG_sense, "-> Parameter list length error\n");
      else if (asc_ascq == 0x2c01)
	DBG (DBG_sense, "-> Too many windows specified\n");
      else if (asc_ascq == 0x2c02)
	DBG (DBG_sense, "-> Invalid combination of windows\n");
      else if (asc_ascq == 0x2c81)
	DBG (DBG_sense, "-> Illegal scanning frame\n");
      else if (asc_ascq == 0x2400)
	DBG (DBG_sense, "-> Invalid field in CDB\n");
      else if (asc_ascq == 0x2481)
	DBG (DBG_sense, "-> Request too many lines of data\n");
      else if (asc_ascq == 0x2000)
	DBG (DBG_sense, "-> Invalid command OP code\n");
      else if (asc_ascq == 0x2501)
	DBG (DBG_sense, "-> LUN not supported\n");
      else if (asc_ascq == 0x2601)
	DBG (DBG_sense, "-> Parameter not supported\n");
      else if (asc_ascq == 0x2602)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Parameter not specified\n");
      else if (asc_ascq == 0x2603)
	DBG (DBG_sense, "-> Parameter value invalid - Invalid threshold\n");
      else if (asc_ascq == 0x2680)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Control command sequence error\n");
      else if (asc_ascq == 0x2681)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Grain setting (halftone pattern\n");
      else if (asc_ascq == 0x2682)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal resolution setting\n");
      else if (asc_ascq == 0x2683)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Invalid filter assignment\n");
      else if (asc_ascq == 0x2684)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal gamma adjustment setting (look-up table)\n");
      else if (asc_ascq == 0x2685)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal offset setting (digital brightness)\n");
      else if (asc_ascq == 0x2686)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal bits per pixel setting\n");
      else if (asc_ascq == 0x2687)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal contrast setting\n");
      else if (asc_ascq == 0x2688)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal paper length setting\n");
      else if (asc_ascq == 0x2689)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal highlight/shadow setting\n");
      else if (asc_ascq == 0x268a)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal exposure time setting (analog brightness)\n");
      else if (asc_ascq == 0x268b)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Invalid device select or device not exist\n");
      else if (asc_ascq == 0x268c)
	DBG (DBG_sense,
	     "-> Parameter value invalid - Illegal color packing\n");
      else if (asc_ascq == 0x3d00)
	DBG (DBG_sense, "-> Invalid bits in identify field\n");



      else if (asc_ascq == 0x4900)
	DBG (DBG_sense, "-> Invalid message\n");
      else if (asc_ascq == 0x8101)
	DBG (DBG_sense, "-> Not enough memory for color packing\n");

      if (len >= 0x11)
	{
	  if (get_RS_SKSV (result) != 0)
	    {
	      if (get_RS_CD (result) == 0)
		{

		  DBG (DBG_sense, "-> illegal parameter in CDB\n");
		}
	      else
		{
		  DBG (DBG_sense,
		       "-> illegal parameter is in the data parameters sent during data out phase\n");
		}

	      DBG (DBG_sense, "-> error detected in byte %d\n",
		   get_RS_field_pointer (result));
	    }
	}
      return SANE_STATUS_IO_ERROR;
      break;


    case 0x06:			/* unit attention */
      if (asc_ascq == 0x2900)
	DBG (DBG_sense, "-> power on, reset or bus device reset\n");
      if (asc_ascq == 0x8200)
	DBG (DBG_sense,
	     "-> unit attention - calibration disable not granted\n");
      if (asc_ascq == 0x8300)
	DBG (DBG_sense, "-> unit attention - calibration will be ignored\n");
      else
	DBG (DBG_sense, "-> unit attention: asc=%d, ascq=%d\n", asc, ascq);
      break;


    case 0x09:			/* vendor specific */
      DBG (DBG_sense, "-> vendor specific sense-code: asc=%d, ascq=%d\n", asc,
	   ascq);
      break;

    case 0x0b:
      if (asc_ascq == 0x0006)
	DBG (DBG_sense, "-> Received ABORT message from initiator\n");
      if (asc_ascq == 0x4800)
	DBG (DBG_sense, "-> Initiator detected error message received\n");
      if (asc_ascq == 0x4300)
	DBG (DBG_sense, "-> Message error\n");
      if (asc_ascq == 0x4500)
	DBG (DBG_sense, "-> Select or re-select error\n");
      else
	DBG (DBG_sense, "-> aborted command: asc=%d, ascq=%d\n", asc, ascq);
      break;

    }

  return SANE_STATUS_IO_ERROR;
}

/* --------------------------- PIE_USB_READ_STATUS ---------------------------- */
/**
 * Issue PIE vendor specific 0xdd PIE_READ_STATUS command
 *
 * @param[in] dn USB device number
 * @param[out] buffer Pointer to buffer receiving 11 bytes
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_INVAL - on every other error
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 */
static SANE_Status
pie_usb_read_status (int dn, unsigned char *buf)
{
  SANE_Status status;
  size_t size;

  DBG (DBG_proc, "pie_usb_read_status\n");

  size = 11;

  status =
    pie_usb_scsi_wrapper (dn, read_statusC, sizeof (read_statusC), buf,
                          &size);
  return status;
}


/*
 * @@ Initialization is rather similar for SCSI and USB scanners
 */

/* ---------------------------------- PIE INIT ---------------------------------- */

static void
pie_init (Pie_Device * dev, SANE_Int is_USB)    /* pie_init is called once while driver-initialization */
{
  DBG (DBG_proc, "init\n");

  dev->cal_info_count = 0;
  dev->cal_info = NULL;
  dev->halftone_list[0] = NULL;
  dev->speed_list[0] = NULL;

  dev->devicename = NULL;
  dev->inquiry_len = 0;
  dev->model = NULL;

  if (is_USB == 0)
    {
#ifdef HAVE_SANEI_SCSI_OPEN_EXTENDED
  DBG (DBG_info,
       "variable scsi buffer size (usage of sanei_scsi_open_extended)\n");
#else
  DBG (DBG_info, "fixed scsi buffer size = %d bytes\n",
       sanei_scsi_max_request_size);
#endif
      dev->scsi_cmd = sanei_scsi_cmd;
    }
  else
    {
      dev->scsi_cmd = pie_usb_scsi_wrapper;
    }
}

/* -------------------------------- PIE PRINT INQUIRY ------------------------- */

static void
pie_print_inquiry (Pie_Device * dev)
{
  DBG (DBG_inquiry, "INQUIRY:\n");
  DBG (DBG_inquiry, "========\n");
  DBG (DBG_inquiry, "\n");
  DBG (DBG_inquiry, "vendor........................: '%s'\n", dev->vendor);
  DBG (DBG_inquiry, "product.......................: '%s'\n", dev->product);
  DBG (DBG_inquiry, "version.......................: '%s'\n", dev->version);

  DBG (DBG_inquiry, "X resolution..................: %d dpi\n",
       dev->inquiry_x_res);
  DBG (DBG_inquiry, "Y resolution..................: %d dpi\n",
       dev->inquiry_y_res);
  DBG (DBG_inquiry, "pixel resolution..............: %d dpi\n",
       dev->inquiry_pixel_resolution);
  DBG (DBG_inquiry, "fb width......................: %f in\n",
       dev->inquiry_fb_width);
  DBG (DBG_inquiry, "fb length.....................: %f in\n",
       dev->inquiry_fb_length);

  DBG (DBG_inquiry, "transparency width............: %f in\n",
       dev->inquiry_trans_width);
  DBG (DBG_inquiry, "transparency length...........: %f in\n",
       dev->inquiry_trans_length);
  DBG (DBG_inquiry, "transparency offset...........: %d,%d\n",
       dev->inquiry_trans_top_left_x, dev->inquiry_trans_top_left_y);

  DBG (DBG_inquiry, "# of halftones................: %d\n",
       dev->inquiry_halftones);

  DBG (DBG_inquiry, "One pass color................: %s\n",
       dev->inquiry_filters & INQ_ONE_PASS_COLOR ? "yes" : "no");

  DBG (DBG_inquiry, "Filters.......................: %s%s%s%s (%02x)\n",
       dev->inquiry_filters & INQ_FILTER_RED ? "Red " : "",
       dev->inquiry_filters & INQ_FILTER_GREEN ? "Green " : "",
       dev->inquiry_filters & INQ_FILTER_BLUE ? "Blue " : "",
       dev->inquiry_filters & INQ_FILTER_NEUTRAL ? "Neutral " : "",
       dev->inquiry_filters);

  DBG (DBG_inquiry, "Color depths..................: %s%s%s%s%s%s (%02x)\n",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_16 ? "16 bit " : "",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_12 ? "12 bit " : "",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_10 ? "10 bit " : "",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_8 ? "8 bit " : "",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_4 ? "4 bit " : "",
       dev->inquiry_color_depths & INQ_COLOR_DEPTH_1 ? "1 bit " : "",
       dev->inquiry_color_depths);

  DBG (DBG_inquiry, "Color Format..................: %s%s%s (%02x)\n",
       dev->inquiry_color_format & INQ_COLOR_FORMAT_INDEX ? "Indexed " : "",
       dev->inquiry_color_format & INQ_COLOR_FORMAT_LINE ? "Line " : "",
       dev->inquiry_color_format & INQ_COLOR_FORMAT_PIXEL ? "Pixel " : "",
       dev->inquiry_color_format);

  DBG (DBG_inquiry, "Image Format..................: %s%s%s%s (%02x)\n",
       dev->inquiry_image_format & INQ_IMG_FMT_OKLINE ? "OKLine " : "",
       dev->inquiry_image_format & INQ_IMG_FMT_BLK_ONE ? "BlackOne " : "",
       dev->inquiry_image_format & INQ_IMG_FMT_MOTOROLA ? "Motorola " : "",
       dev->inquiry_image_format & INQ_IMG_FMT_INTEL ? "Intel" : "",
       dev->inquiry_image_format);

  DBG (DBG_inquiry,
       "Scan Capability...............: %s%s%s%s%d speeds (%02x)\n",
       dev->inquiry_scan_capability & INQ_CAP_PWRSAV ? "PowerSave " : "",
       dev->inquiry_scan_capability & INQ_CAP_EXT_CAL ? "ExtCal " : "",
       dev->inquiry_scan_capability & INQ_CAP_FAST_PREVIEW ? "FastPreview" :
       "",
       dev->inquiry_scan_capability & INQ_CAP_DISABLE_CAL ? "DisCal " : "",
       dev->inquiry_scan_capability & INQ_CAP_SPEEDS,
       dev->inquiry_scan_capability);

  DBG (DBG_inquiry, "Optional Devices..............: %s%s%s%s (%02x)\n",
       dev->inquiry_optional_devices & INQ_OPT_DEV_MPCL ? "MultiPageLoad " :
       "",
       dev->inquiry_optional_devices & INQ_OPT_DEV_TP1 ? "TransModule1 " : "",
       dev->inquiry_optional_devices & INQ_OPT_DEV_TP ? "TransModule " : "",
       dev->inquiry_optional_devices & INQ_OPT_DEV_ADF ? "ADF " : "",
       dev->inquiry_optional_devices);

  DBG (DBG_inquiry, "Enhancement...................: %02x\n",
       dev->inquiry_enhancements);
  DBG (DBG_inquiry, "Gamma bits....................: %d\n",
       dev->inquiry_gamma_bits);

  DBG (DBG_inquiry, "Fast Preview Resolution.......: %d\n",
       dev->inquiry_fast_preview_res);
  DBG (DBG_inquiry, "Min Highlight.................: %d\n",
       dev->inquiry_min_highlight);
  DBG (DBG_inquiry, "Max Shadow....................: %d\n",
       dev->inquiry_max_shadow);
  DBG (DBG_inquiry, "Cal Eqn.......................: %d\n",
       dev->inquiry_cal_eqn);
  DBG (DBG_inquiry, "Min Exposure..................: %d\n",
       dev->inquiry_min_exp);
  DBG (DBG_inquiry, "Max Exposure..................: %d\n",
       dev->inquiry_max_exp);
}


/* ------------------------------ PIE GET INQUIRY VALUES -------------------- */


static void
pie_get_inquiry_values (Pie_Device * dev, unsigned char *buffer)
{
  DBG (DBG_proc, "get_inquiry_values\n");

  dev->inquiry_len = get_inquiry_additional_length (buffer) + 5;

  dev->inquiry_x_res = get_inquiry_max_x_res (buffer);
  dev->inquiry_y_res = get_inquiry_max_y_res (buffer);

  if (dev->inquiry_y_res < 256)
    {
      /* y res is a multiplier */
      dev->inquiry_pixel_resolution = dev->inquiry_x_res;
      dev->inquiry_x_res *= dev->inquiry_y_res;
      dev->inquiry_y_res = dev->inquiry_x_res;
    }
  else
    {
      /* y res really is resolution */
      dev->inquiry_pixel_resolution =
	min (dev->inquiry_x_res, dev->inquiry_y_res);
    }

  dev->inquiry_fb_width =
    (double) get_inquiry_fb_max_scan_width (buffer) /
    dev->inquiry_pixel_resolution;
  dev->inquiry_fb_length =
    (double) get_inquiry_fb_max_scan_length (buffer) /
    dev->inquiry_pixel_resolution;

  dev->inquiry_trans_top_left_x = get_inquiry_trans_x1 (buffer);
  dev->inquiry_trans_top_left_y = get_inquiry_trans_y1 (buffer);

  dev->inquiry_trans_width =
    (double) (get_inquiry_trans_x2 (buffer) -
	      get_inquiry_trans_x1 (buffer)) / dev->inquiry_pixel_resolution;
  dev->inquiry_trans_length =
    (double) (get_inquiry_trans_y2 (buffer) -
	      get_inquiry_trans_y1 (buffer)) / dev->inquiry_pixel_resolution;

  dev->inquiry_halftones = get_inquiry_halftones (buffer) & 0x0f;

  dev->inquiry_filters = get_inquiry_filters (buffer);
  dev->inquiry_color_depths = get_inquiry_color_depths (buffer);
  dev->inquiry_color_format = get_inquiry_color_format (buffer);
  dev->inquiry_image_format = get_inquiry_image_format (buffer);

  dev->inquiry_scan_capability = get_inquiry_scan_capability (buffer);
  dev->inquiry_optional_devices = get_inquiry_optional_devices (buffer);
  dev->inquiry_enhancements = get_inquiry_enhancements (buffer);
  dev->inquiry_gamma_bits = get_inquiry_gamma_bits (buffer);
  dev->inquiry_fast_preview_res = get_inquiry_fast_preview_res (buffer);
  dev->inquiry_min_highlight = get_inquiry_min_highlight (buffer);
  dev->inquiry_max_shadow = get_inquiry_max_shadow (buffer);
  dev->inquiry_cal_eqn = get_inquiry_cal_eqn (buffer);
  dev->inquiry_min_exp = get_inquiry_min_exp (buffer);
  dev->inquiry_max_exp = get_inquiry_max_exp (buffer);

  pie_print_inquiry (dev);

  return;
}

/* ----------------------------- PIE DO INQUIRY ---------------------------- */


static void
pie_do_inquiry (Pie_Device * dev, int sfd, unsigned char *buffer)
{
  size_t size;
  SANE_Status status;

  DBG (DBG_proc, "do_inquiry\n");
  memset (buffer, '\0', 256);	/* clear buffer */

  size = 5;

  set_inquiry_return_size (inquiry.cmd, size);	/* first get only 5 bytes to get size of inquiry_return_block */
  status = (*dev->scsi_cmd) (sfd, inquiry.cmd, inquiry.size, buffer, &size);
  if (status)
    {
      DBG (DBG_error, "pie_do_inquiry: command returned status %s\n",
	   sane_strstatus (status));
    }

  size = get_inquiry_additional_length (buffer) + 5;

  set_inquiry_return_size (inquiry.cmd, size);	/* then get inquiry with actual size */
  status = (*dev->scsi_cmd) (sfd, inquiry.cmd, inquiry.size, buffer, &size);
  if (status)
    {
      DBG (DBG_error, "pie_do_inquiry: command returned status %s\n",
	   sane_strstatus (status));
    }
}

/* ---------------------- PIE IDENTIFY SCANNER ---------------------- */


static int
pie_identify_scanner (Pie_Device * dev, int sfd, int is_USB)
{
  char *vendor, *product, *version;
  char *pp;
  int j, i = 0;
  SANE_Byte usb_model_id = 0;
  unsigned char inquiry_block[256];

  DBG (DBG_proc, "identify_scanner\n");

  pie_do_inquiry (dev, sfd, inquiry_block);	/* get inquiry */

  if (get_inquiry_periph_devtype (inquiry_block) != IN_periph_devtype_scanner)
    {
      return 1;
    }				/* no scanner */

  vendor = dup_inquiry_vendor ((char *) inquiry_block);
  product = dup_inquiry_product ((char *) inquiry_block);
  version = dup_inquiry_version ((char *) inquiry_block);

  pp = &vendor[8];
  vendor[8] = ' ';
  while (*pp == ' ')
    {
      *pp-- = '\0';
    }

  pp = &product[0x10];
  product[0x10] = ' ';
  while (*pp == ' ')
    {
      *pp-- = '\0';
    }

  pp = &version[4];

  version[4] = ' ';
  while (*pp == ' ')
    {
      *pp-- = '\0';
    }

  DBG (DBG_info, "Found %s scanner %s version %s on device %s\n", vendor,
       product, version, dev->devicename);

  while (strncmp ("END_OF_LIST", scanner_str[2 * i], 11) != 0)	/* Now identify full supported scanners */
    {
      if (!strncmp (vendor, scanner_str[2 * i], strlen (scanner_str[2 * i])))
	{
	  if (!strncmp
	      (product, scanner_str[2 * i + 1],
	       strlen (scanner_str[2 * i + 1])))
	    {
	      /* different types of PIE USB scanners use the same USB id and inquiry name
	         so we need to do some subtyping here */
	      if (is_USB)
		{
		  for (j = 0; pie_usb_device_list[j].model != 0; j++)
		    {
		      usb_model_id = get_inquiry_model (inquiry_block);
		      if (pie_usb_device_list[j].model->model_id == usb_model_id)
			dev->model = pie_usb_device_list[j].model;
		    }
		  if (dev->model == NULL)
		    continue;
		}

	      DBG (DBG_info, "found supported scanner\n");

	      if (dev->model == NULL)
		{
		  dev->vendor = vendor;
		  dev->product = product;
		}
	      else
		{
		  free (vendor);
		  free (product);
		  dev->vendor = dev->model->vendor;
		  dev->product = dev->model->model;
		}
	      dev->version = version;
	      pie_get_inquiry_values (dev, inquiry_block);
	      return 0;
	    }
	}
      i++;
    }

  /* A new USB model was recognized, we wish to know about it */
  if (usb_model_id)
    {
      DBG (DBG_info, "You have a scanner which is recognized but not yet\n");
      DBG (DBG_info, "supported by this backend. The model id is %d\n", usb_model_id);
      if (DBG_LEVEL == 197)
        {
          DBG (DBG_info, "You are now working at your own risk!!!\n");
          dev->model = pie_usb_device_list[0].model;
          free (vendor);
          free (product);
          dev->vendor = dev->model->vendor;
          dev->product = dev->model->model;
        }
      else
        {
          DBG (DBG_info, "Please post this output at the sane-devel list.\n");
          dev->vendor = vendor;
          dev->product = product;
        }
      dev->version = version;
      pie_get_inquiry_values (dev, inquiry_block);
      if (DBG_LEVEL == 197)
        return 0;
    }
  return 1;			/* NO SUPPORTED SCANNER: short inquiry-block and unknown scanner */
}


/* ------------------------------- GET SPEEDS ----------------------------- */

static void
pie_get_speeds (Pie_Device * dev)
{
  int speeds = dev->inquiry_scan_capability & INQ_CAP_SPEEDS;

  DBG (DBG_proc, "get_speeds\n");

  if (speeds == 3)
    {
      dev->speed_list[0] = strdup ("Normal");
      dev->speed_list[1] = strdup ("Fine");
      dev->speed_list[2] = strdup ("Pro");
      dev->speed_list[3] = NULL;
    }
  else
    {
      int i;
      char buf[2];

      buf[1] = '\0';

      for (i = 0; i < speeds; i++)
	{
	  buf[0] = '1' + i;
	  dev->speed_list[i] = strdup (buf);
	}

      dev->speed_list[i] = NULL;
    }
}

/* ------------------------------- GET HALFTONES ----------------------------- */

static void
pie_get_halftones (Pie_Device * dev, int sfd)
{
  int i;
  size_t size;
  SANE_Status status;
  unsigned char *data;
  unsigned char buffer[128];

  DBG (DBG_proc, "get_halftones\n");

  for (i = 0; i < dev->inquiry_halftones; i++)
    {
      size = 6;

      set_write_length (swrite.cmd, size);

      memcpy (buffer, swrite.cmd, swrite.size);

      data = buffer + swrite.size;
      memset (data, 0, size);

      set_command (data, READ_HALFTONE);
      set_data_length (data, 2);
      data[4] = i;

      status = (*dev->scsi_cmd) (sfd, buffer, swrite.size + size, NULL, NULL);
      if (status)
	{
	  DBG (DBG_error,
	       "pie_get_halftones: write command returned status %s\n",
	       sane_strstatus (status));
	}
      else
	{
	  /* now read the halftone data */
	  memset (buffer, '\0', sizeof buffer);	/* clear buffer */

	  size = 128;
	  set_read_length (sread.cmd, size);

	  DBG (DBG_info, "doing read\n");
	  status =
	    (*dev->scsi_cmd) (sfd, sread.cmd, sread.size, buffer, &size);
	  if (status)
	    {
	      DBG (DBG_error,
		   "pie_get_halftones: read command returned status %s\n",
		   sane_strstatus (status));
	    }
	  else
	    {
	      unsigned char *s;

	      s = buffer + 8 + buffer[6] * buffer[7];

	      DBG (DBG_info, "halftone %d: %s\n", i, s);

	      dev->halftone_list[i] = strdup ((char *) s);
	    }
	}
    }
  dev->halftone_list[i] = NULL;
}

/* ------------------------------- GET CAL DATA ----------------------------- */

static void
pie_get_cal_info (Pie_Device * dev, int sfd)
{
  size_t size;
  SANE_Status status;
  unsigned char *data;
  unsigned char buffer[280];

  DBG (DBG_proc, "get_cal_info\n");

  if (!(dev->inquiry_scan_capability & INQ_CAP_EXT_CAL))
    return;

  size = 6;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, READ_CAL_INFO);

  status = (*dev->scsi_cmd) (sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error, "pie_get_cal_info: write command returned status %s\n",
	   sane_strstatus (status));
    }
  else
    {
      /* now read the cal data */
      memset (buffer, '\0', sizeof buffer);	/* clear buffer */

      size = 128;
      set_read_length (sread.cmd, size);

      DBG (DBG_info, "doing read\n");
      status = (*dev->scsi_cmd) (sfd, sread.cmd, sread.size, buffer, &size);
      if (status)
	{
	  DBG (DBG_error,
	       "pie_get_cal_info: read command returned status %s\n",
	       sane_strstatus (status));
	}
      else
	{
	  int i;

	  dev->cal_info_count = buffer[4];
	  dev->cal_info =
	    malloc (sizeof (struct Pie_cal_info) * dev->cal_info_count);

	  for (i = 0; i < dev->cal_info_count; i++)
	    {
	      dev->cal_info[i].cal_type = buffer[8 + i * buffer[5]];
	      dev->cal_info[i].send_bits = buffer[9 + i * buffer[5]];
	      dev->cal_info[i].receive_bits = buffer[10 + i * buffer[5]];
	      dev->cal_info[i].num_lines = buffer[11 + i * buffer[5]];
	      dev->cal_info[i].pixels_per_line =
		(buffer[13 + i * buffer[5]] << 8) + buffer[12 +
							   i * buffer[5]];

	      DBG (DBG_info2, "%02x %2d %2d %2d %d\n",
		   dev->cal_info[i].cal_type, dev->cal_info[i].send_bits,
		   dev->cal_info[i].receive_bits, dev->cal_info[i].num_lines,
		   dev->cal_info[i].pixels_per_line);
	    }
	}
    }
}


/* ----------------------------- PIE_USB_ATTACH_OPEN ----------------------------- */
/**
 * @brief Try to open and identify an USB scanner class
 *
 * @param[in] devname name of the device to open
 * @param[out] dn USB device number
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_ACCESS_DENIED - no access to file due to permissions
 * - SANE_STATUS_UNSUPPORTED - if the OS doesn't support detection of ids
 * - SANE_STATUS_INVAL - not in our list of supported devices
 */
static SANE_Status
pie_usb_attach_open (SANE_String_Const devname, SANE_Int * dn)
{
  SANE_Status status;
  SANE_Int vendor, product;
  SANE_Int model = 0;
  int i;

  DBG (DBG_proc, "pie_usb_attach_open: opening `%s'\n", devname);
  status = sanei_usb_open (devname, dn);
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "pie_usb_attach_open: sanei_usb_open failed\n");
      return status;
    }
  DBG (DBG_info, "pie_usb_attach_open: USB device `%s' successfully opened\n",
       devname);

  status = sanei_usb_get_vendor_product (*dn, &vendor, &product);
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error,
           "pie_usb_attach_open: couldn't get vendor and product ids of device `%s': %s\n",
           devname, sane_strstatus (status));
      return status;
    }

  for (i = 0; pie_usb_device_list[i].model != 0; i++)
    {
      if (vendor == pie_usb_device_list[i].vendor &&
          product == pie_usb_device_list[i].product)
        {
          model++;
          break;
        }
    }
  if (model == 0)
    {
      DBG (DBG_error,
           "pie_usb_attach_open: vendor 0x%04x product 0x%04x is not supported by this backend\n",
           vendor, product);
      status = SANE_STATUS_INVAL;
    }

  return status;
}


/* ----------------------------- PIE_USB_TRY_ATTACH ----------------------------- */
/**
 * @brief Callback function for sanei_config_attach_matching_devices
 *
 * @param[in] name device name pattern
 *
 * @return
 * - SANE_STATUS_GOOD
 */
static SANE_Status
pie_usb_try_attach (const char *name)
{
  sanei_usb_attach_matching_devices (name, pie_attach_one);
  return SANE_STATUS_GOOD;
}


/* ------------------------------- ATTACH SCANNER ----------------------------- */

static SANE_Status
pie_attach_scanner (const char *devicename, Pie_Device ** devp)
{
  SANE_Status status;
  SANE_Int USB_model = 1;	/* assume USB scanner */
  Pie_Device *dev;
  int sfd;
  int bufsize;

  DBG (DBG_sane_proc, "pie_attach_scanner: %s\n", devicename);

  for (dev = first_dev; dev; dev = dev->next)
    {
      if (strcmp (dev->sane.name, devicename) == 0)
	{
	  if (devp)
	    {
	      *devp = dev;
	    }
	  return SANE_STATUS_GOOD;
	}
    }

  dev = malloc (sizeof (*dev));
  if (!dev)
    {
      return SANE_STATUS_NO_MEM;
    }

  status = pie_usb_attach_open (devicename, &sfd);	/* try USB scanners first */
  if (status != SANE_STATUS_GOOD)
    {
      USB_model = 0;		/* if failed try SCSI */
#ifdef HAVE_SANEI_SCSI_OPEN_EXTENDED
  bufsize = 16384;		/* 16KB */

  if (sanei_scsi_open_extended
      (devicename, &sfd, pie_sense_handler, dev, &bufsize) != 0)
    {
      DBG (DBG_error, "pie_attach_scanner: open failed\n");
      free (dev);
      return SANE_STATUS_INVAL;
    }

  if (bufsize < 4096)		/* < 4KB */
    {
      DBG (DBG_error,
	   "pie_attach_scanner: sanei_scsi_open_extended returned too small scsi buffer (%d)\n",
	   bufsize);
      sanei_scsi_close (sfd);
      free (dev);
      return SANE_STATUS_NO_MEM;
    }
  DBG (DBG_info,
       "pie_attach_scanner: sanei_scsi_open_extended returned scsi buffer size = %d\n",
       bufsize);
#else
  bufsize = sanei_scsi_max_request_size;

  if (sanei_scsi_open (devicename, &sfd, pie_sense_handler, dev) != 0)
    {
      DBG (DBG_error, "pie_attach_scanner: open failed\n");
      free (dev);

      return SANE_STATUS_INVAL;
    }
#endif
    }

  pie_init (dev, USB_model);	/* preset values in structure dev */

  dev->devicename = strdup (devicename);

  if (pie_identify_scanner (dev, sfd, USB_model) != 0)
    {
      DBG (DBG_error, "pie_attach_scanner: scanner-identification failed\n");
      if (USB_model == 0)
      sanei_scsi_close (sfd);
      else
	sanei_usb_close (sfd);
      free (dev);
      return SANE_STATUS_INVAL;
    }

  if (USB_model == 0)
    {
      pie_get_halftones (dev, sfd);
      pie_get_cal_info (dev, sfd);
      pie_get_speeds (dev);

      dev->scan_mode_list[0] = COLOR_STR;
      dev->scan_mode_list[1] = GRAY_STR;
      dev->scan_mode_list[2] = LINEART_STR;
      dev->scan_mode_list[3] = HALFTONE_STR;
      dev->scan_mode_list[4] = 0;

      dev->bpp_list[0] = 1;
      dev->bpp_list[1] = 8;
      dev->bpp_list[2] = 0;

      dev->sane.type = "flatbed scanner";

      sanei_scsi_close (sfd);
    }
  else
    {
      dev->scan_mode_list[0] = COLOR_STR;
      dev->scan_mode_list[1] = COLOR_IR_STR;
      dev->scan_mode_list[2] = 0;

      dev->sane.type = "film scanner";

      dev->bpp_list[0] = 2;
      dev->bpp_list[1] = 16;
      dev->bpp_list[2] = 8;
      dev->bpp_list[3] = 0;

      sanei_usb_close (sfd);
    }

  dev->ir_sw_list[0] = THE_NONE_STR;
  dev->ir_sw_list[1] = IR_SPECT_STR;
  dev->ir_sw_list[2] = IR_CLEAN_STR;
  dev->ir_sw_list[3] = 0;

  dev->grain_sw_list[0] = 4;
  dev->grain_sw_list[1] = 0;
  dev->grain_sw_list[2] = 1;
  dev->grain_sw_list[3] = 2;
  dev->grain_sw_list[4] = 3;
  dev->grain_sw_list[5] = 0;

  dev->crop_sw_list[0] = THE_NONE_STR;
  dev->crop_sw_list[1] = CROP_OUTER_STR;
  dev->crop_sw_list[2] = CROP_INNER_STR;
  dev->crop_sw_list[3] = 0;

  dev->sane.name = dev->devicename;
  dev->sane.vendor = dev->vendor;
  dev->sane.model = dev->product;

  dev->x_range.min = SANE_FIX (0);
  dev->x_range.quant = SANE_FIX (0);
  dev->x_range.max = SANE_FIX (dev->inquiry_fb_width * MM_PER_INCH);

  dev->y_range.min = SANE_FIX (0);
  dev->y_range.quant = SANE_FIX (0);
  dev->y_range.max = SANE_FIX (dev->inquiry_fb_length * MM_PER_INCH);

  dev->dpi_range.min = SANE_FIX (25);
  dev->dpi_range.quant = SANE_FIX (1);
  dev->dpi_range.max =
    SANE_FIX (max (dev->inquiry_x_res, dev->inquiry_y_res));

  dev->shadow_range.min = SANE_FIX (0);
  dev->shadow_range.quant = SANE_FIX (1);
  dev->shadow_range.max = SANE_FIX (dev->inquiry_max_shadow);

  dev->highlight_range.min = SANE_FIX (dev->inquiry_min_highlight);
  dev->highlight_range.quant = SANE_FIX (1);
  dev->highlight_range.max = SANE_FIX (100);

  dev->exposure_range.min = SANE_FIX (dev->inquiry_min_exp);
  dev->exposure_range.quant = SANE_FIX (1);
  dev->exposure_range.max = SANE_FIX (dev->inquiry_max_exp);

#if 0
  dev->analog_gamma_range.min = SANE_FIX (1.0);
  dev->analog_gamma_range.quant = SANE_FIX (0.01);
  dev->analog_gamma_range.max = SANE_FIX (2.0);

#endif

  dev->next = first_dev;
  first_dev = dev;

  if (devp)
    {
      *devp = dev;
    }

  return SANE_STATUS_GOOD;
}

/* --------------------------- MAX STRING SIZE ---------------------------- */


static size_t
max_string_size (SANE_String_Const strings[])
{
  size_t size, max_size = 0;
  int i;

  for (i = 0; strings[i]; ++i)
    {
      size = strlen (strings[i]) + 1;
      if (size > max_size)
	{
	  max_size = size;
	}
    }

  return max_size;
}


/* --------------------------- INIT OPTIONS ------------------------------- */


static SANE_Status
pie_init_options (Pie_Scanner * scanner)
{
  int i;

  DBG (DBG_sane_proc, "pie_init_options\n");

  memset (scanner->opt, 0, sizeof (scanner->opt));
  memset (scanner->val, 0, sizeof (scanner->val));

  for (i = 0; i < NUM_OPTIONS; ++i)
    {
      scanner->opt[i].size = sizeof (SANE_Word);
      scanner->opt[i].cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
    }

  scanner->opt[OPT_NUM_OPTS].title = SANE_TITLE_NUM_OPTIONS;
  scanner->opt[OPT_NUM_OPTS].desc = SANE_DESC_NUM_OPTIONS;
  scanner->opt[OPT_NUM_OPTS].type = SANE_TYPE_INT;
  scanner->opt[OPT_NUM_OPTS].cap = SANE_CAP_SOFT_DETECT;
  scanner->val[OPT_NUM_OPTS].w = NUM_OPTIONS;

  /* "Mode" group: */
  scanner->opt[OPT_MODE_GROUP].title = "Scan Mode";
  scanner->opt[OPT_MODE_GROUP].desc = "";
  scanner->opt[OPT_MODE_GROUP].type = SANE_TYPE_GROUP;
  scanner->opt[OPT_MODE_GROUP].cap = 0;
  scanner->opt[OPT_MODE_GROUP].constraint_type = SANE_CONSTRAINT_NONE;

  /* scan mode */
  scanner->opt[OPT_MODE].name = SANE_NAME_SCAN_MODE;
  scanner->opt[OPT_MODE].title = SANE_TITLE_SCAN_MODE;
  scanner->opt[OPT_MODE].desc = SANE_DESC_SCAN_MODE;
  scanner->opt[OPT_MODE].type = SANE_TYPE_STRING;
  scanner->opt[OPT_MODE].size =
    max_string_size ((SANE_String_Const *)(void*) scanner->device->scan_mode_list);
  scanner->opt[OPT_MODE].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_MODE].constraint.string_list =
    (SANE_String_Const *)(void*) scanner->device->scan_mode_list;
  scanner->val[OPT_MODE].s =
    (SANE_Char *) strdup (scanner->device->scan_mode_list[1]);

  /* bit depth */
  scanner->opt[OPT_BIT_DEPTH].name = SANE_NAME_BIT_DEPTH;
  scanner->opt[OPT_BIT_DEPTH].title = SANE_TITLE_BIT_DEPTH;
  scanner->opt[OPT_BIT_DEPTH].desc = SANE_DESC_BIT_DEPTH;
  scanner->opt[OPT_BIT_DEPTH].type = SANE_TYPE_INT;
  scanner->opt[OPT_BIT_DEPTH].constraint_type = SANE_CONSTRAINT_WORD_LIST;
  scanner->opt[OPT_BIT_DEPTH].size = sizeof (SANE_Word);
  scanner->opt[OPT_BIT_DEPTH].constraint.word_list =
    scanner->device->bpp_list;
  scanner->val[OPT_BIT_DEPTH].w = scanner->device->bpp_list[1];
  if (scanner->opt[OPT_BIT_DEPTH].constraint.word_list[0] < 2)
    scanner->opt[OPT_BIT_DEPTH].cap |= SANE_CAP_INACTIVE;

  /* x-resolution */
  scanner->opt[OPT_RESOLUTION].name = SANE_NAME_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].title = SANE_TITLE_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].desc = SANE_DESC_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_RESOLUTION].unit = SANE_UNIT_DPI;
  scanner->opt[OPT_RESOLUTION].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_RESOLUTION].constraint.range = &scanner->device->dpi_range;
  scanner->val[OPT_RESOLUTION].w = 1200 << SANE_FIXED_SCALE_SHIFT;

  /* "Geometry" group: */

  scanner->opt[OPT_GEOMETRY_GROUP].title = "Geometry";
  scanner->opt[OPT_GEOMETRY_GROUP].desc = "";
  scanner->opt[OPT_GEOMETRY_GROUP].type = SANE_TYPE_GROUP;
  scanner->opt[OPT_GEOMETRY_GROUP].cap = SANE_CAP_ADVANCED;
  scanner->opt[OPT_GEOMETRY_GROUP].constraint_type = SANE_CONSTRAINT_NONE;

  /* top-left x */
  scanner->opt[OPT_TL_X].name = SANE_NAME_SCAN_TL_X;
  scanner->opt[OPT_TL_X].title = SANE_TITLE_SCAN_TL_X;
  scanner->opt[OPT_TL_X].desc = SANE_DESC_SCAN_TL_X;
  scanner->opt[OPT_TL_X].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_TL_X].unit = SANE_UNIT_MM;
  scanner->opt[OPT_TL_X].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_TL_X].constraint.range = &(scanner->device->x_range);
  scanner->val[OPT_TL_X].w = 0;

  /* top-left y */
  scanner->opt[OPT_TL_Y].name = SANE_NAME_SCAN_TL_Y;
  scanner->opt[OPT_TL_Y].title = SANE_TITLE_SCAN_TL_Y;
  scanner->opt[OPT_TL_Y].desc = SANE_DESC_SCAN_TL_Y;
  scanner->opt[OPT_TL_Y].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_TL_Y].unit = SANE_UNIT_MM;
  scanner->opt[OPT_TL_Y].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_TL_Y].constraint.range = &(scanner->device->y_range);
  scanner->val[OPT_TL_Y].w = 0;

  /* bottom-right x */
  scanner->opt[OPT_BR_X].name = SANE_NAME_SCAN_BR_X;
  scanner->opt[OPT_BR_X].title = SANE_TITLE_SCAN_BR_X;
  scanner->opt[OPT_BR_X].desc = SANE_DESC_SCAN_BR_X;
  scanner->opt[OPT_BR_X].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_BR_X].unit = SANE_UNIT_MM;
  scanner->opt[OPT_BR_X].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_BR_X].constraint.range = &(scanner->device->x_range);
  scanner->val[OPT_BR_X].w = scanner->device->x_range.max;

  /* bottom-right y */
  scanner->opt[OPT_BR_Y].name = SANE_NAME_SCAN_BR_Y;
  scanner->opt[OPT_BR_Y].title = SANE_TITLE_SCAN_BR_Y;
  scanner->opt[OPT_BR_Y].desc = SANE_DESC_SCAN_BR_Y;
  scanner->opt[OPT_BR_Y].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_BR_Y].unit = SANE_UNIT_MM;
  scanner->opt[OPT_BR_Y].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_BR_Y].constraint.range = &(scanner->device->y_range);
  scanner->val[OPT_BR_Y].w = scanner->device->y_range.max;

  /* "enhancement" group: */

  scanner->opt[OPT_ENHANCEMENT_GROUP].title = "Enhancement";
  scanner->opt[OPT_ENHANCEMENT_GROUP].desc = "";
  scanner->opt[OPT_ENHANCEMENT_GROUP].type = SANE_TYPE_GROUP;
  scanner->opt[OPT_ENHANCEMENT_GROUP].cap = 0;
  scanner->opt[OPT_ENHANCEMENT_GROUP].constraint_type = SANE_CONSTRAINT_NONE;

  /* reduce spectal overlap from infrared plane */
  scanner->opt[OPT_SW_IRED].name = IR_NAME_STR;
  scanner->opt[OPT_SW_IRED].title = IR_TITLE_STR;
  scanner->opt[OPT_SW_IRED].desc = IR_DESC_STR;
  scanner->opt[OPT_SW_IRED].type = SANE_TYPE_STRING;
  scanner->opt[OPT_SW_IRED].size =
    max_string_size ((SANE_String_Const *)(void*) scanner->device->ir_sw_list);
  scanner->opt[OPT_SW_IRED].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_SW_IRED].constraint.string_list =
    (SANE_String_Const *)(void*) scanner->device->ir_sw_list;
  scanner->val[OPT_SW_IRED].s =
    (SANE_Char *) strdup (scanner->device->ir_sw_list[2]);

  /* strength of grain filtering */
  scanner->opt[OPT_SW_GRAIN].name = "swgrain";
  scanner->opt[OPT_SW_GRAIN].title = "Attenuate film grain";
  scanner->opt[OPT_SW_GRAIN].desc = "Amount of smothening";
  scanner->opt[OPT_SW_GRAIN].type = SANE_TYPE_INT;
  scanner->opt[OPT_SW_GRAIN].constraint_type = SANE_CONSTRAINT_WORD_LIST;
  scanner->opt[OPT_SW_GRAIN].size = sizeof (SANE_Word);
  scanner->opt[OPT_SW_GRAIN].constraint.word_list =
    scanner->device->grain_sw_list;
  scanner->val[OPT_SW_GRAIN].w = scanner->device->grain_sw_list[1];
  if (scanner->opt[OPT_SW_GRAIN].constraint.word_list[0] < 2)
    scanner->opt[OPT_SW_GRAIN].cap |= SANE_CAP_INACTIVE;

  /* gamma correction, to make image sRGB like */
  scanner->opt[OPT_SW_SRGB].name = "swsrgb";
  scanner->opt[OPT_SW_SRGB].title = "sRGB colors";
  scanner->opt[OPT_SW_SRGB].desc = "Transform image to approximate sRGB color space";
  scanner->opt[OPT_SW_SRGB].type = SANE_TYPE_BOOL;
  scanner->opt[OPT_SW_SRGB].unit = SANE_UNIT_NONE;
  scanner->val[OPT_SW_SRGB].w = SANE_TRUE;

  /* color correction for generic negative film */
  scanner->opt[OPT_SW_NEGA].name = "swnega";
  scanner->opt[OPT_SW_NEGA].title = "Invert colors";
  scanner->opt[OPT_SW_NEGA].desc = "Correct for generic negative film";
  scanner->opt[OPT_SW_NEGA].type = SANE_TYPE_BOOL;
  scanner->opt[OPT_SW_NEGA].unit = SANE_UNIT_NONE;

  /* crop image */
  scanner->opt[OPT_SW_CROP].name = CROP_NAME_STR;
  scanner->opt[OPT_SW_CROP].title = CROP_TITLE_STR;
  scanner->opt[OPT_SW_CROP].desc = CROP_DESC_STR;
  scanner->opt[OPT_SW_CROP].type = SANE_TYPE_STRING;
  scanner->opt[OPT_SW_CROP].size =
    max_string_size ((SANE_String_Const *)(void*) scanner->device->crop_sw_list);
  scanner->opt[OPT_SW_CROP].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_SW_CROP].constraint.string_list =
    (SANE_String_Const *)(void*) scanner->device->crop_sw_list;
  scanner->val[OPT_SW_CROP].s =
    (SANE_Char *) strdup (scanner->device->crop_sw_list[2]);

/* grayscale gamma vector */
  scanner->opt[OPT_GAMMA_VECTOR].name = SANE_NAME_GAMMA_VECTOR;
  scanner->opt[OPT_GAMMA_VECTOR].title = SANE_TITLE_GAMMA_VECTOR;
  scanner->opt[OPT_GAMMA_VECTOR].desc = SANE_DESC_GAMMA_VECTOR;
  scanner->opt[OPT_GAMMA_VECTOR].type = SANE_TYPE_INT;
  scanner->opt[OPT_GAMMA_VECTOR].unit = SANE_UNIT_NONE;
  scanner->opt[OPT_GAMMA_VECTOR].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->val[OPT_GAMMA_VECTOR].wa = scanner->gamma_table[0];
  scanner->opt[OPT_GAMMA_VECTOR].constraint.range = &scanner->gamma_range;
  scanner->opt[OPT_GAMMA_VECTOR].size =
    scanner->gamma_length * sizeof (SANE_Word);
  scanner->opt[OPT_GAMMA_VECTOR].cap |= SANE_CAP_INACTIVE;

  /* red gamma vector */
  scanner->opt[OPT_GAMMA_VECTOR_R].name = SANE_NAME_GAMMA_VECTOR_R;
  scanner->opt[OPT_GAMMA_VECTOR_R].title = SANE_TITLE_GAMMA_VECTOR_R;
  scanner->opt[OPT_GAMMA_VECTOR_R].desc = SANE_DESC_GAMMA_VECTOR_R;
  scanner->opt[OPT_GAMMA_VECTOR_R].type = SANE_TYPE_INT;
  scanner->opt[OPT_GAMMA_VECTOR_R].unit = SANE_UNIT_NONE;
  scanner->opt[OPT_GAMMA_VECTOR_R].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->val[OPT_GAMMA_VECTOR_R].wa = scanner->gamma_table[1];
  scanner->opt[OPT_GAMMA_VECTOR_R].constraint.range = &(scanner->gamma_range);
  scanner->opt[OPT_GAMMA_VECTOR_R].size =
    scanner->gamma_length * sizeof (SANE_Word);

  /* green gamma vector */
  scanner->opt[OPT_GAMMA_VECTOR_G].name = SANE_NAME_GAMMA_VECTOR_G;
  scanner->opt[OPT_GAMMA_VECTOR_G].title = SANE_TITLE_GAMMA_VECTOR_G;
  scanner->opt[OPT_GAMMA_VECTOR_G].desc = SANE_DESC_GAMMA_VECTOR_G;
  scanner->opt[OPT_GAMMA_VECTOR_G].type = SANE_TYPE_INT;
  scanner->opt[OPT_GAMMA_VECTOR_G].unit = SANE_UNIT_NONE;
  scanner->opt[OPT_GAMMA_VECTOR_G].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->val[OPT_GAMMA_VECTOR_G].wa = scanner->gamma_table[2];
  scanner->opt[OPT_GAMMA_VECTOR_G].constraint.range = &(scanner->gamma_range);
  scanner->opt[OPT_GAMMA_VECTOR_G].size =
    scanner->gamma_length * sizeof (SANE_Word);


  /* blue gamma vector */
  scanner->opt[OPT_GAMMA_VECTOR_B].name = SANE_NAME_GAMMA_VECTOR_B;
  scanner->opt[OPT_GAMMA_VECTOR_B].title = SANE_TITLE_GAMMA_VECTOR_B;
  scanner->opt[OPT_GAMMA_VECTOR_B].desc = SANE_DESC_GAMMA_VECTOR_B;
  scanner->opt[OPT_GAMMA_VECTOR_B].type = SANE_TYPE_INT;
  scanner->opt[OPT_GAMMA_VECTOR_B].unit = SANE_UNIT_NONE;
  scanner->opt[OPT_GAMMA_VECTOR_B].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->val[OPT_GAMMA_VECTOR_B].wa = scanner->gamma_table[3];
  scanner->opt[OPT_GAMMA_VECTOR_B].constraint.range = &(scanner->gamma_range);
  scanner->opt[OPT_GAMMA_VECTOR_B].size =
    scanner->gamma_length * sizeof (SANE_Word);

  if (scanner->device->model != NULL)
    {
      scanner->opt[OPT_GAMMA_VECTOR].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_GAMMA_VECTOR_R].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_GAMMA_VECTOR_G].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_GAMMA_VECTOR_B].cap |= SANE_CAP_INACTIVE;
    }
  else
    {
      scanner->opt[OPT_SW_SRGB].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_SW_NEGA].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_SW_IRED].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_SW_CROP].cap |= SANE_CAP_INACTIVE;
      scanner->opt[OPT_SW_GRAIN].cap |= SANE_CAP_INACTIVE;
    }

  /* halftone pattern */
  scanner->opt[OPT_HALFTONE_PATTERN].name = SANE_NAME_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].title = SANE_TITLE_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].desc = SANE_DESC_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].type = SANE_TYPE_STRING;
  scanner->opt[OPT_HALFTONE_PATTERN].size =
    max_string_size ((SANE_String_Const *)(void*) scanner->device->halftone_list);
  scanner->opt[OPT_HALFTONE_PATTERN].constraint_type =
    SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_HALFTONE_PATTERN].constraint.string_list =
    (SANE_String_Const *)(void*) scanner->device->halftone_list;
  scanner->val[OPT_HALFTONE_PATTERN].s =
    (SANE_Char *) strdup (scanner->device->halftone_list[0]);
  scanner->opt[OPT_HALFTONE_PATTERN].cap |= SANE_CAP_INACTIVE;

  /* speed */
  scanner->opt[OPT_SPEED].name = SANE_NAME_SCAN_SPEED;
  scanner->opt[OPT_SPEED].title = SANE_TITLE_SCAN_SPEED;
  scanner->opt[OPT_SPEED].desc = SANE_DESC_SCAN_SPEED;
  scanner->opt[OPT_SPEED].type = SANE_TYPE_STRING;
  scanner->opt[OPT_SPEED].size =
    max_string_size ((SANE_String_Const *)(void*) scanner->device->speed_list);
  scanner->opt[OPT_SPEED].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_SPEED].constraint.string_list =
    (SANE_String_Const *)(void*) scanner->device->speed_list;
  scanner->val[OPT_SPEED].s =
    (SANE_Char *) strdup (scanner->device->speed_list[1]);

  /* lineart threshold */
  scanner->opt[OPT_THRESHOLD].name = SANE_NAME_THRESHOLD;
  scanner->opt[OPT_THRESHOLD].title = SANE_TITLE_THRESHOLD;
  scanner->opt[OPT_THRESHOLD].desc = SANE_DESC_THRESHOLD;
  scanner->opt[OPT_THRESHOLD].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_THRESHOLD].unit = SANE_UNIT_PERCENT;
  scanner->opt[OPT_THRESHOLD].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_THRESHOLD].constraint.range = &percentage_range_100;
  scanner->val[OPT_THRESHOLD].w = SANE_FIX (50);
  scanner->opt[OPT_THRESHOLD].cap |= SANE_CAP_INACTIVE;

  /* "advanced" group: */

  scanner->opt[OPT_ADVANCED_GROUP].title = "Advanced";
  scanner->opt[OPT_ADVANCED_GROUP].desc = "";
  scanner->opt[OPT_ADVANCED_GROUP].type = SANE_TYPE_GROUP;
  scanner->opt[OPT_ADVANCED_GROUP].cap = SANE_CAP_ADVANCED;
  scanner->opt[OPT_ADVANCED_GROUP].constraint_type = SANE_CONSTRAINT_NONE;

  /* preview */
  scanner->opt[OPT_PREVIEW].name = SANE_NAME_PREVIEW;
  scanner->opt[OPT_PREVIEW].title = SANE_TITLE_PREVIEW;
  scanner->opt[OPT_PREVIEW].desc = SANE_DESC_PREVIEW;
  scanner->opt[OPT_PREVIEW].type = SANE_TYPE_BOOL;
  scanner->val[OPT_PREVIEW].w = SANE_FALSE;



  return SANE_STATUS_GOOD;
}

/*
 * @@ Mid level USB functions
 */

/* ---------------------------- PIE_USB_WAIT_SCANNER ----------------------------- */
/**
 * @brief TEST_UNIT_READY until ready or timed out
 *
 * @param[in] scanner points to structure of opened scanner
 * @param[in] secs seconds to wait maximally
 *
 * @return
 * - SANE_STATUS_GOOD - scanner is ready
 * - SANE_STATUS_DEVICE_BUSY - scanner timed out
 * - SANE_STATUS_IO_ERROR - on error
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 * - SANE_STATUS_INVAL - received unexpected value
 */
static SANE_Status
pie_usb_wait_scanner (Pie_Scanner * scanner, int secs)
{
  SANE_Status status;
  int cnt = secs * 16;

  DBG (DBG_proc, "pie_usb_wait_scanner\n");

  do
    {
      status = pie_usb_scsi_wrapper (scanner->sfd, test_unit_ready.cmd,
                                     test_unit_ready.size, NULL, NULL);
      if (status == SANE_STATUS_GOOD)
        return status;
      if (cnt == 0)
        {
          DBG (DBG_warning, "pie_usb_wait_scanner timed out\n");
          return status;
        }
      sleep (1);
      cnt--;

    }
  while (status == SANE_STATUS_DEVICE_BUSY);

  DBG (DBG_error,
       "pie_usb_wait_scanner failed: %s\n", sane_strstatus (status));
  return status;
}


/* -------------------------- PIE_USB_RELEASE_SCANNER ---------------------------- */
/**
 * @brief Release scanner after image aquisition
 *
 * @param[in] data points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - scanner is in parking position
 * - SANE_STATUS_DEVICE_BUSY - scanner timed out
 * - SANE_STATUS_IO_ERROR - on error
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 * - SANE_STATUS_INVAL - Received unexpected value
 *
 * @todo
 * The scanner does not accept the PIE_RELEASE_SCANNER command before
 * it has reached the parking position. So this command should
 * be called via sanei_begin_thread. However, that might not work
 * if the Sane backends have been built with pthread support when
 * the parent process, e.g. a forked one exits. It works with
 * xsane and scanimage.
 */
static int
pie_usb_release_scanner (void * passed)
{
  unsigned char buffer[16];
  unsigned char *data;
  SANE_Status status;
  size_t size = release_scanC[4];
  Pie_Scanner * scanner;
  int cnt = 0;

  DBG (DBG_proc, "pie_usb_release_scanner\n");

  scanner = (Pie_Scanner *) passed;

  /* wait upto 15 secs */
  status = pie_usb_wait_scanner (scanner, 15);
  if (status != SANE_STATUS_GOOD)
    return status;

  /* prepare to issue vendor specific 0xd2 command */
  memcpy (buffer, release_scanC, sizeof (release_scanC));
  data = buffer + sizeof (release_scanC);
  memset (data, 0, size);
  data[0] = 2;
  size += sizeof (release_scanC);

  /* try command for maximally 20 * 0.5 = 10 sec */
  do
    {
      status = pie_usb_scsi_wrapper (scanner->sfd, buffer, size, NULL, NULL);

      if (status != SANE_STATUS_DEVICE_BUSY)
        break;

      if (cnt == 1)
        DBG (DBG_info2,
             "pie_usb_release_scanner: scanner reports %s, waiting ...\n",
             sane_strstatus (status));

      usleep (TUR_WAIT_TIME);
      cnt++;
    }
  while (cnt < 20);

  if (status != SANE_STATUS_GOOD)
    return status;

  pie_power_save (scanner, 15);

  return status;
}

/* ---------------------- PIE_USB_IS_SCANNER_RELEASED ------------------------- */
/**
 * @brief Waits until a pie_usb_release_scanner has finished
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - Gracefully finished
 * - SANE_STATUS_IO_ERROR - Timed out
 */
static SANE_Status
pie_usb_is_scanner_released (Pie_Scanner * scanner)
{
  SANE_Status status = SANE_STATUS_DEVICE_BUSY;
  int cnt = 0;

  DBG (DBG_proc, "pie_usb_is_scanner_released: pid %d\n",
      (int) scanner->parking_pid);

  if (scanner->parking_pid == NO_PID)
    return SANE_STATUS_GOOD;
  else
    do
      {
        status = sanei_thread_get_status (scanner->parking_pid);
        if (status == SANE_STATUS_GOOD)
          break;

        if (cnt == 1)
          DBG (DBG_info2, "pie_usb_is_scanner_released: waiting ...\n");

        cnt++;
        usleep (TUR_WAIT_TIME);
      }
    while (cnt < 20);

  if (status == SANE_STATUS_GOOD)
    {
      scanner->parking_pid = NO_PID;
      DBG (DBG_proc, "pie_usb_is_scanner_released: success\n");
      return status;
    }
  else
    return SANE_STATUS_IO_ERROR;
}


/* --------------------- PIE_USB_IS_SCANNER_INITIALZED ----------------------- */
/**
 * @brief Wait for internal initialization of scanner
 *
 * @param[in] dn USB device number
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @note
 * During its initialization the scanner becomes easily screwed up.
 * This series of commands, however, is answered reliably.
 */
static SANE_Status
pie_usb_is_scanner_initialized (int dn)
{
  unsigned char buffer[16];
  SANE_Status status, state;
  uint32_t sense_kascq;
  int wait_cnt = 240;           /* 240 * 0.5 = 120 seconds */
  DBG (DBG_proc, "pie_usb_is_scanner_initialized\n");
  do
    {
      status =
        pie_usb_scsi_wrapper (dn, test_unit_ready.cmd,
                              test_unit_ready.size, NULL, NULL);
      if (status == SANE_STATUS_IO_ERROR)       /* Not Ready - Warming Up ? */
        {
          state = pie_usb_request_sense (dn, &sense_kascq);
          if (state != SANE_STATUS_GOOD)
            return state;
          if (sense_kascq != 0x020401)
            return status;
          else
            status = SANE_STATUS_DEVICE_BUSY;
        }

      if (status == SANE_STATUS_DEVICE_BUSY)
        {
          usleep (TUR_WAIT_TIME);
          wait_cnt--;
        }

      memset (buffer, 0, 11);
      status = pie_usb_read_status (dn, buffer);
      if (status == SANE_STATUS_IO_ERROR)       /* Not Ready - Warming Up ? */
        {
          state = pie_usb_request_sense (dn, &sense_kascq);
          if (state != SANE_STATUS_GOOD)
            return state;
          if (sense_kascq != 0x020401)
            return status;
          else
            status = SANE_STATUS_DEVICE_BUSY;
        }

      if (status == SANE_STATUS_GOOD)
        {
          DBG_DUMP (DBG_info, buffer, 11);
/*        if ((buffer[5] != 0) || (buffer[9] != 0) || (buffer[10] != 0))*/
          if (buffer[5] != 0)
            status = SANE_STATUS_DEVICE_BUSY;
        }

      if (status == SANE_STATUS_DEVICE_BUSY)
        {
          usleep (TUR_WAIT_TIME);
          wait_cnt--;
        }
    }
  while ((status == SANE_STATUS_DEVICE_BUSY) && (wait_cnt > 0));
  return status;
}


/* -------------------------- PIE_USB_COPY_SENSORS ---------------------------- */
/**
 * @brief Read a vector indicating which sensor elements are used
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy, timed out
 *
 * @note
 * pie_usb_copy_sensors reads a vector consisting of 0x00 and 0x70 before image
 * aquisition. 0x00 indicates that the sensor element is used for the
 * following image. The vector is stored in the scanner structure and
 * used for shading correction.
 */
static SANE_Status
pie_usb_copy_sensors (Pie_Scanner * scanner)
{
  size_t size_read;
  int cnt = 0;
  SANE_Status status;

  DBG (DBG_proc, "pie_usb_copy_sensors\n");

  do
    {
      size_read = scanner->device->cal_info[0].pixels_per_line;
      status =
        pie_usb_scsi_wrapper (scanner->sfd, pie_copyC, sizeof (pie_copyC),
                              scanner->cal_data->sensors, &size_read);
      if (status == SANE_STATUS_GOOD)
        return status;

      if (cnt == 1)
        DBG (DBG_info2,
             "pie_usb_copy_sensors: scanner reports %s, waiting ...\n",
             sane_strstatus (status));

      usleep (TUR_WAIT_TIME);
      cnt++;
    }
  while (cnt < 10);             /* maximally 10 * 0.5 = 5 sec */

  return status;
}

/* ------------------------------ PIE_USB_SCAN -------------------------------- */
/**
 * @brief Perform SCAN command
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy, timed out
 *
 * @note
 * Not all errors after the SCAN command are fatal here.
 */

static SANE_Status
pie_usb_scan (Pie_Scanner * scanner, int start)
{
  SANE_Status status, state;
  uint32_t sense_kascq;

  DBG (DBG_proc, "pie_usb_scan: %d\n", start);

  set_scan_cmd (scan.cmd, start);
  if (start)
    {
      /* wait upto X seconds until returned to start position */
      status = pie_usb_wait_scanner (scanner, 15);
      if (status != SANE_STATUS_GOOD)
        return status;
      do
        {
          status =
            pie_usb_scsi_wrapper (scanner->sfd, scan.cmd, scan.size,
                                  NULL, NULL);
          if (status)
            {
              DBG (DBG_error, "pie_usb_scan: received %s\n",
                   sane_strstatus (status));
              if (status == SANE_STATUS_IO_ERROR)
                {
                  state = pie_usb_request_sense (scanner->sfd, &sense_kascq);
                  if (state != SANE_STATUS_GOOD)
                    return state;
                  if (sense_kascq == 0x020401)  /* Not Ready - Warming Up */
                    status = SANE_STATUS_DEVICE_BUSY;
                  else if (sense_kascq == 0x068200)     /* calibration disable not granted */
                    status = SANE_STATUS_GOOD;
                }
              if (status == SANE_STATUS_DEVICE_BUSY)
                usleep (SCAN_WARMUP_WAIT_TIME);
            }
        }
      while (status == SANE_STATUS_DEVICE_BUSY);
      return status;
    }
  else
    {
      status =
        pie_usb_scsi_wrapper (scanner->sfd, scan.cmd, scan.size, NULL, NULL);
      if (status == SANE_STATUS_IO_ERROR)
        {
          state = pie_usb_request_sense (scanner->sfd, &sense_kascq);
          if (state != SANE_STATUS_GOOD)
            return state;
          if (sense_kascq != 0x0b0006)  /* ABORT message from initiator */
            return status;
        }
      return SANE_STATUS_GOOD;
    }
}


/*----------------------- PIE_USB_SET_WINDOW --------------------------- */
/**
 * @brief Issue SET_SCAN_FRAME via a SCSI WRITE command.
 *
 * @param[in] scanner Pointer to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - on error
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported by the OS
 *
 * @todo
 * Merge back into pie_set_window
 */
static SANE_Status
pie_usb_set_window (Pie_Scanner * scanner)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;
  double x, dpmm;

  DBG (DBG_proc, "pie_usb_set_window\n");

  size = 14;
  set_write_length (swrite.cmd, size);
  memcpy (buffer, swrite.cmd, swrite.size);
  data = buffer + swrite.size;
  memset (data, 0, size);
  set_command (data, SET_SCAN_FRAME);
  set_data_length (data, size - 4);

  data[4] = 0x80;

  dpmm = (double) scanner->device->inquiry_pixel_resolution / MM_PER_INCH;

  if (scanner->device->model->flags & PIE_USB_FLAG_MIRROR_IMAGE)
    {
      x =
        SANE_UNFIX (scanner->device->x_range.max -
                    scanner->val[OPT_BR_X].w) * dpmm;
      set_data (data, 6, (int) x, 2);
      DBG (DBG_info, "TL_X: %d\n", (int) x);

      x =
        SANE_UNFIX (scanner->device->x_range.max -
                    scanner->val[OPT_TL_X].w) * dpmm;
      set_data (data, 10, (int) x, 2);
      DBG (DBG_info, "BR_X: %d\n", (int) x);
    }
  else
    {
      x = SANE_UNFIX (scanner->val[OPT_TL_X].w) * dpmm;
      set_data (data, 6, (int) x, 2);
      DBG (DBG_info, "TL_X: %d\n", (int) x);

      x = SANE_UNFIX (scanner->val[OPT_BR_X].w) * dpmm;
      set_data (data, 10, (int) x, 2);
      DBG (DBG_info, "BR_X: %d\n", (int) x);
    }
  x = SANE_UNFIX (scanner->val[OPT_TL_Y].w) * dpmm;
  set_data (data, 8, (int) x, 2);
  DBG (DBG_info, "TL_Y: %d\n", (int) x);

  x = SANE_UNFIX (scanner->val[OPT_BR_Y].w) * dpmm;
  set_data (data, 12, (int) x, 2);
  DBG (DBG_info, "BR_Y: %d\n", (int) x);

  status =
    pie_usb_scsi_wrapper (scanner->sfd, buffer, swrite.size + size, NULL,
                          NULL);
  if (status)
    {
      DBG (DBG_error,
           "pie_usb_set_window: write command returned status %s\n",
           sane_strstatus (status));
    }

  return status;
}


/*-------------------------- PIE_USB_MODE_SELECT ------------------------------- */
/**
 * @brief Setup and send MODE command.
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @note
 * data[9] sets what sort of calibration sequence the scanner will
 * expect. Its value may differ between different types of scanners.
 */
static SANE_Status
pie_usb_mode_select (Pie_Scanner * scanner)
{
  PIE_USB_Model *model = scanner->device->model;
  SANE_Status status;
  unsigned char buffer[128];
  size_t size;
  unsigned char *data;
  int i, cal;

  DBG (DBG_proc, "pie_usb_mode_select\n");

  size = 16;
  set_mode_length (smode.cmd, size);
  memcpy (buffer, smode.cmd, smode.size);
  data = buffer + smode.size;
  memset (data, 0, size);
  /* size of data */
  data[1] = size - 2;
  /* set resolution required */
  set_data (data, 2, scanner->resolution, 2);
  /* set color filter and color depth */
  data[4] = 0;
  scanner->cal_filter = 0;
  switch (scanner->colormode)
    {
    case RGBI:
      data[4] = INQ_FILTER_IRED;
      scanner->cal_filter = INQ_FILTER_IRED;    /* fall through */
    case RGB:
      if (scanner->device->inquiry_filters & INQ_ONE_PASS_COLOR)
        {
          data[4] |= INQ_ONE_PASS_COLOR;
          scanner->cal_filter |= FILTER_RED | FILTER_GREEN | FILTER_BLUE;
        }
      else
        {
          DBG (DBG_error,
               "pie_usb_mode_select: support for multipass color not yet implemented\n");
          return SANE_STATUS_UNSUPPORTED;
        }
      if (scanner->val[OPT_BIT_DEPTH].w == 16)
        data[5] = INQ_COLOR_DEPTH_16;
      else
        data[5] = INQ_COLOR_DEPTH_8;
      break;
    default:
      DBG (DBG_error, "pie_usb_mode_select: wrong colour format!\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* choose color packing method */
  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    data[6] = INQ_COLOR_FORMAT_INDEX;
#if 0
  /* possible TODO: The scanner can do INQ_COLOR_FORMAT_LINE but our
   * reader routine for it has been neglected */
  else if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_LINE)
    data[6] = INQ_COLOR_FORMAT_LINE;
#endif
  else
    {
      DBG (DBG_error,
           "pie_usb_mode_select: support for pixel packing not yet implemented\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* choose data format */
  if (scanner->device->inquiry_image_format & INQ_IMG_FMT_INTEL)
    data[8] = INQ_IMG_FMT_INTEL;
  else
    {
      DBG (DBG_error,
           "pie_usb_mode_select: support for Motorola format not yet implemented\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* set required calibration and quality */
  i = 0;
  while (scanner->device->speed_list[i] != NULL)
    {
      if (strcmp
          (scanner->device->speed_list[i], scanner->val[OPT_SPEED].s) == 0)
        break;
      i++;
    }

  if (scanner->device->speed_list[i] == NULL)
    i = 0;                      /* neither should happen */
  if (i > 2)
    i = 2;
  cal = i;
  if (cal == 1)
    {
      if (scanner->val[OPT_PREVIEW].w == SANE_TRUE)
        cal = model->op_mode[OPM_PREVIEW];      /*CAL_MODE_FILM_NORMAL; */
      else
        cal = model->op_mode[OPM_QUALITY];      /*CAL_MODE_FILM_HIQUAL; */
    }
  /* skip calibration if no quality in this or last scan */
  if ((cal != model->op_mode[OPM_QUALITY]) &&
      (scanner->cal_mode != model->op_mode[OPM_QUALITY]))
    cal = model->op_mode[OPM_SKIPCAL];
  data[9] = cal;
  scanner->cal_mode = cal;

  /* unsupported for USB film scanners: halftone, threshold */
  /* 12: halftone pattern 0 */
  data[13] = 0x80;              /* lineart threshold */
  data[14] = 0x10;              /* ?? */

  DBG (DBG_info, "pie_usb_mode_select: speed %02x\n", data[9]);
  DBG (DBG_info, "pie_usb_mode_select sending:\n");
  DBG_DUMP (DBG_info, data, size);
  status =
    pie_usb_scsi_wrapper (scanner->sfd, buffer, smode.size + size,
                          NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
           "pie_usb_mode_select: write command returned status %s\n",
           sane_strstatus (status));
    }

  return status;
}


/* ------------------------------------ PIE_USB_GET_PARAMS ------------------------ */
/**
 * @brief Send SCSI PARAM command.
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_INVAL - received unexpected value
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_IO_ERROR - something got screwed up
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_UNSUPPORTED - if the feature is not supported
 *
 * @todo
 * Merge back with pie_get_params
 */
static SANE_Status
pie_usb_get_params (Pie_Scanner * scanner)
{
  unsigned char buffer[128];
  SANE_Status status;
  size_t size = 18;

  DBG (DBG_proc, "pie_usb_get_params\n");

  status = pie_usb_wait_scanner (scanner, 5);
  if (status)
    return status;

  set_param_length (param.cmd, size);

  status =
    pie_usb_scsi_wrapper (scanner->sfd, param.cmd, param.size, buffer, &size);
  if (status)
    {
      DBG (DBG_error, "pie_usb_get_params: command returned status %s\n",
           sane_strstatus (status));
    }
  else
    {
      DBG (DBG_info, "Scan Width:  %d\n", get_param_scan_width (buffer));
      DBG (DBG_info, "Scan Lines:  %d\n", get_param_scan_lines (buffer));
      DBG (DBG_info, "Scan bytes:  %d\n", get_param_scan_bytes (buffer));

      DBG (DBG_info, "Offset 1:    %d\n",
           get_param_scan_filter_offset1 (buffer));
      DBG (DBG_info, "Offset 2:    %d\n",
           get_param_scan_filter_offset2 (buffer));
      DBG (DBG_info, "Scan period: %d\n", get_param_scan_period (buffer));
      DBG (DBG_info, "Xfer rate:   %d\n", get_param_scsi_xfer_rate (buffer));
      DBG (DBG_info, "Avail lines: %d\n",
           get_param_scan_available_lines (buffer));

      scanner->filter_offset1 = get_param_scan_filter_offset1 (buffer);
      scanner->filter_offset2 = get_param_scan_filter_offset2 (buffer);
      scanner->bytes_per_line = get_param_scan_bytes (buffer);

      scanner->params.pixels_per_line = get_param_scan_width (buffer);
      scanner->params.lines = get_param_scan_lines (buffer);

      if (scanner->colormode == RGBI)
        {
#ifdef SANE_FRAME_RGBI
          scanner->params.format = SANE_FRAME_RGBI;
          scanner->params.bytes_per_line = 4 * get_param_scan_bytes (buffer);
#else
          scanner->params.format = SANE_FRAME_RGB;
          scanner->params.bytes_per_line = 3 * get_param_scan_bytes (buffer);
#endif
          scanner->params.depth = scanner->val[OPT_BIT_DEPTH].w;
        }
      else if ((scanner->colormode == RGB) || (scanner->processing & POST_SW_DIRT))
        {
          scanner->params.format = SANE_FRAME_RGB;
          scanner->params.depth = scanner->val[OPT_BIT_DEPTH].w;
          scanner->params.bytes_per_line = 3 * get_param_scan_bytes (buffer);
        }
      else
        {
          DBG (DBG_error, "pie_usb_get_params: wrong colour format!\n");
          return SANE_STATUS_UNSUPPORTED;
        }
      scanner->params.last_frame = 0;
    }
  return status;
}

/*
 * @@ USB calibration functions
 */

/* ------------------------ PIE_USB_CALICALC_HIQUAL ---------------------------- */
/**
 * @brief Calculate gain and exposure for quality mode
 *
 * @param[in] scanner points to structure of opened scanner
 * @param[in] d7cal points to calibration data read by a PIE_READ_CALIBRATION command
 *
 * @note
 * Empirical data show that in quality mode for a color (j) the illumination (Ij)
 * reached with certain gain (gj) and exposure time (tj) settings can be described
 * by (I)  Ij = aj * exp (c * gj^2) * tj . The gain constants gj and desired brightness
 * has to be provided. All other values can be derived from the calibration lines
 * or the 0xd7 read. In quality mode maximal gain is limited and most of the
 * calibration is done with exposure time.
 * The result of this routine is stored in scanner->cal_data.
 */
static void
pie_usb_calicalc_hiqual (Pie_Scanner * scanner,
                         PIE_USB_Calibration_Read * d7cal)
{
  PIE_USB_Model *model = scanner->device->model;
  PIE_USB_Calibration *caldat = scanner->cal_data;
  double fact, dgain;
  int i, tg, tt, tmax;

  SANE_Status status;
  int brightnesses[3];
  int pokebuf[4];
  int pokesiz = 3;

  DBG (DBG_proc, "pie_usb_calicalc_hiqual\n");

  for (i = 0; i < 3; i++)
    brightnesses[i] = caldat->brightness[i];
  status = pie_usb_poke_ints ("/tmp/bright.txt", pokebuf, &pokesiz);
  if (status == SANE_STATUS_GOOD && pokesiz)
    {
      for (i = 0; i < 3; i++)
        brightnesses[i] = pokebuf[i];
      DBG (DBG_info,
           "pie_usb_calicalc_hiqual poked brightness %d, %d, %d\n",
           brightnesses[0], brightnesses[1], brightnesses[2]);
    }

  tmax = 0;
  for (i = 0; i < 3; i++)
    {
      /* overall illumination correction factor
         fact = I / I0 = exp (c * (g^2 - g0^2)) * t / t0 */
      fact = (double) brightnesses[i] / (double) caldat->mean_shade[i];
      /* calculate gain from f^(1/p) = exp (c * (g^2 - g0^2)), the part done by gain */
      dgain =
        log (fact) / (model->gain_const[i] *
                      (double) model->gain_hiqual_part) +
        (double) d7cal->gain[i] * (double) d7cal->gain[i];
      if (dgain < 0)
        tg = model->gain_min;
      else
        {
          tg = sqrt (dgain) + 0.5;
          if (tg < model->gain_min)
            tg = model->gain_min;
          if (tg > model->gain_hiqual_max)
            tg = model->gain_hiqual_max;
        }
      caldat->cal_hiqual.gain[i] = tg;
      DBG (DBG_info,
           "pie_usb_calicalc_hiqual gain[%d] = 0x%02x = %d\n", i, tg, tg);

      /* the rest has to be done by exposure time */
      dgain = fact;
      if (tg != d7cal->gain[i])
        dgain *=
          exp (model->gain_const[i] *
               ((double) d7cal->gain[i] * (double) d7cal->gain[i] -
                (double) tg * (double) tg));
      tt = dgain * (double) d7cal->texp[i] + 0.5;

      /* if exposure time is too short try to redo the gain,
         should not happen very often as "fact" is usually > 1 */
      if (tt < d7cal->t_min)
        {
          tt = d7cal->t_min;
          fact *= (double) tt / (double) d7cal->texp[i];
          dgain =
            log (fact) / model->gain_const[i] +
            (double) d7cal->gain[i] * (double) d7cal->gain[i];
          if (dgain < 0)
            tg = model->gain_min;
          else
            {
              tg = sqrt (dgain) + 0.5;
              if (tg < model->gain_min)
                tg = model->gain_min;
            }
          caldat->cal_hiqual.gain[i] = tg;
          DBG (DBG_info,
               "pie_usb_calicalc_hiqual regain[%d] = 0x%02x = %d\n",
               i, tg, tg);
        }
      caldat->cal_hiqual.texp[i] = tt;
      if (tt > tmax)
        tmax = tt;
      DBG (DBG_info,
           "pie_usb_calicalc_hiqual texp[%d] = 0x%02x = %d\n", i, tt, tt);
    }
  caldat->cal_hiqual.texp_max = tmax;
}

/* ------------------------ PIE_USB_CALICALC_NORMAL ----------------------------
 *
 * @brief Calculate gain and exposure for normal mode
 *
 * @param[in] scanner points to structure of opened scanner
 * @param[in] d7cal points to calibration data read by a PIE_READ_CALIBRATION command
 *
 * @note
 * In normal mode we have to fight nasty offsets with some types of scanners and
 * a rather narrow window for exposure time. Here for a color (j) the illumination
 * (Ij) reached with certain gain (gj) and exposure time (tj) settings can be described
 * by (II)  I = n * aj * exp(c * gj^2) * tj - (n - 1) * 65536 . The gain constant c, offset
 * factor n and desired brightness have to be provided. Gain may be really maximal here,
 * i.e. 0x3f, the exposure time has to be larger than t_min.
 * The result of this routine is stored in scanner->cal_data.
 */
static void
pie_usb_calicalc_normal (Pie_Scanner * scanner,
                         PIE_USB_Calibration_Read * d7cal)
{
  PIE_USB_Model *model = scanner->device->model;
  PIE_USB_Calibration *caldat = scanner->cal_data;
  int tg, tt, i, tmax;

  SANE_Status status;
  int brightnesses[3];
  int pokebuf[4];
  int pokesiz = 3;

  DBG (DBG_proc, "pie_usb_calicalc_normal\n");

  for (i = 0; i < 3; i++)
    brightnesses[i] = caldat->brightness[i];
  status = pie_usb_poke_ints ("/tmp/bright.txt", pokebuf, &pokesiz);
  if (status == SANE_STATUS_GOOD && pokesiz)
    {
      for (i = 0; i < 3; i++)
        brightnesses[i] = pokebuf[i];
      DBG (DBG_info,
           "pie_usb_calicalc_normal poked brightness %d, %d, %d\n",
           brightnesses[0], brightnesses[1], brightnesses[2]);
    }

  tmax = 0;
  for (i = 0; i < 3; i++)
    {
      /* start calculating gain for default texp rounding down.
         Following horrible expression is obtained by setting up equation (II) for the desired
         brightness, setting up quality mode equation (I) using the information from the
         0xd7 read together with the achieved brightness of read calibration lines and solving
         that system for the gain required in normal mode */
      tg = sqrt (log ((((double) brightnesses[i] +
                        ((model->offs_factor[i] -
                          1.0) * 65536)) * (double) d7cal->texp[i]) /
                      (model->offs_factor[i] *
                       (double) model->default_normal.texp[i] *
                       (double) caldat->mean_shade[i])) /
                 model->gain_const[i] +
                 (double) d7cal->gain[i] * (double) d7cal->gain[i]);
      if (tg > 0x3f)
        tg = 0x3f;
      /* here the gain (tg) is smaller than for what it was calculated, so we need
         a longer exposure time. Using the same set of equations as above exposure time is
         calculated for the given gain */
      tt = (((double) brightnesses[i] + ((model->offs_factor[i] -
                                          1.0) * 65536.0)) *
            (double) d7cal->texp[i]) / (model->offs_factor[i] *
                                        (double) caldat->mean_shade[i]) *
        exp (model->gain_const[i] *
             ((double) d7cal->gain[i] * (double) d7cal->gain[i] - tg * tg)) +
        0.5;
      if (tt < d7cal->t_min)
        tt = d7cal->t_min;
      if (tt > model->texp_normal_max)
        tt = model->texp_normal_max;

      caldat->cal_normal.gain[i] = tg;
      caldat->cal_normal.texp[i] = tt;
      if (tt > tmax)
        tmax = tt;
      DBG (DBG_info,
           "pie_usb_calicalc_normal gain[%d] = 0x%02x = %d, texp[%d] = 0x%02x = %d\n",
           i, tg, tg, i, tt, tt);
    }
  caldat->cal_normal.texp_max = tmax;
}


/* ---------------------- PIE_USB_CALICALC_SLOW_DOWN --------------------------- */
/**
 * @brief Calculate coefficient for slowing down the scan
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - calculated coefficient
 *
 * @note
 * At high resolutions to much data seem to be delivered to the scanners' USB
 * interface. So we have to slow down. As we do not have the results from the
 * PARAMS call yet calculate from our internal values.
 */
static unsigned char
pie_usb_calicalc_slow_down (Pie_Scanner * scanner)
{
  double width, x_dpmm;
  int slow_down;

  DBG (DBG_proc, "pie_usb_calicalc_slow_down\n");

  slow_down = 0;
  width = SANE_UNFIX (scanner->val[OPT_BR_X].w - scanner->val[OPT_TL_X].w);
  x_dpmm = SANE_UNFIX (scanner->val[OPT_RESOLUTION].w) / MM_PER_INCH;
  if ((width > 0) && (x_dpmm > 0))
    {
      width *= 3 * x_dpmm;      /* calculate for RGB */
      if (scanner->val[OPT_BIT_DEPTH].w > 8)
        width *= 2;             /* bytes per line */

      if (width > SLOW_START)
        {                       /* slow down value */
          width = (width - SLOW_START) * SLOW_HEIGHT / SLOW_LENGTH;
          if (scanner->cal_mode ==      /* correct for exposure time */
              scanner->device->model->op_mode[OPM_QUALITY])
            width *=
              (double) scanner->device->model->default_hiqual.texp_max /
              (double) scanner->cal_data->cal_hiqual.texp_max;
          else
            width *=
              (double) scanner->device->model->default_normal.texp_max /
              (double) scanner->cal_data->cal_normal.texp_max;
          slow_down = (int) (width + 0.5);
          if (scanner->colormode == RGBI)       /* correct for infrared */
            slow_down -= 2;
          if ((slow_down < 0) || (slow_down >= 16))
            slow_down = 0;
        }
    }
  DBG (DBG_info, "pie_usb_calicalc_slow_down: %d\n", slow_down);
  return slow_down;
}


/* ------------------------ PIE_USB_CALIBRATION_SEND -------------------------- */
/**
 * @brief Read first and then send calibration
 *
 * @param[in] scanner Pointer to structure of opened scanner
 * @param[in] calc_cal If !=0, calculate parameters in between
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_INVAL - on every other error
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 *
 * @note
 * This routine contains the vendor SCSI commands 0xd7 for reading and 0xdc
 * for writing. As in the logs we first read 103 bytes and then write 23 bytes.
 * Only R G and B exposure times (texp), gains and slow_down values are
 * calibrated all other values are copied from the 0xd7 read.
 */
static SANE_Status
pie_usb_calibration_send (Pie_Scanner * scanner, int calc_cal)
{
  PIE_USB_Calibration_Read calD7in;
  PIE_USB_Calibration_Send calDCout;
  size_t size_read = 103;
  size_t size_write = 23;
  SANE_Status status;
  unsigned char pokebuf[64];
  int i, pokesiz;

  DBG (DBG_proc, "pie_usb_calibration_send\n");

  status =                      /* 103 bytes */
    pie_usb_scsi_wrapper (scanner->sfd, read_calibrationC,
                          sizeof (read_calibrationC), &calD7in, &size_read);
  if (status != SANE_STATUS_GOOD)
    return status;

  DBG (DBG_info, "pie_usb_calibration_send received:\n");
  DBG_DUMP (DBG_info, (unsigned char *) &calD7in, 103);

  if (calD7in.illumination[0] == 0)     /* assume same target as for blue */
    calD7in.illumination[0] = calD7in.illumination[2];
  if (calD7in.illumination[1] == 0)     /* assume same target as for blue */
    calD7in.illumination[1] = calD7in.illumination[2];
  for (i = 0; i < 3; i++)
    scanner->cal_data->target_shade[i] = calD7in.illumination[i];
  scanner->cal_data->target_shade[3] = calD7in.illumination[2]; /* ired ?? */

  /* now we have all data to calculate calibration */
  if (calc_cal)
    {
      pie_usb_calicalc_hiqual (scanner, &calD7in);
      pie_usb_calicalc_normal (scanner, &calD7in);
    }
  /* load vector to be sent */
  size_write += 6;
  memcpy (&(calDCout.scsi_cmd), write_calibrationC, 6);
  /* first with what we have/had calculated */
  if (scanner->cal_mode != scanner->device->model->op_mode[OPM_QUALITY])        /* normal mode */
    {
      for (i = 0; i < 3; i++)
        calDCout.texp[i] = scanner->cal_data->cal_normal.texp[i];
      for (i = 0; i < 3; i++)
        calDCout.gain[i] = scanner->cal_data->cal_normal.gain[i];
    }
  else                          /* quality mode */
    {
      for (i = 0; i < 3; i++)
        calDCout.texp[i] = scanner->cal_data->cal_hiqual.texp[i];
      for (i = 0; i < 3; i++)
        calDCout.gain[i] = scanner->cal_data->cal_hiqual.gain[i];
    }

  /* then with values which are usully copied */
  memcpy (calDCout.offset, calD7in.offset, 6);  /* offsets, zero_2 */

  calDCout.some_time[0] = calD7in.some_time;
  /* slow down at high resolutions */
  if (calc_cal
      || (scanner->cal_mode == scanner->device->model->op_mode[OPM_SKIPCAL]))
    calDCout.some_time[1] = pie_usb_calicalc_slow_down (scanner);
  else
    calDCout.some_time[1] = 0;
  calDCout.some_time[2] = 0;

  memcpy (&calDCout.infrared, &calD7in.infrared, sizeof (PIE_USB_cal_ired));
  pokesiz = 12;
  status = pie_usb_poke_bytes ("/tmp/calbytes.txt", pokebuf, &pokesiz);
  if (status == SANE_STATUS_GOOD)
    {
      memcpy (calDCout.texp, &pokebuf[0], 6);
      memcpy (calDCout.gain, &pokebuf[6], 6);
    }

  DBG (DBG_info, "pie_usb_calibration_send sending:\n");
  DBG_DUMP (DBG_info, (unsigned char *) &calDCout, size_write);
  status =
    pie_usb_scsi_wrapper (scanner->sfd, &calDCout, size_write, NULL, NULL);
  return status;
}


/* ---------------------------- PIE_USB_CALIBRATE ----------------------------- */
/**
 * @brief Do a full calibration of the scanner
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - on success
 * - SANE_STATUS_IO_ERROR - if an error occurred during the write
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_INVAL - on every other error
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy
 * - SANE_STATUS_NO_MEM - buffer allocation failed
 *
 * @note
 * This routine is called in opm_quality mode and when modes have
 * changed after the SCAN command. It acts as follows
 * - 1 line of image data is read
 * - test unit ready
 * - 13 lines are read  in indexed RGBI
 * - something should be learned from that
 * - calibration info is read, calculated and sent to the scanner
 * - dump left over lines
 * Number of all lines in scanner->device->cal_info[any].num_lines
 */
static SANE_Status
pie_usb_calibrate (Pie_Scanner * scanner)
{
  unsigned int val;
  unsigned char *rcv_buffer, *src;
  int rcv_length, rcv_lines, rcv_bits;
  int pixels_per_line, average_lines;
  int i, j, k, l;
  int n[4];
  int *dest;
  double dval;
  size_t size;
  SANE_Status status;

  DBG (DBG_proc, "pie_usb_calibrate\n");

  rcv_lines = scanner->device->cal_info[0].num_lines;
  average_lines = rcv_lines - 1;
  if (AVERAGE_CAL_LINES < average_lines)
    average_lines = AVERAGE_CAL_LINES;
  pixels_per_line = scanner->device->cal_info[0].pixels_per_line;
  rcv_length = pixels_per_line;
  rcv_bits = scanner->device->cal_info[0].receive_bits;
  if (rcv_bits > 8)
    rcv_length *= 2;
  if ((scanner->colormode == RGB) || (scanner->colormode == RGBI))
    rcv_length = (rcv_length + 2) * 4;
  else
    return SANE_STATUS_INVAL;

  rcv_buffer = (unsigned char *) malloc (rcv_length);
  if (!rcv_buffer)
    return SANE_STATUS_NO_MEM;

  status = pie_usb_wait_scanner (scanner, 30);
  if (status != SANE_STATUS_GOOD)
    goto freend;
  set_read_length (sread.cmd, 4);       /* a test line */
  size = rcv_length;
  status =
    pie_usb_scsi_wrapper (scanner->sfd, sread.cmd, sread.size,
                          rcv_buffer, &size);
  if (status != SANE_STATUS_GOOD)       /* should not happen */
    {
      status = SANE_STATUS_GOOD;        /* try to recover */
      goto freend;
    }
  rcv_lines--;
  /* this gets the data */
  memset (scanner->cal_data->shades, 0, pixels_per_line * 4 * sizeof (int));
  scanner->cal_data->shades[0] = 0;
  for (i = 0; i < 4; i++)
    n[i] = 0;
  for (k = 0; k < average_lines; k++)
    {
      size = rcv_length;
      status =
        pie_usb_scsi_wrapper (scanner->sfd, sread.cmd, sread.size,
                              rcv_buffer, &size);
      if (status != SANE_STATUS_GOOD)
        goto freend;
      for (i = 0; i < 4; i++)
        {
          if (rcv_bits > 8)
            src = rcv_buffer + i * (pixels_per_line + 1) * 2;
          else
            src = rcv_buffer + i * (pixels_per_line + 2);
          if (*src == 'R')
            l = 0;
          else if (*src == 'G')
            l = 1;
          else if (*src == 'B')
            l = 2;
          else if (*src == 'I')
            l = 3;
          else
            {
              DBG (DBG_error,
                   "pie_usb_calibrate: invalid index byte (%02x)\n", *src);
              DBG_DUMP (DBG_error, src, 32);
              status = SANE_STATUS_INVAL;
              goto freend;
            }
          src += 2;
          dest = scanner->cal_data->shades + l * pixels_per_line;
          if (rcv_bits > 8)
            {
              for (j = 0; j < pixels_per_line; j++)
                {
                  val = *src++;
                  val += (*src++) << 8;
                  *dest++ += val;
                }
            }
          else
            {
              for (j = 0; j < pixels_per_line; j++)
                *dest++ += *src++;
            }
          n[l]++;
        }
      rcv_lines--;
    }
  for (l = 0; l < 4; l++)
    {
      dest = scanner->cal_data->shades + l * pixels_per_line;
      dval = 0;
      for (j = pixels_per_line; j > 0; j--)
        {
          dval += *dest;
          *dest++ /= n[l];
        }
      scanner->cal_data->mean_shade[l] =
        dval / (double) (n[l] * pixels_per_line) + 0.5;
      DBG (DBG_info,
           "pie_usb_calibrate: color %d, mean %d = %04x\n", l,
           scanner->cal_data->mean_shade[l],
           scanner->cal_data->mean_shade[l]);
    }
  if (DBG_LEVEL >= DBG_image)
    pie_usb_shades_to_pnm (scanner, "/tmp/pieshading", 64);
  /* get, calculate and send calibration */
  status = pie_usb_calibration_send (scanner, 1);
  if (status != SANE_STATUS_GOOD)
    goto freend;
  /* we have to read all lines, discard the rest */
  while (rcv_lines > 0)
    {
      size = rcv_length;
      status =
        pie_usb_scsi_wrapper (scanner->sfd, sread.cmd, sread.size,
                              rcv_buffer, &size);
      if (status != SANE_STATUS_GOOD)
        goto freend;
      rcv_lines--;
    }

freend:                 /* ugly, but this may be one of the cases ... */
  free (rcv_buffer);
  return status;
}


/* ------------------------- PIE_USB_CORRECT_SHADING -------------------------- */
/**
 * @brief Do shading and mirror correction on an image line.
 *
 * @param[in] scanner points to structure of opened scanner
 * @param[in] in_buf Array of pointers to separate R, G, B, I input data
 * @param[out] out_buf Array of pointers to corrected data
 * @param[in] pixels Line width
 * @param[in] bits bit depth of data
 * @param[in] start_plane color plane to start with
 * @param[in] end_plane color plane to end with
 *
 * @note
 * This routine should be called as soon as possible after a line
 * has been read as it provides the right endianess.
 */
static void
pie_usb_correct_shading (Pie_Scanner * scanner,
                         unsigned char *in_buf[4], SANEI_IR_bufptr out_buf[4],
                         int pixels, int bits, int start_plane, int end_plane)
{
  SANEI_IR_bufptr buf[4];
  unsigned char *sensors;
  unsigned int val;
  int *shade[4];
  int target[4];
  int cal_pixels;
  int cal_idx = 0;
  int mirror;
  int i, j, k;

  DBG (DBG_proc, "pie_usb_correct_shading: %d to %d\n", start_plane,
       end_plane);

  mirror = scanner->device->model->flags & PIE_USB_FLAG_MIRROR_IMAGE;
  cal_pixels = scanner->device->cal_info[0].pixels_per_line;
  sensors = scanner->cal_data->sensors;
  for (k = start_plane; k <= end_plane; k++)
    {
      shade[k] = scanner->cal_data->shades + k * cal_pixels;
      /* we can take means or targets here,
         targets may lead to values larger than 16 bit */
      target[k] = scanner->cal_data->mean_shade[k];
      if (mirror == 0)
        buf[k] = out_buf[k];
      else if (bits > 8)
        buf[k].b16 = out_buf[k].b16 + pixels - 1;
      else
        buf[k].b8 = out_buf[k].b8 + pixels - 1;
    }

  j = 0;
  for (i = 0; i < pixels; i++)
    {
      while (sensors[j] != 0 && j < cal_pixels)
        j++;
      if (j < cal_pixels)
        {
          cal_idx = j;
          j++;
        }
      if (mirror == 0)
        {
          if (bits > 8)
            for (k = start_plane; k <= end_plane; k++)
              {
                val = *in_buf[k]++;
                val += (*in_buf[k]++) << 8;
                if (val > 4096)
                  {
                    val = (val * target[k]) / shade[k][cal_idx];
                    if (val > 0xffff)
                      val = 0xffff;
                  }
                *buf[k].b16++ = val;
              }
          else
            for (k = start_plane; k <= end_plane; k++)
              {
                val = *in_buf[k]++;
                if (val > 16)
                  {
                    val = (val * target[k]) / shade[k][cal_idx];
                    if (val > 0xff)
                      val = 0xff;
                  }
                *buf[k].b8++ = val;
              }
        }
      else
        {
          if (bits > 8)
            for (k = start_plane; k <= end_plane; k++)
              {
                val = *in_buf[k]++;
                val += (*in_buf[k]++) << 8;
                if (val > 4096)
                  {
                    val = (val * target[k]) / shade[k][cal_idx];
                    if (val > 0xffff)
                      val = 0xffff;
                  }
                *buf[k].b16-- = val;
              }
          else
            for (k = start_plane; k <= end_plane; k++)
              {
                val = *in_buf[k]++;
                if (val > 16)
                  {
                    val = (val * target[k]) / shade[k][cal_idx];
                    if (val > 0xff)
                      val = 0xff;
                  }
                *buf[k].b8-- = val;
              }
        }
    }
}


/*
 * @@ USB image reading and processing
 */

/* ------------------------- PIE_USB_READER_REORDER --------------------------- */
/**
 * @brief Interleave RGB(I) and do color lookup
 *
 * @param[in] scanner Pointer to structure of opened scanner
 * @param[in] in_img Pointer to separate R, G, B, (I) color planes
 * @param[out] out_img Pointer to interleaved RGB(I) data
 * @param[in] planes Number of color planes to work on
 * @param[in] pixels Number of pixels to work on
 */
static void
pie_usb_reader_reorder (Pie_Scanner * scanner, SANEI_IR_bufptr * in_img,
                        SANEI_IR_bufptr out_img, int planes, int pixels)
{
  SANEI_IR_bufptr dest;
  SANEI_IR_bufptr cptr[4];
  uint16_t *gamma_lut;
  int i, k;

  DBG (DBG_proc,
       "pie_usb_reader_reorder:  %d pixels\n", pixels);

  for (i = 0; i < planes; i++)
    cptr[i] = in_img[i];

  dest = out_img;
  if (scanner->processing & POST_SW_COLORS)
    {
      if (scanner->val[OPT_SW_NEGA].w == SANE_TRUE)
        {
          if (scanner->params.depth > 8)
            {
              gamma_lut = scanner->gamma_lut16;
              for (i = pixels; i > 0; i--)
                for (k = 0; k < planes; k++)
                  *dest.b16++ = (uint16_t) (65535 - gamma_lut[*cptr[k].b16++]);
            }
          else
            {
              gamma_lut = scanner->gamma_lut8;
              for (i = pixels; i > 0; i--)
                for (k = 0; k < planes; k++)
                  *dest.b8++ = (uint8_t) (255 - gamma_lut[*cptr[k].b8++]);
            }
        }
      else
        {
          if (scanner->params.depth > 8)
            {
              gamma_lut = scanner->gamma_lut16;
              for (i = pixels; i > 0; i--)
                for (k = 0; k < planes; k++)
                  *dest.b16++ = (uint16_t) gamma_lut[*cptr[k].b16++];
            }
          else
            {
              gamma_lut = scanner->gamma_lut8;
              for (i = pixels; i > 0; i--)
                for (k = 0; k < planes; k++)
                  *dest.b8++ = (uint8_t) gamma_lut[*cptr[k].b8++];
            }
        }
    }
  else
    {
      if (scanner->params.depth > 8)
        {
          for (i = pixels; i > 0; i--)
            for (k = 0; k < planes; k++)
              *dest.b16++ = *cptr[k].b16++;
        }
      else
        {
          for (i = pixels; i > 0; i--)
            for (k = 0; k < planes; k++)
              *dest.b8++ = *cptr[k].b8++;
        }
    }
}


/* ---------------------------- PIE_USB_SW_STORE ------------------------------ */
/**
 * @brief Interleave RGB(I), do color lookup and store data
 *
 * @param[in] scanner Pointer to structure of opened scanner
 * @param[in] in_img Pointer to separate R, G, B, (I) color planes
 * @param[in] planes Number of color planes
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_NO_MEM - no buffer for storing
 */
static SANE_Status
pie_usb_sw_store (Pie_Scanner * scanner,
    SANEI_IR_bufptr * in_img, int planes)
{
  SANE_Status status = SANE_STATUS_GOOD;
  size_t size;

  DBG (DBG_proc,
       "pie_usb_sw_store:  %d lines of %d bytes/line\n",
       scanner->params.lines, scanner->params.bytes_per_line);

  if (scanner->img_buffer.b8)
    free (scanner->img_buffer.b8);
  size = scanner->params.bytes_per_line * scanner->params.lines;
  scanner->img_buffer.b8 = malloc (size);
  if (scanner->img_buffer.b8)
    {
      pie_usb_reader_reorder (scanner, in_img, scanner->img_buffer, planes,
                              scanner->params.pixels_per_line * scanner->params.lines);
      if (DBG_LEVEL >= DBG_image)
        {
          pie_usb_write_pnm_file ("/tmp/RGBi-img.pnm", scanner->img_buffer.b8,
                                  scanner->params.depth, planes,
                                  scanner->params.pixels_per_line,
                                  scanner->params.lines);
        }
    }
  else
    {
      DBG (DBG_error, "pie_usb_sw_store: no buffer\n");
      status = SANE_STATUS_NO_MEM;
    }

  return status;
}


/* -------------------------- PIE_USB_READER_WRITE ---------------------------- */
/**
 * @brief Interleave RGB(I), do color lookup and write data to pipe
 *
 * @param[in] scanner Pointer to structure of opened scanner
 * @param[in] fp Pipe to write to
 * @param[in] in_img Pointer to separate R, G, B, (I) color planes
 * @param[in] planes Number of color planes
 * @param[in] lines Number of lines to work on
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_IO_ERROR - pipe error
 * - SANE_STATUS_NO_MEM - no buffer for reordering
 */
static SANE_Status
pie_usb_reader_write (Pie_Scanner * scanner, FILE * fp,
                      SANEI_IR_bufptr * in_img, int planes, int lines)
{
  SANE_Status status = SANE_STATUS_GOOD;
  SANEI_IR_bufptr reorder;
  size_t size, nwrite;

  DBG (DBG_proc,
       "pie_usb_reader_write:  %d lines of %d bytes/line\n",
       lines, scanner->params.bytes_per_line);

  size = scanner->params.bytes_per_line * lines;
  reorder.b8 = malloc (size);
  if (reorder.b8)
    {
      pie_usb_reader_reorder (scanner, in_img, reorder, planes,
                              scanner->params.pixels_per_line * lines);

      nwrite = fwrite (reorder.b8, 1, size, fp);
      if (nwrite < size)        /* abort */
        {
          DBG (DBG_error, "pie_usb_reader_write: pipe error\n");
          status = SANE_STATUS_IO_ERROR;
        }

      free (reorder.b8);
    }
  else
    {
      DBG (DBG_error, "pie_usb_reader_write: no buffer\n");
      status = SANE_STATUS_NO_MEM;
    }

  return status;
}


/* -------------------------- PIE_USB_SMOOTHEN RGB ---------------------------- */
/**
 * @brief Apply triangular blur to R, G, B image data
 *
 * @param[in] params describes image
 * @param     the_img points to R, G, B color planes
 * @param[in] win_size window size for mean filtering
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 *
 * @note
 * Two successive mean filters are applied as an approximation to
 * gaussian smoothening. Routine may be used to reduce film grain.
 * Input data is replaced.
 */
static SANE_Status
pie_usb_smoothen_rgb (const SANE_Parameters * params,
                      SANEI_IR_bufptr * the_img, int win_size)
{
  SANEI_IR_bufptr cplane[3];    /* R, G, B, (I) gray scale planes */
  SANEI_IR_bufptr plane;
  int depth, i;
  size_t itop;
  SANE_Status ret = SANE_STATUS_NO_MEM;

  for (i = 0; i < 3; i++)
    cplane[i] = the_img[i];

  depth = params->depth;
  itop = params->lines * params->pixels_per_line;
  if (depth > 8)
    plane.b8 = malloc (itop * sizeof (uint16_t));
  else
    plane.b8 = malloc (itop * sizeof (uint8_t));
  if (!plane.b8)
    DBG (5, "pie_usb_smoothen_rgb: Cannot allocate buffer\n");
  else
    {
      for (i = 0; i < 3; i++)
        {
          ret =
            sanei_ir_filter_mean (params, cplane[i], plane, win_size, win_size);
          if (ret != SANE_STATUS_GOOD)
            break;
          ret =
            sanei_ir_filter_mean (params, plane, cplane[i], win_size, win_size);
          if (ret != SANE_STATUS_GOOD)
            break;
        }
      free (plane.b8);
    }
  return ret;
}

/* ------------------------ PIE_USB_SW_FINAL_CROP ----------------------------- */
/**
 * @brief Crop separate color planes of an image
 *
 * @param[in]  parameters describes one color plane of the image
 * @param[in]  scanner Pointer to structure of opened scanner
 * @param[in]  in_img Pointer to R, G, B(, I) color planes
 * @param[in]  top cropping position
 * @param[in]  bot cropping position
 * @param[in]  left cropping position
 * @param[in]  right cropping position
 * @param[in]  planes how many planes to crop
 *
 * @note
 * Updates scanner->params
 */
static SANE_Status
pie_usb_sw_crop_planes (SANE_Parameters * parameters, Pie_Scanner * scanner,
    SANEI_IR_bufptr * in_img, int top, int bot, int left, int right, int planes)
{
  SANEI_IR_bufptr cplane[4];    /* R, G, B, I gray scale planes */
  SANE_Parameters params;
  SANE_Status status;
  int i;

  for (i = 0; i < planes; i++)
    cplane[i] = in_img[i];

  for (i = 0; i < planes; i++)
    {
      memcpy (&params, parameters, sizeof (SANE_Parameters));
      status =
        sanei_magic_crop(&params, cplane[i].b8, top, bot, left, right);
      if (status != SANE_STATUS_GOOD)
        return status;
    }
  memcpy (parameters, &params, sizeof (SANE_Parameters));
  scanner->params.bytes_per_line /= scanner->params.pixels_per_line;
  scanner->params.pixels_per_line = params.pixels_per_line;
  scanner->params.bytes_per_line *= params.pixels_per_line;
  scanner->params.lines = params.lines;

  return SANE_STATUS_GOOD;
}


/* ------------------------ PIE_USB_SW_FINAL_CROP ----------------------------- */
/**
 * @brief Crop the final complete RGB image
 *
 * @param[in]  scanner Pointer to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 *
 * @note
 * sanei_magic's crop routine works best on color corrected images and it has
 * the tendency to crop outside the image edges. If cropping inside the
 * edges is wanted have to estimate.
 */
static SANE_Status
pie_usb_sw_final_crop (Pie_Scanner * scanner)
{
  SANEI_IR_bufptr tmp_img;
  SANE_Parameters params;
  SANE_Status status;
  int resol = scanner->resolution / 16;
  int top, bot, left, right;

  DBG (DBG_sane_proc, "pie_usb_sw_final_crop\n");

  /* reduce image depth for edge detection */
  status =
      sane_ir_to_8bit (&scanner->params, scanner->img_buffer,
          &params, &tmp_img);
  if (status != SANE_STATUS_GOOD)
      return status;

  status = sanei_magic_findEdges(&params, tmp_img.b8,
      resol, resol, &top, &bot, &left, &right);
  if (status != SANE_STATUS_GOOD)
      return status;

  if (strcmp (scanner->val[OPT_SW_CROP].s, CROP_INNER_STR) == 0)
    {
      int width = scanner->params.pixels_per_line;
      int height = scanner->params.lines;
      int it = top;
      int ib = bot;
      int il = left;
      int ir = right;

      resol = scanner->resolution / 100;
      if (it > 2)
        it += resol;
      if (height - ib > 2)
        ib -= resol;
      if (il > 2)
        il += resol;
      if (width - ir > 2)
        ir -= resol;

      if (ib - it > 0)
        {
          top = it;
          bot = ib;
        }
      if (ir - il > 0)
        {
          left = il;
          right = ir;
        }
      DBG (DBG_info, "pie_usb_sw_final_crop: suggested cropping:\n \
          top %d, bot %d, left %d, right %d\n", top, bot, left, right);
    }

  status =
    sanei_magic_crop(&scanner->params, scanner->img_buffer.b8, top, bot, left, right);

  free (tmp_img.b8);
  return status;
}


/* -------------------------- PIE_USB_SW_POST --------------------------------- */
/**
 * @brief Postprocess a scanned R, G, B(, I) image
 *
 * @param[in] scanner Pointer to structure of opened scanner
 * @param[in] in_img Pointer to R, G, B(, I) color planes
 * @param[in] planes number planes to work on
 * @param[in] out_planes number of allowed output planes
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 *
 * @note
 * Currently postprocessing does if applicable
 * - clean the infrared plane from red spectral overlay
 * - remove dirt
 * - smoothen film grain
 * - gain correct, invert colors
 * - crop
 *
 * @note
 * After removing dirt special smoothening (all kinds of film)
 * and cropping (only slide film) routines are used.
 */
static SANE_Status
pie_usb_sw_post (Pie_Scanner * scanner,
    SANEI_IR_bufptr * in_img, int planes, int out_planes)
{
  SANEI_IR_bufptr cplane[4];    /* R, G, B, I gray scale planes */
  SANE_Parameters parameters;   /* describes the image */
  int winsize_smooth;           /* for adapting replaced pixels */
  char filename[64];
  SANE_Status status;
  int smooth, i;

  memcpy (&parameters, &scanner->params, sizeof (SANE_Parameters));
  parameters.format = SANE_FRAME_GRAY;
  parameters.bytes_per_line = parameters.pixels_per_line;
  if (parameters.depth > 8)
    parameters.bytes_per_line *= 2;
  parameters.last_frame = 0;

  DBG (DBG_info, "pie_usb_sw_post: %d ppl, %d lines, %d bits, %d planes, %d dpi\n",
       parameters.pixels_per_line, parameters.lines,
       planes, parameters.depth, scanner->resolution);

  for (i = 0; i < planes; i++)
    cplane[i] = in_img[i];

  /* dirt is rather resolution invariant, so
   * setup resolution dependent parameters
   */
  /* film grain reduction */
  smooth = scanner->val[OPT_SW_GRAIN].w;
  winsize_smooth = (scanner->resolution / 540) | 1;
  /* smoothen whole image or only replaced pixels */
  if (smooth)
    {
      winsize_smooth += 2 * (smooth - 3);       /* even */
      if (winsize_smooth < 3)
        smooth = 0;
    }
  if (winsize_smooth < 3)
    winsize_smooth = 3;
  DBG (DBG_info, "pie_usb_sw_post: winsize_smooth %d\n", winsize_smooth);

  /* RGBI post-processing if selected:
   * 1) remove spectral overlay from ired plane,
   * 2) remove dirt, smoothen if, crop if */
  if (scanner-> processing & POST_SW_IRED_MASK)
    {
      int winsize_filter;               /* primary size of filtering window */
      int size_dilate;                  /* the dirt mask */
      SANE_Byte *thresh_data;
      int static_thresh, too_thresh;    /* static thresholds */

      /* size of filter detecting dirt */
      winsize_filter = (int) (5.0 * (double) scanner->resolution / 300.0) | 1;
      if (winsize_filter < 3)
        winsize_filter = 3;
      /* dirt usually has smooth edges which also need correction */
      size_dilate = scanner->resolution / 1000 + 1;

      /* remove spectral overlay from ired plane */
      status =
        sane_ir_spectral_clean (&parameters, scanner->ln_lut, cplane[0],
                                cplane[3]);
      if (status != SANE_STATUS_GOOD)
        return status;
      if (DBG_LEVEL >= DBG_image)
        {
          snprintf (filename, 63, "/tmp/ir-spectral.pnm");
          pie_usb_write_pnm_file (filename, cplane[3].b8,
                                  parameters.depth, 1,
                                  parameters.pixels_per_line, parameters.lines);
        }
      if (cancel_requ)          /* asynchronous cancel ? */
        return SANE_STATUS_CANCELLED;

      /* remove dirt, smoothen if, crop if */
      if (scanner->processing & POST_SW_DIRT)
        {
          double *norm_histo;
          int crop[4];

          /* first detect large dirt by a static threshold */
          status =
              sanei_ir_create_norm_histogram (&parameters, cplane[3], &norm_histo);
          if (status != SANE_STATUS_GOOD)
            {
              DBG (DBG_error, "pie_usb_sw_post: no buffer\n");
              return SANE_STATUS_NO_MEM;
            }
          /* generate a "bimodal" static threshold */
          status =
            sanei_ir_threshold_yen (&parameters, norm_histo, &static_thresh);
          if (status != SANE_STATUS_GOOD)
            return status;
          /* generate traditional static threshold */
          status =
            sanei_ir_threshold_otsu (&parameters, norm_histo, &too_thresh);
          if (status != SANE_STATUS_GOOD)
            return status;
          /* choose lower one */
          if (too_thresh < static_thresh)
            static_thresh = too_thresh;
          free (norm_histo);

          /* then generate dirt mask with adaptive thresholding filter
           * and add the dirt from the static threshold */
          status =                  /* last two parameters: 10, 50 detects more, 20, 75 less */
            sanei_ir_filter_madmean (&parameters, cplane[3], &thresh_data,
                                     winsize_filter, 20, 100);
          if (status != SANE_STATUS_GOOD)
            return status;
          sanei_ir_add_threshold (&parameters, cplane[3], thresh_data,
                                  static_thresh);
          if (DBG_LEVEL >= DBG_image)
            {
              snprintf (filename, 63, "/tmp/ir-threshold.pnm");
              pie_usb_write_pnm_file (filename, thresh_data,
                                      8, 1, parameters.pixels_per_line,
                                      parameters.lines);
            }
          if (cancel_requ)          /* asynchronous cancel ? */
            return SANE_STATUS_CANCELLED;

          /* replace the dirt and smoothen film grain and crop if possible */
          if (((scanner->processing & POST_SW_CROP) != 0) &&
              (scanner->val[OPT_SW_NEGA].w == SANE_FALSE))
            {
              status =
                  sanei_ir_dilate_mean (&parameters, cplane, thresh_data,
                      500, size_dilate, winsize_smooth, smooth,
                      (strcmp (scanner->val[OPT_SW_CROP].s, CROP_INNER_STR) == 0),
                      crop);
              if (status != SANE_STATUS_GOOD)
                return status;
              status =
                  pie_usb_sw_crop_planes (&parameters, scanner, cplane,
                      crop[0], crop[1], crop[2], crop[3], 3);
              if (status != SANE_STATUS_GOOD)
                return status;
            }
          else
            {
              status =
                  sanei_ir_dilate_mean (&parameters, cplane, thresh_data, 500,
                      size_dilate, winsize_smooth, smooth, 0, NULL);
              if (status != SANE_STATUS_GOOD)
                return status;
            }
          smooth = 0;
          free (thresh_data);
        }
    }   /* scanner-> processing & POST_SW_IRED_MASK */

  /* smoothen remaining cases */
  if (smooth)
    pie_usb_smoothen_rgb (&parameters, cplane, winsize_smooth);

  status = pie_usb_sw_store (scanner, cplane, out_planes);
  if (status != SANE_STATUS_GOOD)
    return status;

  if (((scanner->processing & POST_SW_CROP) != 0) &&
      (((scanner->processing & POST_SW_DIRT) == 0) ||
          (scanner->val[OPT_SW_NEGA].w == SANE_TRUE)))
    status = pie_usb_sw_final_crop (scanner);

  return status;
}


/* ------------------------- PIE_USB_READER_INDEXED --------------------------- */
/**
 * @brief Read indexed image data from scanner
 *
 * @param[in] scanner points to structure of opened scanner
 * @param[in] fp pipe to write output
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_IO_ERROR - if an error occurred during I/O
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy, timed out
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 *
 * @note
 * Necessarily can not be read from as the scanner does
 * not sequentially send the R, G, B, I planes. Several lines may
 * have to be stored before a complete one can be assembled.
 */
static SANE_Status
pie_usb_reader_indexed (Pie_Scanner * scanner, FILE * fp)
{
  char idx_char[4] = {
    'R', 'G', 'B', 'I'
  };
  char *idx_ptr[4];
  char *iend_ptr[4];
  int idx_buf[4];
  int idx_found;
  unsigned char *src[4], *buffer;
  unsigned char *wrt_ptr, *end_ptr;
  int bytes_per_line, bytes_per_color;
  int chunk_lines, lines_todo;
  int read_lines, write_lines;
  SANE_Status status;
  int irgb;                     /* number of color planes = values per pixel */
  int irgb_out;                 /* limit output as long as SANE disregards infrared */
  SANE_Bool flag_accumulate;    /* 1 : store all data read */
  SANE_Bool flag_postprocess;   /* 0 : write immediately */
  int request_data, i, j;
  size_t chunk_size, size;

  struct timeval start_time, end_time;
  long int elapsed_time;
  SANEI_IR_bufptr color_buf[4];
  SANEI_IR_bufptr color_ptr[4];
  char filename[64];

  DBG (DBG_read,
      "pie_usb_reader_indexed reading %d lines of %d bytes/line in mode %d, processing %d\n",
      scanner->params.lines, scanner->params.bytes_per_line,
      scanner->colormode, scanner->processing);

  flag_postprocess = ((scanner->processing & POST_SW_ACCUM_MASK) != 0);
  flag_accumulate = flag_postprocess;
  if (DBG_LEVEL >= DBG_image)
    flag_accumulate = SANE_TRUE;

  bytes_per_color = scanner->bytes_per_line + 2;
  if (scanner->colormode == RGB)
    {
      irgb = 3;
      irgb_out = 3;
      bytes_per_line = scanner->params.bytes_per_line + 6;
    }
  else if (scanner->colormode == RGBI)
    {
      if (scanner->processing & POST_SW_DIRT)
        {
          irgb = 4;
          irgb_out = 3;
          bytes_per_line = scanner->params.bytes_per_line + bytes_per_color + 6;
        }
      else
        {
          irgb = 4;
#ifdef SANE_FRAME_RGBI
          irgb_out = 4;
          bytes_per_line = scanner->params.bytes_per_line + 8;
#else
          irgb_out = 3;
          bytes_per_line = scanner->params.bytes_per_line + bytes_per_color + 6;
#endif
        }
    }
  else
    {
      DBG (DBG_error,
           "pie_usb_reader_indexed: wrong colour format!\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* split the image read into reasonably sized chunks */
  chunk_lines = (scanner->params.lines + 7) / 8;
  chunk_size = chunk_lines * bytes_per_line;
  if (chunk_size > BUFFER_MAXSIZE)      /* hardware limitation ? */
    {
      chunk_lines = BUFFER_MAXSIZE / bytes_per_line;
      chunk_size = chunk_lines * bytes_per_line;
    }

  size = bytes_per_line *       /* deskewing needs a minimum */
    (scanner->filter_offset1 + (scanner->filter_offset2 + 3) * 2);
  if (chunk_size < size)
    {
      chunk_lines = (size + bytes_per_line - 1) / bytes_per_line;
      chunk_size = chunk_lines * bytes_per_line;
    }

  if (chunk_lines > scanner->params.lines)      /* not larger than image */
    {
      chunk_lines = scanner->params.lines;
      chunk_size = chunk_lines * bytes_per_line;
    }

  buffer = malloc (chunk_size * 2);
  if (!buffer)
    {
      DBG (DBG_error, "pie_usb_reader_indexed: no buffer\n");
      return SANE_STATUS_NO_MEM;
    }

  size = scanner->bytes_per_line;
  if (flag_accumulate)
    size *= scanner->params.lines;
  for (i = 0; i < irgb; i++)
    {
      color_buf[i].b8 = malloc (size);
      if (!color_buf[i].b8)
        {
          DBG (DBG_error, "pie_usb_reader_indexed: no buffers\n");
          for (j = 0; j < i; j++)
            free (color_buf[j].b8);
          free (buffer);
          return SANE_STATUS_NO_MEM;
        }
      color_ptr[i] = color_buf[i];
    }
  gettimeofday (&start_time, NULL);

  /* read one buffer in advance */
  lines_todo = chunk_lines;
  set_read_length (sread.cmd, lines_todo * irgb);
  size = lines_todo * bytes_per_line;
  do
    {
      status =
        pie_usb_scsi_wrapper (scanner->sfd, sread.cmd, sread.size,
                              buffer, &size);
    }
  while (status);
  DBG_DUMP (DBG_dump, buffer, 32);
  write_lines = scanner->params.lines;
  read_lines = write_lines - lines_todo;
  wrt_ptr = buffer + chunk_size;
  end_ptr = wrt_ptr + chunk_size;
  for (i = 0; i < irgb; i++)
    {
      idx_ptr[i] = (char *) buffer;
      iend_ptr[i] = idx_ptr[i] + chunk_size;
      idx_buf[i] = 1;
      src[i] = NULL;
    }
  request_data = 0;
  idx_found = 0;
  while (write_lines)
    {
      if (cancel_requ)
        {
          DBG (DBG_info, "pie_usb_reader_indexed: cancelled\n");
          status = SANE_STATUS_CANCELLED;
          break;
        }
      for (i = 0; i < irgb; i++)        /* find indices */
        {
          while (src[i] == NULL)
            {
              if (*idx_ptr[i] == idx_char[i])
                {
                  src[i] = (unsigned char *) idx_ptr[i] + 2;
                  idx_found++;
                }
              /* advance pointers unconditionally */
              idx_ptr[i] += bytes_per_color;
              if (idx_ptr[i] >= iend_ptr[i])
                {
                  /* check for wrap */
                  if (idx_ptr[i] >= (char *) end_ptr)
                    idx_ptr[i] = (char *) buffer;
                  /* maintain private "end of buffer" */
                  iend_ptr[i] = idx_ptr[i] + chunk_size;
                  idx_buf[i]--;
                  /* request buffer fill if necessary */
                  if (idx_buf[i] == 0)
                    {
                      request_data = 1;
                      break;
                    }
                }
            }
        }

      if (idx_found == irgb)    /* success, reorder and line(s) */
        {
          write_lines--;
/*        for (i = 0; i < irgb; i++)
            memcpy(color_ptr[i].b8, src[i], scanner->bytes_per_line);
*/
          pie_usb_correct_shading (scanner, src, color_ptr,
                                   scanner->params.pixels_per_line,
                                   scanner->params.depth, 0, irgb - 1);
          if (!flag_postprocess)
            status =
                pie_usb_reader_write (scanner, fp, color_ptr,
                                      irgb_out, 1);
          if (flag_accumulate)
            {
              if (write_lines == 0)
                {
                  if (DBG_LEVEL >= DBG_image)
                    for (i = 0; i < irgb; i++)
                      {
                        snprintf (filename, 63, "/tmp/color-%d.pnm", i);
                        status =
                          pie_usb_write_pnm_file (filename, color_buf[i].b8,
                                                  scanner->params.depth, 1,
                                                  scanner->params.pixels_per_line,
                                                  scanner->params.lines);
                        if (status != SANE_STATUS_GOOD)
                          break;
                      }
                  if (flag_postprocess)
                    status =
                        pie_usb_sw_post (scanner, color_buf, irgb, irgb_out);
                }
              else
                for (i = 0; i < irgb; i++)
                  color_ptr[i].b8 += scanner->bytes_per_line;
            }
          if (status != SANE_STATUS_GOOD)
            {
              write_lines = 0;
              request_data = 0;
            }

          for (i = 0; i < irgb; i++)    /* setup for next line */
            src[i] = NULL;
          idx_found = 0;
        }

      if (request_data)         /* read next data */
        {
          if (read_lines)
            {
              lines_todo = chunk_lines;
              if (lines_todo > read_lines)
                lines_todo = read_lines;
              set_read_length (sread.cmd, lines_todo * irgb);
              size = lines_todo * bytes_per_line;
              do
                {
                  status =
                    pie_usb_scsi_wrapper (scanner->sfd, sread.cmd,
                                          sread.size, wrt_ptr, &size);
                }
              while (status);
              DBG_DUMP (DBG_dump, wrt_ptr, 32);
              read_lines -= lines_todo;
              wrt_ptr += chunk_size;
              if (wrt_ptr >= end_ptr)
                wrt_ptr = buffer;
              for (i = 0; i < irgb; i++)
                idx_buf[i]++;
              request_data = 0;
            }
          else if (write_lines)
            {
              DBG (DBG_error,
                   "pie_usb_reader_indexed: deskew failed for %d lines\n",
                   write_lines);
              write_lines = 0;
            }
        }
    }
  gettimeofday (&end_time, NULL);
  elapsed_time = ((end_time.tv_sec * 1000000 + end_time.tv_usec)
                  - (start_time.tv_sec * 1000000 +
                     start_time.tv_usec)) / 1000;
  DBG (DBG_read,
       "pie_usb_reader_indexed finished %d bytes in %ld ms, returning %s\n",
       (scanner->params.lines - read_lines) * scanner->params.bytes_per_line,
       elapsed_time, sane_strstatus (status));

  for (i = 0; i < irgb; i++)
    free (color_buf[i].b8);
  free (buffer);
  return status;
}

#if 0
/* ----------------------- PIE_USB_READER_PROCESS_LINE ------------------------ */

/* Currently not used, and unused by the Windows programs. The scanners seem to support
   INQ_COLOR_FORMAT_LINE delivering ordered RGB data for one line */

static int
pie_usb_reader_process_line (Pie_Scanner * scanner, FILE * fp)
{
  int status;
  int lines, lines_todo, chunk_lines;
  unsigned char *buffer, *reorder;
  size_t size, chunk_size;
  DBG (DBG_read,
       "pie_usb_reader_process reading %d lines of %d bytes/line\n",
       scanner->params.lines, scanner->params.bytes_per_line);
  chunk_lines = (scanner->params.lines + 7) / 8;
  chunk_size = chunk_lines * scanner->params.bytes_per_line;
  if (chunk_size > BUFFER_MAXSIZE)
    {
      chunk_lines = BUFFER_MAXSIZE / scanner->params.bytes_per_line;
      chunk_size = chunk_lines * scanner->params.bytes_per_line;
    }
  buffer = malloc (chunk_size);
  reorder = malloc (chunk_size);
  if (!buffer || !reorder)
    {
      free (buffer);
      free (reorder);
      return SANE_STATUS_NO_MEM;
    }

  lines = scanner->params.lines;
  while (lines > 0)
    {
      lines_todo = chunk_lines;
      if (lines_todo > lines)
        lines_todo = lines;
      set_read_length (sread.cmd, lines_todo);
      size = lines_todo * scanner->params.bytes_per_line;
      do
        {
          status =
            pie_usb_scsi_wrapper (scanner->sfd, sread.cmd,
                                  sread.size, buffer, &size);
        }
      while (status);
      DBG_DUMP (DBG_dump, buffer, 64);
      if (scanner->colormode == RGB)
        {
          int offset = scanner->bytes_per_line;
          unsigned char *dest = reorder;
          unsigned char *src = buffer;
          int i, j;
          if (scanner->params.depth > 8)
            {
              for (j = lines_todo; j > 0; j--)
                {
                  for (i = scanner->params.pixels_per_line; i > 0; i--)
                    {
                      *dest++ = *src;
                      *dest++ = *(src + 1);
                      *dest++ = *(src + offset);
                      *dest++ = *(src + offset + 1);
                      *dest++ = *(src + 2 * offset);
                      *dest++ = *(src + 2 * offset + 1);
                      src++;
                      src++;
                    }
                  src += offset * 2;
                }
            }
          else
            {
              for (j = lines_todo; j > 0; j--)
                {
                  for (i = scanner->params.pixels_per_line; i > 0; i--)
                    {
                      *dest++ = *src;
                      *dest++ = *(src + offset);
                      *dest++ = *(src + 2 * offset);
                      src++;
                    }
                  src += offset * 2;
                }
            }
          fwrite (reorder, 1, size, fp);
        }
      else
        {
          fwrite (buffer, 1, size, fp);
        }
      lines -= lines_todo;
    }

  free (buffer);
  free (reorder);
  return 0;
}
#endif


/* ------------------------ PIE_USB_DO_CANCEL ------------------------------ */
/**
 * @brief Perform actions necessary to abort scan
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_CANCELLED All done
 */
static SANE_Status
pie_usb_do_cancel (Pie_Scanner * scanner, int park)
{

  DBG (DBG_sane_proc, "pie_usb_do_cancel\n");

  if (scanner->scanning)
    {
      scanner->scanning = SANE_FALSE;

      if (scanner->processing & POST_SW_ACCUM_MASK)     /* single threaded case */
        {
          cancel_requ = 1;
        }
      else                              /* threaded or forked case */
        {
          if (scanner->pipe >= 0)   /* cancel or error */
            close (scanner->pipe);

          if (scanner->reader_pid != NO_PID)
            {
#if defined USE_PTHREAD || defined HAVE_OS2_H || defined __BEOS__
              cancel_requ = 1;
#else
              sanei_thread_kill (scanner->reader_pid);
#endif
              sanei_thread_waitpid (scanner->reader_pid, 0);
              scanner->reader_pid = NO_PID;
              DBG (DBG_sane_info, "pie_usb_do_cancel: reader thread finished\n");
            }

          if (scanner->pipe >= 0)   /* cancel or error */
            {
              scanner->pipe = -1;
              pie_usb_scan (scanner, 0);
            }
        }
    }
  /* greatly improves handling when forked: watch scanner as it returns,
   * needs to be checked before another USB command is sent */
  if ((scanner->parking_pid == NO_PID) && park)
    scanner->parking_pid =
        sanei_thread_begin (pie_usb_release_scanner, (void *) scanner);

  return SANE_STATUS_CANCELLED;
}


/* ----------------------------- PIE_USB_READER_PROCESS_SIGTERM_HANDLER  -------- */
/**
 * @brief Set a variable to indicate cancel request
 */
static RETSIGTYPE
pie_usb_reader_process_sigterm_handler (__sane_unused__ int signal)
{
  cancel_requ = 1;
  return;
}


/* ------------------------ PIE_USB_READER_UNTHREADED ------------------------- */
/**
 * @brief Read and store whole image
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_IO_ERROR - if an error occurred during I/O
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy, timed out
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 * - SANE_STATUS_UNSUPPORTED - wrong color format requested
 */
static SANE_Status
pie_usb_reader_unthreaded (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_sane_proc, "pie_usb_reader_unthreaded\n");

  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    {
      cancel_requ = 0;              /* assume not canceled yet */

      status = pie_usb_reader_indexed (scanner, NULL);
      if (status != SANE_STATUS_GOOD)
        {
          pie_usb_scan (scanner, 0);
          scanner->scanning = SANE_FALSE;
        }
      else
        {
          scanner->total_bytes_stored = scanner->params.bytes_per_line *
              scanner->params.lines;
          scanner->total_bytes_read = 0;
        }
      return status;
    }
  else
    return SANE_STATUS_UNSUPPORTED;
}


/* ------------------------- PIE_USB_READER_PROCESS --------------------------- */
/**
 * @brief Setup child process / thread for reading and piping out an image
 *
 * @param[in] data Pointer to structure of opened scanner
 *
 * @ return
 * - SANE_STATUS_GOOD - success
 * - SANE_STATUS_IO_ERROR - if an error occurred during I/O
 * - SANE_STATUS_INVAL - unexpected value was read
 * - SANE_STATUS_EOF - if zero bytes have been read
 * - SANE_STATUS_DEVICE_BUSY - scanner is busy, timed out
 * - SANE_STATUS_NO_MEM - could not allocate buffer
 * - SANE_STATUS_UNSUPPORTED - color format not supported
 *
 * @note
 * This routine is quite useful if data can be delivered continuously
 * to the frontend, it is useless if the image has to be cropped.
 */
static int
pie_usb_reader_process (void *data)     /* executed as a child process */
{
  int status;
  FILE *fp;
  Pie_Scanner *scanner;
  sigset_t ignore_set;
  struct SIGACTION act;

  scanner = (Pie_Scanner *) data;

  if (sanei_thread_is_forked ())
    {

      close (scanner->pipe);

      sigfillset (&ignore_set);
      sigdelset (&ignore_set, SIGTERM);
#if defined (__APPLE__) && defined (__MACH__)
      sigdelset (&ignore_set, SIGUSR2);
#endif
      sigprocmask (SIG_SETMASK, &ignore_set, 0);

      memset (&act, 0, sizeof (act));
      sigaction (SIGTERM, &act, 0);
    }

  DBG (DBG_sane_proc, "pie_usb_reader_process started\n");

  cancel_requ = 0;              /* assume not cancelled yet */
  memset (&act, 0, sizeof (act));       /* define SIGTERM-handler */
  act.sa_handler = pie_usb_reader_process_sigterm_handler;
  sigaction (SIGTERM, &act, 0);

  fp = fdopen (scanner->reader_fds, "w");
  if (!fp)
    return SANE_STATUS_IO_ERROR;

  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    status = pie_usb_reader_indexed (scanner, fp);
  else
    status = SANE_STATUS_UNSUPPORTED;

  close (scanner->reader_fds);

  DBG (DBG_sane_info, "pie_usb_reader_process: finished reading data\n");

  return status;
}


/*
 * @@ pie_usb_sane_xy routines replace large parts of the sane_xy functions
 */

/* ------------------------------ PIE_USB_SANE_CONTROL_OPTION -------------------- */
/**
 * @brief Set or inquire the current value of an option
 *
 * Arguments and return are the same as required for sane_control_option.
 */
static SANE_Status
pie_usb_sane_control_option (SANE_Handle handle, SANE_Int option,
                             SANE_Action action, void *val, SANE_Int * info)
{
  Pie_Scanner *scanner = handle;
  SANE_Status status;
  SANE_Word cap;
  SANE_Int l_info;
  SANE_String_Const name;

  l_info = 0;
  if (info)
    *info = l_info;

  if (scanner->scanning)
    return SANE_STATUS_DEVICE_BUSY;

  if ((unsigned) option >= NUM_OPTIONS)
    return SANE_STATUS_INVAL;

  cap = scanner->opt[option].cap;
  if (!SANE_OPTION_IS_ACTIVE (cap))
    return SANE_STATUS_INVAL;

  name = scanner->opt[option].name;
  if (!name)
    name = "(no name)";

  if (action == SANE_ACTION_GET_VALUE)
    {

      DBG (DBG_sane_option, "get %s [#%d]\n", name, option);

      switch (option)
        {
          /* word options: */
        case OPT_NUM_OPTS:
        case OPT_BIT_DEPTH:
        case OPT_RESOLUTION:
        case OPT_TL_X:
        case OPT_TL_Y:
        case OPT_BR_X:
        case OPT_BR_Y:
        case OPT_PREVIEW:
        case OPT_SW_GRAIN:
        case OPT_SW_SRGB:
        case OPT_SW_NEGA:
          *(SANE_Word *) val = scanner->val[option].w;
          break;

          /* string options: */
        case OPT_MODE:
        case OPT_SPEED:
        case OPT_SW_IRED:
        case OPT_SW_CROP:
          strcpy (val, scanner->val[option].s);
        }

      return SANE_STATUS_GOOD;
    }
  else if (action == SANE_ACTION_SET_VALUE)
    {
      switch (scanner->opt[option].type)
        {
        case SANE_TYPE_INT:
          DBG (DBG_sane_option, "set %s [#%d] to %d\n", name, option,
               *(SANE_Word *) val);
          break;

        case SANE_TYPE_FIXED:
          DBG (DBG_sane_option, "set %s [#%d] to %f\n", name, option,
               SANE_UNFIX (*(SANE_Word *) val));
          break;

        case SANE_TYPE_STRING:
          DBG (DBG_sane_option, "set %s [#%d] to %s\n", name, option,
               (char *) val);
          break;

        case SANE_TYPE_BOOL:
          DBG (DBG_sane_option, "set %s [#%d] to %d\n", name, option,
               *(SANE_Word *) val);
          break;

        default:
          DBG (DBG_sane_option, "set %s [#%d]\n", name, option);
        }

      if (!SANE_OPTION_IS_SETTABLE (cap))
        return SANE_STATUS_INVAL;

      status = sanei_constrain_value (scanner->opt + option, val, &l_info);
      if (status != SANE_STATUS_GOOD)
        return status;

      switch (option)
        {
          /* (mostly) side-effect-free word options: */
        case OPT_RESOLUTION:
        case OPT_TL_X:
        case OPT_TL_Y:
        case OPT_BR_X:
        case OPT_BR_Y:
          l_info |= SANE_INFO_RELOAD_PARAMS;
          /* fall through */
        case OPT_NUM_OPTS:
        case OPT_PREVIEW:
        case OPT_BIT_DEPTH:
        case OPT_SW_GRAIN:
        case OPT_SW_SRGB:
        case OPT_SW_NEGA:
          scanner->val[option].w = *(SANE_Word *) val;
          break;
          /* (mostly) side-effect-free string options: */
        case OPT_SPEED:
        case OPT_SW_IRED:
        case OPT_SW_CROP:
          if (scanner->val[option].s)
            free (scanner->val[option].s);
          scanner->val[option].s = (SANE_Char *) strdup (val);
          break;
          /* options with side-effects: */
        case OPT_MODE:
          {
            int is_rgbi = (strcmp (val, COLOR_IR_STR) == 0);

            if (scanner->val[option].s)
              free (scanner->val[option].s);
            scanner->val[option].s = (SANE_Char *) strdup (val);
            l_info |= SANE_INFO_RELOAD_OPTIONS | SANE_INFO_RELOAD_PARAMS;

            if (is_rgbi)
              {
                scanner->opt[OPT_SW_IRED].cap &= ~SANE_CAP_INACTIVE;
              }
            else
              {
                        scanner->opt[OPT_SW_IRED].cap |= SANE_CAP_INACTIVE;
              }
            break;
          }
        }

      if (info)
        *info = l_info;
      return SANE_STATUS_GOOD;
    }                           /* else */
  return SANE_STATUS_INVAL;
}


/* ------------------------------------ PIE_USB_SANE_START ------------------------------ */
/**
 * @brief Initiates aquisition of an image
 *
 * @param[in] scanner Pointer to structure of opened scanner
 *
 * Return is as required for sane_start.
 */
static SANE_Status
pie_usb_sane_start (Pie_Scanner * scanner)
{
  const char *mode;
  const char *prired;
  SANE_Status status;
  int fds[2];

  DBG (DBG_sane_init, "pie_usb_sane_start\n");

  mode = scanner->val[OPT_MODE].s;
  prired = scanner->val[OPT_SW_IRED].s;

  status = pie_usb_is_scanner_released (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;
  pie_power_save (scanner, 0);

  scanner->colormode = RGB;
  scanner->processing = 0;
  if (scanner->val[OPT_PREVIEW].w != SANE_TRUE)
    {
      if (strcmp (mode, COLOR_IR_STR) == 0)
        {
          scanner->colormode = RGBI;
          if (strcmp (prired, IR_CLEAN_STR) == 0)
            scanner->processing |= POST_SW_DIRT;
          else
            if (strcmp (prired, IR_SPECT_STR) == 0)
              scanner->processing |= POST_SW_IRED;
        }
      if (scanner->val[OPT_SW_GRAIN].w != 0)
        scanner->processing |= POST_SW_GRAIN;
      if (strcmp (scanner->val[OPT_SW_CROP].s, THE_NONE_STR) != 0)
        scanner->processing |= POST_SW_CROP;
    }
  if ((scanner->val[OPT_SW_SRGB].w == SANE_TRUE) ||
      (scanner->val[OPT_SW_NEGA].w == SANE_TRUE))
    scanner->processing |= POST_SW_COLORS;

  /* get and set geometric values for scanning */
  scanner->resolution = SANE_UNFIX (scanner->val[OPT_RESOLUTION].w);

  scanner->scanning = SANE_TRUE;        /* definetly ! */

  status = pie_send_exposure (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_send_highlight_shadow (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_usb_set_window (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_usb_calibration_send (scanner, 0);
  if (status != SANE_STATUS_GOOD)
    return status;
  status = pie_usb_mode_select (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;

  status = pie_usb_scan (scanner, 1);
  if (status != SANE_STATUS_GOOD)
    return status;

  /* if calibration data is there !! we have to get it */
  if (scanner->cal_mode != scanner->device->model->op_mode[OPM_SKIPCAL])
    {
      status = pie_usb_calibrate (scanner);
      if (status != SANE_STATUS_GOOD)
        return status;
    }
  status = pie_usb_copy_sensors (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;

  status = pie_usb_get_params (scanner);
  if (status != SANE_STATUS_GOOD)
    return status;

  if (scanner->processing & POST_SW_ACCUM_MASK)
    {
      if (scanner->img_buffer.b8)       /* reset buffer */
        free (scanner->img_buffer.b8);
      scanner->img_buffer.b8 = NULL;
      scanner->total_bytes_stored = 0;
      scanner->total_bytes_read = 0;

      if (scanner->processing & POST_SW_CROP)
        return pie_usb_reader_unthreaded (scanner);
      else
        return SANE_STATUS_GOOD;
    }
  else
    {
      if (pipe (fds) < 0)           /* create a pipe, fds[0]=read-fd, fds[1]=write-fd */
        {
          DBG (DBG_error, "pie_usb_sane_start: could not create pipe\n");
          scanner->scanning = SANE_FALSE;
          pie_usb_scan (scanner, 0);
          return SANE_STATUS_IO_ERROR;
        }

      scanner->pipe = fds[0];
      scanner->reader_fds = fds[1];
      scanner->reader_pid =
        sanei_thread_begin (pie_usb_reader_process, (void *) scanner);

      if (scanner->reader_pid == NO_PID)
        {
          DBG (DBG_error, "pie_usb_sane_start: sanei_thread_begin failed (%s)\n",
               strerror (errno));
          return SANE_STATUS_NO_MEM;
        }

      if (sanei_thread_is_forked ())
        {
          close (scanner->reader_fds);
          scanner->reader_fds = -1;
        }

      return SANE_STATUS_GOOD;
    }
}


/* ---------------------------------- PIE_USB_SANE_READ ------------------------------ */
/**
 * @brief Read image data out of buffer or from the device
 *
 * @param[in] scanner points to structure of opened scanner
 *
 * Other arguments and return as required for sane_read.
 */
static SANE_Status
pie_usb_sane_read (Pie_Scanner * scanner, SANE_Byte * buf, SANE_Int max_len,
                   SANE_Int * len)
{
  SANE_Status status;
  ssize_t nread;

  DBG (DBG_sane_proc, "pie_usb_sane_read\n");

  if (!(scanner->scanning))     /* OOPS, not scanning */
    return SANE_STATUS_CANCELLED;

  if (scanner->processing & POST_SW_ACCUM_MASK)
    {
      if (scanner->total_bytes_stored == 0)
        {
          status = pie_usb_reader_unthreaded (scanner);
          if (status != SANE_STATUS_GOOD)
            return status;
        }

      nread = max_len;
      if(scanner->total_bytes_read + nread > scanner->total_bytes_stored)
        nread = scanner->total_bytes_stored - scanner->total_bytes_read;
      if (nread <= 0)
        return SANE_STATUS_EOF;

      DBG (DBG_sane_info, "pie_usb_sane_read: copy %ld bytes\n", (long) nread);
      memcpy (buf, scanner->img_buffer.b8 + scanner->total_bytes_read, nread);
      scanner->total_bytes_read += nread;
      *len = nread;
      return SANE_STATUS_GOOD;
    }
  else
    {
      /* threaded or forked, read from pipe */
      nread = read (scanner->pipe, buf, max_len);
      DBG (DBG_sane_info, "pie_usb_sane_read: read %ld bytes\n", (long) nread);

      if (nread < 0)
        {
          if (errno == EAGAIN)
            {
              DBG (DBG_sane_info, "pie_usb_sane_read: EAGAIN\n");
              return SANE_STATUS_GOOD;
            }
          else
            {
              pie_usb_do_cancel (scanner, SANE_TRUE);  /* error, stop scanner */
              return SANE_STATUS_IO_ERROR;
            }
        }

      *len = nread;

      if (nread == 0)               /* EOF */
        {
          /* presumably normal close of pipe, tell
             pie_usb_do_cancel to ommit some commands */
          close (scanner->pipe);
          scanner->pipe = -1;
          pie_usb_do_cancel (scanner, SANE_FALSE);
          return SANE_STATUS_EOF;
        }

      return SANE_STATUS_GOOD;
    }
}


/* ----------------------------------- PIE_USB_SANE_OPEN ------------------------------ */
/**
 * @brief Establish a connection to a scanner
 *
 * @param[in] scanner points to structure of to be opened scanner
 * @param[out] handle points to structure of opened scanner
 *
 * @return
 * - SANE_STATUS_GOOD on success
 * - SANE_STATUS_INVAL scanner could not be opened
 * - SANE_STATUS_NO_MEM no buffer for precalculated data
 */
static SANE_Status
pie_usb_sane_open (Pie_Scanner * scanner, SANE_Handle * handle)
{
  SANE_Status status;
  double di;
  int buf_size, i;

  DBG (DBG_sane_proc, "pie_usb_sane_open started\n");

  status = pie_usb_attach_open (scanner->device->sane.name, &(scanner->sfd));
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "pie_usb_sane_open: open failed\n");
      return SANE_STATUS_INVAL;
    }

  /* During initial internal calibration of the scanner we can not issue all commands to
     complete our setup so the frontend will have to wait at some point. We have to
     query the scanner for initalizing options here. */
  status = pie_usb_is_scanner_initialized (scanner->sfd);
  if (status != SANE_STATUS_GOOD)
    return status;

  if (scanner->device->cal_info_count == 0)
    {
      pie_get_halftones (scanner->device, scanner->sfd);
      pie_get_cal_info (scanner->device, scanner->sfd);
      pie_get_speeds (scanner->device);
    }
  scanner->cal_mode = scanner->device->model->op_mode[OPM_QUALITY];     /* calibrate the first time */
  scanner->reader_pid = NO_PID;
  scanner->parking_pid = NO_PID;

  scanner->cal_data = malloc (sizeof (PIE_USB_Calibration));
  if (!scanner->cal_data)
    return SANE_STATUS_NO_MEM;
  buf_size = scanner->device->cal_info[0].pixels_per_line;
  scanner->cal_data->sensors = malloc (buf_size);
  if (!scanner->cal_data->sensors)
    return SANE_STATUS_NO_MEM;
  buf_size *= 4 * sizeof (int);
  scanner->cal_data->shades = malloc (buf_size);
  if (!scanner->cal_data->shades)
    return SANE_STATUS_NO_MEM;

  memcpy (&scanner->cal_data->cal_hiqual,
          &scanner->device->model->default_hiqual,
          sizeof (PIE_USB_Calibration_Set));
  memcpy (&scanner->cal_data->cal_normal,
          &scanner->device->model->default_normal,
          sizeof (PIE_USB_Calibration_Set));
  for (i = 0; i < 3; i++)
    scanner->cal_data->brightness[i] =
      scanner->device->model->default_brightness;

  /* gamma lookup tables */
  scanner->gamma_lut8 = malloc (256 * sizeof (double));
  if (!scanner->gamma_lut8)
    return SANE_STATUS_NO_MEM;
  di = 255.0 / pow (255.0, CONST_GAMMA);
  for (i = 0; i < 256; i++)
    scanner->gamma_lut8[i] = di * pow ((double) i, CONST_GAMMA);
  scanner->gamma_lut16 = malloc (65536 * sizeof (double));
  if (!scanner->gamma_lut16)
    return SANE_STATUS_NO_MEM;
  di = 65535.0 / pow (65535.0, CONST_GAMMA);
  for (i = 0; i < 65536; i++)
    scanner->gamma_lut16[i] = di * pow ((double) i, CONST_GAMMA);

  pie_init_options (scanner);

  /* ln lookup table for infrared cleaning */
  status = sane_ir_ln_table (65536, &scanner->ln_lut);
  if (status != SANE_STATUS_GOOD)
    return status;

  /* storage for software processing of whole image */
  scanner->img_buffer.b8 = NULL;
  scanner->total_bytes_stored = 0;
  scanner->total_bytes_read = 0;

  scanner->next = first_handle; /* insert newly opened handle into list of open handles: */
  first_handle = scanner;
  *handle = scanner;

  return SANE_STATUS_GOOD;
}


/* -------------------------------- PIE_USB_SANE_CLOSE ----------------------------- */
/**
 * @brief Terminate the association between scanner and scanner structure
 *
 * @param[in] scanner points to structure of to be closed scanner
 */
static void
pie_usb_sane_close (Pie_Scanner * scanner)
{
  DBG (DBG_sane_proc, "pie_usb_sane_close started\n");

  if (scanner->scanning)        /* stop scan if still scanning */
    pie_usb_do_cancel (scanner, SANE_TRUE);

  pie_usb_is_scanner_released (scanner);    /* not yet in parking position? */

  if (scanner->sfd >= 0)
    {
      sanei_usb_reset (scanner->sfd);   /* close USB */
      sanei_usb_close (scanner->sfd);
    }

  if (scanner->cal_data)        /* free calibration data */
    {
      if (scanner->cal_data->shades)
        free (scanner->cal_data->shades);
      if (scanner->cal_data->sensors)
        free (scanner->cal_data->sensors);
      free (scanner->cal_data);
    }

  /* ln lookup table for infrared cleaning */
  free (scanner->ln_lut);
  /* gamma lookup tables */
  free (scanner->gamma_lut8);
  free (scanner->gamma_lut16);

  /* whole last image */
  if (scanner->img_buffer.b8)
    free (scanner->img_buffer.b8);

  return;
}


/*
 * @@ Original SCSI functions: Some of them call the pie_usb code
 *    using the (*scanner->device->scsi_cmd) function pointer.
 */

/*------------------------- PIE POWER SAVE -----------------------------*/

static SANE_Status
pie_power_save (Pie_Scanner * scanner, int time)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "pie_power_save: %d min\n", time);

  size = 6;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, SET_POWER_SAVE_CONTROL);
  set_data_length (data, size - 4);
  data[4] = time & 0x7f;

  status =
    (*scanner->device->scsi_cmd) (scanner->sfd, buffer, swrite.size + size,
				  NULL, NULL);
  if (status)
    {
      DBG (DBG_error, "pie_power_save: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}


/*------------------------- PIE SEND EXPOSURE ONE -----------------------------*/


static SANE_Status
pie_send_exposure_one (Pie_Scanner * scanner, int filter, int value)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "pie_send_exposure_one\n");

  size = 8;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, SET_EXP_TIME);
  set_data_length (data, size - 4);

  data[4] = filter;

  set_data (data, 6, (int) value, 2);

  status =
    (*scanner->device->scsi_cmd) (scanner->sfd, buffer, swrite.size + size,
				  NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
	   "pie_send_exposure_one: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}

/*------------------------- PIE SEND EXPOSURE -----------------------------*/

static SANE_Status
pie_send_exposure (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_proc, "pie_send_exposure\n");

  status = pie_send_exposure_one (scanner, FILTER_RED, 100);
  if (status)
    return status;

  status = pie_send_exposure_one (scanner, FILTER_GREEN, 100);
  if (status)
    return status;

  status = pie_send_exposure_one (scanner, FILTER_BLUE, 100);
  if (status)
    return status;

  return SANE_STATUS_GOOD;
}


/*------------------------- PIE SEND HIGHLIGHT/SHADOW ONE -----------------------------*/

static SANE_Status
pie_send_highlight_shadow_one (Pie_Scanner * scanner, int filter,
			       int highlight, int shadow)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "pie_send_highlight_shadow_one\n");

  size = 8;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, SET_EXP_TIME);
  set_data_length (data, size - 4);

  data[4] = filter;

  data[6] = highlight;
  data[7] = shadow;

  status =
    (*scanner->device->scsi_cmd) (scanner->sfd, buffer, swrite.size + size,
				  NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
	   "pie_send_highlight_shadow_one: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}

/*------------------------- PIE SEND HIGHLIGHT/SHADOW -----------------------------*/

static SANE_Status
pie_send_highlight_shadow (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_proc, "pie_send_highlight_shadow\n");

  status = pie_send_highlight_shadow_one (scanner, FILTER_RED, 100, 0);
  if (status)
    return status;

  status = pie_send_highlight_shadow_one (scanner, FILTER_GREEN, 100, 0);
  if (status)
    return status;

  status = pie_send_highlight_shadow_one (scanner, FILTER_BLUE, 100, 0);
  if (status)
    return status;

  return SANE_STATUS_GOOD;
}

/*------------------------- PIE PERFORM CAL ----------------------------*/

static SANE_Status
pie_perform_cal (Pie_Scanner * scanner, int cal_index)
{
  long *red_result;
  long *green_result;
  long *blue_result;
  long *neutral_result;
  long *result = NULL;
  int rcv_length, send_length;
  int rcv_lines, rcv_bits, send_bits;
  int pixels_per_line;
  int i;
  unsigned char *rcv_buffer, *rcv_ptr;
  unsigned char *send_buffer, *send_ptr;
  size_t size;
  int fullscale;
  int cal_limit;
  int k;
  int filter;
  SANE_Status status;

  DBG (DBG_proc, "pie_perform_cal\n");

  pixels_per_line = scanner->device->cal_info[cal_index].pixels_per_line;
  rcv_length = pixels_per_line;
  send_length = pixels_per_line;

  rcv_bits = scanner->device->cal_info[cal_index].receive_bits;
  if (rcv_bits > 8)
    rcv_length *= 2;		/* 2 bytes / sample */

  send_bits = scanner->device->cal_info[cal_index].send_bits;
  if (send_bits > 8)
    send_length *= 2;		/* 2 bytes / sample */

  rcv_lines = scanner->device->cal_info[cal_index].num_lines;

  send_length += 2;		/* space for filter at start */

  if (scanner->colormode == RGB)
    {
      rcv_lines *= 3;
      send_length *= 3;
      rcv_length += 2;		/* 2 bytes for index at front of data (only in RGB??) */
    }

  send_length += 4;		/* space for header at start of data */

  /* alllocate buffers for the receive data, the result buffers, and for the send data */
  rcv_buffer = (unsigned char *) malloc (rcv_length);

  red_result = (long *) calloc (pixels_per_line, sizeof (long));
  green_result = (long *) calloc (pixels_per_line, sizeof (long));
  blue_result = (long *) calloc (pixels_per_line, sizeof (long));
  neutral_result = (long *) calloc (pixels_per_line, sizeof (long));

  if (!rcv_buffer || !red_result || !green_result || !blue_result
      || !neutral_result)
    {
      /* at least one malloc failed, so free all buffers (free accepts NULL) */
      free (rcv_buffer);
      free (red_result);
      free (green_result);
      free (blue_result);
      free (neutral_result);
      return SANE_STATUS_NO_MEM;
    }

  /* read the cal data a line at a time, and accumulate into the result arrays */
  while (rcv_lines--)
    {
      /* TUR */
      status = pie_wait_scanner (scanner);
      if (status)
	{
	  free (rcv_buffer);
	  free (red_result);
	  free (green_result);
	  free (blue_result);
	  free (neutral_result);
	  return status;
	}

      set_read_length (sread.cmd, 1);
      size = rcv_length;

      DBG (DBG_info, "pie_perform_cal: reading 1 line (%lu bytes)\n",
	   (u_long) size);

      status =
	sanei_scsi_cmd (scanner->sfd, sread.cmd, sread.size, rcv_buffer,
			&size);

      if (status)
	{
	  DBG (DBG_error,
	       "pie_perform_cal: read command returned status %s\n",
	       sane_strstatus (status));
	  free (rcv_buffer);
	  free (red_result);
	  free (green_result);
	  free (blue_result);
	  free (neutral_result);
	  return status;
	}

      DBG_DUMP (DBG_dump, rcv_buffer, 32);

      /* which result buffer does this line belong to? */
      if (scanner->colormode == RGB)
	{
	  if (*rcv_buffer == 'R')
	    result = red_result;
	  else if (*rcv_buffer == 'G')
	    result = green_result;
	  else if (*rcv_buffer == 'B')
	    result = blue_result;
	  else if (*rcv_buffer == 'N')
	    result = neutral_result;
	  else
	    {
	      DBG (DBG_error, "pie_perform_cal: invalid index byte (%02x)\n",
		   *rcv_buffer);
	      DBG_DUMP (DBG_error, rcv_buffer, 32);
	      free (rcv_buffer);
	      free (red_result);
	      free (green_result);
	      free (blue_result);
	      free (neutral_result);
	      return SANE_STATUS_INVAL;
	    }
	  rcv_ptr = rcv_buffer + 2;
	}
      else
	{
	  /* monochrome - no bytes indicating filter here */
	  result = neutral_result;
	  rcv_ptr = rcv_buffer;
	}

      /* now add the values in this line to the result array */
      for (i = 0; i < pixels_per_line; i++)
	{
	  result[i] += *rcv_ptr++;
	  if (rcv_bits > 8)
	    {
	      result[i] += (*rcv_ptr++) << 8;
	    }
	}
    }

  /* got all the cal data, now process it ready to send back */
  free (rcv_buffer);
  send_buffer = (unsigned char *) malloc (send_length + swrite.size);

  if (!send_buffer)
    {
      free (red_result);
      free (green_result);
      free (blue_result);
      free (neutral_result);
      return SANE_STATUS_NO_MEM;
    }

  rcv_lines = scanner->device->cal_info[cal_index].num_lines;
  fullscale = (1 << rcv_bits) - 1;
  cal_limit = fullscale / (1 << scanner->device->inquiry_cal_eqn);
  k = (1 << scanner->device->inquiry_cal_eqn) - 1;

  /* set up scsi command and data */
  size = send_length;

  memcpy (send_buffer, swrite.cmd, swrite.size);
  set_write_length (send_buffer, size);

  set_command (send_buffer + swrite.size, SEND_CAL_DATA);
  set_data_length (send_buffer + swrite.size, size - 4);

  send_ptr = send_buffer + swrite.size + 4;

  for (filter = FILTER_NEUTRAL; filter <= FILTER_BLUE; filter <<= 1)
    {

      /* only send data for filter we expect to send */
      if (!(filter & scanner->cal_filter))
	continue;

      set_data (send_ptr, 0, filter, 2);
      send_ptr += 2;

      if (scanner->colormode == RGB)
	{
	  switch (filter)
	    {
	    case FILTER_RED:
	      result = red_result;
	      break;

	    case FILTER_GREEN:
	      result = green_result;
	      break;

	    case FILTER_BLUE:
	      result = blue_result;
	      break;

	    case FILTER_NEUTRAL:
	      result = neutral_result;
	      break;
	    }
	}
      else
	result = neutral_result;

      /* for each pixel */
      for (i = 0; i < pixels_per_line; i++)
	{
	  long x;

	  /* make average */
	  x = result[i] / rcv_lines;

	  /* ensure not overflowed */
	  if (x > fullscale)
	    x = fullscale;

	  /* process according to required calibration equation */
	  if (scanner->device->inquiry_cal_eqn)
	    {
	      if (x <= cal_limit)
		x = fullscale;
	      else
		x = ((fullscale - x) * fullscale) / (x * k);
	    }

	  if (rcv_bits > send_bits)
	    x >>= (rcv_bits - send_bits);
	  else if (send_bits > rcv_bits)
	    x <<= (send_bits - rcv_bits);

	  /* put result into send buffer */
	  *send_ptr++ = x;
	  if (send_bits > 8)
	    *send_ptr++ = x >> 8;
	}
    }

  /* now send the data back to scanner */

  /* TUR */
  status = pie_wait_scanner (scanner);
  if (status)
    {
      free (red_result);
      free (green_result);
      free (blue_result);
      free (neutral_result);
      free (send_buffer);
      return status;
    }

  DBG (DBG_info, "pie_perform_cal: sending cal data (%lu bytes)\n",
       (u_long) size);
  DBG_DUMP (DBG_dump, send_buffer, 64);

  status =
    sanei_scsi_cmd (scanner->sfd, send_buffer, swrite.size + size, NULL,
		    NULL);
  if (status)
    {
      DBG (DBG_error, "pie_perform_cal: write command returned status %s\n",
	   sane_strstatus (status));
      free (red_result);
      free (green_result);
      free (blue_result);
      free (neutral_result);
      free (send_buffer);
      return status;
    }

  free (red_result);
  free (green_result);
  free (blue_result);
  free (neutral_result);
  free (send_buffer);

  return SANE_STATUS_GOOD;
}

/*------------------------- PIE DO CAL -----------------------------*/

static SANE_Status
pie_do_cal (Pie_Scanner * scanner)
{
  SANE_Status status;
  int cal_index;

  DBG (DBG_proc, "pie_do_cal\n");

  if (scanner->device->inquiry_scan_capability & INQ_CAP_EXT_CAL)
    {
      for (cal_index = 0; cal_index < scanner->device->cal_info_count;
	   cal_index++)
	if (scanner->device->cal_info[cal_index].cal_type ==
	    scanner->cal_mode)
	  {
	    status = pie_perform_cal (scanner, cal_index);
	    if (status != SANE_STATUS_GOOD)
	      return status;
	  }
    }

  return SANE_STATUS_GOOD;
}

/*------------------------- PIE DWNLD GAMMA ONE -----------------------------*/

static SANE_Status
pie_dwnld_gamma_one (Pie_Scanner * scanner, int filter, SANE_Int * table)
{
  unsigned char *buffer;
  size_t size;
  SANE_Status status;
  unsigned char *data;
  int i;

  DBG (DBG_proc, "pie_dwnld_gamma_one\n");

  /* TUR */
  status = pie_wait_scanner (scanner);
  if (status)
    {
      return status;
    }

  if (scanner->device->inquiry_gamma_bits > 8)
    size = scanner->gamma_length * 2 + 6;
  else
    size = scanner->gamma_length + 6;

  buffer = malloc (size + swrite.size);
  if (!buffer)
    return SANE_STATUS_NO_MEM;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, DWNLD_GAMMA_TABLE);
  set_data_length (data, size - 4);

  data[4] = filter;

  for (i = 0; i < scanner->gamma_length; i++)
    {
      if (scanner->device->inquiry_gamma_bits > 8)
	{
	  set_data (data, 6 + 2 * i, table ? table[i] : i, 2);
	}
      else
	{
	  set_data (data, 6 + i, table ? table[i] : i, 1);
	}
    }

  DBG_DUMP (DBG_dump, data, 128);

  status =
    sanei_scsi_cmd (scanner->sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
	   "pie_dwnld_gamma_one: write command returned status %s\n",
	   sane_strstatus (status));
    }

  free (buffer);

  return status;
}

/*------------------------- PIE DWNLD GAMMA -----------------------------*/

static SANE_Status
pie_dwnld_gamma (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_proc, "pie_dwnld_gamma\n");

  if (scanner->colormode == RGB)
    {
      status =
	pie_dwnld_gamma_one (scanner, FILTER_RED, scanner->gamma_table[1]);
      if (status)
	return status;


      status =
	pie_dwnld_gamma_one (scanner, FILTER_GREEN, scanner->gamma_table[2]);
      if (status)
	return status;

      status =
	pie_dwnld_gamma_one (scanner, FILTER_BLUE, scanner->gamma_table[3]);
      if (status)
	return status;
    }
  else
    {
      SANE_Int *table;

      /* if lineart or half tone, force gamma to be one to one by passing NULL */
      if (scanner->colormode == GRAYSCALE)
	table = scanner->gamma_table[0];
      else
	table = NULL;

      status = pie_dwnld_gamma_one (scanner, FILTER_GREEN, table);
      if (status)
	return status;
    }

  usleep (DOWNLOAD_GAMMA_WAIT_TIME);

  return SANE_STATUS_GOOD;
}

/*------------------------- PIE SET WINDOW -----------------------------*/

static SANE_Status
pie_set_window (Pie_Scanner * scanner)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;
  double x, dpmm;

  DBG (DBG_proc, "pie_set_window\n");

  size = 14;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, SET_SCAN_FRAME);
  set_data_length (data, size - 4);

  data[4] = 0x80;
  if (scanner->colormode == HALFTONE)
    data[4] |= 0x40;

  dpmm = (double) scanner->device->inquiry_pixel_resolution / MM_PER_INCH;

  x = SANE_UNFIX (scanner->val[OPT_TL_X].w) * dpmm;
  set_data (data, 6, (int) x, 2);
  DBG (DBG_info, "TL_X: %d\n", (int) x);

  x = SANE_UNFIX (scanner->val[OPT_TL_Y].w) * dpmm;
  set_data (data, 8, (int) x, 2);
  DBG (DBG_info, "TL_Y: %d\n", (int) x);

  x = SANE_UNFIX (scanner->val[OPT_BR_X].w) * dpmm;
  set_data (data, 10, (int) x, 2);
  DBG (DBG_info, "BR_X: %d\n", (int) x);

  x = SANE_UNFIX (scanner->val[OPT_BR_Y].w) * dpmm;
  set_data (data, 12, (int) x, 2);
  DBG (DBG_info, "BR_Y: %d\n", (int) x);

  status =
    sanei_scsi_cmd (scanner->sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error, "pie_set_window: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}


/*------------------------- PIE MODE SELECT -----------------------------*/

static SANE_Status
pie_mode_select (Pie_Scanner * scanner)
{

  SANE_Status status;
  unsigned char buffer[128];
  size_t size;
  unsigned char *data;
  int i;

  DBG (DBG_proc, "pie_mode_select\n");

  size = 14;

  set_mode_length (smode.cmd, size);

  memcpy (buffer, smode.cmd, smode.size);

  data = buffer + smode.size;
  memset (data, 0, size);

  /* size of data */
  data[1] = size - 2;

  /* set resolution required */
  set_data (data, 2, scanner->resolution, 2);

  /* set color filter and color depth */
  switch (scanner->colormode)
    {
    case RGB:
      if (scanner->device->inquiry_filters & INQ_ONE_PASS_COLOR)
	{
	  data[4] = INQ_ONE_PASS_COLOR;
	  scanner->cal_filter = FILTER_RED | FILTER_GREEN | FILTER_BLUE;
	}
      else
	{
	  DBG (DBG_error,
	       "pie_mode_select: support for multipass color not yet implemented\n");
	  return SANE_STATUS_UNSUPPORTED;
	}
      data[5] = INQ_COLOR_DEPTH_8;
      break;

    case GRAYSCALE:
    case LINEART:
    case HALFTONE:
      /* choose which filter to use for monochrome mode */
      if (scanner->device->inquiry_filters & INQ_FILTER_NEUTRAL)
	{
	  data[4] = FILTER_NEUTRAL;
	  scanner->cal_filter = FILTER_NEUTRAL;
	}
      else if (scanner->device->inquiry_filters & INQ_FILTER_GREEN)
	{
	  data[4] = FILTER_GREEN;
	  scanner->cal_filter = FILTER_GREEN;
	}
      else if (scanner->device->inquiry_filters & INQ_FILTER_RED)
	{
	  data[4] = FILTER_RED;
	  scanner->cal_filter = FILTER_RED;
	}
      else if (scanner->device->inquiry_filters & INQ_FILTER_BLUE)
	{
	  data[4] = FILTER_BLUE;
	  scanner->cal_filter = FILTER_BLUE;
	}
      else
	{
	  DBG (DBG_error,
	       "pie_mode_select: scanner doesn't appear to support monochrome\n");
	  return SANE_STATUS_UNSUPPORTED;
	}

      if (scanner->colormode == GRAYSCALE)
	data[5] = INQ_COLOR_DEPTH_8;
      else
	data[5] = INQ_COLOR_DEPTH_1;
      break;
    }

  /* choose color packing method */
  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_LINE)
    data[6] = INQ_COLOR_FORMAT_LINE;
  else if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    data[6] = INQ_COLOR_FORMAT_INDEX;
  else
    {
      DBG (DBG_error,
	   "pie_mode_select: support for pixel packing not yet implemented\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* choose data format */
  if (scanner->device->inquiry_image_format & INQ_IMG_FMT_INTEL)
    data[8] = INQ_IMG_FMT_INTEL;
  else
    {
      DBG (DBG_error,
	   "pie_mode_select: support for Motorola format not yet implemented\n");
      return SANE_STATUS_UNSUPPORTED;
    }

  /* set required speed */
  i = 0;
  while (scanner->device->speed_list[i] != NULL)
    {
      if (strcmp (scanner->device->speed_list[i], scanner->val[OPT_SPEED].s)
	  == 0)
	break;
      i++;
    }

  if (scanner->device->speed_list[i] == NULL)
    data[9] = 0;
  else
    data[9] = i;

  scanner->cal_mode = CAL_MODE_FLATBED;

  /* if preview supported, ask for preview, limit resolution to max for fast preview */
  if (scanner->val[OPT_PREVIEW].w
      && (scanner->device->inquiry_scan_capability & INQ_CAP_FAST_PREVIEW))
    {
      DBG (DBG_info, "pie_mode_select: setting preview\n");
      scanner->cal_mode |= CAL_MODE_PREVIEW;
      data[9] |= INQ_CAP_FAST_PREVIEW;
      data[9] &= ~INQ_CAP_SPEEDS;
      if (scanner->resolution > scanner->device->inquiry_fast_preview_res)
	set_data (data, 2, scanner->device->inquiry_fast_preview_res, 2);
    }


  /* set required halftone pattern */
  i = 0;
  while (scanner->device->halftone_list[i] != NULL)
    {
      if (strcmp
	  (scanner->device->halftone_list[i],
	   scanner->val[OPT_HALFTONE_PATTERN].s) == 0)
	break;
      i++;
    }

  if (scanner->device->halftone_list[i] == NULL)
    data[12] = 0;		/* halftone pattern */
  else
    data[12] = i;

  data[13] = SANE_UNFIX (scanner->val[OPT_THRESHOLD].w) * 255 / 100;	/* lineart threshold */

  DBG (DBG_info, "pie_mode_select: speed %02x\n", data[9]);
  DBG (DBG_info, "pie_mode_select: halftone %d\n", data[12]);
  DBG (DBG_info, "pie_mode_select: threshold %02x\n", data[13]);

  status =
    sanei_scsi_cmd (scanner->sfd, buffer, smode.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error, "pie_mode_select: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}


/*------------------------- PIE SCAN -----------------------------*/

static SANE_Status
pie_scan (Pie_Scanner * scanner, int start)
{
  SANE_Status status;

  DBG (DBG_proc, "pie_scan\n");

  /* TUR */
  status = pie_wait_scanner (scanner);
  if (status)
    {
      return status;
    }

  set_scan_cmd (scan.cmd, start);

  do
    {
      status = sanei_scsi_cmd (scanner->sfd, scan.cmd, scan.size, NULL, NULL);
      if (status)
	{
	  DBG (DBG_error, "pie_scan: write command returned status %s\n",
	       sane_strstatus (status));
	  usleep (SCAN_WARMUP_WAIT_TIME);
	}
    }
  while (start && status);

  usleep (SCAN_WAIT_TIME);

  return status;
}


/* --------------------------------------- PIE WAIT SCANNER -------------------------- */


static SANE_Status
pie_wait_scanner (Pie_Scanner * scanner)
{
  SANE_Status status;
  int cnt = 0;

  DBG (DBG_proc, "wait_scanner\n");

  do
    {
      if (cnt > 100)		/* maximal 100 * 0.5 sec = 50 sec */
	{
	  DBG (DBG_warning, "scanner does not get ready\n");
	  return -1;
	}
      /* test unit ready */
      status =
	sanei_scsi_cmd (scanner->sfd, test_unit_ready.cmd,
			test_unit_ready.size, NULL, NULL);

      cnt++;

      if (status)
	{
	  if (cnt == 1)
	    {
	      DBG (DBG_info2, "scanner reports %s, waiting ...\n",
		   sane_strstatus (status));
	    }

	  usleep (TUR_WAIT_TIME);
	}
    }
  while (status != SANE_STATUS_GOOD);

  DBG (DBG_info, "scanner ready\n");


  return status;
}


/* -------------------------------------- PIE GET PARAMS -------------------------- */


static SANE_Status
pie_get_params (Pie_Scanner * scanner)
{
  SANE_Status status;
  size_t size;
  unsigned char buffer[128];

  DBG (DBG_proc, "pie_get_params\n");

  status = pie_wait_scanner (scanner);
  if (status)
    return status;

  if (scanner->device->inquiry_image_format & INQ_IMG_FMT_OKLINE)
    size = 16;
  else

    size = 14;

  set_param_length (param.cmd, size);

  status =
    sanei_scsi_cmd (scanner->sfd, param.cmd, param.size, buffer, &size);

  if (status)
    {
      DBG (DBG_error, "pie_get_params: command returned status %s\n",
	   sane_strstatus (status));
    }
  else
    {
      DBG (DBG_info, "Scan Width:  %d\n", get_param_scan_width (buffer));
      DBG (DBG_info, "Scan Lines:  %d\n", get_param_scan_lines (buffer));
      DBG (DBG_info, "Scan bytes:  %d\n", get_param_scan_bytes (buffer));

      DBG (DBG_info, "Offset 1:    %d\n",
	   get_param_scan_filter_offset1 (buffer));
      DBG (DBG_info, "Offset 2:    %d\n",
	   get_param_scan_filter_offset2 (buffer));
      DBG (DBG_info, "Scan period: %d\n", get_param_scan_period (buffer));
      DBG (DBG_info, "Xfer rate:   %d\n", get_param_scsi_xfer_rate (buffer));
      if (scanner->device->inquiry_image_format & INQ_IMG_FMT_OKLINE)
	DBG (DBG_info, "Avail lines: %d\n",
	     get_param_scan_available_lines (buffer));

      scanner->filter_offset1 = get_param_scan_filter_offset1 (buffer);
      scanner->filter_offset2 = get_param_scan_filter_offset2 (buffer);
      scanner->bytes_per_line = get_param_scan_bytes (buffer);

      scanner->params.pixels_per_line = get_param_scan_width (buffer);
      scanner->params.lines = get_param_scan_lines (buffer);

      switch (scanner->colormode)
	{
	case RGB:
	  scanner->params.format = SANE_FRAME_RGB;
	  scanner->params.depth = 8;
	  scanner->params.bytes_per_line = 3 * get_param_scan_bytes (buffer);
	  break;

	case GRAYSCALE:
	  scanner->params.format = SANE_FRAME_GRAY;
	  scanner->params.depth = 8;
	  scanner->params.bytes_per_line = get_param_scan_bytes (buffer);
	  break;

	case HALFTONE:
	case LINEART:
	  scanner->params.format = SANE_FRAME_GRAY;
	  scanner->params.depth = 1;
	  scanner->params.bytes_per_line = get_param_scan_bytes (buffer);
	  break;
	}

      scanner->params.last_frame = 0;
    }

  return status;
}


/* -------------------------------------- PIE GRAB SCANNER -------------------------- */


static SANE_Status
pie_grab_scanner (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_proc, "grab_scanner\n");


  status = pie_wait_scanner (scanner);
  if (status)
    return status;

  status =
    sanei_scsi_cmd (scanner->sfd, reserve_unit.cmd, reserve_unit.size, NULL,
		    NULL);


  if (status)
    {
      DBG (DBG_error, "pie_grab_scanner: command returned status %s\n",
	   sane_strstatus (status));
    }
  else
    {
      DBG (DBG_info, "scanner reserved\n");
    }

  return status;
}


/* ------------------------------------ PIE GIVE SCANNER -------------------------- */


static SANE_Status
pie_give_scanner (Pie_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_info2, "trying to release scanner ...\n");

  status =
    sanei_scsi_cmd (scanner->sfd, release_unit.cmd, release_unit.size, NULL,
		    NULL);
  if (status)
    {
      DBG (DBG_error, "pie_give_scanner: command returned status %s\n",
	   sane_strstatus (status));
    }
  else
    {
      DBG (DBG_info, "scanner released\n");
    }
  return status;
}


/* ------------------- PIE READER PROCESS INDEXED ------------------- */

static int
pie_reader_process_indexed (Pie_Scanner * scanner, FILE * fp)
{
  int status;
  int lines;
  unsigned char *buffer, *reorder = NULL;
  unsigned char *red_buffer = NULL, *green_buffer = NULL;
  unsigned char *red_in = NULL, *red_out = NULL;
  unsigned char *green_in = NULL, *green_out = NULL;
  int red_size = 0, green_size = 0;
  int bytes_per_line;
  int red_count = 0, green_count = 0;

  size_t size;

  DBG (DBG_read, "reading %d lines of %d bytes/line (indexed)\n",
       scanner->params.lines, scanner->params.bytes_per_line);

  lines = scanner->params.lines;
  bytes_per_line = scanner->bytes_per_line;

  /* allocate receive buffer */
  buffer = malloc (bytes_per_line + 2);
  if (!buffer)
    {
      return SANE_STATUS_NO_MEM;
    }

  /* allocate deskew buffers for RGB mode */
  if (scanner->colormode == RGB)
    {
      lines *= 3;

      red_size = bytes_per_line * (scanner->filter_offset1 +
				   scanner->filter_offset2 + 2);
      green_size = bytes_per_line * (scanner->filter_offset2 + 2);

      DBG (DBG_info2,
	   "pie_reader_process_indexed: alloc %d lines (%d bytes) for red buffer\n",
	   red_size / bytes_per_line, red_size);
      DBG (DBG_info2,
	   "pie_reader_process_indexed: alloc %d lines (%d bytes) for green buffer\n",
	   green_size / bytes_per_line, green_size);

      reorder = malloc (scanner->params.bytes_per_line);
      red_buffer = malloc (red_size);
      green_buffer = malloc (green_size);

      if (!reorder || !red_buffer || !green_buffer)
	{
	  free (buffer);
	  free (reorder);
	  free (red_buffer);
	  free (green_buffer);
	  return SANE_STATUS_NO_MEM;
	}

      red_in = red_out = red_buffer;
      green_in = green_out = green_buffer;
    }

  while (lines--)
    {
      set_read_length (sread.cmd, 1);
      size = bytes_per_line + 2;

      do
	{
	  status =
	    sanei_scsi_cmd (scanner->sfd, sread.cmd, sread.size, buffer,
			    &size);
	}
      while (status);

      DBG_DUMP (DBG_dump, buffer, 64);

      if (scanner->colormode == RGB)
	{
	  /* we're assuming that we get red before green before blue here */
	  switch (*buffer)
	    {
	    case 'R':
	      /* copy to red buffer */
	      memcpy (red_in, buffer + 2, bytes_per_line);

	      /* advance in pointer, and check for wrap */
	      red_in += bytes_per_line;
	      if (red_in >= (red_buffer + red_size))
		red_in = red_buffer;

	      /* increment red line count */
	      red_count++;
	      DBG (DBG_info2,
		   "pie_reader_process_indexed: got a red line (%d)\n",
		   red_count);
	      break;

	    case 'G':
	      /* copy to green buffer */
	      memcpy (green_in, buffer + 2, bytes_per_line);

	      /* advance in pointer, and check for wrap */
	      green_in += bytes_per_line;
	      if (green_in >= (green_buffer + green_size))
		green_in = green_buffer;

	      /* increment green line count */
	      green_count++;
	      DBG (DBG_info2,
		   "pie_reader_process_indexed: got a green line (%d)\n",
		   green_count);
	      break;

	    case 'B':
	      /* check we actually have red and green data available */
	      if (!red_count || !green_count)
		{
		  DBG (DBG_error,
		       "pie_reader_process_indexed: deskew buffer empty (%d %d)\n",
		       red_count, green_count);
		  return SANE_STATUS_INVAL;
		}
	      red_count--;
	      green_count--;

	      DBG (DBG_info2,
		   "pie_reader_process_indexed: got a blue line\n");

	      {
		int i;
		unsigned char *red, *green, *blue, *dest;

		/* now pack the pixels lines into RGB format */
		dest = reorder;
		red = red_out;
		green = green_out;
		blue = buffer + 2;

		for (i = bytes_per_line; i > 0; i--)
		  {
		    *dest++ = *red++;
		    *dest++ = *green++;
		    *dest++ = *blue++;
		  }
		fwrite (reorder, 1, scanner->params.bytes_per_line, fp);

		/* advance out pointers, and check for wrap */
		red_out += bytes_per_line;
		if (red_out >= (red_buffer + red_size))
		  red_out = red_buffer;
		green_out += bytes_per_line;
		if (green_out >= (green_buffer + green_size))
		  green_out = green_buffer;
	      }
	      break;

	    default:
	      DBG (DBG_error,
		   "pie_reader_process_indexed: bad filter index\n");
	    }
	}
      else
	{
	  DBG (DBG_info2,
	       "pie_reader_process_indexed: got a line (%lu bytes)\n",
	       (u_long) size);

	  /* just send the data on, assume filter bytes not present as per calibration case */
	  fwrite (buffer, 1, scanner->params.bytes_per_line, fp);
	}
    }

  free (buffer);
  free (reorder);
  free (red_buffer);
  free (green_buffer);
  return 0;
}

/* ----------------------------- PIE_READER_PROCESS_FMTLINE -------------------- */

static int
pie_reader_process_fmtline (Pie_Scanner * scanner, FILE * fp)
{
  int status;
  int lines;
  unsigned char *buffer, *reorder;
  size_t size;

  DBG (DBG_read, "reading %d lines of %d bytes/line\n", scanner->params.lines,
       scanner->params.bytes_per_line);

  buffer = malloc (scanner->params.bytes_per_line);
  reorder = malloc (scanner->params.bytes_per_line);
  if (!buffer || !reorder)
    {
      free (buffer);
      free (reorder);
      return SANE_STATUS_NO_MEM;
    }

  lines = scanner->params.lines;

  while (lines--)
    {
      set_read_length (sread.cmd, 1);
      size = scanner->params.bytes_per_line;

      do
	{
	  status =
	    sanei_scsi_cmd (scanner->sfd, sread.cmd, sread.size, buffer,
			    &size);
	}
      while (status);

      DBG_DUMP (DBG_dump, buffer, 64);

      if (scanner->colormode == RGB)
	{
	  int i;
	  unsigned char *src, *dest;
	  int offset;

	  dest = reorder;
	  src = buffer;
	  offset = scanner->params.pixels_per_line;

	  for (i = scanner->params.pixels_per_line; i > 0; i--)
	    {
	      *dest++ = *src;
	      *dest++ = *(src + offset);
	      *dest++ = *(src + 2 * offset);
	      src++;
	    }
	  fwrite (reorder, 1, scanner->params.bytes_per_line, fp);
	}
      else
	{
	  fwrite (buffer, 1, scanner->params.bytes_per_line, fp);
	}

      fflush (fp);
    }

  free (buffer);
  free (reorder);

  return 0;
}



/* ------------------------------- PIE_READER_PROCESS_SIGTERM_HANDLER  ---------- */


static RETSIGTYPE
pie_reader_process_sigterm_handler (int signal)
{
  DBG (DBG_sane_info, "pie_reader_process: terminated by signal %d\n", signal);

#ifdef HAVE_SANEI_SCSI_OPEN_EXTENDED
  sanei_scsi_req_flush_all ();	/* flush SCSI queue */
#else
  sanei_scsi_req_flush_all ();	/* flush SCSI queue */
#endif

  _exit (SANE_STATUS_GOOD);
}



/* ---------------------------- PIE_READER_PROCESS --------------------------- */


static int
pie_reader_process (void *data)	/* executed as a child process */
{
  int status;
  FILE *fp;
  Pie_Scanner *scanner;
  sigset_t ignore_set;
  struct SIGACTION act;

  scanner = (Pie_Scanner *) data;
  
  if (sanei_thread_is_forked ())
    {

      close (scanner->pipe);

      sigfillset (&ignore_set);
      sigdelset (&ignore_set, SIGTERM);
#if defined (__APPLE__) && defined (__MACH__)
      sigdelset (&ignore_set, SIGUSR2);
#endif
      sigprocmask (SIG_SETMASK, &ignore_set, 0);

      memset (&act, 0, sizeof (act));
      sigaction (SIGTERM, &act, 0);
  }
  
  DBG (DBG_sane_proc, "pie_reader_process started\n");

  memset (&act, 0, sizeof (act));	/* define SIGTERM-handler */
  act.sa_handler = pie_reader_process_sigterm_handler;
  sigaction (SIGTERM, &act, 0);

  fp = fdopen (scanner->reader_fds, "w");
  if (!fp)
    {
      return SANE_STATUS_IO_ERROR;
    }

  DBG (DBG_sane_info, "pie_reader_process: starting to READ data\n");

  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_LINE)
    status = pie_reader_process_fmtline (scanner, fp);
  else if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    status = pie_reader_process_indexed (scanner, fp);
  else
    status = SANE_STATUS_UNSUPPORTED;

  fclose (fp);

  DBG (DBG_sane_info, "pie_reader_process: finished reading data\n");

  return status;
}


/* ------------------------------ PIE_ATTACH_ONE -------------------------------- */

/* callback function for sanei_usb_attach_matching_devices */
static SANE_Status
pie_attach_one (const char *name)
{
  pie_attach_scanner (name, 0);
  return SANE_STATUS_GOOD;
}


/* --------------------------- PIE_CLOSE_PIPE -------------------------------- */

static SANE_Status
pie_close_pipe (Pie_Scanner * scanner)
{
  DBG (DBG_sane_proc, "pie_close_pipe\n");

  if (scanner->pipe >= 0)
    {
      close (scanner->pipe);
      scanner->pipe = -1;
    }

  return SANE_STATUS_EOF;
}



/* -------------------------- PIE_DO_CANCEL -------------------------------- */


static SANE_Status
pie_do_cancel (Pie_Scanner * scanner)
{
  DBG (DBG_sane_proc, "pie_do_cancel\n");

  scanner->scanning = SANE_FALSE;

  if (scanner->reader_pid != NO_PID)
    {
      DBG (DBG_sane_info, "killing pie_reader_process\n");
      sanei_thread_kill (scanner->reader_pid);
      sanei_thread_waitpid (scanner->reader_pid, 0);
      scanner->reader_pid = NO_PID;
      DBG (DBG_sane_info, "pie_reader_process killed\n");
    }

  if (scanner->sfd >= 0)
    {
      pie_scan (scanner, 0);

      pie_power_save (scanner, 15);

      pie_give_scanner (scanner);	/* reposition and release scanner */

      DBG (DBG_sane_info, "closing scannerdevice filedescriptor\n");
      sanei_scsi_close (scanner->sfd);
      scanner->sfd = -1;
    }

  return SANE_STATUS_CANCELLED;
}


/*
 * @@ sane_xy functions: Most of them call their pie_usb counterparts for USB scanners
 */

/* --------------------------------------- SANE INIT ---------------------------------- */

SANE_Status
sane_init (SANE_Int * version_code,
	   SANE_Auth_Callback __sane_unused__ authorize)
{
  char dev_name[PATH_MAX];
  size_t len;
  FILE *fp;

  DBG_INIT ();

  DBG (DBG_sane_init, "sane_init() build %d\n", BUILD);

  if (version_code)
    *version_code = SANE_VERSION_CODE (SANE_CURRENT_MAJOR, V_MINOR, BUILD);

  /* initialize usb use */
  sanei_usb_init ();
  /* initialize infrared handling */
  sanei_ir_init ();
  /* initialize magic handling */
  sanei_magic_init ();

  fp = sanei_config_open (PIE_CONFIG_FILE);
  if (!fp)
    {
      pie_attach_scanner ("/dev/scanner", 0);	/* no config-file: /dev/scanner */
      return SANE_STATUS_GOOD;
    }

  while (sanei_config_read (dev_name, sizeof (dev_name), fp))
    {
      if (dev_name[0] == '#')
	{
	  continue;
	}			/* ignore line comments */

      len = strlen (dev_name);

      if (!len)			/* ignore empty lines */
	{
	  continue;
	}

      DBG (DBG_sane_proc, "sane_init() trying %s\n", dev_name);
      sanei_config_attach_matching_devices (dev_name, pie_usb_try_attach);
    }

  fclose (fp);

  return SANE_STATUS_GOOD;
}


/* ----------------------------------------- SANE EXIT ---------------------------------- */


void
sane_exit (void)
{
  Pie_Device *dev, *next;
  int i;

  DBG (DBG_sane_init, "sane_exit()\n");

  for (dev = first_dev; dev; dev = next)
    {
      next = dev->next;
      free (dev->devicename);
      if (dev->model == NULL)
	{
	  free (dev->vendor);
	  free (dev->product);
	}
      free (dev->version);
      if (dev->cal_info)
      free (dev->cal_info);
      i = 0;
      while (dev->halftone_list[i] != NULL)
	free (dev->halftone_list[i++]);
      i = 0;
      while (dev->speed_list[i] != NULL)
	free (dev->speed_list[i++]);

      free (dev);
    }

  first_dev = NULL;

  if (devlist)
    {
      free (devlist);
      devlist = NULL;
    }
}


/* ------------------------------------------ SANE GET DEVICES --------------------------- */


SANE_Status
sane_get_devices (const SANE_Device *** device_list,
    		  SANE_Bool __sane_unused__ local_only)
{
  Pie_Device *dev;
  int i;

  DBG (DBG_sane_init, "sane_get_devices\n");

  i = 0;
  for (dev = first_dev; dev; dev = dev->next)
    i++;

  if (devlist)
    {
      free (devlist);
    }

  devlist = malloc ((i + 1) * sizeof (devlist[0]));
  if (!devlist)
    {
      return SANE_STATUS_NO_MEM;
    }

  i = 0;

  for (dev = first_dev; dev; dev = dev->next)
    {
      devlist[i++] = &dev->sane;
    }

  devlist[i] = NULL;

  *device_list = devlist;

  return SANE_STATUS_GOOD;
}


/* --------------------------------------- SANE OPEN ---------------------------------- */

SANE_Status
sane_open (SANE_String_Const devicename, SANE_Handle * handle)
{
  Pie_Device *dev;
  SANE_Status status;
  Pie_Scanner *scanner;
  int i, j;

  DBG (DBG_sane_init, "sane_open(%s)\n", devicename);

  if (devicename[0])		/* search for devicename */
    {
      for (dev = first_dev; dev; dev = dev->next)
	{
	  if (strcmp (dev->sane.name, devicename) == 0)
	    {
	      break;
	    }
	}

      if (!dev)
	{
	  status = pie_attach_scanner (devicename, &dev);
	  if (status != SANE_STATUS_GOOD)
	    {
	      return status;
	    }
	}
    }
  else
    {
      dev = first_dev;		/* empty devicename -> use first device */
    }


  if (!dev)
    {
      return SANE_STATUS_INVAL;
    }

  scanner = malloc (sizeof (*scanner));
  if (!scanner)

    {
      return SANE_STATUS_NO_MEM;
    }

  memset (scanner, 0, sizeof (*scanner));

  scanner->device = dev;
  scanner->sfd = -1;
  scanner->pipe = -1;

  if (scanner->device->model != NULL)	/* USB film scanners */
    return pie_usb_sane_open (scanner, handle);

  scanner->gamma_length = 1 << (scanner->device->inquiry_gamma_bits);

  DBG (DBG_sane_info, "Using %d bits for gamma input\n",
       scanner->device->inquiry_gamma_bits);

  scanner->gamma_range.min = 0;
  scanner->gamma_range.max = scanner->gamma_length - 1;
  scanner->gamma_range.quant = 0;

  scanner->gamma_table[0] =
    (SANE_Int *) malloc (scanner->gamma_length * sizeof (SANE_Int));
  scanner->gamma_table[1] =
    (SANE_Int *) malloc (scanner->gamma_length * sizeof (SANE_Int));
  scanner->gamma_table[2] =
    (SANE_Int *) malloc (scanner->gamma_length * sizeof (SANE_Int));
  scanner->gamma_table[3] =
    (SANE_Int *) malloc (scanner->gamma_length * sizeof (SANE_Int));

  for (i = 0; i < 4; ++i)	/* gamma_table[0,1,2,3] */
    {
      for (j = 0; j < scanner->gamma_length; ++j)
	{
	  scanner->gamma_table[i][j] = j;
	}
    }

  pie_init_options (scanner);

  scanner->next = first_handle;	/* insert newly opened handle into list of open handles: */
  first_handle = scanner;

  *handle = scanner;

  return SANE_STATUS_GOOD;
}


/* ------------------------------------ SANE CLOSE --------------------------------- */


void
sane_close (SANE_Handle handle)
{
  Pie_Scanner *prev, *scanner;

  DBG (DBG_sane_init, "sane_close\n");

  /* remove handle from list of open handles: */
  prev = 0;

  for (scanner = first_handle; scanner; scanner = scanner->next)
    {
      if (scanner == handle)
	{
	  break;
	}

      prev = scanner;
    }

  if (!scanner)
    {
      DBG (DBG_error, "close: invalid handle %p\n", handle);
      return;			/* oops, not a handle we know about */
    }

  if (prev)
    {
      prev->next = scanner->next;
    }
  else
    {
      first_handle = scanner->next;
    }

  if (scanner->device->model != NULL)	/* USB film scanners */
    {
      pie_usb_sane_close (scanner);
    }
  else
    {
      if (scanner->scanning)	/* stop scan if still scanning */
        {
          pie_do_cancel (handle);
        }

      free (scanner->gamma_table[0]);	/* free custom gamma tables */
      free (scanner->gamma_table[1]);
      free (scanner->gamma_table[2]);
      free (scanner->gamma_table[3]);
    }
  free (scanner->val[OPT_MODE].s);
  free (scanner->val[OPT_SPEED].s);
  free (scanner->val[OPT_HALFTONE_PATTERN].s);
  free (scanner->val[OPT_SW_IRED].s);
  free (scanner->val[OPT_SW_CROP].s);

  scanner->bufsize = 0;

  free (scanner);		/* free scanner */
}


/* ---------------------------------- SANE GET OPTION DESCRIPTOR ----------------- */

const SANE_Option_Descriptor *
sane_get_option_descriptor (SANE_Handle handle, SANE_Int option)
{
  Pie_Scanner *scanner = handle;

  DBG (DBG_sane_option, "sane_get_option_descriptor %d\n", option);

  if ((unsigned) option >= NUM_OPTIONS)
    {
      return 0;
    }

  return scanner->opt + option;
}


/* ---------------------------------- SANE CONTROL OPTION ------------------------ */

SANE_Status
sane_control_option (SANE_Handle handle, SANE_Int option, SANE_Action action,
		     void *val, SANE_Int * info)
{
  Pie_Scanner *scanner = handle;
  SANE_Status status;
  SANE_Word cap;
  SANE_String_Const name;

  if (scanner->device->model != NULL)
    return pie_usb_sane_control_option (handle, option, action, val, info);

  if (info)
    {
      *info = 0;
    }

  if (scanner->scanning)
    {
      return SANE_STATUS_DEVICE_BUSY;
    }

  if ((unsigned) option >= NUM_OPTIONS)
    {
      return SANE_STATUS_INVAL;
    }

  cap = scanner->opt[option].cap;
  if (!SANE_OPTION_IS_ACTIVE (cap))
    {
      return SANE_STATUS_INVAL;
    }

  name = scanner->opt[option].name;
  if (!name)
    {
      name = "(no name)";
    }

  if (action == SANE_ACTION_GET_VALUE)
    {

      DBG (DBG_sane_option, "get %s [#%d]\n", name, option);

      switch (option)
	{
	  /* word options: */
	case OPT_NUM_OPTS:
	case OPT_RESOLUTION:
	case OPT_TL_X:
	case OPT_TL_Y:
	case OPT_BR_X:
	case OPT_BR_Y:
	case OPT_PREVIEW:
	case OPT_THRESHOLD:
	  *(SANE_Word *) val = scanner->val[option].w;
	  return SANE_STATUS_GOOD;

	  /* word-array options: */
	case OPT_GAMMA_VECTOR:
	case OPT_GAMMA_VECTOR_R:
	case OPT_GAMMA_VECTOR_G:
	case OPT_GAMMA_VECTOR_B:
	  memcpy (val, scanner->val[option].wa, scanner->opt[option].size);
	  return SANE_STATUS_GOOD;

#if 0
	  /* string options: */
	case OPT_SOURCE:
#endif
	case OPT_MODE:
	case OPT_HALFTONE_PATTERN:
	case OPT_SPEED:
	  strcpy (val, scanner->val[option].s);
	  return SANE_STATUS_GOOD;
	}
    }
  else if (action == SANE_ACTION_SET_VALUE)
    {
      switch (scanner->opt[option].type)
	{
	case SANE_TYPE_INT:
	  DBG (DBG_sane_option, "set %s [#%d] to %d\n", name, option,
	       *(SANE_Word *) val);
	  break;

	case SANE_TYPE_FIXED:
	  DBG (DBG_sane_option, "set %s [#%d] to %f\n", name, option,
	       SANE_UNFIX (*(SANE_Word *) val));
	  break;

	case SANE_TYPE_STRING:
	  DBG (DBG_sane_option, "set %s [#%d] to %s\n", name, option,
	       (char *) val);
	  break;

	case SANE_TYPE_BOOL:
	  DBG (DBG_sane_option, "set %s [#%d] to %d\n", name, option,
	       *(SANE_Word *) val);
	  break;

	default:
	  DBG (DBG_sane_option, "set %s [#%d]\n", name, option);
	}

      if (!SANE_OPTION_IS_SETTABLE (cap))
	{
	  return SANE_STATUS_INVAL;
	}

      status = sanei_constrain_value (scanner->opt + option, val, info);
      if (status != SANE_STATUS_GOOD)
	{
	  return status;
	}

      switch (option)
	{
	  /* (mostly) side-effect-free word options: */
	case OPT_RESOLUTION:
	case OPT_TL_X:
	case OPT_TL_Y:
	case OPT_BR_X:
	case OPT_BR_Y:
	  if (info)
	    {
	      *info |= SANE_INFO_RELOAD_PARAMS;
	    }
	  /* fall through */
	case OPT_NUM_OPTS:
	case OPT_PREVIEW:
	case OPT_THRESHOLD:
	  scanner->val[option].w = *(SANE_Word *) val;
	  return SANE_STATUS_GOOD;

	  /* side-effect-free word-array options: */
	case OPT_GAMMA_VECTOR:
	case OPT_GAMMA_VECTOR_R:
	case OPT_GAMMA_VECTOR_G:
	case OPT_GAMMA_VECTOR_B:
	  memcpy (scanner->val[option].wa, val, scanner->opt[option].size);
	  return SANE_STATUS_GOOD;

	  /* options with side-effects: */

	case OPT_MODE:
	  {
	    int halftoning;

	    if (scanner->val[option].s)
	      {
		free (scanner->val[option].s);
	      }

	    scanner->val[option].s = (SANE_Char *) strdup (val);

	    if (info)
	      {
		*info |= SANE_INFO_RELOAD_OPTIONS | SANE_INFO_RELOAD_PARAMS;
	      }

	    scanner->opt[OPT_HALFTONE_PATTERN].cap |= SANE_CAP_INACTIVE;


	    scanner->opt[OPT_GAMMA_VECTOR].cap |= SANE_CAP_INACTIVE;
	    scanner->opt[OPT_GAMMA_VECTOR_R].cap |= SANE_CAP_INACTIVE;
	    scanner->opt[OPT_GAMMA_VECTOR_G].cap |= SANE_CAP_INACTIVE;
	    scanner->opt[OPT_GAMMA_VECTOR_B].cap |= SANE_CAP_INACTIVE;
	    scanner->opt[OPT_THRESHOLD].cap |= SANE_CAP_INACTIVE;

	    halftoning = (strcmp (val, HALFTONE_STR) == 0);

	    if (halftoning || strcmp (val, LINEART_STR) == 0)
	      {			/* one bit modes */
		if (halftoning)
		  {		/* halftoning modes */
		    scanner->opt[OPT_HALFTONE_PATTERN].cap &=
		      ~SANE_CAP_INACTIVE;
		  }
		else
		  {		/* lineart modes */
		  }
		scanner->opt[OPT_THRESHOLD].cap &= ~SANE_CAP_INACTIVE;
	      }
	    else
	      {			/* multi-bit modes(gray or color) */
	      }

	    if ((strcmp (val, LINEART_STR) == 0)
		|| (strcmp (val, HALFTONE_STR) == 0)
		|| (strcmp (val, GRAY_STR) == 0))
	      {
		scanner->opt[OPT_GAMMA_VECTOR].cap &= ~SANE_CAP_INACTIVE;
	      }
	    else if (strcmp (val, COLOR_STR) == 0)
	      {
		/* scanner->opt[OPT_GAMMA_VECTOR].cap &= ~SANE_CAP_INACTIVE; */
		scanner->opt[OPT_GAMMA_VECTOR_R].cap &= ~SANE_CAP_INACTIVE;
		scanner->opt[OPT_GAMMA_VECTOR_G].cap &= ~SANE_CAP_INACTIVE;
		scanner->opt[OPT_GAMMA_VECTOR_B].cap &= ~SANE_CAP_INACTIVE;
	      }
	    return SANE_STATUS_GOOD;
	  }

	case OPT_SPEED:
	case OPT_HALFTONE_PATTERN:
	  {
	    if (scanner->val[option].s)
	      {
		free (scanner->val[option].s);
	      }

	    scanner->val[option].s = (SANE_Char *) strdup (val);

	    return SANE_STATUS_GOOD;
	  }
	}
    }				/* else */
  return SANE_STATUS_INVAL;
}


/* ------------------------------------ SANE GET PARAMETERS ------------------------ */


SANE_Status
sane_get_parameters (SANE_Handle handle, SANE_Parameters * params)
{
  Pie_Scanner *scanner = handle;
  const char *mode;

  DBG (DBG_sane_info, "sane_get_parameters\n");

  if (!scanner->scanning)
    {				/* not scanning, so lets use recent values */
      double width, length, x_dpi, y_dpi;

      memset (&scanner->params, 0, sizeof (scanner->params));

      width =
	SANE_UNFIX (scanner->val[OPT_BR_X].w - scanner->val[OPT_TL_X].w);
      length =
	SANE_UNFIX (scanner->val[OPT_BR_Y].w - scanner->val[OPT_TL_Y].w);
      x_dpi = SANE_UNFIX (scanner->val[OPT_RESOLUTION].w);
      y_dpi = x_dpi;

#if 0
      if ((scanner->val[OPT_RESOLUTION_BIND].w == SANE_TRUE)
	  || (scanner->val[OPT_PREVIEW].w == SANE_TRUE))
	{
	  y_dpi = x_dpi;
	}
#endif
      if (x_dpi > 0.0 && y_dpi > 0.0 && width > 0.0 && length > 0.0)
	{
	  double x_dots_per_mm = x_dpi / MM_PER_INCH;
	  double y_dots_per_mm = y_dpi / MM_PER_INCH;

	  scanner->params.pixels_per_line = width * x_dots_per_mm;
	  scanner->params.lines = length * y_dots_per_mm;
	}
    }

  mode = scanner->val[OPT_MODE].s;

  if (strcmp (mode, LINEART_STR) == 0 || strcmp (mode, HALFTONE_STR) == 0)
    {
      scanner->params.format = SANE_FRAME_GRAY;
      scanner->params.bytes_per_line =
	(scanner->params.pixels_per_line + 7) / 8;
      scanner->params.depth = 1;
    }
  else if (strcmp (mode, GRAY_STR) == 0)
    {
      scanner->params.format = SANE_FRAME_GRAY;
      scanner->params.bytes_per_line = scanner->params.pixels_per_line;
      scanner->params.depth = scanner->val[OPT_BIT_DEPTH].w;
    }
  else if ((strcmp (mode, COLOR_STR) == 0) ||           /* RGB */
	  ((strcmp (mode, COLOR_IR_STR) == 0) &&        /* RGB with infrared processing */
	   (strcmp (scanner->val[OPT_SW_IRED].s, IR_CLEAN_STR) == 0)))
    {
      scanner->params.format = SANE_FRAME_RGB;
      scanner->params.bytes_per_line = 3 * scanner->params.pixels_per_line;
      scanner->params.depth = scanner->val[OPT_BIT_DEPTH].w;
    }
  else				/* pure RGBI */
    {
#ifdef SANE_FRAME_RGBI
      scanner->params.format = SANE_FRAME_RGBI;
      scanner->params.bytes_per_line = 4 * scanner->params.pixels_per_line;
#else
      scanner->params.format = SANE_FRAME_RGB;
      scanner->params.bytes_per_line = 3 * scanner->params.pixels_per_line;
#endif
      scanner->params.depth = scanner->val[OPT_BIT_DEPTH].w;
    }
  if (scanner->params.depth > 8)
    scanner->params.bytes_per_line *= 2;

  scanner->params.last_frame = (scanner->params.format != SANE_FRAME_RED
				&& scanner->params.format !=
				SANE_FRAME_GREEN);

  if (params)
    {
      *params = scanner->params;
    }

  return SANE_STATUS_GOOD;
}


/* ----------------------------------------- SANE START --------------------------------- */


SANE_Status
sane_start (SANE_Handle handle)
{
  Pie_Scanner *scanner = handle;
  int fds[2];
  const char *mode;
  int status;

  DBG (DBG_sane_init, "sane_start\n");

  /* Check for inconsistencies */

  if (scanner->val[OPT_TL_X].w > scanner->val[OPT_BR_X].w)
    {
      DBG (0, "sane_start: %s (%.1f mm) is bigger than %s (%.1f mm) "
              "-- aborting\n",
	   scanner->opt[OPT_TL_X].title,
	   SANE_UNFIX (scanner->val[OPT_TL_X].w),
	   scanner->opt[OPT_BR_X].title,
	   SANE_UNFIX (scanner->val[OPT_BR_X].w));
      return SANE_STATUS_INVAL;
    }
  if (scanner->val[OPT_TL_Y].w > scanner->val[OPT_BR_Y].w)
    {
      DBG (0, "sane_start: %s (%.1f mm) is bigger than %s (%.1f mm) "
	      "-- aborting\n",
	   scanner->opt[OPT_TL_Y].title,
	   SANE_UNFIX (scanner->val[OPT_TL_Y].w),
	   scanner->opt[OPT_BR_Y].title,
	   SANE_UNFIX (scanner->val[OPT_BR_Y].w));
      return SANE_STATUS_INVAL;
    }

  if (scanner->device->model != NULL)	/* USB film scanners */
    return pie_usb_sane_start (scanner);

  mode = scanner->val[OPT_MODE].s;

  if (scanner->sfd < 0)		/* first call, don`t run this routine again on multi frame or multi image scan */
    {
#ifdef HAVE_SANEI_SCSI_OPEN_EXTENDED
      int scsi_bufsize = 131072;	/* 128KB */

      if (sanei_scsi_open_extended
	  (scanner->device->sane.name, &(scanner->sfd), pie_sense_handler,
	   scanner->device, &scsi_bufsize) != 0)

	{
	  DBG (DBG_error, "sane_start: open failed\n");
	  return SANE_STATUS_INVAL;
	}

      if (scsi_bufsize < 32768)	/* < 32KB */
	{
	  DBG (DBG_error,
	       "sane_start: sanei_scsi_open_extended returned too small scsi buffer (%d)\n",
	       scsi_bufsize);
	  sanei_scsi_close ((scanner->sfd));
	  return SANE_STATUS_NO_MEM;
	}
      DBG (DBG_info,
	   "sane_start: sanei_scsi_open_extended returned scsi buffer size = %d\n",
	   scsi_bufsize);


      scanner->bufsize = scsi_bufsize;
#else
      if (sanei_scsi_open
	  (scanner->device->sane.name, &(scanner->sfd), pie_sense_handler,
	   scanner->device) != SANE_STATUS_GOOD)
	{
	  DBG (DBG_error, "sane_start: open of %s failed:\n",
	       scanner->device->sane.name);
	  return SANE_STATUS_INVAL;
	}

      /* there is no need to reallocate the buffer because the size is fixed */
#endif

#if 0
      if (pie_check_values (scanner->device) != 0)
	{
	  DBG (DBG_error, "ERROR: invalid scan-values\n");
	  scanner->scanning = SANE_FALSE;
	  pie_give_scanner (scanner);	/* reposition and release scanner */
	  sanei_scsi_close (scanner->sfd);
	  scanner->sfd = -1;
	  return SANE_STATUS_INVAL;
	}
#endif
#if 0
      scanner->params.bytes_per_line = scanner->device->row_len;
      scanner->params.pixels_per_line = scanner->device->width_in_pixels;
      scanner->params.lines = scanner->device->length_in_pixels;

      sane_get_parameters (scanner, 0);

      DBG (DBG_sane_info, "x_resolution (dpi)      = %u\n",
	   scanner->device->x_resolution);
      DBG (DBG_sane_info, "y_resolution (dpi)      = %u\n",
	   scanner->device->y_resolution);
      DBG (DBG_sane_info, "x_coordinate_base (dpi) = %u\n",
	   scanner->device->x_coordinate_base);
      DBG (DBG_sane_info, "y_coordinate_base (dpi) = %u\n",
	   scanner->device->y_coordinate_base);
      DBG (DBG_sane_info, "upper_left_x (xbase)    = %d\n",
	   scanner->device->upper_left_x);
      DBG (DBG_sane_info, "upper_left_y (ybase)    = %d\n",
	   scanner->device->upper_left_y);
      DBG (DBG_sane_info, "scanwidth    (xbase)    = %u\n",
	   scanner->device->scanwidth);
      DBG (DBG_sane_info, "scanlength   (ybase)    = %u\n",
	   scanner->device->scanlength);
      DBG (DBG_sane_info, "width in pixels         = %u\n",
	   scanner->device->width_in_pixels);
      DBG (DBG_sane_info, "length in pixels        = %u\n",
	   scanner->device->length_in_pixels);
      DBG (DBG_sane_info, "bits per pixel/color    = %u\n",
	   scanner->device->bits_per_pixel);
      DBG (DBG_sane_info, "bytes per line          = %d\n",
	   scanner->params.bytes_per_line);
      DBG (DBG_sane_info, "pixels_per_line         = %d\n",
	   scanner->params.pixels_per_line);
      DBG (DBG_sane_info, "lines                   = %d\n",
	   scanner->params.lines);
#endif

      /* grab scanner */
      if (pie_grab_scanner (scanner))
	{
	  sanei_scsi_close (scanner->sfd);
	  scanner->sfd = -1;
	  DBG (DBG_warning,
	       "WARNING: unable to reserve scanner: device busy\n");
	  return SANE_STATUS_DEVICE_BUSY;
	}

      scanner->scanning = SANE_TRUE;

      pie_power_save (scanner, 0);
    }				/* ------------ end of first call -------------- */


  if (strcmp (mode, LINEART_STR) == 0)
    {
      scanner->colormode = LINEART;
    }
  else if (strcmp (mode, HALFTONE_STR) == 0)
    {
      scanner->colormode = HALFTONE;
    }
  else if (strcmp (mode, GRAY_STR) == 0)
    {
      scanner->colormode = GRAYSCALE;
    }
  else if (strcmp (mode, COLOR_STR) == 0)
    {
      scanner->colormode = RGB;
    }

  /* get and set geometric values for scanning */
  scanner->resolution = SANE_UNFIX (scanner->val[OPT_RESOLUTION].w);

  pie_set_window (scanner);
  pie_send_exposure (scanner);
  pie_mode_select (scanner);
  pie_send_highlight_shadow (scanner);

  pie_scan (scanner, 1);

  status = pie_do_cal (scanner);
  if (status)
    return status;

  /* send gammacurves */

  pie_dwnld_gamma (scanner);

  pie_get_params (scanner);

  if (pipe (fds) < 0)		/* create a pipe, fds[0]=read-fd, fds[1]=write-fd */
    {
      DBG (DBG_error, "ERROR: could not create pipe\n");
      scanner->scanning = SANE_FALSE;
      pie_scan (scanner, 0);
      pie_give_scanner (scanner);	/* reposition and release scanner */
      sanei_scsi_close (scanner->sfd);
      scanner->sfd = -1;
      return SANE_STATUS_IO_ERROR;
    }

  scanner->pipe       = fds[0];
  scanner->reader_fds = fds[1];
  scanner->reader_pid = sanei_thread_begin (pie_reader_process, (void *) scanner);

  if (scanner->reader_pid == NO_PID)
    {
      DBG (1, "sane_start: sanei_thread_begin failed (%s)\n",
             strerror (errno));
      return SANE_STATUS_NO_MEM;
    }

  if (sanei_thread_is_forked ())
    {
      close (scanner->reader_fds);
      scanner->reader_fds = -1;
    }

  return SANE_STATUS_GOOD;
}


/* -------------------------------------- SANE READ ---------------------------------- */


SANE_Status
sane_read (SANE_Handle handle, SANE_Byte * buf, SANE_Int max_len,
	   SANE_Int * len)
{
  Pie_Scanner *scanner = handle;
  ssize_t nread;

  *len = 0;

  if (scanner->device->model != NULL)	/* USB film scanners */
    return pie_usb_sane_read (scanner, buf, max_len, len);

  nread = read (scanner->pipe, buf, max_len);
  DBG (DBG_sane_info, "sane_read: read %ld bytes\n", (long) nread);

  if (!(scanner->scanning))	/* OOPS, not scanning */
    {
      return pie_do_cancel (scanner);
    }

  if (nread < 0)
    {
      if (errno == EAGAIN)
	{
	  DBG (DBG_sane_info, "sane_read: EAGAIN\n");
	  return SANE_STATUS_GOOD;
	}
      else
	{
	  pie_do_cancel (scanner);	/* we had an error, stop scanner */
	  return SANE_STATUS_IO_ERROR;
	}
    }

  *len = nread;

  if (nread == 0)		/* EOF */
    {
      pie_do_cancel (scanner);

      return pie_close_pipe (scanner);	/* close pipe */
    }

  return SANE_STATUS_GOOD;
}


/* ------------------------------------- SANE CANCEL -------------------------------- */


void
sane_cancel (SANE_Handle handle)
{
  Pie_Scanner *scanner = handle;

  DBG (DBG_sane_init, "sane_cancel\n");

  if (scanner->device->model == NULL)
    {
      if (scanner->scanning)
        pie_do_cancel (scanner);
    }
  else			/* USB film scanners */
    pie_usb_do_cancel (scanner, SANE_TRUE);
}


/* -------------------------------------- SANE SET IO MODE --------------------------- */


SANE_Status
sane_set_io_mode (SANE_Handle handle, SANE_Bool non_blocking)
{
  Pie_Scanner *scanner = handle;

  DBG (DBG_sane_init, "sane_set_io_mode: non_blocking=%d\n", non_blocking);

  if (!scanner->scanning)
    {
      return SANE_STATUS_INVAL;
    }

  if (fcntl (scanner->pipe, F_SETFL, non_blocking ? O_NONBLOCK : 0) < 0)
    {
      return SANE_STATUS_IO_ERROR;
    }

  return SANE_STATUS_GOOD;
}


/* --------------------------------------- SANE GET SELECT FD ------------------------- */


SANE_Status
sane_get_select_fd (SANE_Handle handle, SANE_Int * fd)
{
  Pie_Scanner *scanner = handle;

  DBG (DBG_sane_init, "sane_get_select_fd\n");

  if (!scanner->scanning)
    {
      return SANE_STATUS_INVAL;
    }
  *fd = scanner->pipe;

  return SANE_STATUS_GOOD;
}

