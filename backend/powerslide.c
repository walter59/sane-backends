/* sane - Scanner Access Now Easy.

   powerslide.c, roughly based on pie.c

   Backend for Pacific Image Electronics PowerSlide 3600/5000
   (Sold in Germany a Reflecta DigitDia 4000/5000)

   Copyright (C) 2012 Klaus Kämpf

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
   If you do not wish that, delete this exception notice.  */

/*
 * 2012-07-03 Started
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
#include <unistd.h>

#include "../include/sane/sane.h"
#include "../include/sane/sanei.h"
#include "../include/sane/saneopts.h"
#include "../include/sane/sanei_usb.h"
#include "../include/sane/sanei_debug.h"

#define BACKEND_NAME	powerslide
#include "../include/sane/sanei_backend.h"
#include "../include/sane/sanei_config.h"

# include "../include/sane/sanei_thread.h"

#include "powerslide-scsidef.h"

#define min(x,y) (((x) < (y)) ? (x) : (y))

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

#define BUILD 1

#define POWERSLIDE_CONFIG_FILE "powerslide.conf"

/* wait times in usec */

#define DOWNLOAD_GAMMA_WAIT_TIME 1000
#define SCAN_WARMUP_WAIT_TIME 1000
#define SCAN_WAIT_TIME 1000
#define TUR_WAIT_TIME 1000

/* from libieee1284 */
#define C1284_NSTROBE 0x01
#define C1284_NINIT 0x04

/* usb bRequest */
#define POWERSLIDE_USB_REQ_MANY 0x04 /* multiple bytes */
#define POWERSLIDE_USB_REQ_ONE 0x0c /* single byte */

/* usb wValue aka register */
#define POWERSLIDE_USB_SIZE_REG 0x0082
#define POWERSLIDE_USB_SCSI_STATUS 0x0084
#define POWERSLIDE_USB_SCSI_CMD 0x0085
#define POWERSLIDE_USB_VAL_CTRL 0x0087
#define POWERSLIDE_USB_VAL_DATA 0x0088

/* names of scanners that are supported because */
/* the inquiry_return_block is ok and driver is tested */

static char *scanner_str[] = {
  "PIE", "MS Scanner",
  "END_OF_LIST"
};


/* options supported by the scanner */

enum Powerslide_Option
{
  OPT_NUM_OPTS = 0,

  /* ------------------------------------------- */
  OPT_MODE_GROUP,
  OPT_MODE,
  OPT_RESOLUTION,


  /* ------------------------------------------- */

  OPT_GEOMETRY_GROUP,
  OPT_TL_X,			/* top-left x */
  OPT_TL_Y,			/* top-left y */
  OPT_BR_X,			/* bottom-right x */
  OPT_BR_Y,			/* bottom-right y */

  /* ------------------------------------------- */

  OPT_ENHANCEMENT_GROUP,

  OPT_HALFTONE_PATTERN,
  OPT_SPEED,
  OPT_THRESHOLD,

  OPT_GAMMA_VECTOR,
  OPT_GAMMA_VECTOR_R,
  OPT_GAMMA_VECTOR_G,
  OPT_GAMMA_VECTOR_B,

  /* ------------------------------------------- */

  OPT_ADVANCED_GROUP,
  OPT_PREVIEW,

  /* must come last: */
  NUM_OPTIONS
};




/* This defines the information needed during calibration */

struct Powerslide_cal_info
{
  int cal_type;
  int receive_bits;
  int send_bits;
  int num_lines;
  int pixels_per_line;
};


/* This structure holds the information about a physical scanner */

typedef struct Powerslide_Device
{
  struct Powerslide_Device *next;

  char *devicename;		/* name of the scanner device from powerslide.conf */
  char *usbname;		/* usb name of the scanner device, for sanei_usb_open() */
  SANE_Int usb;                 /* opened USB device, -1 if closed */

  char vendor[9];		/* will be xxxxx */
  char product[17];		/* e.g. "SuperVista_S12" or so */
  char version[5];		/* e.g. V1.3 */

  SANE_Device sane;
  SANE_Range dpi_range;
  SANE_Range x_range;
  SANE_Range y_range;

  SANE_Range exposure_range;
  SANE_Range shadow_range;
  SANE_Range highlight_range;

  int inquiry_len;		/* length of inquiry return block */

  int inquiry_x_res;		/* maximum x-resolution */
  int inquiry_y_res;		/* maximum y-resolution */
  int inquiry_pixel_resolution;
  double inquiry_fb_width;	/* flatbed width in inches */
  double inquiry_fb_length;	/* flatbed length in inches */

  int inquiry_trans_top_left_x;
  int inquiry_trans_top_left_y;
  double inquiry_trans_width;	/* transparency width in inches */
  double inquiry_trans_length;	/* transparency length in inches */

  int inquiry_halftones;	/* number of halftones supported */
  int inquiry_filters;		/* available colour filters */
  int inquiry_color_depths;	/* available colour depths */
  int inquiry_color_format;	/* colour format from scanner */
  int inquiry_image_format;	/* image data format */
  int inquiry_scan_capability;	/* additional scanner features, number of speeds */
  int inquiry_optional_devices;	/* optional devices */
  int inquiry_enhancements;	/* enhancements */
  int inquiry_gamma_bits;	/* no of bits used for gamma table */
  int inquiry_fast_preview_res;	/* fast preview resolution */
  int inquiry_min_highlight;	/* min highlight % that can be used */
  int inquiry_max_shadow;	/* max shadow % that can be used */
  int inquiry_cal_eqn;		/* which calibration equation to use */
  int inquiry_min_exp;		/* min exposure % */
  int inquiry_max_exp;		/* max exposure % */

  SANE_String scan_mode_list[7];	/* holds names of types of scan (color, ...) */

  SANE_String halftone_list[17];	/* holds the names of the halftone patterns from the scanner */

  SANE_String speed_list[9];	/* holds the names of available speeds */

  int cal_info_count;		/* number of calibration info sets */
  struct Powerslide_cal_info *cal_info;	/* points to the actual calibration information */
}
Powerslide_Device;

/* This structure holds information about an instance of an 'opened' scanner */

typedef struct Powerslide_Scanner
{
  struct Powerslide_Scanner *next;
  Powerslide_Device *device;		/* pointer to physical scanner */

  int sfd;			/* scanner file desc. */
  int bufsize;			/* max scsi buffer size */

  SANE_Option_Descriptor opt[NUM_OPTIONS];	/* option descriptions for this instance */
  Option_Value val[NUM_OPTIONS];	/* option settings for this instance */
  SANE_Int *gamma_table[4];	/* gamma tables for this instance */
  SANE_Range gamma_range;
  int gamma_length;		/* size of gamma table */

  int scanning;			/* true if actually doing a scan */
  SANE_Parameters params;

  SANE_Pid reader_pid;
  int pipe;
  int reader_fds;
  
  int colormode;		/* whether RGB, GRAY, LINEART, HALFTONE */
  int resolution;
  int cal_mode;			/* set to value to compare cal_info mode to */

  int cal_filter;		/* set to indicate which filters will provide data for cal */

  int filter_offset1;		/* offsets between colors in indexed scan mode */
  int filter_offset2;

  int bytes_per_line;		/* number of bytes per line */

}
Powerslide_Scanner;

static const SANE_Range percentage_range_100 = {
  0 << SANE_FIXED_SCALE_SHIFT,	/* minimum */
  100 << SANE_FIXED_SCALE_SHIFT,	/* maximum */
  0 << SANE_FIXED_SCALE_SHIFT	/* quantization */
};

static Powerslide_Device *first_dev = NULL;
static Powerslide_Scanner *first_handle = NULL;
static const SANE_Device **devlist = NULL;



static SANE_Status powerslide_wait_scanner (Powerslide_Scanner * scanner);


/* ---------------------------------- POWERSLIDE DUMP_BUFFER ---------------------------------- */

#define DBG_DUMP(level, buf, n)	{ if (DBG_LEVEL >= (level)) powerslide_dump_buffer(level,buf,n); }

static void
powerslide_dump_buffer (int level, unsigned char *buf, int n)
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

/* ---------------------------------- POWERSLIDE LOWLEVEL ---------------------------------- */

/* ---------------------------------- IEEE1284 via USB ---------------------------------- */

/*
 * powerslide_ieee1284_control_init - set IEEE1284 control to init
 *
 */
static SANE_Status
powerslide_ieee1284_control_init(SANE_Int usb)
{
  SANE_Int status;
  static SANE_Byte init[1] = { C1284_NINIT };
  DBG (DBG_proc, "powerslide_ieee1284_control_init\n");
  status = sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT, POWERSLIDE_USB_REQ_ONE,
				POWERSLIDE_USB_VAL_CTRL, 0, 1, init );
  usleep(3000);
  return status;
}

/*
 * powerslide_ieee1284_control_strobe - issue IEEE1284 strobe
 *
 */
static SANE_Status
powerslide_ieee1284_control_strobe(SANE_Int usb)
{
  static SANE_Byte strobe[1] = { C1284_NINIT|C1284_NSTROBE };
  SANE_Int status;
  DBG (DBG_proc, "powerslide_ieee1284_control_strobe\n");
  status = sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT, POWERSLIDE_USB_REQ_ONE,
				  POWERSLIDE_USB_VAL_CTRL, 0, 1, strobe );
  usleep(3000);
  if (status == SANE_STATUS_GOOD)
    {
      status = powerslide_ieee1284_control_init(usb);
    }
  return status;
}

/*
 * powerslide_ieee1284_command_write - write single command byte to IEEE1284 command register
 *
 */
static SANE_Status
powerslide_ieee1284_command_write(SANE_Int usb, SANE_Byte cmd)
{
  SANE_Int status;
  static SANE_Byte buf[1];
  buf[0] = cmd;
  DBG (DBG_proc, "powerslide_ieee1284_command_write\n");

  status = sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT, POWERSLIDE_USB_REQ_ONE,
				POWERSLIDE_USB_VAL_DATA, 0, 1, buf );
  usleep(3000);
  return status;
}

/*
 * powerslide_command prefix
 * Issue 'ieee1284' prefix sequence
 *
 */
static SANE_Status
powerslide_ieee1284_command_prefix(SANE_Int usb)
{
  static SANE_Byte prefix_sequence[] = { 0xff, 0xaa, 0x55, 0x00, 0xff, 0x87, 0x78 };
  DBG (DBG_proc, "powerslide_ieee1284_command_prefix\n");
  int prefix_sequence_length = sizeof(prefix_sequence);
  int i;
  SANE_Int status;
  
  for (i = 0; i < prefix_sequence_length; ++i)
    {
      status = powerslide_ieee1284_command_write(usb, prefix_sequence[i]);
      if (status != SANE_STATUS_GOOD)
        break;
    }
  return status;
}

/*
 * powerslide_command
 * Issue 'ieee1284' command
 *
 */

static SANE_Status
powerslide_ieee1284_command(SANE_Int usb, SANE_Byte command)
{
  SANE_Int status;
  DBG (DBG_proc, "powerslide_ieee1284_command\n");

  while (1)
    {
      status = powerslide_ieee1284_command_prefix(usb);
      if (status != SANE_STATUS_GOOD)
        break;
      status = powerslide_ieee1284_command_write(usb, command);
      if (status != SANE_STATUS_GOOD)
        break;
      status = powerslide_ieee1284_control_strobe(usb);
      if (status != SANE_STATUS_GOOD)
        break;
      status = powerslide_ieee1284_command_write(usb, 0xff);
      if (status != SANE_STATUS_GOOD)
        break;
      break;
    }
  if (status != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "usb write failed\n");
    }
  DBG (DBG_proc, "powerslide_ieee1284_command returns %d\n", status);
  return status;
}

/*
 * Issue 'Addr' via ieee1284
 *
 */

static SANE_Status
powerslide_ieee1284_addr(SANE_Int usb)
{
  DBG (DBG_proc, "powerslide_ieee1284_addr\n");
  return powerslide_ieee1284_command(usb, 0x00);
}


/*
 * Issue 'Reset' via ieee1284
 *
 */

static SANE_Status
powerslide_ieee1284_reset(SANE_Int usb)
{
  DBG (DBG_proc, "powerslide_ieee1284_reset\n");
  return powerslide_ieee1284_command(usb, 0x30);
}


/*
 * powerslide_scsi_command_write - write single command byte to SCSI register
 *
 */
static SANE_Status
powerslide_scsi_command_write(SANE_Int usb, SANE_Byte cmd)
{
  static SANE_Byte buf[1];
  buf[0] = cmd;
  DBG (DBG_proc, "powerslide_scsi_command_write\n");
  /* wIndex 0x0001 - unknown */
  return sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT, POWERSLIDE_USB_REQ_ONE,
				POWERSLIDE_USB_SCSI_CMD, 0x0001, 1, buf );
}


/*
 * powerslide_scsi_size_write - write size buffer to SCSI register
 *
 */
static SANE_Status
powerslide_scsi_size_write(SANE_Int usb, SANE_Int size, SANE_Byte *buf)
{
  SANE_Int status;
  DBG (DBG_proc, "powerslide_scsi_size_write\n");
  /* wIndex 0x00a4 - unknown */
  status = sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT, POWERSLIDE_USB_REQ_MANY,
				POWERSLIDE_USB_SIZE_REG, 0x00a4, size, buf );
  usleep(3000);
  return status;
}

/*
 * powerslide_scsi_status_read - read single status byte from SCSI register
 *
 * @return: -1 on error
 */
static SANE_Int
powerslide_scsi_status_read(SANE_Int usb)
{
  SANE_Byte status;
  DBG (DBG_proc, "powerslide_scsi_status_read\n");
  if (sanei_usb_control_msg (usb, USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_IN, POWERSLIDE_USB_REQ_ONE,
				POWERSLIDE_USB_SCSI_STATUS, 0, 1, &status ) != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "sanei_usb_control_msg failed with '%s'\n", sane_strstatus(status));
      sanei_usb_reset(usb);
      return -1;
    }
  return status;
}

/*
 * Issue 'Scsi' via ieee1284
 *
 */

static SANE_Status
powerslide_ieee1284_scsi(SANE_Int usb, SANE_Int scsi_len, SANE_Byte *scsi_buf)
{
  SANE_Int status;
  SANE_Int i;
  SANE_Int expected_size;
  SANE_Int scsi_status;
  static SANE_Byte sizebuf[8] = { 0 };

  DBG (DBG_proc, "powerslide_ieee1284_scsi: len %d, cmd 0x%02x\n", scsi_len, *scsi_buf);
  DBG_DUMP (DBG_proc, scsi_buf, scsi_len);	  
  powerslide_ieee1284_reset(usb);
  usleep(500);
  powerslide_ieee1284_reset(usb);
  usleep(500);
  powerslide_ieee1284_addr(usb);
  usleep(500);

  expected_size = scsi_buf[4];

  DBG (DBG_proc, "powerslide_ieee1284_scsi, cmd 0x%02x, scsi_len %d, expected 0x%02x\n", *scsi_buf, scsi_len, expected_size);
  status = powerslide_ieee1284_command(usb, 0xe0);

  for (i = 0; i < scsi_len; ++i)
    {
      if (status != SANE_STATUS_GOOD)
        {
	  DBG (DBG_error, "powerslide_ieee1284_scsi: failed with %d:'%s' at i %d\n", status, sane_strstatus(status), i);
          return status;
	}
      status = powerslide_scsi_command_write(usb, *scsi_buf++);
    }
  scsi_status = powerslide_scsi_status_read(usb);
  if (scsi_status != 1)
    {
      DBG (DBG_error, "Wrong status: 0x%02x\n", scsi_status);
      return SANE_STATUS_CANCELLED;
    }
  sizebuf[5] = expected_size;
  status = powerslide_scsi_size_write(usb, 8, sizebuf);

  return status;
}


/* ---------------------------------- SCSI via IEEE1284 ---------------------------------- */

/* ---------------------------- SENSE_HANDLER ------------------------------ */


static SANE_Status
sense_handler (__sane_unused__ int scsi_fd, unsigned char *result, __sane_unused__ void *arg)	/* is called by sanei_scsi */
{
  unsigned char asc, ascq, sensekey;
  int asc_ascq, len;
  /* Powerslide_Device *dev = arg; */

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


/* -------------------------------- POWERSLIDE PRINT INQUIRY ------------------------- */


static void
powerslide_print_inquiry (Powerslide_Device * dev)
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


/* ------------------------------ POWERSLIDE GET INQUIRY VALUES -------------------- */


static void
powerslide_get_inquiry_values (Powerslide_Device * dev, unsigned char *buffer)
{
  DBG (DBG_proc, "get_inquiry_values\n");

  dev->inquiry_len = get_inquiry_additional_length (buffer) + 5;

  get_inquiry_vendor ((char *) buffer, dev->vendor);
  dev->vendor[8] = '\0';
  get_inquiry_product ((char *) buffer, dev->product);
  dev->product[16] = '\0';
  get_inquiry_version ((char *) buffer, dev->version);
  dev->version[4] = '\0';

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

  powerslide_print_inquiry (dev);

  return;
}

/* ----------------------------- POWERSLIDE DO INQUIRY ---------------------------- */


static SANE_Int
powerslide_do_inquiry (int usb, SANE_Int *size, SANE_Byte *inquiry)
{
  SANE_Status status;
  SANE_Byte scsi[6] = { 0x12, 0x00, 0x00, 0x00, 0x84, 0x00 };
  size_t bufsize = 512;
  SANE_Byte buf[512];
  DBG (DBG_proc, "do_inquiry: size 0x%02x, inquiry @ %p\n", *size, inquiry);

  scsi[4] = *size;

  status = powerslide_ieee1284_scsi (usb, sizeof(scsi), scsi);
  if (status != SANE_STATUS_GOOD)
    return status;
  usleep(3000);
  sanei_usb_set_endpoint (usb, USB_ENDPOINT_TYPE_BULK, 1);
  status = sanei_usb_read_bulk (usb, buf, &bufsize);
  DBG (DBG_proc, "read_bulk: status %d, %d bytes:\n", status, bufsize);
  DBG_DUMP (DBG_proc, buf, bufsize);
  if (status == SANE_STATUS_GOOD)
    {
      *size = bufsize;
      memcpy (inquiry, buf, bufsize);
    }
  sanei_usb_set_endpoint (usb, USB_ENDPOINT_TYPE_CONTROL, 0);

  return status;
}

/* ---------------------- POWERSLIDE IDENTIFY SCANNER ---------------------- */


static int
powerslide_identify_scanner (Powerslide_Device * dev)
{
  char vendor[9];
  char product[0x11];
  char version[5];
  char *pp;
  int i = 0;
  SANE_Int status;
  SANE_Byte inquiry_block[132];
  SANE_Int inquiry_size = sizeof(inquiry_block);
  memset (inquiry_block, '\0', inquiry_size);	/* clear buffer */

  DBG (DBG_proc, "powerslide_identify_scanner: inquiry_size %d\n", inquiry_size);

  status = powerslide_do_inquiry (dev->usb, &inquiry_size, inquiry_block);	/* get inquiry */

  if (get_inquiry_periph_devtype (inquiry_block) != IN_periph_devtype_scanner)
    {
      return 1;
    }				/* no scanner */

  get_inquiry_vendor ((char *) inquiry_block, vendor);
  get_inquiry_product ((char *) inquiry_block, product);
  get_inquiry_version ((char *) inquiry_block, version);

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
	      DBG (DBG_info, "found supported scanner\n");

	      powerslide_get_inquiry_values (dev, inquiry_block);
	      return 0;
	    }
	}
      i++;
    }

  return 1;			/* NO SUPPORTED SCANNER: short inquiry-block and unknown scanner */
}

/* ------------------------------- ATTACH POWERSLIDE ----------------------------- */

static const char *usbName;

/* called from attach_scanner() via sanei_usb_find_devices() */
static SANE_Status
attach_powerslide (const char *usbname)
{
  DBG (DBG_sane_proc, "attach_powerslide: %s\n", usbname);
  usbName = strdup(usbname);
  return SANE_STATUS_GOOD;
}

/* ------------------------------- ATTACH SCANNER ----------------------------- */

static SANE_Status
attach_scanner (const char *devicename)
{
  Powerslide_Device *dev;
  SANE_Int vendor;
  SANE_Int product;

  DBG (DBG_sane_proc, "attach_scanner: %s\n", devicename);

  for (dev = first_dev; dev; dev = dev->next)
    {
      if (strcmp (dev->devicename, devicename) == 0)
	{
	  return SANE_STATUS_GOOD;
	}
    }

  if (sscanf(devicename, "usb 0x%x 0x%x", &vendor, &product) != 2)
    {
      DBG (DBG_error, "attach_scanner: Bad config line '%s', should be 'usb 0xVVVV 0xPPPP'\n", devicename);
      return SANE_STATUS_INVAL;
    }

  if (sanei_usb_find_devices (vendor, product, attach_powerslide) != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "attach_scanner: Cannot find USB vendor 0x%04x, product 0x%04x'\n", vendor, product);
      return SANE_STATUS_INVAL;
    }
      
  dev = calloc (1, sizeof (*dev));
  if (!dev)
    { 
      return SANE_STATUS_NO_MEM;
    }

  dev->usb = -1;

  dev->devicename = strdup(devicename);
  dev->usbname = usbName;

  dev->next = first_dev;
  first_dev = dev;

  return SANE_STATUS_GOOD;
}


static SANE_Status
powerslide_open( Powerslide_Device *dev)
{
  DBG (DBG_info, "powerslide_open: opening %s\n", dev->usbname);

  if (sanei_usb_open (dev->usbname, &dev->usb) != SANE_STATUS_GOOD)
    {
      DBG (DBG_error, "powerslide_open: Cannot open scanner device %s\n", dev->usbname);
      return SANE_STATUS_INVAL;
    }

  if (powerslide_identify_scanner (dev) != 0)
    {
      DBG (DBG_error, "powerslide_open: scanner-identification failed\n");
      sanei_usb_close (dev->usb);
      return SANE_STATUS_INVAL;
    }

  dev->sane.name = dev->devicename;
  dev->sane.vendor = dev->vendor;
  dev->sane.model = dev->product;
  dev->sane.type = "Multiple slide scanner";

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
init_options (Powerslide_Scanner * scanner)
{
  int i;

  DBG (DBG_sane_proc, "init_options\n");

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
    max_string_size ((SANE_String_Const *) scanner->device->scan_mode_list);
  scanner->opt[OPT_MODE].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_MODE].constraint.string_list =
    (SANE_String_Const *) scanner->device->scan_mode_list;
  scanner->val[OPT_MODE].s =
    (SANE_Char *) strdup (scanner->device->scan_mode_list[0]);

  /* x-resolution */
  scanner->opt[OPT_RESOLUTION].name = SANE_NAME_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].title = SANE_TITLE_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].desc = SANE_DESC_SCAN_RESOLUTION;
  scanner->opt[OPT_RESOLUTION].type = SANE_TYPE_FIXED;
  scanner->opt[OPT_RESOLUTION].unit = SANE_UNIT_DPI;
  scanner->opt[OPT_RESOLUTION].constraint_type = SANE_CONSTRAINT_RANGE;
  scanner->opt[OPT_RESOLUTION].constraint.range = &scanner->device->dpi_range;
  scanner->val[OPT_RESOLUTION].w = 100 << SANE_FIXED_SCALE_SHIFT;

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

  /* halftone pattern */
  scanner->opt[OPT_HALFTONE_PATTERN].name = SANE_NAME_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].title = SANE_TITLE_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].desc = SANE_DESC_HALFTONE_PATTERN;
  scanner->opt[OPT_HALFTONE_PATTERN].type = SANE_TYPE_STRING;
  scanner->opt[OPT_HALFTONE_PATTERN].size =
    max_string_size ((SANE_String_Const *) scanner->device->halftone_list);
  scanner->opt[OPT_HALFTONE_PATTERN].constraint_type =
    SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_HALFTONE_PATTERN].constraint.string_list =
    (SANE_String_Const *) scanner->device->halftone_list;
  scanner->val[OPT_HALFTONE_PATTERN].s =
    (SANE_Char *) strdup (scanner->device->halftone_list[0]);
  scanner->opt[OPT_HALFTONE_PATTERN].cap |= SANE_CAP_INACTIVE;

  /* speed */
  scanner->opt[OPT_SPEED].name = SANE_NAME_SCAN_SPEED;
  scanner->opt[OPT_SPEED].title = SANE_TITLE_SCAN_SPEED;
  scanner->opt[OPT_SPEED].desc = SANE_DESC_SCAN_SPEED;
  scanner->opt[OPT_SPEED].type = SANE_TYPE_STRING;
  scanner->opt[OPT_SPEED].size =
    max_string_size ((SANE_String_Const *) scanner->device->speed_list);
  scanner->opt[OPT_SPEED].constraint_type = SANE_CONSTRAINT_STRING_LIST;
  scanner->opt[OPT_SPEED].constraint.string_list =
    (SANE_String_Const *) scanner->device->speed_list;
  scanner->val[OPT_SPEED].s =
    (SANE_Char *) strdup (scanner->device->speed_list[0]);

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


/*------------------------- POWERSLIDE POWER SAVE -----------------------------*/

static SANE_Status
powerslide_power_save (Powerslide_Scanner * scanner, int time)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "powerslide_power_save: %d min\n", time);

  size = 6;

  set_write_length (swrite.cmd, size);

  memcpy (buffer, swrite.cmd, swrite.size);

  data = buffer + swrite.size;
  memset (data, 0, size);

  set_command (data, SET_POWER_SAVE_CONTROL);
  set_data_length (data, size - 4);
  data[4] = time & 0x7f;

  status =
    sanei_scsi_cmd (scanner->sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error, "powerslide_power_save: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}

/*------------------------- POWERSLIDE SEND EXPOSURE ONE -----------------------------*/


static SANE_Status
powerslide_send_exposure_one (Powerslide_Scanner * scanner, int filter, int value)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "powerslide_send_exposure_one\n");

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
    sanei_scsi_cmd (scanner->sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
	   "powerslide_send_exposure_one: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}

/*------------------------- POWERSLIDE SEND EXPOSURE -----------------------------*/

static SANE_Status
powerslide_send_exposure (Powerslide_Scanner * scanner)
{
  SANE_Status status;

  DBG (DBG_proc, "powerslide_send_exposure\n");

  status = powerslide_send_exposure_one (scanner, FILTER_RED, 100);
  if (status)
    return status;

  status = powerslide_send_exposure_one (scanner, FILTER_GREEN, 100);
  if (status)
    return status;

  status = powerslide_send_exposure_one (scanner, FILTER_BLUE, 100);
  if (status)
    return status;

  return SANE_STATUS_GOOD;
}


/*------------------------- POWERSLIDE SEND HIGHLIGHT/SHADOW ONE -----------------------------*/

static SANE_Status
powerslide_send_highlight_shadow_one (Powerslide_Scanner * scanner, int filter,
			       int highlight, int shadow)
{
  unsigned char buffer[128];
  size_t size;
  SANE_Status status;
  unsigned char *data;

  DBG (DBG_proc, "powerslide_send_highlight_shadow_one\n");

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
    sanei_scsi_cmd (scanner->sfd, buffer, swrite.size + size, NULL, NULL);
  if (status)
    {
      DBG (DBG_error,
	   "powerslide_send_highlight_shadow_one: write command returned status %s\n",
	   sane_strstatus (status));
    }

  return status;
}

/*------------------------- POWERSLIDE SEND HIGHLIGHT/SHADOW -----------------------------*/

static SANE_Status
powerslide_send_highlight_shadow (Powerslide_Scanner * scanner)
{
  return SANE_STATUS_GOOD;
}

/*------------------------- POWERSLIDE PERFORM CAL ----------------------------*/

static SANE_Status
powerslide_perform_cal (Powerslide_Scanner * scanner, int cal_index)
{

  return SANE_STATUS_GOOD;
}

/*------------------------- POWERSLIDE DO CAL -----------------------------*/

static SANE_Status
powerslide_do_cal (Powerslide_Scanner * scanner)
{
  return SANE_STATUS_GOOD;
}

/*------------------------- POWERSLIDE DWNLD GAMMA ONE -----------------------------*/

static SANE_Status
powerslide_dwnld_gamma_one (Powerslide_Scanner * scanner, int filter, SANE_Int * table)
{

  return SANE_STATUS_GOOD;
}

/*------------------------- POWERSLIDE DWNLD GAMMA -----------------------------*/

static SANE_Status
powerslide_dwnld_gamma (Powerslide_Scanner * scanner)
{
  DBG (DBG_proc, "powerslide_dwnld_gamma\n");

  return SANE_STATUS_GOOD;
}

/*------------------------- POWERSLIDE SET WINDOW -----------------------------*/

static SANE_Status
powerslide_set_window (Powerslide_Scanner * scanner)
{
  SANE_Status status = SANE_STATUS_GOOD;

  DBG (DBG_proc, "powerslide_set_window\n");

  return status;
}


/*------------------------- POWERSLIDE MODE SELECT -----------------------------*/

static SANE_Status
powerslide_mode_select (Powerslide_Scanner * scanner)
{

  SANE_Status status = SANE_STATUS_GOOD;

  DBG (DBG_proc, "powerslide_mode_select\n");

  return status;
}


/*------------------------- POWERSLIDE SCAN -----------------------------*/

static SANE_Status
powerslide_scan (Powerslide_Scanner * scanner, int start)
{
  SANE_Status status = SANE_STATUS_GOOD;

  DBG (DBG_proc, "powerslide_scan\n");


  return status;
}


/* --------------------------------------- POWERSLIDE WAIT SCANNER -------------------------- */


static SANE_Status
powerslide_wait_scanner (Powerslide_Scanner * scanner)
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
      /* FIXME */
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

/* ------------------- POWERSLIDE READER PROCESS INDEXED ------------------- */

static int
powerslide_reader_process_indexed (Powerslide_Scanner * scanner, FILE * fp)
{
  DBG (DBG_read, "reading %d lines of %d bytes/line (indexed)\n",
       scanner->params.lines, scanner->params.bytes_per_line);

  return 0;
}

/* --------------------------------- POWERSLIDE READER PROCESS ------------------------ */

static int
powerslide_reader_process (Powerslide_Scanner * scanner, FILE * fp)
{
  DBG (DBG_read, "reading %d lines of %d bytes/line\n", scanner->params.lines,
       scanner->params.bytes_per_line);

  return 0;
}



/* --------------------------------- READER PROCESS SIGTERM HANDLER  ------------ */


static RETSIGTYPE
reader_process_sigterm_handler (int signal)
{
  DBG (DBG_sane_info, "reader_process: terminated by signal %d\n", signal);

#ifdef HAVE_SANEI_SCSI_OPEN_EXTENDED
  sanei_scsi_req_flush_all ();	/* flush SCSI queue */
#else
  sanei_scsi_req_flush_all ();	/* flush SCSI queue */
#endif

  _exit (SANE_STATUS_GOOD);
}



/* ------------------------------ READER PROCESS ----------------------------- */


static int
reader_process ( void *data )	/* executed as a child process */
{
  int status;
  FILE *fp;
  Powerslide_Scanner * scanner;
  sigset_t ignore_set;
  struct SIGACTION act;

  scanner = (Powerslide_Scanner *)data;
  
  if (sanei_thread_is_forked ()) {

      close ( scanner->pipe );

      sigfillset (&ignore_set);
      sigdelset (&ignore_set, SIGTERM);
#if defined (__APPLE__) && defined (__MACH__)
      sigdelset (&ignore_set, SIGUSR2);
#endif
      sigprocmask (SIG_SETMASK, &ignore_set, 0);

      memset (&act, 0, sizeof (act));
      sigaction (SIGTERM, &act, 0);
  }
  
  DBG (DBG_sane_proc, "reader_process started\n");

  memset (&act, 0, sizeof (act));	/* define SIGTERM-handler */
  act.sa_handler = reader_process_sigterm_handler;
  sigaction (SIGTERM, &act, 0);

  fp = fdopen (scanner->reader_fds, "w");
  if (!fp)
    {
      return SANE_STATUS_IO_ERROR;
    }

  DBG (DBG_sane_info, "reader_process: starting to READ data\n");

  if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_LINE)
    status = powerslide_reader_process (scanner, fp);
  else if (scanner->device->inquiry_color_format & INQ_COLOR_FORMAT_INDEX)
    status = powerslide_reader_process_indexed (scanner, fp);
  else
    status = SANE_STATUS_UNSUPPORTED;

  fclose (fp);

  DBG (DBG_sane_info, "reader_process: finished reading data\n");

  return status;
}


/* ----------------------------- CLOSE PIPE ---------------------------------- */


static SANE_Status
close_pipe (Powerslide_Scanner * scanner)
{
  DBG (DBG_sane_proc, "close_pipe\n");

  if (scanner->pipe >= 0)
    {
      close (scanner->pipe);
      scanner->pipe = -1;
    }

  return SANE_STATUS_EOF;
}



/* ---------------------------- DO CANCEL ---------------------------------- */


static SANE_Status
do_cancel (Powerslide_Scanner * scanner)
{
  DBG (DBG_sane_proc, "do_cancel\n");

  scanner->scanning = SANE_FALSE;

  if (scanner->reader_pid != -1)
    {
      DBG (DBG_sane_info, "killing reader_process\n");
      sanei_thread_kill (scanner->reader_pid);
      sanei_thread_waitpid (scanner->reader_pid, 0);
      scanner->reader_pid = -1;
      DBG (DBG_sane_info, "reader_process killed\n");
    }

  return SANE_STATUS_CANCELLED;
}



/* --------------------------------------- SANE INIT ---------------------------------- */


SANE_Status
sane_init (SANE_Int * version_code, SANE_Auth_Callback __sane_unused__ authorize)
{
  char dev_name[PATH_MAX];
  size_t len;
  FILE *fp;

  DBG_INIT ();

  DBG (DBG_sane_init, "sane_init() build %d\n", BUILD);

  if (version_code)
    *version_code = SANE_VERSION_CODE (SANE_CURRENT_MAJOR, V_MINOR, BUILD);

  fp = sanei_config_open (POWERSLIDE_CONFIG_FILE);
  if (!fp)
    {
      DBG(1, "Could not open config file: %s: %s\n", POWERSLIDE_CONFIG_FILE, strerror(errno));
      return SANE_STATUS_INVAL;
    }

  sanei_usb_init();

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

      sanei_config_attach_matching_devices (dev_name, attach_scanner);
    }

  fclose (fp);

  return SANE_STATUS_GOOD;
}


/* ----------------------------------------- SANE EXIT ---------------------------------- */


void
sane_exit (void)
{
  Powerslide_Device *dev, *next;
  int i;

  DBG (DBG_sane_init, "sane_exit()\n");

  /* sanei_usb_exit(); */

  for (dev = first_dev; dev; dev = next)
    {
      next = dev->next;
      free (dev->devicename);
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
sane_get_devices (const SANE_Device *** device_list, SANE_Bool __sane_unused__ local_only)
{
  Powerslide_Device *dev;
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
  Powerslide_Device *dev;
  SANE_Status status;
  Powerslide_Scanner *scanner;
  int i, j;

  DBG (DBG_sane_init, "sane_open(%s)\n", devicename);

  if (devicename[0])		/* search for devicename */
    {
      for (dev = first_dev; dev; dev = dev->next)
	{
	  if (strcmp (dev->devicename, devicename) == 0)
	    {
	      break;
	    }
	}

      if (!dev)
	{
	  status = attach_scanner (devicename);
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


  if (!dev || !dev->usbname)
    {
      return SANE_STATUS_INVAL;
    }

  DBG (DBG_sane_init, "sane_open: using %s (usb %s)\n", devicename, dev->usbname);
  
  status = powerslide_open (dev);
  if (status != SANE_STATUS_GOOD)
    {
      return status;
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

  init_options (scanner);

  scanner->next = first_handle;	/* insert newly opened handle into list of open handles: */
  first_handle = scanner;

  *handle = scanner;

  return SANE_STATUS_GOOD;
}


/* ------------------------------------ SANE CLOSE --------------------------------- */


void
sane_close (SANE_Handle handle)
{
  Powerslide_Scanner *prev, *scanner;

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

  if (scanner->scanning)	/* stop scan if still scanning */
    {
      do_cancel (handle);
    }

  if (prev)
    {
      prev->next = scanner->next;
    }
  else
    {
      first_handle = scanner->next;
    }

  if (scanner->device->usb >= 0)
    {
      sanei_usb_close(scanner->device->usb);
      scanner->device->usb = -1;
    }

  free (scanner->gamma_table[0]);	/* free custom gamma tables */
  free (scanner->gamma_table[1]);
  free (scanner->gamma_table[2]);
  free (scanner->gamma_table[3]);
  free (scanner->val[OPT_MODE].s);
  free (scanner->val[OPT_SPEED].s);
  free (scanner->val[OPT_HALFTONE_PATTERN].s);

  scanner->bufsize = 0;

  free (scanner);		/* free scanner */
}


/* ---------------------------------- SANE GET OPTION DESCRIPTOR ----------------- */

const SANE_Option_Descriptor *
sane_get_option_descriptor (SANE_Handle handle, SANE_Int option)
{
  Powerslide_Scanner *scanner = handle;

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
  Powerslide_Scanner *scanner = handle;
  SANE_Status status;
  SANE_Word cap;
  SANE_String_Const name;

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
        default:
	  ;
	 /* nothing */
        } /* switch(option) */
    }				/* else */
  return SANE_STATUS_INVAL;
}


/* ------------------------------------ SANE GET PARAMETERS ------------------------ */


SANE_Status
sane_get_parameters (SANE_Handle handle, SANE_Parameters * params)
{
  DBG (DBG_sane_info, "sane_get_parameters\n");

  return SANE_STATUS_GOOD;
}


/* ----------------------------------------- SANE START --------------------------------- */


SANE_Status
sane_start (SANE_Handle handle)
{
  DBG (DBG_sane_init, "sane_start\n");

  return SANE_STATUS_GOOD;
}


/* -------------------------------------- SANE READ ---------------------------------- */


SANE_Status
sane_read (SANE_Handle handle, SANE_Byte * buf, SANE_Int max_len,
	   SANE_Int * len)
{
  Powerslide_Scanner *scanner = handle;
  ssize_t nread;

  *len = 0;

  nread = read (scanner->pipe, buf, max_len);
  DBG (DBG_sane_info, "sane_read: read %ld bytes\n", (long) nread);

  if (!(scanner->scanning))	/* OOPS, not scanning */
    {
      return do_cancel (scanner);
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
	  do_cancel (scanner);	/* we had an error, stop scanner */
	  return SANE_STATUS_IO_ERROR;
	}
    }

  *len = nread;

  if (nread == 0)		/* EOF */
    {
      do_cancel (scanner);

      return close_pipe (scanner);	/* close pipe */
    }

  return SANE_STATUS_GOOD;
}


/* ------------------------------------- SANE CANCEL -------------------------------- */


void
sane_cancel (SANE_Handle handle)
{
  Powerslide_Scanner *scanner = handle;

  DBG (DBG_sane_init, "sane_cancel\n");

  if (scanner->scanning)
    {
      do_cancel (scanner);
    }
}


/* -------------------------------------- SANE SET IO MODE --------------------------- */


SANE_Status
sane_set_io_mode (SANE_Handle handle, SANE_Bool non_blocking)
{
  Powerslide_Scanner *scanner = handle;

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
  Powerslide_Scanner *scanner = handle;

  DBG (DBG_sane_init, "sane_get_select_fd\n");


  return SANE_STATUS_GOOD;
}
