/* sane - Scanner Access Now Easy.

   pie_usb.h

   Copyright (C) 2012 Michael Rickmann <mrickma@gwdg.de>
   
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
*/

/** @file pie_usb.h
 * This file contains definitions for the USB code for
 * PIE SF film scanners. For documentation of the code
 * see pie.c. Here the more general things are explained.
 *
 * Exposure time (texp) and gain calibration is done in software for
 * each of the R, G, B channels between two custom SCSI commands.
 * The first 0xd7 named PIE_READ_CALIBRATION reads what the
 * scanner suggest, the second 0xdc named PIE_WRITE_CALIBRATION
 * sends what has been calculated. Commands sent during a scan:
 * - 1-3) SET_EXP_TIME         x3
 * - 4-6) SET_HIGHLIGHT_SHADOW    x3
 * - 5) READ_CAL_INFO
 * - 6) SET_SCAN_FRAME
 *      - 7) PIE_READ_CALIBRATION
 *      - 8) PIE_WRITE_CALIBRATION
 *      - 9) MODE
 *      - 10) SCAN
 *              - 11) READ            1 line
 *              - 12) TEST_UNIT_READY
 *              - 13) READ            13 lines
 *              - 14) PIE_READ_CALIBRATION
 *              - 15) PIE_WRITE_CALIBRATION
 *              - 16) READ            31 lines
 *      - 17) COPY                 sensors
 * - 18) PARAM
 * - 19) READ                    image
 *
 * Steps 7 to 17 are done for calibration. Steps 11 to 16
 * are only done for full calibration, i.e. for
 * steps 7, 8 an initial set or the result of the last full
 * calibration is used. Full calibration mode is termed
 * OPM_QUALITY below, the short sequence may be OPM_PREVIEW
 * or OPM_SKIPCAL.
 *
 * The best documentation of what the 0xd7 and 0xdc commands
 * contain is found in Jan Vleeshouwers post "Reflecta Crystalscan /
 * ProScan 7200 update" at sane-devel Tue Dec 20 22:26:52 UTC 2011:
 * http://lists.alioth.debian.org/pipermail/sane-devel/2011-December/029337.html
 * Jan disassembled a PIE ROM.
 *
 * Jan named the 0xd7 command READ GAIN OFFSET and the 0xdc one
 * SET GAIN OFFSET. Here they are phrased PIE_READ_CALIBRATION and
 * PIE_WRITE_CALIBRATION because the Windows programs do all calibration
 * by changing exposure time (texp below, Jan's Timer count) and gain
 * values. In USB snoops, the offsets are the same in read to write.
 *
 * PIE_USB_Calibration_Read contains what is read for calibration.
 *
 * PIE_USB_Calibration_Send contains what is written for calibration.
 */

#ifndef PIE_USB_H
#define PIE_USB_H


/* ----------------------------------------------------------------
 * defines for the SCSI over USB interface
 */

#define BULKIN_MAXSIZE          0xFFF0
#define BUFFER_MAXSIZE          0x7F000
#define AVERAGE_CAL_LINES       13

/* USB control message values */

#define REQUEST_TYPE_IN         (USB_TYPE_VENDOR | USB_DIR_IN)
#define REQUEST_TYPE_OUT        (USB_TYPE_VENDOR | USB_DIR_OUT)

/* the following have a taste of genesys */
#define REQUEST_REGISTER        0x0c
#define REQUEST_BUFFER          0x04

#define VALUE_BUFFER            0x82
#define VALUE_READ_REGISTER     0x84
#define VALUE_WRITE_REGISTER    0x85
#define VALUE_INIT_1            0x87
#define VALUE_INIT_2            0x88
#define VALUE_BUF_ENDACCESS     0x8c
#define VALUE_GET_REGISTER      0x8e
/* fortunately 0, index of USB commands */
#define INDEX                   0x00

/* e.g.
   control  0x40 0x0c 0x87 0x00 len     1 wrote 0x04
            REQUEST_TYPE_OUT
                 REQUEST_REGISTER
                      VALUE_INIT_1
                           INDEX
*/

/**
 * @brief USB control sequence element
 */
typedef struct PIE_USB_Value_Data
{
  SANE_Byte bValue;     /**< value to send */
  SANE_Byte bData;      /**< data to send */
} PIE_USB_Value_Data;


/**
 * @brief Modes affecting calibration and quality
 *
 * - Before the first scan after opening,
 * - during OPM_QUALITY scanning
 * - and after having changed from  OPM_QUALITY to some other mode
 *
 * the full calibration sequence has to be done by the code !!!
 * The actual values sent may be different for different scanner models.
 */
enum PIE_USB_Operation_Mode
{
  OPM_PREVIEW,          /**< low quality preview */
  OPM_SKIPCAL,          /**< low quality, full calibration is skipped */
  OPM_QUALITY           /**< high quality, always fully calibrate before scan */
};


/** @brief slowdown values for calibration
 *
 * No need yet to make them model specific.
 * Construct a slope SLOW_HEIGHT / SLOW_LENGTH beginning
 * at SLOW_START bytes per line, for infrared result is lower.
 * This quirk seems to satisfy a limitation of the physical
 * SCSI to USB interface within the scanner
 */
#define SLOW_START 26700.0
#define SLOW_LENGTH 5340.0
#define SLOW_HEIGHT 8.0
#define SLOW_IRED -2

/**
 * @brief Infrared extension, not critical
 */
typedef struct PIE_USB_cal_ired
{
  uint16_t texp;                /**< infrared exposure time */
  unsigned char offset;         /**< infrared offset */
  unsigned char zero;
  unsigned char gain;           /**< infrared gain */
} __attribute__ ((__packed__)) PIE_USB_cal_ired;


/**
 * @brief Read by vendor specific 0xd7 SCSI command "PIE_READ_CALIBRATION"
 */
typedef struct PIE_USB_Calibration_Read
{
  unsigned char zero_1[54];
  /* values describing internal calibration */
  uint16_t illumination[3];     /**< R G B targeted illumination, [0] and [1] may be zero */
  uint16_t texp[3];             /**< R G B exposure times */
  unsigned char offset[3];      /**< R G B offsets */
  unsigned char zero_2[3];
  /** for a Reflecta ProScan 7200 in quality mode
   *  the read R G B gain values are usually one less than the written ones */
  unsigned char gain[3];
  unsigned char some_time;      /**< slow down value, may correspond to some_time[0] sent */
  uint16_t t_min;               /**< ?? minimal exposure time ?? */
  unsigned char no_idea[20];
  PIE_USB_cal_ired infrared;    /**< infrared extension */
} __attribute__ ((__packed__)) PIE_USB_Calibration_Read;

/**
 * @brief Written by vendor specific 0xdc SCSI command "PIE_WRITE_CALIBRATION"
 */
typedef struct PIE_USB_Calibration_Send
{
  /** header */
  unsigned char scsi_cmd[6];
  /** texp values are inversely correlated with gain values below,
      marked differences between different types of scanners,
      relatively high in quality mode, R G B */
  uint16_t texp[3];
  /** R G B offset usually copied from calibration read */
  unsigned char offset[3];
  /** usually copied from calibration read */
  unsigned char zero_2[3];
  /** R G B gain values inversely correlate with texp values */
  unsigned char gain[3];
  unsigned char some_time[3];
  /* infrared bytes are usually copied from calibration read */
  PIE_USB_cal_ired infrared;
} __attribute__ ((__packed__)) PIE_USB_Calibration_Send;


/**
 * @brief Handle calibration in software
 */
typedef struct PIE_USB_Calibration_Set
{
  uint16_t texp[3];             /**< exposure times, currently only RGB */
  uint16_t texp_max;            /**< maximum, needed for slow down correction */
  unsigned char gain[3];        /**< gain, currently only R, G, B */
} PIE_USB_Calibration_Set;


/**
 * @brief Hold data from the last calibration
 *
 * Initially loaded with default sets for exposure and gain but not shading
 */
typedef struct PIE_USB_Calibration
{
  int brightness[3];            /**< at next calibration tune scanner to this illumination */
  int *shades;                  /**< RGBI, summed up calibration lines, dimension [4][pixels] */
  unsigned char *sensors;       /**< array of 0x00 or 0x70, 0x00 means sensor element active */
  int mean_shade[4];            /**< RGBI, mean brightness of calibration lines */
  /** illumination targets, target_shade[2] always sent from scanner, ir ?? currently faked */
  int target_shade[4];
  PIE_USB_Calibration_Set cal_hiqual;   /**< calculated set for quality mode */
  PIE_USB_Calibration_Set cal_normal;   /**< calculated set for normal mode */
} PIE_USB_Calibration;


/**
 * @brief Describe a PIE USB scanner
 *
 * There are several different PIE SF scanners which are different from each other.
 * The following is an approach to hold model specific values.
 *
 * @todo
 * Define some of the constraints constant
 */
typedef struct PIE_USB_Model
{
  SANE_String vendor;
  SANE_String model;
  SANE_Byte model_id;           /**< for subtyping models with the same USB id */
  SANE_Word flags;              /**< hacks are needed for this scanner */
  unsigned char op_mode[3];     /**< operation modes for byte 0x09 of MODE command */
  SANE_Int default_brightness;  /**< default targeted illumination */
  double gain_const[3];         /**< brightness = f * exp(gain_const * gain^2) * time, !! critical !! */
  /** normal mode only,
     brightness = offs_factor * f * exp(gain_const * gain^2) * time - (offs_factor - 1) * 65536
     , different between types of scanners, !! critical !! */
  double offs_factor[3];
  int gain_min;                 /**< 6 bit, put a sensible default here, e.g. 0x10 */
  /** 6 bit, quality mode only, put a sensible default here, e.g. 0x28, higher values result in noisy images */
  int gain_hiqual_max;
  int texp_normal_max;          /**< maximal exposure time seems to be a 12 bit number */
  /** quality mode only, >= 2, higher values lead to more calibration by exposure time, about 10 is typical */
  int gain_hiqual_part;
  /** initial calibration for quality mode, not critical, can be left as the default
     if calibration is not reliable or omitted */
  PIE_USB_Calibration_Set default_hiqual;
  /** similar as default_hiqual, normal mode, not critical */
  PIE_USB_Calibration_Set default_normal;
} PIE_USB_Model;

/**
 * @brief Distinguish between different scanner models
 */
typedef struct PIE_USB_Device_Entry
{
  SANE_Word vendor;             /**< USB vendor identifier */
  SANE_Word product;            /**< USB product identifier */
  PIE_USB_Model *model;         /**< Scanner model information */
} PIE_USB_Device_Entry;

/* model peculiarities going into the flags value */
#define PIE_USB_FLAG_MIRROR_IMAGE       (1 << 0)        /**> mirror image and X-window settings */
#define PIE_USB_FLAG_XRES_FAKE          (1 << 1)        /**> highest X-resolution has to be emulated */

/** constant gamma as guessed from Silverfast (tm) scans */
#define CONST_GAMMA     0.454545455

#endif /* not PIE_USB_H */

