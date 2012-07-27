/* ----------------------------------------------------------------------------- */

/* sane - Scanner Access Now Easy.

    pie-usbdef.h: SUBS-definiton header file for PIE scanner driver.

    Copyright (C) 2012 Klaus Kaempf, based on pie-scsidef.h by Simon Munton

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

/* --------------------------------------------------------------------------------------------------------- */


#ifndef PIE_USBDEF_H
#define PIE_USBDEF_H

/*
 * Low level defines
 *
 */

/* from libieee1284 */
#define C1284_NSTROBE 0x01
#define C1284_NINIT   0x04

/* usb via ieee1284 */
#define PIE_IEEE1284_ADDR  0x00
#define PIE_IEEE1284_RESET 0x30
#define PIE_IEEE1284_SCSI  0xe0

/* usb bRequest */
#define PIE_USB_REQ_MANY 0x04 /* multiple bytes */
#define PIE_USB_REQ_ONE  0x0c /* single byte */

/* usb wValue aka register */
#define PIE_USB_SIZE_REG    0x0082
#define PIE_USB_SCSI_STATUS 0x0084
#define PIE_USB_SCSI_CMD    0x0085
#define PIE_USB_VAL_CTRL    0x0087
#define PIE_USB_VAL_DATA    0x0088

/* */
#define PIE_USB_WRITE (USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_OUT)
#define PIE_USB_READ  (USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_IN)

/* --------------------------------------------------------------------------------------------------------- */

#ifndef PIE_SCSIDEF_H
#include "pie-scsidef.h"
#endif

/*
 * Interesting links: http://lists.alioth.debian.org/pipermail/sane-devel/2011-December/029337.html
 *
 */

#define PIE_SCSI_CMD_LEN 6 /* all SCSI commands are 6 bytes */

/*
 * SCSI defines
 *
 */

#define PIE_SCSI_STATE_BAD  -1   /* usb protocol failure */
#define PIE_SCSI_STATE_OK   0x00 /* ok, done */
#define PIE_SCSI_STATE_LEN  0x01 /* read: send expected length */
#define PIE_SCSI_STATE_2    0x02 /* ?, ok for next cmd */
#define PIE_SCSI_STATE_BUSY 0x03 /* busy, wait for PIE_SCSI_STATE_OK */
#define PIE_SCSI_STATE_8    0x08 /* ?, ok for next cmd */

#define SCSI_CMD_SLIDE_CTRL 0xd1
static unsigned char slide_ctrlC[] = { SCSI_CMD_SLIDE_CTRL, 0x00, 0x00, 0x00, 0x04, 0x00 };
static scsiblk slide_ctrl = { slide_ctrlC, sizeof(slide_ctrlC) };
#define PIE_SLIDE_NEXT 0x04
#define PIE_SLIDE_PREV 0x05
#define PIE_SLIDE_LOAD 0x10
#define PIE_SLIDE_RELOAD 0x40

#define SCSI_CMD_READ_REVERSE 0x12
static unsigned char read_reverseC[] = { SCSI_CMD_READ_REVERSE, 0x00, 0x00, 0x00, 0x12, 0x00 };
static scsiblk read_reverse = { read_reverseC, sizeof(read_reverseC) };

#define SCSI_CMD_COPY_DATA 0x18
static unsigned char copy_dataC[] = { SCSI_CMD_COPY_DATA, 0x00, 0x00, 0x1d, 0x1a, 0x00 };
static scsiblk copy_data = { copy_dataC, sizeof(copy_dataC) };

#define SCSI_CMD_READ_GAIN_OFFSET 0xd7
static unsigned char read_gain_offsetC[] = { SCSI_CMD_READ_GAIN_OFFSET, 0x00, 0x00, 0x00, 0x67, 0x00 };
static scsiblk read_gain_offset = { read_gain_offsetC, sizeof(read_gain_offsetC) };

#define SCSI_CMD_SET_GAIN_OFFSET 0xdc
static unsigned char set_gain_offsetC[] = { SCSI_CMD_SET_GAIN_OFFSET, 0x00, 0x00, 0x00, 0x1d, 0x00 };
static scsiblk set_gain_offset = { set_gain_offsetC, sizeof(set_gain_offsetC) };

#define SCSI_CMD_READ_STATUS 0xdd
static unsigned char read_statusC[] = { SCSI_CMD_READ_STATUS, 0x00, 0x00, 0x00, 0x0c, 0x00 };
static scsiblk read_status = { read_statusC, sizeof(read_statusC) };

#endif /* PIE_USBDEF_H */
