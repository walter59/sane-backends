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

#define PIE_SCSI_CMD_LEN 6 /* all SCSI commands are 6 bytes */

/*
 * SCSI defines
 *
 */

#define PIE_SCSI_STATE_BAD  -1   /* usb protocol failure */
#define PIE_SCSI_STATE_OK   0x00 /* ok, done */
#define PIE_SCSI_STATE_LEN  0x01 /* read: send expected length */
#define PIE_SCSI_STATE_BUSY 0x03 /* busy, wait for PIE_SCSI_STATE_OK */

/* some kind of 'ping' ? */
#define SCSI_CMD_PING 0xdd
static unsigned char pingC[] = { SCSI_CMD_PING, 0x00, 0x00, 0x00, 0x0c, 0x00 };
static scsiblk ping = { pingC, sizeof(pingC) };



#endif /* PIE_USBDEF_H */
