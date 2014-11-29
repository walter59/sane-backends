/*
 * File:   pieusb_usb.h
 * Author: Jan Vleeshouwers
 *
 * Created on August 20, 2012, 7:47 PM
 */

#ifndef PIEUSB_USB_H
#define	PIEUSB_USB_H

/* Structures used by the USB functions */

struct Pieusb_Command_Status {
    SANE_Status sane_status;
    SANE_Byte senseKey; /* sense key: see Pieusb_Sense */
    SANE_Byte senseCode; /* sense code */
    SANE_Byte senseQualifier; /* sense code qualifier */
};

void commandScanner(SANE_Int device_number, SANE_Byte command[], SANE_Byte data[], SANE_Int size, struct Pieusb_Command_Status *status);

SANE_Byte getByte(SANE_Byte* array, SANE_Byte offset);
void setByte(SANE_Byte val, SANE_Byte* array, SANE_Byte offset);
SANE_Int getShort(SANE_Byte* array, SANE_Byte offset);
void setShort(SANE_Word val, SANE_Byte* array, SANE_Byte offset);
SANE_Int getInt(SANE_Byte* array, SANE_Byte offset);
void setInt(SANE_Word val, SANE_Byte* array, SANE_Byte offset);
void getBytes(SANE_Byte* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
void setBytes(SANE_Byte* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
void getShorts(SANE_Word* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
void setShorts(SANE_Word* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);

#endif	/* PIEUSB_USB_H */

