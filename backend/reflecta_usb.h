/* 
 * File:   reflecta_0.h
 * Author: Jan Vleeshouwers
 *
 * Created on August 20, 2012, 7:47 PM
 */

#ifndef REFLECTA_0_H
#define	REFLECTA_0_H

/* Structures used by the USB functions */

struct Reflecta_Command_Status {
    SANE_Status sane_status;
    SANE_Byte senseKey; /* sense data, see Reflecta_Sense */
    SANE_Byte senseCode;
    SANE_Byte senseQualifier;
};

/* USB functions */

static SANE_Status ctrloutbyte(SANE_Int device_number, SANE_Int port, SANE_Byte b);
static SANE_Status ctrloutint(SANE_Int device_number, unsigned int size);
static SANE_Status ctrlinbyte(SANE_Int device_number, SANE_Byte* b);
static SANE_Status bulkin(SANE_Int device_number, SANE_Byte* data, unsigned int size);
static void commandScanner(SANE_Int device_number, SANE_Byte command[], SANE_Byte data[], SANE_Int size, struct Reflecta_Command_Status *status);
static void commandScannerRepeat(SANE_Int device_number, SANE_Byte command[], SANE_Byte data[], SANE_Int size, struct Reflecta_Command_Status *status, int repeat);
static SANE_Status interpretStatus(SANE_Byte status[]);

static SANE_Byte getByte(SANE_Byte* array, SANE_Byte offset);
static void setByte(SANE_Byte val, SANE_Byte* array, SANE_Byte offset);
static SANE_Int getShort(SANE_Byte* array, SANE_Byte offset);
static void setShort(SANE_Word val, SANE_Byte* array, SANE_Byte offset);
static SANE_Int getInt(SANE_Byte* array, SANE_Byte offset);
static void setInt(SANE_Word val, SANE_Byte* array, SANE_Byte offset);
static void getBytes(SANE_Byte* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
static void setBytes(SANE_Byte* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
static void getShorts(SANE_Word* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);
static void setShorts(SANE_Word* val, SANE_Byte* array, SANE_Byte offset, SANE_Byte count);

#endif	/* REFLECTA_0_H */

