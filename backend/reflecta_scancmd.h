/* 
 * File:   reflecta_scancmd.h
 * Author: Jan Vleeshouwers
 *
 * Created on August 20, 2012, 7:56 PM
 */

#ifndef REFLECTA_SCANCMD_H
#define	REFLECTA_SCANCMD_H

/* =========================================================================
 * 
 * Data-structures used by scanner commands
 * 
 * ========================================================================= */

/* Data returned from a SCSI INQUIRY command. */
/* See SCSI-2 p141 table 45, 46, 47 */
/* 2-byte short ints are represented by 4-byte SANE_Int types*/
struct Reflecta_Scanner_Properties {
    SANE_Byte deviceType; /* 0x06 = scanner */
    SANE_Byte additionalLength; /* including this byte: 0xb4 = 180, so total structure 184 bytes */
    SANE_Char vendor[9]; /* actually 8 bytes, not null-terminated ...PIE     ... */
    SANE_Char product[17]; /* actually 16 bytes, null-terminated ...SF Scanner... */
    SANE_Char productRevision[5]; /* actually 4 bytes, not null-terminated ...1.70... */
    /* 1st Vendor-specific block, 20 bytes, see pie_get_inquiry_values(), partially: */
    SANE_Int maxResolutionX; /* 7200 maximum scan resolution in x direction */
    SANE_Int maxResolutionY; /* 7200 maximum scan resolution in y direction */
    SANE_Int maxScanWidth; /* 10680 flatbed_max_scan_width (& calibration block size) */
    SANE_Int maxScanHeight; /* 6888 flatbed_max_scan_height */
    SANE_Byte filters; /* 0x9e = 10011110 ?-0-0-OnePassColor-B-G-R-N => additional infrared? */
    SANE_Byte colorDepths; /* 0x35 = 00110101 0-0-16-12-10-8-4-1 */
    SANE_Byte colorFormat; /* 0x07 = 00000111 0-0-0-0-0-Index-Line-Pixel */
    SANE_Byte imageFormat; /* 0x09 = 00001001 0-0-0-0-OKLine-BlkOne-Motorola-Intel */
    SANE_Byte scanCapability;
      /* 0x4b = 01001011 PowerSave-ExtCal-0-FastPreview-DisableCal-[CalSpeeds=3]
       * PowerSave: no
       * ExtCal: yes => 
       * FastPreview: no
       * DisableCal: yes => can calibration be disabled?
       * CalSpeeds: 3 => 1 line, 13 lines, 31 lines */
    SANE_Byte optionalDevices;
      /* 0x61 = 01100001 MultiPageLoad-?-?-0-0-TransModule1-TransModule-AutoDocFeeder => additional? 
       * MultiPageLoad: no
       * ?: yes
       * ?: yes
       * TransModule1: no
       * TransModule: no
       * AutoDocFeeder: yes */
    SANE_Byte enhancements; /* 0x02 = no info in pie.c */
    SANE_Byte gammaBits; /* 0x0c = 00001100 = 12 ? used when downloading gamma table ... does not happen */
    SANE_Byte lastFilter; /* 0x00 = ? no info in pie.c, not used */
    SANE_Int previewScanResolution; /* 0x2c01 = 300 fast preview scan resolution */
    /* Reserved (56-95) */
    /* SANE_Byte div_56[40]; */
    /* 2nd vendor specific block (36 bytes at offset 96) */
    SANE_Char firmwareVersion[5]; /* actually 4 bytes, not null terminated "1.05" */
    SANE_Byte halftones; /* 0x08 = halftones (4 LSbits) = 00001000 ? */
    SANE_Byte minumumHighlight; /* 0x64 = 100 */
    SANE_Byte maximumShadow; /* 0x64 = 100 */
    SANE_Byte calibrationEquation; /* 0x01 ? see pie_perform_cal() */
    SANE_Int maximumExposure; /* 0xc409 = 2500 (units?) */
    SANE_Int minimumExposure; /* 0x6400 = 100 (units?) */
    SANE_Int x0; /* 0xd002 = 720 transparency top left x */
    SANE_Int y0; /* 0x8604 = 1158 transparency top left y */
    SANE_Int x1; /* 0xbc10 = 4284 transparency bottom right x */
    SANE_Int y1; /* 0xc015 = 5568 transparency bottom right y */
    SANE_Int model; /* 0x3000 => model number */
    /* SANE_Int div_118; 0x0000 meaning? */
    SANE_Char production[24]; /* null terminated */
    SANE_Byte signature[40]; /* null terminated */
};

struct Reflecta_Sense {
    /* 14 bytes according to SCSI-2 p158, table 67 (p469 ASC/Q alphabetically) */
    SANE_Byte errorCode; /* 0x70 or 0x71 */
    SANE_Byte segment;
    SANE_Byte senseKey; /* sense key is actually this value & 0x0F - table 69 */
    SANE_Byte info[4];
    SANE_Byte addLength; /* should be 0x07 (remaining struct length including this byte) */
    SANE_Byte cmdInfo[4]; /* command specific information */
    SANE_Byte senseCode; /* abbreviated ASC - table 71 */
    SANE_Byte senseQualifier; /* abbreviated ASCQ - table 71 */
};

struct Reflecta_Scanner_State {
    SANE_Byte buttonPushed; /* 0x01 if pushed */
    SANE_Byte warmingUp; /* 0x01 if warming up, 0x00 if not */
    SANE_Byte scanning; /* bit 6 set if SCAN active, bit 7 motor direction inverted (not analysed in detail) */
};

struct Reflecta_Scan_Parameters {
    SANE_Int width; /* Number of pixels on a scan line */
    SANE_Int lines; /* Number of lines in the scan. Value depends on color format. */
    SANE_Int bytes; /* Number of bytes on a scan line. Value depends on color format. */
    SANE_Byte filterOffset1; /* 0x08 in the logs, but may also be set to 0x16, they seem to be used in “line”-format only. */
    SANE_Byte filterOffset2; /* 0x08 in the logs, but may also be set to 0x16, they seem to be used in “line”-format only. */
    SANE_Int period; /* Seems to be unused */
    SANE_Int scsiTransferRate; /* Don't use, values cannot be trusted */
    SANE_Int availableLines; /* The number of currently available scanned lines. Value depends on color format. Returns a value >0 if PARAM is called while scanning is in progress */
    SANE_Byte motor; /* Motor direction in bit 0 */
};

struct Reflecta_Mode {
    /* SANE_Byte size; of remaining data, not useful */
    SANE_Int resolution; /* in dpi */
    SANE_Byte passes;
      /* 0x80 = One pass color; 0x90 = One pass RGBI; 
       * bit 7 : one-pass-color bit (equivalent to RGB all set?)
       * bit 6 & 5: unused
       * bit 4 : Infrared
       * bit 3 : Blue
       * bit 2 : Green
       * bit 1 : Red
       * bit 0: Neutral (not supported, ignored) */
    SANE_Byte colorDepth;
      /* 0x04 = 8, 0x20 = 16 bit
       * bit 7 & 6 : 0 (unused)
       * bit 5 : 16 bit
       * bit 4 : 12 bit
       * bit 3 : 10 bit
       * bit 2 : 8 bit
       * bit 1 : 4 bit
       * bit 0 : 1 bit */
    SANE_Byte colorFormat;
      /* 0x04 = index, cf. INQUIRY
       * bit 7-3 : 0 (unused)
       * bit 2 : Index = scanned data are lines preceeded by a two-byte index, 'RR', 'GG', 'BB', or 'II'
       * bit 1 : Line =  scanned data are lines in RGBI order
       * bit 0 : Pixel = scanned data are RGBI-pixels */
    SANE_Byte byteOrder; /* 0x01 = Intel; only bit 0 used */
    SANE_Bool sharpen; /* byte 9 bit 1 */
    SANE_Bool skipCalibration; /* byte 9 bit 3 */
    SANE_Bool fastInfrared; /* byte 9 bit 7 */
      /* bit 7 : “fast infrared” flag
       * bit 6,5,4 : 0 (unused)
       * bit 3 : “skip calibration” flag (skip collecting shading information)
       * bit 2 : 0 (unused)
       * bit 1 : “sharpen” flag (only effective with fastInfrared off, one-pass-color and no extra BADF-entries)
       * bit 0 : 0 (unused) */
    SANE_Byte halftonePattern; /* 0x00 = no halftone pattern */
    SANE_Byte lineThreshold; /* 0xFF = 100% */
};

struct Reflecta_Settings {
    SANE_Int saturationLevel[3];
      /* The average pixel values for the three colors Red, Green and Blue,
       * which are the result of optimizing the Timer 1 counts so that Red and
       * Blue values are least 90% of full scale (58981) and the Green value is
       * at least 80% (52428). These levels are only determined during warming up. */
    SANE_Int exposureTime[4];
      /* Optimized exposure times for Red, Green and Blue. The exposure times are
       * Timer 1 counts which define when Timer 1 interrupts. These values are
       * only determined at startup.
       * Exposure time for Infrared. The value is optimized and set at startup
       * with the other exposure times. Quite often, it is subsequently reset to
       * a default value (0x0B79). */
    SANE_Word offset[4];
      /* Optimized offsets for Red, Green and Blue. See above. These values are
       * also updated before outputting the CCD-mask.
       * Element 4 is offset for Infrared. */
    SANE_Word gain[4];
      /* Optimized gains for Red, Green and Blue. See the remark for
       * exposureTime above. Element 4 is gain for Infrared. */
    SANE_Byte light;
      /* Current light level. The stability of the light source is tested during
       * warming up. The check starts with a light value 7 or 6, and decrements
       * it when the light warms up. At a light value of 4, the scanner produces
       * stable scans (i.e. successive “white” scan values don't differ more
       * than 0x200). */
    SANE_Int minimumExposureTime;
      /* Fixed value: 0x0b79 (2937) */
    SANE_Byte extraEntries; 
    SANE_Byte doubleTimes;
      /* Originally 20 unused bytes (uninitialized memory)
       * To complete the mapping to the Reflecta_Settings_Condensed struct,
       * the last two bytes are given an explicit meaning. */
    /* SANE_Int exposureTimeIR; */
    /* SANE_Byte offsetIR; */
    /* SANE_Byte gainIR; */
};

/* Not used, Reflecta_Settings contains the same fields (after a bit of juggling) */
struct Reflecta_Settings_Condensed {
    SANE_Int exposureTime[4]; /* => Reflecta_Settings.exposureTime */
    SANE_Byte offset[4]; /* => Reflecta_Settings.offset */
    SANE_Byte gain[4]; /* => Reflecta_Settings.gain */
    SANE_Byte light; /* => Reflecta_Settings.light */
    SANE_Byte extraEntries; /* => Reflecta_Settings.extraEntries */
    SANE_Byte doubleTimes; /* => Reflecta_Settings.doubleTimes */
};

struct Reflecta_Halftone_Pattern {
    SANE_Int code; /* 0x91 */
    /*TODO */
};

struct Reflecta_Scan_Frame {
    SANE_Int code; /* 0x92 */
    SANE_Int size; /* number of bytes in rest of structure */
    SANE_Int index; /* scan frame index (0-7) */
    SANE_Int x0; /* top left, is origin */
    SANE_Int y0;
    SANE_Int x1; /* bottom right */
    SANE_Int y1;
};

struct Reflecta_Exposure_Time_Color {
    SANE_Int filter; /* color mask 0x02, 0x04 or 0x08 for R, G, B */
    SANE_Int value; /* relative exposure time 0 - 100 */
};

struct Reflecta_Exposure_Time {
    SANE_Int code; /* 0x93 */
    SANE_Int size; /* number of bytes in rest of structure */
    struct Reflecta_Exposure_Time_Color color[3]; /* not all elements may actually be used */
};

struct Reflecta_Highlight_Shadow_Color {
    SANE_Int filter; /* color mask 0x02, 0x04 or 0x08 for R, G, B */
    SANE_Byte highlightValue; /* range unknown, value is not used */
    SANE_Byte shadowValue; /* range unknown, value is not used */   
};

struct Reflecta_Highlight_Shadow {
    SANE_Int code; /* 0x94 */
    SANE_Int size; /* number of bytes in rest of structure */
    struct Reflecta_Highlight_Shadow_Color color[3];
};

struct Reflecta_Shading_Parameters_Info {
    SANE_Byte type; /* 0x00, 0x08, 0x10, 0x20 */
    SANE_Byte sendBits; /* 0x10 = 16 */
    SANE_Byte recieveBits; /* 0x10 = 16 */
    SANE_Byte nLines; /* 0x2D = 45 */
    SANE_Int pixelsPerLine; /* 0x14dc = 5340 */  
};

struct Reflecta_Shading_Parameters {
    SANE_Int code; /* 0x95 */
    SANE_Int size; /* number of bytes in rest of structure (0x1c=28) */
    SANE_Byte calInfoCount; /* number of individual info structures (=0x04) */
    SANE_Byte calInfoSize; /* size of individual info structure (=0x06) */
    SANE_Int div_6; /* 0x0004, meaning not clear */
    struct Reflecta_Shading_Parameters_Info cal[4];
};

typedef struct Reflecta_Scanner_Properties Reflecta_Scanner_Properties;

/* Scanner commands */

void cmdIsUnitReady(SANE_Int device_number, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdGetSense(SANE_Int device_number, struct Reflecta_Sense* sense, struct Reflecta_Command_Status *status);

void cmdGetHalftonePattern(SANE_Int device_number, SANE_Int index, struct Reflecta_Halftone_Pattern* pattern, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdGetScanFrame(SANE_Int device_number, SANE_Int index, struct Reflecta_Scan_Frame* frame, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdGetRelativeExposureTime(SANE_Int device_number, SANE_Int colorbits, struct Reflecta_Exposure_Time* time, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdGetHighlightShadow(SANE_Int device_number, SANE_Int colorbits, struct Reflecta_Highlight_Shadow* hgltshdw, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdGetShadingParameters(SANE_Int device_number, SANE_Int index, struct Reflecta_Shading_Parameters* shading, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdGetScannedLines(SANE_Int device_number, SANE_Byte* data, SANE_Int lines, SANE_Int size, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdSetHalftonePattern(SANE_Int device_number, SANE_Int index, struct Reflecta_Halftone_Pattern* pattern, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdSetScanFrame(SANE_Int device_number, SANE_Int index, struct Reflecta_Scan_Frame* frame, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdSetRelativeExposureTime(SANE_Int device_number, struct Reflecta_Exposure_Time* time, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdSetHighlightShadow(SANE_Int device_number, struct Reflecta_Highlight_Shadow* hgltshdw, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdSetCCDMask(SANE_Int device_number, SANE_Byte colorbits, SANE_Byte* mask, struct Reflecta_Command_Status *status, SANE_Int repeat);

/*
void cmdPrepareHalftonePattern(SANE_Int device_number, SANE_Int index, struct Reflecta_Command_Status *status);
void cmdPrepareScanFrame(SANE_Int device_number, SANE_Int index, struct Reflecta_Command_Status *status);
void cmdPrepareRelativeExposureTime(SANE_Int device_number, SANE_Int colorbits, struct Reflecta_Command_Status *status);
void cmdPrepareHighlightShadow(SANE_Int device_number, SANE_Int colorbits, struct Reflecta_Command_Status *status);
void cmdPrepareShadingParameters(SANE_Int device_number, struct Reflecta_Command_Status *status);
*/

void cmdGetScanParameters(SANE_Int device_number, struct Reflecta_Scan_Parameters* parameters, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdDoInquiry(SANE_Int device_number, struct Reflecta_Scanner_Properties* inq, SANE_Byte size, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdSetMode(SANE_Int device_number, struct Reflecta_Mode* mode, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdGetCCDMask(SANE_Int device_number, SANE_Byte* mask, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdGetMode(SANE_Int device_number, struct Reflecta_Mode* mode, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdStartScan(SANE_Int device_number, struct Reflecta_Command_Status *status, SANE_Int repeat);
void cmdStopScan(SANE_Int device_number, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdSetScanHead(SANE_Int device_number, SANE_Int mode, SANE_Int steps, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdGetGainOffset(SANE_Int device_number, struct Reflecta_Settings* settings, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdSetGainOffset(SANE_Int device_number, struct Reflecta_Settings* settings, struct Reflecta_Command_Status *status, SANE_Int repeat);

void cmdGetState(SANE_Int device_number, struct Reflecta_Scanner_State* state, struct Reflecta_Command_Status *status, SANE_Int repeat);

/* Utility */

void setCommand(SANE_Byte* command_bytes, SANE_Byte command, SANE_Word size);

SANE_String senseDescription(struct Reflecta_Sense* sense);

#endif	/* REFLECTA_SCANCMD_H */

