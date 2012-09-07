/* 
 * File:   reflecta_specific.h
 * Author: Jan Vleeshouwers
 *
 * Created on August 20, 2012, 8:02 PM
 */

#ifndef REFLECTA_SPECIFIC_H
#define	REFLECTA_SPECIFIC_H

/* Settings for scan modes available to SANE */
/* In addition to those defined in sane.h */
#define SANE_VALUE_SCAN_MODE_RGBI    "Color+Infrared"

/* --------------------------------------------------------------------------
 *
 * DEVICE DEFINITION STRUCTURES
 * 
 * --------------------------------------------------------------------------*/

/* Options supported by the scanner */

enum Reflecta_Option
{
    OPT_NUM_OPTS = 0,
    /* ------------------------------------------- */
    OPT_MODE_GROUP,
    OPT_MODE,                   /* scan mode */
    OPT_BIT_DEPTH,              /* number of bits to encode a color */
    OPT_RESOLUTION,             /* number of pixels per inch */
    /* ------------------------------------------- */
    OPT_GEOMETRY_GROUP,
    OPT_TL_X,			/* top-left x */
    OPT_TL_Y,			/* top-left y */
    OPT_BR_X,			/* bottom-right x */
    OPT_BR_Y,			/* bottom-right y */
    /* ------------------------------------------- */
    OPT_ENHANCEMENT_GROUP,
    OPT_HALFTONE_PATTERN,       /* halftone pattern to use (see halftone_list) */
    OPT_THRESHOLD,              /* halftone threshold */
    OPT_SHARPEN,                /* create a sharper scan at the cost of scan time */
    OPT_SKIP_CALIBRATION,       /* skip the auto-calibration phase before the scan */
    OPT_FAST_INFRARED,          /* scan infrared channel faster but less accurate */
    /* ------------------------------------------- */
    OPT_ADVANCED_GROUP,
    OPT_PREVIEW,                /* scan a preview before the actual scan */
    OPT_SHADINGDATA,            /* output shading data */
    OPT_CCDMASK,                /* output CCD mask */
    OPT_EXPOSURE,               /* exposure times for R, G, B and I (a 4-element array) */
    OPT_GAIN,                   /* gain for R, G, B and I (a 4-element array)*/
    OPT_OFFSET,                 /* offset for R, G, B and I (a 4-element array) */
    /* must come last: */
    NUM_OPTIONS
};

/* Forward declaration */
struct Reflecta_Shading_Parameters;

/* Device characteristics of a Reflecta USB scanner */
struct Reflecta_Device_Definition
{
    struct Reflecta_Device_Definition *next;

    SANE_Device sane; 
      /* name = string like "libusb:001:006" == NO! this should be "CrystalScan 7200" or "ProScan 7200"...
       * vendor = "PIE/Reflecta"
       * model = "CrystalScan 7200" or "ProScan 7200"
       * type = "film scanner" */
    /* char *devicename; => sane->name */
    /* char *vendor; => sane->vendor */
    /* char *product; => sane->model */
    SANE_Word vendorId;
    SANE_Word productId;
      /* USB id's like 0x05e3 0x0145, see reflecta.conf */
    SANE_String version; /* INQUIRY productRevision */
    SANE_Byte model; /* INQUIRY model */
    
    /* Ranges for various quantities */
    SANE_Range dpi_range;
    SANE_Range x_range;
    SANE_Range y_range;
    SANE_Range exposure_range; /* Unit is a 8051 machine cycle, which is approximately 1 us. (Exactly: 12 cycles at 11.059 Mhz = 1.085 us.) */

    SANE_Range shadow_range;
    SANE_Range highlight_range;

    /* Enumerated ranges vor various quantities */
    SANE_String scan_mode_list[7]; /* names of scan modes (see saneopts.h) */
    SANE_Word bpp_list[5];	   /* bit depths  */
    SANE_String halftone_list[17]; /* names of the halftone patterns from the scanner */
    SANE_String speed_list[9];	   /* names of available speeds */

    /* Maximum resolution values */
    int maximum_resolution_x;	   /* maximum x-resolution */
    int maximum_resolution_y;	   /* maximum y-resolution */
    int maximum_resolution;

    /* Geometry */
    double scan_bed_width;	   /* flatbed width in inches (horizontal) */
    double scan_bed_height;	   /* flatbed height in inches (vertical) */
    int slide_top_left_x;          /* top-left location of slide w.r.t. scan bed */
    int slide_top_left_y;          /* top-left location of slide w.r.t. scan bed */
    double slide_width;	           /* transparency width in inches */
    double slide_height;           /* transparency length in inches */

    /* Integer and bit-encoded properties */
    int halftone_patterns;	   /* number of halftones supported */
    int color_filters;	           /* available colour filters: Infrared-0-0-OnePassColor-B-G-R-N */
    int color_depths;	           /* available colour depths: 0-0-16-12-10-8-4-1 */
    int color_formats;	           /* colour data format: 0-0-0-0-0-Index-Line-Pixel */
    int image_formats;	           /* image data format: 0-0-0-0-OKLine-BlkOne-Motorola-Intel */
    int scan_capabilities;         /* additional scanner features, number of speeds: PowerSave-ExtCal-0-FastPreview-DisableCal-[CalSpeeds=3] */
    int optional_devices;          /* optional devices: MultiPageLoad-?-?-0-0-TransModule1-TransModule-AutoDocFeeder */
    int enhancements;	           /* enhancements: unknown coding */
    int gamma_bits;	           /* no of bits used for gamma table */
    int fast_preview_resolution;   /* fast preview resolution */
    int minimum_highlight;	   /* min highlight % that can be used */
    int maximum_shadow;  	   /* max shadow % that can be used */
    int calibration_equation;      /* which calibration equation to use */
    int minimum_exposure;	   /* min exposure */
    int maximum_exposure;	   /* max exposure */

    int shading_info_count;	   /* number of shading information sets */
    struct Reflecta_Shading_Parameters *shading_parameters; /* array with shading data parameters */
};

typedef struct Reflecta_Device_Definition Reflecta_Device_Definition;

/* --------------------------------------------------------------------------
 *
 * CURRENTLY ACTIVE DEVICES
 * 
 * --------------------------------------------------------------------------*/

/* This structure holds information about an instance of an active scanner */

struct Reflecta_Scanner
{
    struct Reflecta_Scanner *next;
    struct Reflecta_Device_Definition *device; /* pointer to device definition */

    int device_number; /* scanner device number (as determined by USB) */
    
    /* SANE option descriptions and settings for this scanner instance */
    SANE_Option_Descriptor opt[NUM_OPTIONS]; 
    Option_Value val[NUM_OPTIONS];

    /* Scan state */
    struct Reflecta_Scanner_State state;
    SANE_Int scanning; /* true if busy scanning */
    SANE_Int cancel_request; /* if true, scanner should terminate a scan */
    
    /* Scan settings */
    struct Reflecta_Mode mode;
    struct Reflecta_Settings settings;
    struct Reflecta_Scan_Frame frame;
    SANE_Parameters scan_parameters; /* derived */

    /* Shading data and CCD-mask */
    SANE_Byte *shading_buffer;
    SANE_Byte *ccd_mask;
    
    /* Reading buffer */
    struct Reflecta_Read_Buffer buffer;
};

typedef struct Reflecta_Scanner Reflecta_Scanner;

/* Reflecta specific */

static void reflecta_initialize_device_definition (Reflecta_Device_Definition* dev, Reflecta_Scanner_Properties* inq, const char* devicename, SANE_Word vendor_id, SANE_Word product_id, SANE_Int devnr);
static SANE_Status reflecta_usb_get_device_number (SANE_String_Const devname, SANE_Int* device_number);
static SANE_Status reflecta_usb_check_vendor_product (SANE_String_Const devname, SANE_Int device_number);
static void reflecta_copy_inquiry_values (Reflecta_Device_Definition * dev, Reflecta_Scanner_Properties* inq);
static void reflecta_print_inquiry (Reflecta_Device_Definition * dev);
static void reflecta_do_inquiry (Reflecta_Device_Definition * dev, int sfd, Reflecta_Scanner_Properties* inq);
static void reflecta_dump_buffer (int level, unsigned char *buf, int n);
static SANE_Status init_options (Reflecta_Scanner * scanner);
static size_t max_string_size (SANE_String_Const strings[]);
static SANE_Status reflecta_on_cancel (Reflecta_Scanner * scanner);
static SANE_Status checkWarmedUp(SANE_Int devNr, SANE_Int maxWait);
static SANE_Status reflecta_parse_config_line(const char* config_line, SANE_Word* vendor_id, SANE_Word* product_id, SANE_Word* model_number);
static SANE_Status reflecta_supported_device_list_add(SANE_Word vendor_id, SANE_Word product_id, SANE_Word model_number);
static SANE_Bool reflecta_supported_device_list_contains(SANE_Word vendor_id, SANE_Word product_id, SANE_Word model_number);
static SANE_Status find_device_callback (const char *devicename);
static void reflecta_get_halftones (Reflecta_Device_Definition * dev, int sfd);


#endif	/* REFLECTA_SPECIFIC_H */

