/*
 * INCLUDE FILE FOR REFLECTA.C
 */

#include "reflecta_specific.h"


/* --------------------------------------------------------------------------
 *
 * SPECIFIC REFLECTA 
 * 
 * --------------------------------------------------------------------------*/

/* Scanner settings for colors to scan */
#define SCAN_ONE_PASS_COLOR          0x80
#define SCAN_FILTER_INFRARED         0x10
#define SCAN_FILTER_BLUE             0x08
#define SCAN_FILTER_GREEN            0x04
#define SCAN_FILTER_RED              0x02
#define SCAN_FILTER_NEUTRAL          0x01

/* Settings for color depth of scan */
#define SCAN_COLOR_DEPTH_16          0x20
#define SCAN_COLOR_DEPTH_12          0x10
#define SCAN_COLOR_DEPTH_10          0x08
#define SCAN_COLOR_DEPTH_8           0x04
#define SCAN_COLOR_DEPTH_4           0x02
#define SCAN_COLOR_DEPTH_1           0x01

/* Settings for format of the scanned data */
#define SCAN_COLOR_FORMAT_INDEX      0x04
#define SCAN_COLOR_FORMAT_LINE       0x02
#define SCAN_COLOR_FORMAT_PIXEL      0x01

/* Settings for byte order */
#define SCAN_IMG_FMT_OKLINE          0x08
#define SCAN_IMG_FMT_BLK_ONE         0x04
#define SCAN_IMG_FMT_MOTOROLA        0x02
#define SCAN_IMG_FMT_INTEL           0x01

/* Settings for scanner capabilities */
#define SCAN_CAP_PWRSAV              0x80
#define SCAN_CAP_EXT_CAL             0x40
#define SCAN_CAP_FAST_PREVIEW        0x10
#define SCAN_CAP_DISABLE_CAL         0x08
#define SCAN_CAP_SPEEDS              0x07

/* Available scanner options */
#define SCAN_OPT_DEV_MPCL            0x80
#define SCAN_OPT_DEV_TP1             0x04
#define SCAN_OPT_DEV_TP              0x02
#define SCAN_OPT_DEV_ADF             0x01

/* Options */
#define SANE_NAME_EXPOSURE           "exposure-time"
#define SANE_TITLE_EXPOSURE          "Exposure time"
#define SANE_DESC_EXPOSURE           "The time the 4 different color filters of the CCD are exposed (R,G,B,I)"
#define SANE_EXPOSURE_DEFAULT        2937
#define SANE_NAME_GAIN               "gain"
#define SANE_TITLE_GAIN              "Gain"
#define SANE_DESC_GAIN               "The gain of the signal processor for the 4 CCD color filters (R,G,B,I)"
#define SANE_GAIN_DEFAULT            0x13
#define SANE_NAME_OFFSET             "offset"
#define SANE_TITLE_OFFSET            "Offset"
#define SANE_DESC_OFFSET             "The offset of the signal processor for the 4 CCD color filters (R,G,B,I)"
#define SANE_OFFSET_DEFAULT          0

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

static const SANE_Range percentage_range_100 = {
  0 << SANE_FIXED_SCALE_SHIFT,	  /* minimum */
  100 << SANE_FIXED_SCALE_SHIFT,  /* maximum */
  0 << SANE_FIXED_SCALE_SHIFT	  /* quantization */
};

/* From the firmware disassembly */
static const SANE_Range gain_range = {
  0,	  /* minimum */
  63,     /* maximum */
  0	  /* quantization */
};

/* From the firmware disassembly */
static const SANE_Range offset_range = {
  0,      /* minimum */
  255,    /* maximum */
  0	  /* quantization */
};

/**
 * Callback called whenever a connected USB device reports a supported vendor
 * and product id combination.
 * Used by sane_init() and by sane_open().
 * 
 * @param name Device name which has required vendor and product id
 * @return SANE_STATUS_GOOD
 */
static SANE_Status
find_device_callback (const char *devicename)
{
    struct Reflecta_Command_Status status;
    SANE_Status r;
    Reflecta_Device_Definition *dev;
    int device_number; /* index in usb devices list maintained by sani_usb */
    Reflecta_Scanner_Properties inq;

    DBG (DBG_sane_proc, "find_device_callback: %s\n", devicename);

    /* Check if device is present in the Reflecta device list */
    for (dev = definition_list_head; dev; dev = dev->next) {
        if (strcmp (dev->sane.name, devicename) == 0) {
	    return SANE_STATUS_GOOD;
        }
    }    

    /* If not, create a new device struct */
    dev = malloc (sizeof (*dev));
    if (!dev) {
        return SANE_STATUS_NO_MEM;
    }
    
    /* Get device number: index of the device in the sanei_usb devices list */
    r = sanei_usb_open (devicename, &device_number);
    if (r != SANE_STATUS_GOOD) {
        free (dev);
        DBG (DBG_error, "find_device_callback: sanei_usb_open failed\n");
        return r;
    }
    
    /* Get device properties */
    
    cmdDoInquiry(device_number,&inq,5,&status,5);
    if (status.sane_status != SANE_STATUS_GOOD) {
        free (dev);
        DBG (DBG_error, "find_device_callback: get scanner properties (5 bytes) failed\n");
        sanei_usb_close (device_number);
        return status.sane_status;
    }
    cmdDoInquiry(device_number,&inq,inq.additionalLength+4,&status,5);
    if (status.sane_status != SANE_STATUS_GOOD) {
        free (dev);
        DBG (DBG_error, "find_device_callback: get scanner properties failed\n");
        sanei_usb_close (device_number);
        return status.sane_status;
    }
    
    /* Check model number */
    if (inq.model != reflecta_supported_usb_device.model) {
        free (dev);
        DBG (DBG_error, "find_device_callback: wrong model number %d\n", inq.model);
        return SANE_STATUS_INVAL;
    }
    
    /* Initialize device definition */  
    reflecta_initialize_device_definition(dev,&inq,devicename,reflecta_supported_usb_device.vendor,reflecta_supported_usb_device.product,device_number);
    
    /* Output */
    reflecta_print_inquiry (dev);

    /* Found a supported scanner, put it in the definitions list*/
    DBG (DBG_error, "find_device_callback: success\n");
    dev->next = definition_list_head;
    definition_list_head = dev;
    return SANE_STATUS_GOOD;
}

/**
 * Full initialization of a Reflecta_Device structure from INQUIRY data.
 * The function is used in find_device_callback(), so when sane_init() or
 * sane_open() is called.
 * 
 * @param dev
 */
static void
reflecta_initialize_device_definition (Reflecta_Device_Definition* dev, Reflecta_Scanner_Properties* inq, const char* devicename,
        SANE_Word vendor_id, SANE_Word product_id, SANE_Int devnr)
{
    char* pp;
    
    /* Initialize device definition */    
    dev->next = NULL;
    dev->sane.name = strdup(devicename);

    /* Create 0-terminated string without trailing spaces for vendor */
    dev->sane.vendor = (SANE_Char*)malloc(9);
    strncpy((SANE_String)dev->sane.vendor, inq->vendor,8);
    pp = ((SANE_String)dev->sane.vendor)+8;
    *pp-- = '\0';
    while (*pp == ' ') *pp-- = '\0';

    /* Create 0-terminated string without trailing spaces for model */
    dev->sane.model = (SANE_Char*)malloc(17);
    strncpy((SANE_String)dev->sane.model,inq->product,16);
    pp = ((SANE_String)dev->sane.model)+16;
    *pp-- = '\0';
    while (*pp == ' ') *pp-- = '\0';
    
    dev->sane.type = "film scanner";
    dev->vendorId = vendor_id;
    dev->productId = product_id;

    /* Create 0-terminated strings without trailing spaces for revision */
    dev->version = (SANE_Char*)malloc(5);
    strncpy(dev->version,inq->productRevision,4);
    pp = dev->version+4;
    *pp-- = '\0';
    while (*pp == ' ') *pp-- = '\0';

    dev->model = inq->model;

    /* Maximum resolution values */
    dev->maximum_resolution_x = inq->maxResolutionX; 
    dev->maximum_resolution_y = inq->maxResolutionY;
    if (dev->maximum_resolution_y < 256) {
        /* y res is a multiplier */
        dev->maximum_resolution = dev->maximum_resolution_x;
        dev->maximum_resolution_x *= dev->maximum_resolution_y;
        dev->maximum_resolution_y = dev->maximum_resolution_x;
    } else {
      /* y res really is resolution */
      dev->maximum_resolution = min (dev->maximum_resolution_x, dev->maximum_resolution_y);
    }

    /* Geometry */
    dev->scan_bed_width = (double) inq->maxScanWidth / dev->maximum_resolution;
    dev->scan_bed_height = (double) inq->maxScanHeight / dev->maximum_resolution;
    dev->slide_top_left_x = inq->x0;
    dev->slide_top_left_y = inq->y0;
    dev->slide_width = (double) (inq->x1 - inq->x0) / dev->maximum_resolution;
    dev->slide_height = (double) (inq->y1 - inq->y0) / dev->maximum_resolution;

    /* Integer and bit-encoded properties */
    dev->halftone_patterns = inq->halftones & 0x0f;
    dev->color_filters = inq->filters;
    dev->color_depths = inq->colorDepths;
    dev->color_formats = inq->colorFormat;
    dev->image_formats = inq->imageFormat;
    dev->scan_capabilities = inq->scanCapability;
    dev->optional_devices = inq->optionalDevices;
    dev->enhancements = inq->enhancements;
    dev->gamma_bits = inq->gammaBits;
    dev->fast_preview_resolution = inq->previewScanResolution;
    dev->minimum_highlight = inq->minumumHighlight; 
    dev->maximum_shadow = inq->maximumShadow;
    dev->calibration_equation = inq->calibrationEquation;
    dev->minimum_exposure = inq->minimumExposure;
    dev->maximum_exposure = inq->maximumExposure*2; /* *2 to solve the strange situation that the default value is out of range */
    
    /* Ranges for various quantities */
    dev->x_range.min = SANE_FIX (0);
    dev->x_range.quant = SANE_FIX (0);
    dev->x_range.max = SANE_FIX (dev->scan_bed_width * MM_PER_INCH);

    dev->y_range.min = SANE_FIX (0);
    dev->y_range.quant = SANE_FIX (0);
    dev->y_range.max = SANE_FIX (dev->scan_bed_height * MM_PER_INCH);

    dev->dpi_range.min = SANE_FIX (25);
    dev->dpi_range.quant = SANE_FIX (1);
    dev->dpi_range.max = SANE_FIX (max (dev->maximum_resolution_x, dev->maximum_resolution_y));

    dev->shadow_range.min = SANE_FIX (0);
    dev->shadow_range.quant = SANE_FIX (1);
    dev->shadow_range.max = SANE_FIX (dev->maximum_shadow);

    dev->highlight_range.min = SANE_FIX (dev->minimum_highlight);
    dev->highlight_range.quant = SANE_FIX (1);
    dev->highlight_range.max = SANE_FIX (100);

    dev->exposure_range.min = dev->minimum_exposure;
    dev->exposure_range.quant = 1;
    dev->exposure_range.max = dev->maximum_exposure;
    
    /* Enumerated ranges vor various quantities */
    /*TODO: create from inq->filters */
    dev->scan_mode_list[0] = SANE_VALUE_SCAN_MODE_LINEART;
    dev->scan_mode_list[1] = SANE_VALUE_SCAN_MODE_HALFTONE;
    dev->scan_mode_list[2] = SANE_VALUE_SCAN_MODE_GRAY;
    dev->scan_mode_list[3] = SANE_VALUE_SCAN_MODE_COLOR;
    dev->scan_mode_list[4] = SANE_VALUE_SCAN_MODE_RGBI;
    dev->scan_mode_list[5] = 0;
    
    /*TODO: create from inq->colorDepths */
    dev->bpp_list[0] = 4; /* count */
    dev->bpp_list[1] = 1;
    dev->bpp_list[2] = 8;
    dev->bpp_list[3] = 12;
    dev->bpp_list[4] = 16;

    /*TODO: implement */
    reflecta_get_halftones (dev, devnr);

}

/**
 * Output device definition.
 * The function is used in find_device_callback(), so when sane_init() or
 * sane_open() is called.
 * 
 * @param dev Device to output
 */
static void
reflecta_print_inquiry (Reflecta_Device_Definition * dev)
{
  DBG (DBG_inquiry, "INQUIRY:\n");
  DBG (DBG_inquiry, "========\n");
  DBG (DBG_inquiry, "\n");
  DBG (DBG_inquiry, "vendor........................: '%s'\n", dev->sane.vendor);
  DBG (DBG_inquiry, "product.......................: '%s'\n", dev->sane.model);
  DBG (DBG_inquiry, "version.......................: '%s'\n", dev->version);

  DBG (DBG_inquiry, "X resolution..................: %d dpi\n",
       dev->maximum_resolution_x);
  DBG (DBG_inquiry, "Y resolution..................: %d dpi\n",
       dev->maximum_resolution_y);
  DBG (DBG_inquiry, "pixel resolution..............: %d dpi\n",
       dev->maximum_resolution);
  DBG (DBG_inquiry, "fb width......................: %f in\n",
       dev->scan_bed_width);
  DBG (DBG_inquiry, "fb length.....................: %f in\n",
       dev->scan_bed_height);

  DBG (DBG_inquiry, "transparency width............: %f in\n",
       dev->slide_width);
  DBG (DBG_inquiry, "transparency length...........: %f in\n",
       dev->slide_height);
  DBG (DBG_inquiry, "transparency offset...........: %d,%d\n",
       dev->slide_top_left_x, dev->slide_top_left_y);

  DBG (DBG_inquiry, "# of halftones................: %d\n",
       dev->halftone_patterns);

  DBG (DBG_inquiry, "One pass color................: %s\n",
       dev->color_filters & SCAN_ONE_PASS_COLOR ? "yes" : "no");

  DBG (DBG_inquiry, "Filters.......................: %s%s%s%s%s (%02x)\n",
       dev->color_filters & SCAN_FILTER_INFRARED ? "Infrared " : "",
       dev->color_filters & SCAN_FILTER_RED ? "Red " : "",
       dev->color_filters & SCAN_FILTER_GREEN ? "Green " : "",
       dev->color_filters & SCAN_FILTER_BLUE ? "Blue " : "",
       dev->color_filters & SCAN_FILTER_NEUTRAL ? "Neutral " : "",
       dev->color_filters);

  DBG (DBG_inquiry, "Color depths..................: %s%s%s%s%s%s (%02x)\n",
       dev->color_depths & SCAN_COLOR_DEPTH_16 ? "16 bit " : "",
       dev->color_depths & SCAN_COLOR_DEPTH_12 ? "12 bit " : "",
       dev->color_depths & SCAN_COLOR_DEPTH_10 ? "10 bit " : "",
       dev->color_depths & SCAN_COLOR_DEPTH_8 ? "8 bit " : "",
       dev->color_depths & SCAN_COLOR_DEPTH_4 ? "4 bit " : "",
       dev->color_depths & SCAN_COLOR_DEPTH_1 ? "1 bit " : "",
       dev->color_depths);

  DBG (DBG_inquiry, "Color Format..................: %s%s%s (%02x)\n",
       dev->color_formats & SCAN_COLOR_FORMAT_INDEX ? "Indexed " : "",
       dev->color_formats & SCAN_COLOR_FORMAT_LINE ? "Line " : "",
       dev->color_formats & SCAN_COLOR_FORMAT_PIXEL ? "Pixel " : "",
       dev->color_formats);

  DBG (DBG_inquiry, "Image Format..................: %s%s%s%s (%02x)\n",
       dev->image_formats & SCAN_IMG_FMT_OKLINE ? "OKLine " : "",
       dev->image_formats & SCAN_IMG_FMT_BLK_ONE ? "BlackOne " : "",
       dev->image_formats & SCAN_IMG_FMT_MOTOROLA ? "Motorola " : "",
       dev->image_formats & SCAN_IMG_FMT_INTEL ? "Intel" : "",
       dev->image_formats);

  DBG (DBG_inquiry,
       "Scan Capability...............: %s%s%s%s%d speeds (%02x)\n",
       dev->scan_capabilities & SCAN_CAP_PWRSAV ? "PowerSave " : "",
       dev->scan_capabilities & SCAN_CAP_EXT_CAL ? "ExtCal " : "",
       dev->scan_capabilities & SCAN_CAP_FAST_PREVIEW ? "FastPreview" :
       "",
       dev->scan_capabilities & SCAN_CAP_DISABLE_CAL ? "DisCal " : "",
       dev->scan_capabilities & SCAN_CAP_SPEEDS,
       dev->scan_capabilities);

  DBG (DBG_inquiry, "Optional Devices..............: %s%s%s%s (%02x)\n",
       dev->optional_devices & SCAN_OPT_DEV_MPCL ? "MultiPageLoad " :
       "",
       dev->optional_devices & SCAN_OPT_DEV_TP1 ? "TransModule1 " : "",
       dev->optional_devices & SCAN_OPT_DEV_TP ? "TransModule " : "",
       dev->optional_devices & SCAN_OPT_DEV_ADF ? "ADF " : "",
       dev->optional_devices);

  DBG (DBG_inquiry, "Enhancement...................: %02x\n",
       dev->enhancements);
  DBG (DBG_inquiry, "Gamma bits....................: %d\n",
       dev->gamma_bits);

  DBG (DBG_inquiry, "Fast Preview Resolution.......: %d\n",
       dev->fast_preview_resolution);
  DBG (DBG_inquiry, "Min Highlight.................: %d\n",
       dev->minimum_highlight);
  DBG (DBG_inquiry, "Max Shadow....................: %d\n",
       dev->maximum_shadow);
  DBG (DBG_inquiry, "Cal Eqn.......................: %d\n",
       dev->calibration_equation);
  DBG (DBG_inquiry, "Min Exposure..................: %d\n",
       dev->minimum_exposure);
  DBG (DBG_inquiry, "Max Exposure..................: %d\n",
       dev->maximum_exposure);
}

/**
 * Initiaize scanner options from the device definition and from exposure,
 * gain and offset defaults. The function is called by sane_open(), when no
 * optimized settings are available yet. The scanner object is fully
 * initialized in sane_start().
 * 
 * @param scanner Scanner to initialize
 * @return SANE_STATUS_GOOD
 */
static SANE_Status
init_options (Reflecta_Scanner* scanner)
{
    int i;

    DBG (DBG_sane_proc, "init_options\n");

    memset (scanner->opt, 0, sizeof (scanner->opt));
    memset (scanner->val, 0, sizeof (scanner->val));

    for (i = 0; i < NUM_OPTIONS; ++i) {
        scanner->opt[i].size = sizeof (SANE_Word);
        scanner->opt[i].cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
    }

    /* Number of options (a pseudo-option) */
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
    scanner->opt[OPT_MODE].size = max_string_size ((SANE_String_Const *) scanner->device->scan_mode_list);
    scanner->opt[OPT_MODE].constraint_type = SANE_CONSTRAINT_STRING_LIST;
    scanner->opt[OPT_MODE].constraint.string_list = (SANE_String_Const *) scanner->device->scan_mode_list;
    scanner->val[OPT_MODE].s = (SANE_Char *) strdup (scanner->device->scan_mode_list[3]); /* default RGB */

    /* bit depth */
    scanner->opt[OPT_BIT_DEPTH].name = SANE_NAME_BIT_DEPTH;
    scanner->opt[OPT_BIT_DEPTH].title = SANE_TITLE_BIT_DEPTH;
    scanner->opt[OPT_BIT_DEPTH].desc = SANE_DESC_BIT_DEPTH;
    scanner->opt[OPT_BIT_DEPTH].type = SANE_TYPE_INT;
    scanner->opt[OPT_BIT_DEPTH].constraint_type = SANE_CONSTRAINT_WORD_LIST;
    scanner->opt[OPT_BIT_DEPTH].size = sizeof (SANE_Word);
    scanner->opt[OPT_BIT_DEPTH].constraint.word_list = scanner->device->bpp_list;
    scanner->val[OPT_BIT_DEPTH].w = scanner->device->bpp_list[2];

    /* resolution */
    scanner->opt[OPT_RESOLUTION].name = SANE_NAME_SCAN_RESOLUTION;
    scanner->opt[OPT_RESOLUTION].title = SANE_TITLE_SCAN_RESOLUTION;
    scanner->opt[OPT_RESOLUTION].desc = SANE_DESC_SCAN_RESOLUTION;
    scanner->opt[OPT_RESOLUTION].type = SANE_TYPE_FIXED;
    scanner->opt[OPT_RESOLUTION].unit = SANE_UNIT_DPI;
    scanner->opt[OPT_RESOLUTION].constraint_type = SANE_CONSTRAINT_RANGE;
    scanner->opt[OPT_RESOLUTION].constraint.range = &scanner->device->dpi_range;
    scanner->val[OPT_RESOLUTION].w = scanner->device->fast_preview_resolution << SANE_FIXED_SCALE_SHIFT;

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

    /* "Enhancement" group: */
    scanner->opt[OPT_ENHANCEMENT_GROUP].title = "Enhancement";
    scanner->opt[OPT_ENHANCEMENT_GROUP].desc = "";
    scanner->opt[OPT_ENHANCEMENT_GROUP].type = SANE_TYPE_GROUP;
    scanner->opt[OPT_ENHANCEMENT_GROUP].cap = 0;
    scanner->opt[OPT_ENHANCEMENT_GROUP].constraint_type = SANE_CONSTRAINT_NONE;

    /* halftone pattern */
    scanner->opt[OPT_HALFTONE_PATTERN].name = SANE_NAME_HALFTONE_PATTERN;
    scanner->opt[OPT_HALFTONE_PATTERN].title = SANE_TITLE_HALFTONE_PATTERN;
    scanner->opt[OPT_HALFTONE_PATTERN].desc = SANE_DESC_HALFTONE_PATTERN;
    scanner->opt[OPT_HALFTONE_PATTERN].type = SANE_TYPE_STRING;
    scanner->opt[OPT_HALFTONE_PATTERN].size = max_string_size ((SANE_String_Const *) scanner->device->halftone_list);
    scanner->opt[OPT_HALFTONE_PATTERN].constraint_type = SANE_CONSTRAINT_STRING_LIST;
    scanner->opt[OPT_HALFTONE_PATTERN].constraint.string_list = (SANE_String_Const *) scanner->device->halftone_list;
    scanner->val[OPT_HALFTONE_PATTERN].s = (SANE_Char *) strdup (scanner->device->halftone_list[0]);
    scanner->opt[OPT_HALFTONE_PATTERN].cap |= SANE_CAP_INACTIVE; /* Not implemented, and only meaningful at depth 1 */

    /* lineart threshold */
    scanner->opt[OPT_THRESHOLD].name = SANE_NAME_THRESHOLD;
    scanner->opt[OPT_THRESHOLD].title = SANE_TITLE_THRESHOLD;
    scanner->opt[OPT_THRESHOLD].desc = SANE_DESC_THRESHOLD;
    scanner->opt[OPT_THRESHOLD].type = SANE_TYPE_FIXED;
    scanner->opt[OPT_THRESHOLD].unit = SANE_UNIT_PERCENT;
    scanner->opt[OPT_THRESHOLD].constraint_type = SANE_CONSTRAINT_RANGE;
    scanner->opt[OPT_THRESHOLD].constraint.range = &percentage_range_100;
    scanner->val[OPT_THRESHOLD].w = SANE_FIX (50);
    scanner->opt[OPT_THRESHOLD].cap |= SANE_CAP_INACTIVE; /* Not implemented, and only meaningful at depth 1 */

    /* create a sharper scan at the cost of scan time */
    scanner->opt[OPT_SHARPEN].name = "sharpen";
    scanner->opt[OPT_SHARPEN].title = "Sharpen scan";
    scanner->opt[OPT_SHARPEN].desc = "Sharpen scan by taking more time to discharge the CCD.";
    scanner->opt[OPT_SHARPEN].type = SANE_TYPE_BOOL;
    scanner->opt[OPT_SHARPEN].unit = SANE_UNIT_NONE;
    scanner->opt[OPT_SHARPEN].constraint_type = SANE_CONSTRAINT_NONE;
    scanner->val[OPT_SHARPEN].b = SANE_TRUE;
    scanner->opt[OPT_SHARPEN].cap |= SANE_CAP_SOFT_SELECT;
    
    /* skip the auto-calibration phase before the scan */
    scanner->opt[OPT_SKIP_CALIBRATION].name = "skip-calibration";
    scanner->opt[OPT_SKIP_CALIBRATION].title = "Skip auto-calibration";
    scanner->opt[OPT_SKIP_CALIBRATION].desc = "Skip auto-calibration before scanning image. Option may be overridden by scanner.";
    scanner->opt[OPT_SKIP_CALIBRATION].type = SANE_TYPE_BOOL;
    scanner->opt[OPT_SKIP_CALIBRATION].unit = SANE_UNIT_NONE;
    scanner->opt[OPT_SKIP_CALIBRATION].constraint_type = SANE_CONSTRAINT_NONE;
    scanner->val[OPT_SKIP_CALIBRATION].b = SANE_FALSE;
    scanner->opt[OPT_SKIP_CALIBRATION].cap |= SANE_CAP_SOFT_SELECT;
    
    /* scan infrared channel faster but less accurate */
    scanner->opt[OPT_FAST_INFRARED].name = "fast-infrared";
    scanner->opt[OPT_FAST_INFRARED].title = "Fast infrared scan";
    scanner->opt[OPT_FAST_INFRARED].desc = "Do not reposition scan head before scanning infrared line. Results in an infrared offset which may deteriorate IR dust and scratch removal.";
    scanner->opt[OPT_FAST_INFRARED].type = SANE_TYPE_BOOL;
    scanner->opt[OPT_FAST_INFRARED].unit = SANE_UNIT_NONE;
    scanner->opt[OPT_FAST_INFRARED].constraint_type = SANE_CONSTRAINT_NONE;
    scanner->val[OPT_FAST_INFRARED].b = SANE_FALSE;
    scanner->opt[OPT_FAST_INFRARED].cap |= SANE_CAP_SOFT_SELECT;
    
    /* "Advanced" group: */
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

    /* save shading data */
    scanner->opt[OPT_SHADINGDATA].name = "save-shading-data";
    scanner->opt[OPT_SHADINGDATA].title = "Save shading data";
    scanner->opt[OPT_SHADINGDATA].desc = "Save shading data in 'reflecta.shading'";
    scanner->opt[OPT_SHADINGDATA].type = SANE_TYPE_BOOL;
    scanner->val[OPT_SHADINGDATA].w = SANE_FALSE;
    
    /* save CCD mask */
    scanner->opt[OPT_CCDMASK].name = "save-ccdmask";
    scanner->opt[OPT_CCDMASK].title = "Save CCD mask";
    scanner->opt[OPT_CCDMASK].desc = "Save CCD mask 'reflecta.ccd'";
    scanner->opt[OPT_CCDMASK].type = SANE_TYPE_BOOL;
    scanner->val[OPT_CCDMASK].w = SANE_FALSE;

    /* exposure times for R, G, B and I */
    scanner->opt[OPT_EXPOSURE].name = SANE_NAME_EXPOSURE;
    scanner->opt[OPT_EXPOSURE].title = SANE_TITLE_EXPOSURE;
    scanner->opt[OPT_EXPOSURE].desc = SANE_DESC_EXPOSURE;
    scanner->opt[OPT_EXPOSURE].type = SANE_TYPE_INT;
    scanner->opt[OPT_EXPOSURE].unit = SANE_UNIT_MICROSECOND;
    scanner->opt[OPT_EXPOSURE].constraint_type = SANE_CONSTRAINT_RANGE;
    scanner->opt[OPT_EXPOSURE].constraint.range = &(scanner->device->exposure_range);
    for (i=0; i<4; i++) scanner->settings.exposureTime[i] = SANE_EXPOSURE_DEFAULT;
    scanner->val[OPT_EXPOSURE].wa = scanner->settings.exposureTime;
    scanner->opt[OPT_EXPOSURE].size = 4*sizeof(SANE_Word);
    
    /* gain for R, G, B and I */
    scanner->opt[OPT_GAIN].name = SANE_NAME_GAIN;
    scanner->opt[OPT_GAIN].title = SANE_TITLE_GAIN;
    scanner->opt[OPT_GAIN].desc = SANE_DESC_GAIN;
    scanner->opt[OPT_GAIN].type = SANE_TYPE_INT;
    scanner->opt[OPT_GAIN].unit = SANE_UNIT_NONE;
    scanner->opt[OPT_GAIN].constraint_type = SANE_CONSTRAINT_RANGE;
    scanner->opt[OPT_GAIN].constraint.range = &gain_range;
    for (i=0; i<4; i++) scanner->settings.gain[i] = SANE_GAIN_DEFAULT;
    scanner->val[OPT_GAIN].wa = scanner->settings.gain;
    scanner->opt[OPT_GAIN].size = 4*sizeof(SANE_Word);
    
    /* offsets for R, G, B and I */
    scanner->opt[OPT_OFFSET].name = SANE_NAME_OFFSET;
    scanner->opt[OPT_OFFSET].title = SANE_TITLE_OFFSET;
    scanner->opt[OPT_OFFSET].desc = SANE_DESC_OFFSET;
    scanner->opt[OPT_OFFSET].type = SANE_TYPE_INT;
    scanner->opt[OPT_OFFSET].unit = SANE_UNIT_NONE;
    scanner->opt[OPT_OFFSET].constraint_type = SANE_CONSTRAINT_RANGE;
    scanner->opt[OPT_OFFSET].constraint.range = &offset_range;
    for (i=0; i<4; i++) scanner->settings.offset[i] = SANE_OFFSET_DEFAULT;
    scanner->val[OPT_OFFSET].wa = scanner->settings.offset;
    scanner->opt[OPT_OFFSET].size = 4*sizeof(SANE_Word);
    
    return SANE_STATUS_GOOD;
}

/**
 * Parse line from config file into a vendor id, product id and a model number
 * 
 * @param config_line Text to parse
 * @param vendor_id 
 * @param product_id
 * @param model_number
 * @return SANE_STATUS_GOOD, or SANE_STATUS_INVAL in case of a parse error
 */
static SANE_Status
reflecta_parse_config_line(const char* config_line, SANE_Word* vendor_id, SANE_Word* product_id, SANE_Word* model_number)
{
    char *vendor_id_string, *product_id_string, *model_number_string;

    if (strncmp (config_line, "usb ", 4) != 0) {
        return SANE_STATUS_INVAL;
    }
    /* Detect vendor-id */
    config_line += 4;
    config_line = sanei_config_skip_whitespace (config_line);
    if (*config_line) {
        config_line = sanei_config_get_string (config_line, &vendor_id_string);
        if (vendor_id_string) {
            *vendor_id = strtol (vendor_id_string, 0, 0);
            free (vendor_id_string);
        } else {
            return SANE_STATUS_INVAL;
        }
        config_line = sanei_config_skip_whitespace (config_line);
    } else {
        return SANE_STATUS_INVAL;
    }
    /* Detect product-id */
    config_line = sanei_config_skip_whitespace (config_line);
    if (*config_line) {
        config_line = sanei_config_get_string (config_line, &product_id_string);
        if (product_id_string) {
            *product_id = strtol (product_id_string, 0, 0);
            free (product_id_string);
        } else {
            return SANE_STATUS_INVAL;
        }
        config_line = sanei_config_skip_whitespace (config_line);
    } else {
        return SANE_STATUS_INVAL;
    }
    /* Detect product-id */
    config_line = sanei_config_skip_whitespace (config_line);
    if (*config_line) {
        config_line = sanei_config_get_string (config_line, &model_number_string);
        if (model_number_string) {
            *model_number = strtol (model_number_string, 0, 0);
            free (model_number_string);
        } else {
            return SANE_STATUS_INVAL;
        }
        config_line = sanei_config_skip_whitespace (config_line);
    } else {
        return SANE_STATUS_INVAL;
    }
    return SANE_STATUS_GOOD;
}

/**
 * Check if current list of supported devices contains the given specifications.
 * 
 * @param vendor_id
 * @param product_id
 * @param model_number
 * @return 
 */
static SANE_Bool
reflecta_supported_device_list_contains(SANE_Word vendor_id, SANE_Word product_id, SANE_Word model_number)
{
    int i = 0;
    while (reflecta_supported_usb_device_list[i].vendor != 0) {
        if (reflecta_supported_usb_device_list[i].vendor == vendor_id
              && reflecta_supported_usb_device_list[i].product == product_id
              && reflecta_supported_usb_device_list[i].model == model_number) {
            return SANE_TRUE;
        }
        i++;
    }
    return SANE_FALSE;
}

/**
 * Add the given specifications to the current list of supported devices 
 * @param vendor_id
 * @param product_id
 * @param model_number
 * @return 
 */
static SANE_Status
reflecta_supported_device_list_add(SANE_Word vendor_id, SANE_Word product_id, SANE_Word model_number)
{
    int i = 0;
    struct Reflecta_USB_Device_Entry* dl;
    
    while (reflecta_supported_usb_device_list[i].vendor != 0) {
        i++;
    }    
    /* i is index of last entry */
    dl = realloc(reflecta_supported_usb_device_list,i+2); /* Add one entry to list */
    if (dl == NULL) {
        return SANE_STATUS_INVAL;
    }
    /* Copy values */
    reflecta_supported_usb_device_list = dl;
    reflecta_supported_usb_device_list[i].vendor = vendor_id;
    reflecta_supported_usb_device_list[i].product = product_id;
    reflecta_supported_usb_device_list[i].model = model_number;
    reflecta_supported_usb_device_list[i+1].vendor = 0;
    reflecta_supported_usb_device_list[i+1].product = 0;
    reflecta_supported_usb_device_list[i+1].model = 0;
    return SANE_STATUS_GOOD;
}

static void reflecta_get_halftones (Reflecta_Device_Definition * dev, int sfd)
{
    /* halftone_list */
    dev->halftone_list[0] = "53lpi 45d ROUND"; /* 8x8 pattern */
    dev->halftone_list[1] = "70lpi 45d ROUND"; /* 6x6 pattern */
    dev->halftone_list[2] = "75lpi Hori. Line"; /* 4x4 pattern */
    dev->halftone_list[3] = "4X4 BAYER"; /* 4x4 pattern */
    dev->halftone_list[4] = "4X4 SCROLL"; /* 4x4 pattern */
    dev->halftone_list[5] = "5x5 26 Levels"; /* 5x5 pattern */
    dev->halftone_list[6] = "4x4 SQUARE"; /* 4x4 pattern */
    dev->halftone_list[7] = "5x5 TILE"; /* 5x5 pattern */
    dev->halftone_list[8] = 0;
}

/**
 * Actions to perform when a cancel request has been received.
 * 
 * @param scanner scanner to stop scanning
 * @return SANE_STATUS_CANCELLED
 */
static SANE_Status reflecta_on_cancel (Reflecta_Scanner * scanner)
{
    struct Reflecta_Command_Status status;
    
    cmdStopScan(scanner->device_number, &status, 5);
    cmdSetScanHead(scanner->device_number, 1, 0, &status, 10);
    buffer_delete(&scanner->buffer);
    scanner->scanning = SANE_FALSE;
    return SANE_STATUS_CANCELLED;
}

/**
 * Determine maximum lengt of a set of strings.
 * 
 * @param strings Set of strings
 * @return maximum length
 */
static size_t
max_string_size (SANE_String_Const strings[])
{
    size_t size, max_size = 0;
    int i;

    for (i = 0; strings[i]; ++i) {
        size = strlen (strings[i]) + 1;
        if (size > max_size) {
            max_size = size;
        }
    }

    return max_size;
}

