/* 
 * File:   reflecta.c
 * Author: jan
 *
 * Created on July 22, 2012, 2:22 PM
 * 
 * SANE interface to two Reflecta USB scanners:
 * - CrystalScan 7200 (model id 0x30)
 * - ProScan 7200 (model id 0x36)
 */

/* --------------------------------------------------------------------------
 *
 * INCLUDES
 * 
 * --------------------------------------------------------------------------*/

/* Standard includes for various utiliy functions */
#include <stdio.h> /* for FILE */
#include <string.h> /* for strlen */
#include <stdlib.h> /* for NULL */
extern char *strdup (const char *s);
extern char *strndup (const char *s, size_t n);
#include <unistd.h> /* usleep */
extern int usleep (__useconds_t useconds);
#include <stdint.h>

/* Configutation defines */
#include "../include/sane/config.h"

/* SANE includes */
#include "../include/sane/sane.h"
#include "../include/sane/saneopts.h"
#include "../include/sane/sanei_usb.h"
#include "../include/sane/sanei_config.h"
#include "../include/sane/sanei_thread.h"

/* Backend includes */
#define BACKEND_NAME reflecta
#include "../include/sane/sanei_backend.h"
#include "reflecta.h"
#include "sane.h"

extern void write_tiff_rgbi_header (FILE *fptr, int width, int height, int depth, int resolution, const char *icc_profile);

/* --------------------------------------------------------------------------
 *
 * DEFINES
 * 
 * --------------------------------------------------------------------------*/

/* Build number of this backend */
#define BUILD 1 

/* Configuration filename */
#define REFLECTA_CONFIG_FILE "reflecta.conf"

/* Debug error levels */
#define DBG_error0       0
#define DBG_error        1
#define DBG_sense        2
#define DBG_warning      3
#define DBG_inquiry      4
#define DBG_info         5
#define DBG_info2        6
#define DBG_proc         7
#define DBG_read         8
#define DBG_sane_init   10
#define DBG_sane_proc   11
#define DBG_sane_info   12
#define DBG_sane_option 13
#define DBG_dump	14

#define READ_BUFFER_SIZE 200000



/* =========================================================================
 * 
 * Defines
 * 
 * ========================================================================= */

/* Additional SANE status code */
#define SANE_STATUS_CHECK_CONDITION 14 /* add to SANE_status enum */

/* --------------------------------------------------------------------------
 *
 * SUPPORTED DEVICES SPECIFICS
 * 
 * --------------------------------------------------------------------------*/

/* List of default supported scanners by vendor-id, product-id and model number.
 * A default list will be created in sane_init(), and entries in the config file
 *  will be added to it. */

struct Reflecta_USB_Device_Entry
{
    SANE_Word vendor;		/* USB vendor identifier */
    SANE_Word product;		/* USB product identifier */
    SANE_Word model;		/* USB model number */
    SANE_Int device_number;     /* USB device number if the device is present */
};
static struct Reflecta_USB_Device_Entry* reflecta_supported_usb_device_list = NULL;
static struct Reflecta_USB_Device_Entry reflecta_supported_usb_device; /* for searching */


/* --------------------------------------------------------------------------
 *
 * FUNCTION PROTOTYPES & STRUCTURE DEFINITIONS
 * 
 * --------------------------------------------------------------------------*/

#include "reflecta_usb.h"
#include "reflecta_scancmd.h"
#include "reflecta_buffer.h"
#include "reflecta_specific.h"

/* --------------------------------------------------------------------------
 *
 * LISTS OF ACTIVE DEVICE DEFINITIONS AND SCANNERS
 * 
 * --------------------------------------------------------------------------*/

static Reflecta_Device_Definition *definition_list_head = NULL;
static Reflecta_Scanner *first_handle = NULL;
static const SANE_Device **devlist = NULL;

/* --------------------------------------------------------------------------
 *
 * SANE INTERFACE
 * 
 * --------------------------------------------------------------------------*/

/**
 * Initializes the debugging system, the USB system, the version code and
 * 'attaches' available scanners, i.e. creates device definitions for all
 * scanner devices found. 
 * 
 * @param version_code
 * @param authorize
 * @return SANE_STATUS_GOOD
 */
SANE_Status
sane_init (SANE_Int * version_code, SANE_Auth_Callback __sane_unused__ authorize)
{
    FILE *fp;
    char config_line[PATH_MAX];
    SANE_Word vendor_id;
    SANE_Word product_id;
    SANE_Word model_number;
    SANE_Status status;
    int i;
   
    /* Initialize debug logging */
    DBG_INIT ();

    DBG (DBG_sane_init, "sane_init() build %d\n", BUILD);

    /* Set version code to current major, minor and build number */
    /* TODO: use V_MINOR instead or SANE_CURRENT_MINOR? If so, why?  */
    if (version_code)
        *version_code = SANE_VERSION_CODE (SANE_CURRENT_MAJOR, SANE_CURRENT_MINOR, BUILD);

    /* Initialize usb */
    sanei_usb_init ();
    
    /* What's the use of a config file here, if all that is done with the information
     * is checking it against hard coded values? We should assume that the config
     * file contains appropriate scanner identifications, or we should ignore the
     * config file altogether. Let's adopt the first option. A user who modifies
     * the config file, may use the backend to address any USB device, even ones
     * that don't work. */

    /* Create default list */
    reflecta_supported_usb_device_list = calloc(3,sizeof(struct Reflecta_USB_Device_Entry));
    /* Reflecta CrystalScan 7200, model number 0x30 */
    reflecta_supported_usb_device_list[0].vendor = 0x05e3;
    reflecta_supported_usb_device_list[0].product = 0x0145;
    reflecta_supported_usb_device_list[0].model = 0x31;
    /* Reflecta ProScan 7200, model number 0x36 */
    reflecta_supported_usb_device_list[1].vendor = 0x05e3;
    reflecta_supported_usb_device_list[1].product = 0x0145;
    reflecta_supported_usb_device_list[1].model = 0x36;
    /* end of list */
    reflecta_supported_usb_device_list[2].vendor = 0;
    reflecta_supported_usb_device_list[2].product = 0;
    reflecta_supported_usb_device_list[2].model = 0;

/*
    for (i=0; i<3; i++) {
        DBG(DBG_info,"%03d: %04x %04x %02x\n", i,
            reflecta_supported_usb_device_list[i].vendor,
            reflecta_supported_usb_device_list[i].product,
            reflecta_supported_usb_device_list[i].model);
    }
*/
    
    /* Add entries from config file */
    fp = sanei_config_open (REFLECTA_CONFIG_FILE);
    if (!fp) {
        DBG (DBG_info, "sane_init() did not find a config file, using default list of supported devices\n");
    } else {
        while (sanei_config_read (config_line, sizeof (config_line), fp)) {
            /* Ignore line comments and empty lines */
            if (config_line[0] == '#') continue;
            if (strlen (config_line) == 0) continue;
            /* Ignore lines which do not begin with 'usb ' */
            if (strncmp (config_line, "usb ", 4) != 0) continue;
            /* Parse vendor-id, product-id and model number and add to list */
            DBG (DBG_sane_proc, "sane_init() config file parsing %s\n", config_line);
            status = reflecta_parse_config_line(config_line, &vendor_id, &product_id, &model_number);
            if (status == SANE_STATUS_GOOD) {
                DBG (DBG_info, "sane_init() config file lists device %04x %04x %02x\n",vendor_id, product_id, model_number);
                if (!reflecta_supported_device_list_contains(vendor_id, product_id, model_number)) {
                    DBG (DBG_info, "sane_init() adding device %04x %04x %02x\n",vendor_id, product_id, model_number);
                    reflecta_supported_device_list_add(vendor_id, product_id, model_number);
                } else {
                    DBG (DBG_sane_proc, "sane_init() list already contains %04x %04x %02x\n", vendor_id, product_id, model_number);
                }
            } else {
                DBG (DBG_sane_proc, "sane_init() config file parsing %s: error\n", config_line);
            }
	}
        fclose (fp);
    }
        
    /* Loop through supported device list */
    i = 0;
    while (reflecta_supported_usb_device_list[i].vendor != 0) {
        /* Check if the supported device is present. If so, create a device
         * definition structure for it.
         * The variable reflecta_supported_usb_device is set to current values,
         * which are used in the callback. */
        reflecta_supported_usb_device.vendor = reflecta_supported_usb_device_list[i].vendor;
        reflecta_supported_usb_device.product = reflecta_supported_usb_device_list[i].product;
        reflecta_supported_usb_device.model = reflecta_supported_usb_device_list[i].model;
        reflecta_supported_usb_device.device_number = -1; /* No device number (yet) */
        DBG( DBG_info, "sane_init() looking for Reflecta scanner %04x %04x model %02x\n",reflecta_supported_usb_device.vendor,reflecta_supported_usb_device.product,reflecta_supported_usb_device.model);
        sanei_usb_find_devices (reflecta_supported_usb_device_list[i].vendor, reflecta_supported_usb_device_list[i].product, find_device_callback);
        /* If opened, close it again here.
         * sanei_usb_find_devices() ignores errors from find_device_callback(), so the device may already be closed. */
/* not necessary, callback closes device
        if (reflecta_supported_usb_device.device_number >= 0) {
            sanei_usb_close (reflecta_supported_usb_device.device_number);
        }
*/
        i++;
    }    
    return SANE_STATUS_GOOD;    
}

/**
 * Backend exit.
 * Clean up allocated memory.
 */
void
sane_exit (void)
{
    Reflecta_Device_Definition *dev, *next;

    DBG (DBG_sane_init, "sane_exit()\n");

    for (dev = definition_list_head; dev; dev = next) {
        next = dev->next;
        free((SANE_String)dev->sane.name);
        free((SANE_String)dev->sane.vendor);
        free((SANE_String)dev->sane.model);
        free (dev->version);
        free (dev);
    }
    definition_list_head = NULL;

    if (devlist) {
        free (devlist);
        devlist = NULL;
    }
}

/**
 * Create a SANE device list from the device list generated by sane_init().
 * 
 * @param device_list List of SANE_Device elements
 * @param local_only If true, disregard network scanners. Not applicable for USB scanners.
 * @return SANE_STATUS_GOOD, or SANE_STATUS_NO_MEM if the list cannot be allocated
 */
SANE_Status
sane_get_devices (const SANE_Device *** device_list, SANE_Bool __sane_unused__ local_only)
{
    Reflecta_Device_Definition *dev;
    int i;

    DBG (DBG_sane_init, "sane_get_devices\n");

    /* Create SANE_DEVICE list from device list created in sane_init() */
    i = 0;
    for (dev = definition_list_head; dev; dev = dev->next) {
        i++;
    }
    if (devlist) {
        free (devlist);
    }
    devlist = malloc ((i + 1) * sizeof (devlist[0]));
    if (!devlist) {
        return SANE_STATUS_NO_MEM;
    }
    i = 0;
    for (dev = definition_list_head; dev; dev = dev->next) {
        devlist[i++] = &dev->sane;
    }
    devlist[i] = NULL;
    *device_list = devlist;
    return SANE_STATUS_GOOD;
}

/**
 * Open the scanner with the given devicename and return a handle to it, which
 * is a pointer to a Reflecta_Scanner struct. The handle will be an input to
 * a couple of other functions of the SANE interface.
 * 
 * @param devicename Name of the device, corresponds to SANE_Device.name
 * @param handle handle to scanner (pointer to a Reflecta_Scanner struct)
 * @return SANE_STATUS_GOOD if the device has been opened
 */
SANE_Status
sane_open (SANE_String_Const devicename, SANE_Handle * handle)
{
    Reflecta_Device_Definition *dev;
    SANE_Status status;
    Reflecta_Scanner *scanner, *s;

    DBG (DBG_sane_init, "sane_open(%s)\n", devicename);

    /* Search for devicename */
    if (devicename[0]) {
        for (dev = definition_list_head; dev; dev = dev->next) {
	  if (strcmp (dev->sane.name, devicename) == 0) {
	      break;
	  }
        }
        if (!dev) {
            /* Is it a valid USB device? */
            SANE_Word vendor;
            SANE_Word product;
            int i = 0;

            status = sanei_usb_get_vendor_product_byname(devicename,&vendor,&product);
            if (status != SANE_STATUS_GOOD) {
                DBG (DBG_error, "sane_open: sanei_usb_get_vendor_product_byname failed %s\n",devicename);
                return status;
            }            
            /* Get vendor-product-model & verify that is is supported */
            /* Loop through supported device list */
            while (reflecta_supported_usb_device_list[i].vendor != 0) {
                /* Check if vendor and product match */
                if (reflecta_supported_usb_device_list[i].vendor == vendor
                        && reflecta_supported_usb_device_list[i].product == product) {
                    /* Check if a supported device is present
                     * If so, create a device definition structure for it. */
                    /* Set reflecta_supported_usb_device to current values: these are used in callback */
                    reflecta_supported_usb_device.vendor = vendor;
                    reflecta_supported_usb_device.product = product;
                    reflecta_supported_usb_device.model = reflecta_supported_usb_device_list[i].model;
                    reflecta_supported_usb_device.device_number = -1;
                    sanei_usb_find_devices (vendor, product, find_device_callback);
                    if (reflecta_supported_usb_device.device_number == -1) {
                        /* Did not succeed in opening the USB device, which is an error.
                         * This error is not caught by sanei_usb_find_devices(), so handle
                         * it here. */
                        DBG (DBG_error, "sane_open: sanei_usb_find_devices did not open device %s\n",devicename);
                        return SANE_STATUS_INVAL;
/* not necessary, callback closes device
                    } else {
                        sanei_usb_close(reflecta_supported_usb_device.device_number);
*/
                    }
                }
                i++;
            }    
            /* Now rescan the device list to see if it is present */
            for (dev = definition_list_head; dev; dev = dev->next) {
              if (strcmp (dev->sane.name, devicename) == 0) {
                  break;
              }
            }
	}
    } else {
        /* empty devicename -> use first device */
        dev = definition_list_head;		
    }
    /* If no device found, return error */
    if (!dev) {
        return SANE_STATUS_INVAL;
    }
    
    /* Now create a scanner structure to return */
    
    /* Check if we are not opening the same scanner again. */
    for (s = first_handle; s; s = s->next) {
        if (s->device->sane.name == devicename) {
            *handle = s;
            return SANE_STATUS_GOOD;
        }
    }
    
    /* Create a new scanner instance */
    scanner = malloc (sizeof (*scanner));
    if (!scanner) {
        return SANE_STATUS_NO_MEM;
    }
    memset (scanner, 0, sizeof (*scanner));
    scanner->device = dev;
    sanei_usb_open(dev->sane.name, &scanner->device_number);
    scanner->cancel_request = 0;
    /* Options and buffers */
    init_options (scanner);
    scanner->shading_buffer = (SANE_Byte*)malloc((5340*2+2)*45*4);
    scanner->ccd_mask = (SANE_Byte*)malloc(5340);
    /* Insert newly opened handle into list of open handles: */
    scanner->next = first_handle;	
    first_handle = scanner;
    
    *handle = scanner;
    return SANE_STATUS_GOOD;
}

/**
 * Close the scanner and remove the scanner from the list of active scanners.
 * 
 * @param handle Scanner handle
 */
void
sane_close (SANE_Handle handle)
{
    Reflecta_Scanner *prev, *scanner;
    struct Reflecta_Command_Status status;

    DBG (DBG_sane_init, "sane_close\n");

    /* Find handle in list of open handles: */
    prev = 0;
    for (scanner = first_handle; scanner; scanner = scanner->next)  {
        if (scanner == handle) {
            break;
        }
        prev = scanner;
    }
    /* Not a handle we know about. This may happen since all different backend
     * scanner instances are all cast to SANE_Handle (a void pointer) */
    if (!scanner) {
        DBG (DBG_error, "close: invalid handle %p\n", handle);
        return;			
    }
    
    /* Stop scan if still scanning */
    if (scanner->scanning) {
        cmdStopScan(scanner->device_number, &status, 5);
        cmdSetScanHead(scanner->device_number, 1, 0, &status, 10);
        scanner->scanning = SANE_FALSE;
    }
    
    /* USB scanners may be still open here */
    if (scanner->device_number >= 0) {
        sanei_usb_reset (scanner->device_number);
        sanei_usb_close (scanner->device_number);
    }
    /* Remove handle from list */
    if (prev) {
        prev->next = scanner->next;
    } else {
        first_handle = scanner->next;
    }

    /* Free scanner related allocated memory and the scanner itself */
    /*TODO: check if complete */
    free (scanner->buffer.buffer);
    free (scanner->ccd_mask);
    free (scanner->shading_buffer);
    free (scanner->val[OPT_MODE].s);
    free (scanner->val[OPT_HALFTONE_PATTERN].s);
    free (scanner);		
}

/**
 * Get option descriptor. Return the option descriptor with the given index
 * 
 * @param handle Scanner handle
 * @param option Index of option descriptor to return
 * @return The option descriptor
 */
const SANE_Option_Descriptor *
sane_get_option_descriptor (SANE_Handle handle, SANE_Int option)
{
    Reflecta_Scanner *scanner = handle;

    DBG (DBG_sane_option, "sane_get_option_descriptor %d\n", option);

    if ((unsigned) option >= NUM_OPTIONS)
    {
      return 0;
    }

    return scanner->opt + option;
}

/**
 * Set or inquire the current value of option number 'option' of the device
 * represented by the given handle.
 *
 * @param handle Scanner handle
 * @param option Index of option to set or get
 * @param action Determines if the option value is read or set
 * @param val Pointer to value to set or get
 * @param info About set result. May be NULL.
 * @return SANE_STATUS_GOOD, or SANE_STATUS_INVAL if a parameter cannot be set
 */
SANE_Status
sane_control_option (SANE_Handle handle, SANE_Int option, SANE_Action action,
		     void *val, SANE_Int * info)
{
    Reflecta_Scanner *scanner = handle;
    SANE_Status status;
    SANE_Word cap;
    SANE_String_Const name;

    if (info) {
        *info = 0;
    }

    /* Don't set or get options while the scanner is busy */
    if (scanner->scanning) {
        DBG(DBG_sane_option,"Device busy scanning, no option returned\n");
        return SANE_STATUS_DEVICE_BUSY;
    }

    /* Check if option index is between bounds */
    if ((unsigned) option >= NUM_OPTIONS) {
        DBG(DBG_sane_option,"Index too large, no option returned\n");
        return SANE_STATUS_INVAL;
    }

    /* Check if option is switched on */
    cap = scanner->opt[option].cap;
    if (!SANE_OPTION_IS_ACTIVE (cap))
    {
        DBG(DBG_sane_option,"Option inactive (%s)\n", scanner->opt[option].name);
        return SANE_STATUS_INVAL;
    }

    /* Get name of option */
    name = scanner->opt[option].name;
    if (!name)
    {
      name = "(no name)";
    }

    /* */
    switch (action) {
        case SANE_ACTION_GET_VALUE:
            
            DBG (DBG_sane_option, "get %s [#%d]\n", name, option);

            switch (option) {

                /* word options: */
                case OPT_NUM_OPTS:
                case OPT_BIT_DEPTH:
                case OPT_RESOLUTION:
                case OPT_TL_X:
                case OPT_TL_Y:
                case OPT_BR_X:
                case OPT_BR_Y:
                case OPT_THRESHOLD:
                case OPT_SHARPEN:
                case OPT_SKIP_CALIBRATION:
                case OPT_FAST_INFRARED:
                case OPT_PREVIEW:
                case OPT_SHADINGDATA:
                case OPT_CCDMASK:
                    *(SANE_Word *) val = scanner->val[option].w;
                    DBG (DBG_sane_option, "get %s [#%d] val=%d\n", name, option,scanner->val[option].w);
                    return SANE_STATUS_GOOD;

                /* word-array options: => for expposure gain offset?
                case OPT_GAMMA_VECTOR:
                case OPT_GAMMA_VECTOR_R:
                case OPT_GAMMA_VECTOR_G:
                case OPT_GAMMA_VECTOR_B: */
                case OPT_GAIN:
                case OPT_OFFSET:
                case OPT_EXPOSURE:
                    memcpy (val, scanner->val[option].wa, scanner->opt[option].size);
                    return SANE_STATUS_GOOD;

                case OPT_MODE:
                case OPT_HALFTONE_PATTERN:
                    strcpy (val, scanner->val[option].s);
                    DBG (DBG_sane_option, "get %s [#%d] val=%s\n", name, option,scanner->val[option].s);
                    return SANE_STATUS_GOOD;
            }
            break;
            
        case SANE_ACTION_SET_VALUE:
            
            switch (scanner->opt[option].type) {
                case SANE_TYPE_INT:
                    DBG (DBG_sane_option, "set %s [#%d] to %d, size=%d\n", name, option, *(SANE_Word *) val, scanner->opt[option].size);
                    break;
                case SANE_TYPE_FIXED:
                    DBG (DBG_sane_option, "set %s [#%d] to %f\n", name, option, SANE_UNFIX (*(SANE_Word *) val));
                    break;
                case SANE_TYPE_STRING:
                    DBG (DBG_sane_option, "set %s [#%d] to %s\n", name, option, (char *) val);
                    break;
                case SANE_TYPE_BOOL:
                    DBG (DBG_sane_option, "set %s [#%d] to %d\n", name, option, *(SANE_Word *) val);
                    break;
                default:
                    DBG (DBG_sane_option, "set %s [#%d]\n", name, option);
            }
            /* Check if option can be set */
            if (!SANE_OPTION_IS_SETTABLE (cap)) {
              return SANE_STATUS_INVAL;
            }
            /* Check if new value within bounds */
            status = sanei_constrain_value (scanner->opt + option, val, info);
            if (status != SANE_STATUS_GOOD) {
              return status;
            }
            /* Set option and handle info return */
            switch (option)
            {
                /* (mostly) side-effect-free word options: */
                case OPT_BIT_DEPTH:
                case OPT_RESOLUTION:
                case OPT_TL_X:
                case OPT_TL_Y:
                case OPT_BR_X:
                case OPT_BR_Y:
                case OPT_SHARPEN:
                case OPT_SKIP_CALIBRATION:
                case OPT_FAST_INFRARED:
                    if (info) {
                        *info |= SANE_INFO_RELOAD_PARAMS;
                    }
                  /* fall through */
                case OPT_NUM_OPTS:
                case OPT_PREVIEW:
                case OPT_SHADINGDATA:
                case OPT_CCDMASK:
                case OPT_THRESHOLD:
                    scanner->val[option].w = *(SANE_Word *) val;
                    return SANE_STATUS_GOOD;

                /* side-effect-free word-array options:
                case OPT_GAMMA_VECTOR:
                case OPT_GAMMA_VECTOR_R:
                case OPT_GAMMA_VECTOR_G:
                case OPT_GAMMA_VECTOR_B:
                 */
                case OPT_GAIN:
                case OPT_OFFSET:
                case OPT_EXPOSURE:
                    memcpy (scanner->val[option].wa, val, scanner->opt[option].size);
                    return SANE_STATUS_GOOD;

                /* options with side-effects: */

                case OPT_MODE:
                {
                    /* Free current setting */
                    if (scanner->val[option].s) {
                        free (scanner->val[option].s);
                    }
                    /* New setting */
                    scanner->val[option].s = (SANE_Char *) strdup (val);
                    /* Info */
                    if (info) {
                        *info |= SANE_INFO_RELOAD_OPTIONS | SANE_INFO_RELOAD_PARAMS;
                    }
                    return SANE_STATUS_GOOD;
                }

                case OPT_HALFTONE_PATTERN:
                {
                     /* Free current setting */
                    if (scanner->val[option].s) {
                        free (scanner->val[option].s);
                    }
                    /* New setting */
                    scanner->val[option].s = (SANE_Char *) strdup (val);
                    return SANE_STATUS_GOOD;
                  }
            }
            
            break;
        default:
            return SANE_STATUS_INVAL;
            break;
    }
}

/**
 * Obtain the current scan parameters. The returned parameters are guaranteed
 * to be accurate between the time a scan has been started (sane start() has
 * been called) and the completion of that request. Outside of that window, the
 * returned values are best-effort estimates of what the parameters will be when
 * sane start() gets invoked. - says the SANE standard.
 *
 * @param handle Scanner handle
 * @param params Scan parameters
 * @return SANE_STATUS_GOOD
 */
SANE_Status
sane_get_parameters (SANE_Handle handle, SANE_Parameters * params)
{
    Reflecta_Scanner *scanner = handle;
    struct Reflecta_Scan_Parameters parameters;
    struct Reflecta_Command_Status status;
    const char *mode;
    
    DBG (DBG_sane_info, "sane_get_parameters\n");
    
    if (params) {
        if (scanner->scanning) {
            *params = scanner->scan_parameters;
        } else {
            cmdGetScanParameters(scanner->device_number,&parameters, &status, 5);
            if (status.sane_status != SANE_STATUS_GOOD) {
                return SANE_STATUS_IO_ERROR;
            }    
            mode = scanner->val[OPT_MODE].s;
            if (strcmp(mode,SANE_VALUE_SCAN_MODE_LINEART)==0) {
                scanner->scan_parameters.format = SANE_FRAME_GRAY;
                scanner->scan_parameters.depth = 1;
                scanner->scan_parameters.bytes_per_line = parameters.bytes;
                /*TODO check; unsure what LINEART passes should be */
            } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_HALFTONE)==0) {
                scanner->scan_parameters.format = SANE_FRAME_GRAY;
                scanner->scan_parameters.depth = 1;
                scanner->scan_parameters.bytes_per_line = parameters.bytes;
                /*TODO check; unsure what HALFTONE passes should be */
            } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_GRAY)==0) {
                scanner->scan_parameters.format = SANE_FRAME_GRAY;
                scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
                scanner->scan_parameters.bytes_per_line = parameters.bytes;
                /*TODO check; unsure what GRAY passes should be => change get_parameters() as well! */
            } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_RGBI)==0) {
                scanner->scan_parameters.format = SANE_FRAME_RGBI;
                scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
                scanner->scan_parameters.bytes_per_line = 4*parameters.bytes;
            } else { /* SANE_VALUE_SCAN_MODE_COLOR */
                scanner->scan_parameters.format = SANE_FRAME_RGB;
                scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
                scanner->scan_parameters.bytes_per_line = 3*parameters.bytes;
            }
            scanner->scan_parameters.lines = parameters.lines;
            scanner->scan_parameters.pixels_per_line = parameters.width;
            scanner->scan_parameters.last_frame = SANE_TRUE;
        }
    }

    return SANE_STATUS_GOOD;
}

/**
 * Initiates aquisition of an image from the scanner.
 * Carry out Scan Phase 1: calibration
 * (Scan Phase 2: line-by-line scan & read is not implemented)
 *

 * @param handle Scanner handle
 * @return 
 */
SANE_Status
sane_start (SANE_Handle handle)
{
    struct Reflecta_Scanner *scanner = handle;
    struct Reflecta_Sense sense;
    struct Reflecta_Scan_Parameters parameters;
    struct Reflecta_Command_Status status;
    double dpmm;
    SANE_Byte colors;
    const char *mode;
    
    DBG(DBG_info,"sane_start()\n");
    
    /* ----------------------------------------------------------------------
     * 
     * Exit if currently scanning
     * 
     * ---------------------------------------------------------------------- */
    if (scanner->scanning) {
        DBG(DBG_error,"sane_start(): scanner is already scanning, exiting\n");
        return SANE_STATUS_DEVICE_BUSY;
    }

    /* ----------------------------------------------------------------------
     * 
     * Check for option inconsistencies
     * 
     * ---------------------------------------------------------------------- */
    if (scanner->val[OPT_TL_X].w > scanner->val[OPT_BR_X].w) {
        DBG (0, "sane_start: %s (%.1f mm) is bigger than %s (%.1f mm) -- aborting\n",
	   scanner->opt[OPT_TL_X].title,
	   SANE_UNFIX (scanner->val[OPT_TL_X].w),
	   scanner->opt[OPT_BR_X].title,
	   SANE_UNFIX (scanner->val[OPT_BR_X].w));
        return SANE_STATUS_INVAL;
    }
    if (scanner->val[OPT_TL_Y].w > scanner->val[OPT_BR_Y].w) {
        DBG (0, "sane_start: %s (%.1f mm) is bigger than %s (%.1f mm) -- aborting\n",
	   scanner->opt[OPT_TL_Y].title,
	   SANE_UNFIX (scanner->val[OPT_TL_Y].w),
	   scanner->opt[OPT_BR_Y].title,
	   SANE_UNFIX (scanner->val[OPT_BR_Y].w));
        return SANE_STATUS_INVAL;
    }
    /*TODO: is this all? */

    /* ----------------------------------------------------------------------
     * 
     * Exit if not warmed up
     * 
     * ---------------------------------------------------------------------- */
    cmdGetState(scanner->device_number, &(scanner->state), &status, 10);
    if (status.sane_status != SANE_STATUS_GOOD) {
        DBG(DBG_error,"sane_start(): warmed up check returns status %s\n",  sane_strstatus(status.sane_status));
        return SANE_STATUS_IO_ERROR;
    }
    if (scanner->state.warmingUp) {
        DBG(DBG_error,"sane_start(): warming up, exiting\n");
        /* Seen SANE_STATUS_WARMING_UP in scanimage => enabled */
        sleep(2); /* scanimage does not pause, so do it here */
        return SANE_STATUS_WARMING_UP;
    }

    /* ----------------------------------------------------------------------
     * 
     * Standard run does;
     * - set exposure time 0x0A/0x13
     * - set highlight shadow 0x0A/0x14
     * - read shading parameters 0x0A/0x95/0x08
     * - set scan frame 0x0A/0x12
     *   "12 00 0a00 80 00 0300 0000 b829 e31a"
     *    => 0:12 1:0 2:10 4:80 5:0 6:3 8:0 10:10680 12:6883
     * - read gain offset 0xD7
     * - set gain offset 0xDC
     * - set mode 0x15
     *   "00 0f   2c01 80   04  04  00 01    0a     00 00 00  80  10 00"
     *       size res  pass dpt frm    ord   bitmap       ptn thr
     *       15   300  RGB  8   inx    intel 1=sharpen    0   128
     *                                       3=skipcal
     * 
     * ---------------------------------------------------------------------- */
    /* Scan frame */
    dpmm = (double) scanner->device->maximum_resolution / MM_PER_INCH;
    scanner->frame.x0 = SANE_UNFIX(scanner->val[OPT_TL_X].w) * dpmm;
    scanner->frame.y0 = SANE_UNFIX(scanner->val[OPT_TL_Y].w) * dpmm;
    scanner->frame.x1 = SANE_UNFIX(scanner->val[OPT_BR_X].w) * dpmm;
    scanner->frame.y1 = SANE_UNFIX(scanner->val[OPT_BR_Y].w) * dpmm;
    scanner->frame.code = 0x12;
    scanner->frame.index = 0x00;
    scanner->frame.size = 0x0A;
    cmdSetScanFrame(scanner->device_number,0,&(scanner->frame), &status, 0);
    DBG(DBG_info,"sane_start(): cmdSetScanFrame status %s\n",sane_strstatus(status.sane_status));
    if (status.sane_status != SANE_STATUS_GOOD) {
        return SANE_STATUS_IO_ERROR;
    }    
    /*TODO: might use command to get initial settings as well */
    /* cmdGetOptimizedSettings(scanner->device_number,&scanner->settings); */
    scanner->settings.exposureTime[0] = scanner->val[OPT_EXPOSURE].wa[0];
    scanner->settings.exposureTime[1] = scanner->val[OPT_EXPOSURE].wa[1];
    scanner->settings.exposureTime[2] = scanner->val[OPT_EXPOSURE].wa[2];
    scanner->settings.exposureTime[3] = scanner->val[OPT_EXPOSURE].wa[3]; /* Infrared */
    scanner->settings.offset[0] = scanner->val[OPT_OFFSET].wa[0];
    scanner->settings.offset[1] = scanner->val[OPT_OFFSET].wa[1];
    scanner->settings.offset[2] = scanner->val[OPT_OFFSET].wa[2];
    scanner->settings.offset[3] = scanner->val[OPT_OFFSET].wa[3]; /* Infrared */
    scanner->settings.gain[0] = scanner->val[OPT_GAIN].wa[0];
    scanner->settings.gain[1] = scanner->val[OPT_GAIN].wa[1];
    scanner->settings.gain[2] = scanner->val[OPT_GAIN].wa[2];
    scanner->settings.gain[3] = scanner->val[OPT_GAIN].wa[3]; /* Infrared */
    scanner->settings.light = 0x04;
    scanner->settings.extraEntries = 0x00;
    scanner->settings.doubleTimes = 0x00;
    cmdSetGainOffset(scanner->device_number,&(scanner->settings), &status, 0);
    DBG(DBG_info,"sane_start(): cmdSetSettings status %s\n",sane_strstatus(status.sane_status));
    if (status.sane_status != SANE_STATUS_GOOD) {
        return SANE_STATUS_IO_ERROR;
    }    
    /* Mode settings & basic initialization of SANE parameters */
    mode = scanner->val[OPT_MODE].s;
    if (strcmp(mode,SANE_VALUE_SCAN_MODE_LINEART)==0) {
        scanner->mode.passes = 0x04; /* G */
        scanner->mode.colorFormat = 0x01;
        scanner->scan_parameters.format = SANE_FRAME_GRAY;
        scanner->scan_parameters.depth = 1;
        /*TODO check; unsure what LINEART passes should be */
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_HALFTONE)==0) {
        scanner->mode.passes = 0x04; /* G */
        scanner->mode.colorFormat = 0x01;
        scanner->scan_parameters.format = SANE_FRAME_GRAY;
        scanner->scan_parameters.depth = 1;
        /*TODO check; unsure what HALFTONE passes should be */
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_GRAY)==0) {
        scanner->mode.passes = 0x04; /* G? */
        scanner->mode.colorFormat = 0x01;
        scanner->scan_parameters.format = SANE_FRAME_GRAY;
        scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
        /*TODO check; unsure what GRAY passes should be => change get_parameters() as well! */
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_RGBI)==0) {
        scanner->mode.passes = 0x90;
        scanner->mode.colorFormat = 0x04; /* pixel format does not seem to support infrared */
        scanner->scan_parameters.format = SANE_FRAME_RGBI;
        scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
    } else { /* SANE_VALUE_SCAN_MODE_COLOR */
        scanner->mode.passes = 0x80;
        scanner->mode.colorFormat = 0x04; /* pixel format might be an alternative */
        scanner->scan_parameters.format = SANE_FRAME_RGB;
        scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;
    }
    scanner->mode.resolution = SANE_UNFIX(scanner->val[OPT_RESOLUTION].w);
    switch (scanner->val[OPT_BIT_DEPTH].w) {
        case 1: scanner->mode.colorDepth = 0x01; break;
        case 4: scanner->mode.colorDepth = 0x02; break;
        case 8: scanner->mode.colorDepth = 0x04; break;
        case 10: scanner->mode.colorDepth = 0x08; break;
        case 12: scanner->mode.colorDepth = 0x10; break;
        case 16: scanner->mode.colorDepth = 0x20; break;
    }
    scanner->mode.byteOrder = 0x01; /* 0x01 = Intel; only bit 0 used */
    scanner->mode.sharpen = scanner->val[OPT_SHARPEN].b;
    scanner->mode.skipCalibration = scanner->val[OPT_SKIP_CALIBRATION].b;
    scanner->mode.fastInfrared = scanner->val[OPT_FAST_INFRARED].b;
    scanner->mode.halftonePattern = scanner->val[OPT_HALFTONE_PATTERN].w;
    scanner->mode.lineThreshold = SANE_UNFIX(scanner->val[OPT_THRESHOLD].w) * 0xFF; /* 0xFF = 100% */
    cmdSetMode(scanner->device_number,&(scanner->mode), &status, 0);
    DBG(DBG_info,"sane_start(): cmdSetMode status %s\n",sane_strstatus(status.sane_status));
    if (status.sane_status != SANE_STATUS_GOOD) {
        return SANE_STATUS_IO_ERROR;
    }    

    /* Enter SCAN phase 1 */
    
    /* ----------------------------------------------------------------------
     * 
     * Start scan & wait until device ready
     * 
     * ---------------------------------------------------------------------- */
    scanner->scanning = SANE_TRUE;
    scanner->cancel_request = SANE_FALSE;
    cmdStartScan(scanner->device_number, &status, 10);
    /* Default status check */
    if (status.sane_status == SANE_STATUS_GOOD) {
        /* OK, proceed */
    } else if (status.sane_status == SANE_STATUS_CHECK_CONDITION) {
        /* May be a case of overriding skip calibration */
        if (scanner->mode.skipCalibration && status.senseCode!=0x06 && status.senseKey==0x82 && status.senseQualifier==0x00) {
            scanner->mode.skipCalibration = SANE_FALSE;
        } else {
            /* Other sense */
            DBG(DBG_error,"sane_start(): sense %02x:%02x-%02x\n",status.senseCode,status.senseKey,status.senseQualifier);
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_IO_ERROR;
        }
    } else {
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }
    /* Wait loop 1 */
    cmdIsUnitReady(scanner->device_number, &status, 60);
    if (status.sane_status != SANE_STATUS_GOOD) {
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }
    /* Wait loop 2*/
    cmdIsUnitReady(scanner->device_number, &status, 60);
    if (status.sane_status != SANE_STATUS_GOOD) {
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }
    
    /* Process shading data if requested */
    if (!scanner->mode.skipCalibration) {
        
        /* Handle cancel request */
        if (scanner->cancel_request) {
            cmdStopScan(scanner->device_number, &status, 5);
            cmdSetScanHead(scanner->device_number, 1, 0, &status, 10);
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_CANCELLED;
        }

        /* ------------------------------------------------------------------
         * 
         * Get and set gain and offset
         * 
         * ------------------------------------------------------------------ */
        cmdGetGainOffset(scanner->device_number,&scanner->settings, &status,10);
        if (status.sane_status != SANE_STATUS_GOOD) {
            cmdStopScan(scanner->device_number, &status, 5);
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_IO_ERROR;
        }    
        cmdSetGainOffset(scanner->device_number,&scanner->settings, &status,10);
        if (status.sane_status != SANE_STATUS_GOOD) {
            cmdStopScan(scanner->device_number, &status, 5);
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_IO_ERROR;
        }    

        /* ------------------------------------------------------------------
         * 
         * Obtain shading data & wait until device ready
         * 
         * ------------------------------------------------------------------ */
        SANE_Int lines = 4 * 45; /* 4 colors, 45 scan lines */
        SANE_Int line_size = 10682; /* 5340 pixels on a line, 2 bytes (16 bits) per pixel + 2 index bytes */
        cmdGetScannedLines(scanner->device_number,scanner->shading_buffer,lines,lines*line_size, &status, 5);
        if (status.sane_status != SANE_STATUS_GOOD) {
            cmdStopScan(scanner->device_number, &status, 5);
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_IO_ERROR;
        }
        
        /* Export shading data as TIFF */
        if (scanner->val[OPT_SHADINGDATA].b) {
            struct Reflecta_Read_Buffer shading;
            SANE_Char* lboff = scanner->shading_buffer;
            SANE_Int bpl = 5340*2+2;
            SANE_Int n;
            SANE_Int bufsize = 5340*45*2*4;
            buffer_create(&shading, 5340, 45, 0x0F, 16, SANE_FALSE, bufsize);
            for (n=0; n<4*45; n++) {
                if (buffer_put(&shading, lboff, bpl) == 0) {
                    break;
                }
                lboff += bpl;
            }
            FILE* fs = fopen("reflecta.shading", "w");
            write_tiff_rgbi_header (fs, 5340, 45, 16, 3600, NULL);
            fwrite(shading.buffer, 1, bufsize, fs);
            fclose(fs);
            buffer_delete(&shading);
        }
        
        /* Wait loop */
        cmdIsUnitReady(scanner->device_number, &status, 60);
        if (status.sane_status != SANE_STATUS_GOOD) {
            scanner->scanning = SANE_FALSE;
            return SANE_STATUS_IO_ERROR;
        }
    }

    /* Enter SCAN phase 2 */
    
    /* Line-by-line scan phase not implemented */
    
    /* Enter SCAN phase 3 */
    
    /* Handle cancel request */
    if (scanner->cancel_request) {
        cmdStopScan(scanner->device_number, &status, 5);
        cmdSetScanHead(scanner->device_number, 1, 0, &status, 10);
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_CANCELLED;
    }
    
    /* ----------------------------------------------------------------------
     * 
     * Get CCD mask
     * 
     * ---------------------------------------------------------------------- */
    cmdGetCCDMask(scanner->device_number, scanner->ccd_mask, &status, 20);
    if (status.sane_status != SANE_STATUS_GOOD) {
        cmdStopScan(scanner->device_number, &status, 5);
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }    
    /* Wait loop */
    cmdIsUnitReady(scanner->device_number, &status, 60);
    if (status.sane_status != SANE_STATUS_GOOD) {
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }

    /* Save CCD mask */
    if (scanner->val[OPT_CCDMASK].b) {
        FILE* fs = fopen("reflecta.ccd", "w");
        fwrite(scanner->ccd_mask, 1, 5340, fs);
        fclose(fs);
    }
    
    /* Enter SCAN phase 4 */

    /* ----------------------------------------------------------------------
     * 
     * Read scan parameters & wait until ready for reading
     * 
     * ---------------------------------------------------------------------- */
    cmdGetScanParameters(scanner->device_number,&parameters, &status, 5);
    if (status.sane_status != SANE_STATUS_GOOD) {
        cmdStopScan(scanner->device_number, &status, 5);
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }    
    /* Wait loop */
    cmdIsUnitReady(scanner->device_number, &status, 60);
    if (status.sane_status != SANE_STATUS_GOOD) {
        scanner->scanning = SANE_FALSE;
        return SANE_STATUS_IO_ERROR;
    }
    /* Use response from cmdGetScanParameters() for final initialization of SANE parameters
     * Note: interpretation of parameters.width varies with colorFormat setting. This
     * is initialized above. */
    mode = scanner->val[OPT_MODE].s;
    if (strcmp(mode,SANE_VALUE_SCAN_MODE_LINEART)==0) {
        scanner->scan_parameters.bytes_per_line = parameters.bytes;
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_HALFTONE)==0) {
        scanner->scan_parameters.bytes_per_line = parameters.bytes;
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_GRAY)==0) {
        scanner->scan_parameters.bytes_per_line = parameters.bytes;
        /*TODO check; unsure what GRAY passes should be => change get_parameters() as well! */
    } else if(strcmp(mode,SANE_VALUE_SCAN_MODE_RGBI)==0) {
        scanner->scan_parameters.bytes_per_line = 4*parameters.bytes;
    } else { /* SANE_VALUE_SCAN_MODE_COLOR */
        scanner->scan_parameters.bytes_per_line = 3*parameters.bytes;
    }
    scanner->scan_parameters.lines = parameters.lines;
    scanner->scan_parameters.pixels_per_line = parameters.width;
    scanner->scan_parameters.last_frame = SANE_TRUE;
    scanner->scan_parameters.depth = scanner->val[OPT_BIT_DEPTH].w;    
    
    /* Temporary error exit */
    if (scanner->mode.colorFormat != 0x04) {
        DBG(DBG_error,"sane_start(): currently only equiped to handle INDEX color format\n");
        return SANE_STATUS_INVAL;
    }
    
    /* Prepare buffer */
    switch (scanner->mode.passes) {
        case 0x02: colors = 0x01; break;
        case 0x04: colors = 0x02; break;
        case 0x08: colors = 0x04; break;
        case 0x10: colors = 0x08; break;
        case 0x80: colors = 0x07; break;
        case 0x90: colors = 0x0F; break;
    }
    buffer_create(&(scanner->buffer), scanner->scan_parameters.pixels_per_line, scanner->scan_parameters.lines,
      colors, scanner->scan_parameters.depth, SANE_FALSE, READ_BUFFER_SIZE);
    
    /* Check if buffer is sufficiently large - larger? */
    if (READ_BUFFER_SIZE < scanner->scan_parameters.bytes_per_line) {
        DBG(DBG_error,"sane_start(): scanner buffer too small (%d, need at least %d)\n",READ_BUFFER_SIZE,scanner->scan_parameters.bytes_per_line);
        return SANE_STATUS_INVAL;
    }
    
    DBG(DBG_info,"sane_start(): SANE parameters\n");
    DBG(DBG_info," format = %d\n",scanner->scan_parameters.format);
    DBG(DBG_info," last_frame = %d\n",scanner->scan_parameters.last_frame);
    DBG(DBG_info," bytes_per_line = %d\n",scanner->scan_parameters.bytes_per_line);
    DBG(DBG_info," pixels_per_line = %d\n",scanner->scan_parameters.pixels_per_line);
    DBG(DBG_info," lines = %d\n",scanner->scan_parameters.lines);
    DBG(DBG_info," depth = %d\n",scanner->scan_parameters.depth);
    
    /* This ends the initial phases. See sane_read() for the rest. */

    return SANE_STATUS_GOOD;
    
}

/**
 * Read image data from the scanner.
 *
 * SCAN phase 4: scan slide and output scan data
 *
 * @param handle
 * @param buf
 * @param max_len
 * @param len
 * @return 
 */
SANE_Status
sane_read (SANE_Handle handle, SANE_Byte * buf, SANE_Int max_len, SANE_Int * len)
{
    
    struct Reflecta_Scanner *scanner = handle;
    struct Reflecta_Command_Status status;
    struct Reflecta_Scan_Parameters parameters;
    int k, n;
    SANE_Int bpl; /* bytes per line */
    
    DBG(DBG_info,"sane_read(): reading image data (requested %d bytes)\n",max_len);

    /* No reading if not scanning */
    if (!scanner->scanning) {
        *len = 0;
        return SANE_STATUS_IO_ERROR; /* SANE standard does not allow a SANE_STATUS_INVAL return */
    }
    
    /* Enter SCAN phase 4 */
    
    /* Handle cancel request */
    if (scanner->cancel_request) {
        return reflecta_on_cancel(scanner);
    }

    /* Return image data, read from scanner if necessary */
    SANE_Int return_size;
    if (scanner->buffer.nRead == scanner->buffer.sizeImage) {
        /* Return EOF since all data of this frame has already been read */
        cmdSetScanHead(scanner->device_number, 1, 0, &status, 10);
        scanner->scanning = SANE_FALSE;
        *len = 0;
        return SANE_STATUS_EOF;
    } else if (scanner->buffer.nData >= max_len) {
        /* Already enough data to return, do not read */
        DBG(DBG_info,"sane_read(): buffer suffices (contains %d, requested %d)\n", scanner->buffer.nData, max_len);
        return_size = max_len;
    } else if (scanner->buffer.nRead + scanner->buffer.nData == scanner->buffer.sizeImage) {
        /* All the remaining data is in the buffer, do not read */
        DBG(DBG_info,"sane_read(): buffer suffices (contains %d, requested %d, last batch though)\n", scanner->buffer.nData, max_len);
        return_size = scanner->buffer.nData;
    } else {
        /* Data must be read from the scanner or the remaining buffer data must
         * be returned.
         * Determine the number of lines that should be read. Try to read the
         * maximum amount of lines that fit into the buffer, which is usually
         * more than the scanner has available. Apart from that, if one of the
         * colors lags behind, reading the available lines may not result in
         * having complete lines available for output.
         * But: if we don't read one or more lines here, that is an error.
         * That means we try reading at least once, but loop until the buffer
         * contains data.
         */
        do {
            /* Determine room in buffer */
            SANE_Int linesToRead = 0;
            int room_in_buffer = 0;
            for (k=0; k<scanner->buffer.nHeight; k++) {
                if (scanner->buffer.complete[k] == 0) room_in_buffer++;
            }
            /* Single color lines read in 'index' mode
             * Don't attempt to use full buffer, since the colors do not
             * arrive in a fixed regular order. */
            linesToRead = scanner->buffer.nColors * room_in_buffer / 2;
            /* Check: eror exit if linesToRead == 0 */
            if (linesToRead == 0) {
                DBG(DBG_error,"sane_read(): no room in buffer (buffer size = %d)\n", scanner->buffer.size);
                *len = 0;
                return SANE_STATUS_IO_ERROR;
            }
            DBG(DBG_info,"sane_read(): trying to read %d single color lines\n", linesToRead);
            /* Get available lines. Wait & retry until this number is larger
             * than zero. The SANE standard requires us to return at least one
             * byte... */
            int linesAvailable = 0;
            int tries = 0;
            while (linesAvailable == 0 && tries < 10) {
                cmdGetScanParameters(scanner->device_number,&parameters, &status, 5);
                if (status.sane_status != SANE_STATUS_GOOD) {
                    /* Error, return */
                    *len = 0;
                    return SANE_STATUS_IO_ERROR;
                }
                tries++;
                linesAvailable = parameters.availableLines;
                if (linesAvailable == 0) sleep(2);
            }
            /* Read at most the available lines */
            if (linesToRead > parameters.availableLines) {
                linesToRead = parameters.availableLines;
            }
            DBG(DBG_info,"sane_read(): available lines = %d => reading %d lines\n", parameters.availableLines, linesToRead);
            /* Check: if linesToRead == 0 that is an error */
            if (linesToRead == 0) {
                DBG(DBG_error,"sane_read(): scanner does not seem to have any more data available, perhaps time-out\n");
                *len = 0;
                return SANE_STATUS_IO_ERROR;
            }
            /* Read lines */
            /* The amount of bytes_per_line varies with color format setting; only 'index' implemented */
            bpl = scanner->scan_parameters.bytes_per_line/scanner->buffer.nColors + 2; /* Index bytes! */
            DBG(DBG_info,"sane_read(): reading lines: bytes per line = %d\n",bpl);
            SANE_Char* linebuf = (SANE_Char*)malloc(linesToRead*bpl);
            cmdGetScannedLines(scanner->device_number, linebuf, linesToRead, linesToRead*bpl, &status, 5);
            if (status.sane_status != SANE_STATUS_GOOD ) {
                /* Error, return */
                *len = 0;
                return SANE_STATUS_IO_ERROR;
            }
            /* Copy into official buffer */
            SANE_Char* lboff = linebuf;
            for (n=0; n<linesToRead; n++) {
                if (buffer_put(&scanner->buffer, lboff, bpl) == 0) {
                    /* Error, return */
                    *len = 0;
                    return SANE_STATUS_IO_ERROR;
                }
                lboff += bpl;
            }
            free(linebuf);
        } while (scanner->buffer.nData == 0);
        /* Determine how many bytes to return */
        return_size = scanner->buffer.nData < max_len ? scanner->buffer.nData : max_len;
    }
    
    /* Check */
    if (return_size == 0 && scanner->buffer.nRead < scanner->buffer.sizeImage) {
        DBG(DBG_error,"sane_read(): unable to service read request, %d bytes in frame, %d read\n", scanner->buffer.sizeImage, scanner->buffer.nRead);
    }
    
    /* Return the available data: Output return_size bytes from buffer */
    buffer_get(&scanner->buffer, buf, max_len, len);
    DBG(DBG_info,"sane_read(): currently read %.2f lines of %d\n", (double)scanner->buffer.nWritten/(scanner->buffer.nColors*scanner->buffer.nSingleColorLineWidth), scanner->scan_parameters.lines);
    DBG(DBG_info,"sane_read(): returning %d bytes (requested %d), returned %d of %d \n", *len, max_len,scanner->buffer.nRead, scanner->buffer.sizeImage);
    return SANE_STATUS_GOOD;

}

/**
 * Request cancellation of current scanning process.
 * 
 * @param handle Scanner handle
 */
void
sane_cancel (SANE_Handle handle)
{
    struct Reflecta_Scanner *scanner = handle;

    DBG (DBG_sane_init, "sane_cancel\n");

    if (scanner->scanning) {
        scanner->cancel_request = 1;
    }
}

/**
 * Set the I/O mode of handle h. The I/O mode can be either blocking or
 * non-blocking, but for USB devices, only blocking mode is supported.
 *
 * @param handle Scanner handle
 * @param non_blocking
 * @return SANE_STATUS_UNSUPPORTED;
 */
SANE_Status
sane_set_io_mode (SANE_Handle handle, SANE_Bool non_blocking)
{
    /* Reflecta_Scanner *scanner = handle; */

    DBG (DBG_proc, "sane_set_io_mode: handle = %p, non_blocking = %s\n", handle, non_blocking == SANE_TRUE ? "true" : "false");

    if (non_blocking) {
	return SANE_STATUS_UNSUPPORTED;
    }

    return SANE_STATUS_GOOD;
}

/**
 * Obtain a file-descriptor for the scanner that is readable if image data is
 * available. The select file-descriptor is returned in *fd.
 * The function has not been implemented yet.
 *
 * @param handle Scanner handle
 * @param fd File descriptor with imae data
 * @return SANE_STATUS_INVAL
 */
SANE_Status
sane_get_select_fd (SANE_Handle handle, SANE_Int * fd)
{
    DBG(DBG_info,"sane_get_select_fd(): not supported (only for non-blocking IO)\n");
    return SANE_STATUS_UNSUPPORTED;
}


/* --------------------------------------------------------------------------
 *
 * SPECIFIC REFLECTA 
 * 
 * --------------------------------------------------------------------------*/

#include "reflecta_buffer.c"

#include "reflecta_specific.c"

/* =========================================================================
 * 
 * Reflecta scanner commands
 * 
 * ========================================================================= */

#include "reflecta_scancmd.c"

/* =========================================================================
 * 
 * USB functions
 * 
 * ========================================================================= */

#include "reflecta_usb.c"