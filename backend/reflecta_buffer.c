/*
 * INCLUDE FILE FOR REFLECTA.C
 */

#include "reflecta_specific.h"
#include "reflecta_buffer.h"

/* READER */

/**
 * Initialize the buffer.
 * A scanner has a Reflecta_Read_Buffer struct as one of its members.
 * 
 * @param buffer the buffer to initialize
 * @param width number of pixels on a line (row)
 * @param height number of lines in the buffer (pixels in a column)
 * @param colors bitmap specifying the colors in the scanned data (bitmap: 0000 IBGR)
 * @param depth number of bits of a color 
 * @param bigendian how to store multi-byte values: bigendian if true
 */
static void buffer_create(struct Reflecta_Read_Buffer* buffer, SANE_Int width, SANE_Int height, SANE_Byte colors, SANE_Byte depth, SANE_Bool bigendian, SANE_Int maximum_size)
{
    int k, line_size;
    
    /* Base parameters */
    buffer->nWidth = width;
    buffer->nColors = 0;
    if (colors & 0x01) { buffer->colors[buffer->nColors++] = 'R'; }
    if (colors & 0x02) { buffer->colors[buffer->nColors++] = 'G'; }
    if (colors & 0x04) { buffer->colors[buffer->nColors++] = 'B'; }
    if (colors & 0x08) { buffer->colors[buffer->nColors++] = 'I'; }
    for (k=buffer->nColors; k<4; k++) { buffer->colors[k] = '\0'; }
    if (buffer->nColors == 0) {
        DBG(DBG_error, "buffer_create(): no colors specified\n");
        return;
    }
    buffer->nDepth = depth;
    if (depth < 1 || depth > 16) {
        DBG(DBG_error, "buffer_create(): unsupported depth %d\n", depth);
        return;
    }
    buffer->bigendian = bigendian;
    /* Buffer space */
    if (buffer->nDepth == 1) {
        /* Assume packed per color, so each color gets an integer amount of bytes */
        buffer->nSingleColorLineWidth = (buffer->nWidth * buffer->nDepth + 7) / 8;
    } else {
        buffer->nSingleColorLineWidth = buffer->nWidth * (buffer->nDepth<=8 ? 1 : 2);
    }
    /* Size takes maximum into account */
    line_size = buffer->nColors * buffer->nSingleColorLineWidth;
    buffer->sizeImage = line_size * height;
    buffer->nHeight = (height * line_size > maximum_size) ? maximum_size/line_size : height;
    buffer->size = buffer->nHeight * line_size;
    buffer->buffer = (SANE_Byte*)malloc(buffer->size);
    /* State */
    buffer->pRead = buffer->buffer;
    buffer->iRead = -1;
    for (k=0; k<4; k++) { buffer->iWrite[k] = 0; }
    buffer->complete = (SANE_Byte*)malloc(buffer->nHeight);
    for (k=0; k<buffer->nHeight; k++) { buffer->complete[k] = 0; }
    /* Statistics */
    buffer->nRead = 0;
    buffer->nWritten = 0;
    buffer->nData = 0;
    
    DBG(DBG_info,"Read buffer created: w=%d h=%d ncol=%d depth=%d bigend=%d\n",
      buffer->nWidth, buffer->nHeight, buffer->nColors, buffer->nDepth, buffer->bigendian);
}

static void buffer_delete(struct Reflecta_Read_Buffer* buffer)
{
    free(buffer->buffer);
    free(buffer->complete);
    buffer->nWidth = 0;
    buffer->nHeight = 0;
    buffer->nDepth = 0;
    buffer->nColors = 0;
    buffer->bigendian = SANE_FALSE;
    
    DBG(DBG_info,"Read buffer deleted\n");
}

/**
 * Add an indexed line to the reader buffer.
 * 
 * @param buffer
 * @param line
 * @param size Number of bytes in line, including two index bytes
 * @return 1 if successful, 0 if not
 */
static SANE_Int buffer_put(struct Reflecta_Read_Buffer* buffer, SANE_Byte* line, SANE_Int size)
{

    SANE_Int i, k;
    SANE_Byte* p;
    
    DBG(DBG_info,"buffer_put() entered\n");

    /* Check index code */
    i = -1;
    for (k=0; k<4; k++) {
        if (buffer->colors[k] != '\0' && buffer->colors[k] == *line) {
            i = k;
            break;
        }
    }
    if (i == -1) {
        DBG(DBG_error, "buffer_put(): color '%c' not specified when buffer was created\n", *line);
        return 0;
    }
    DBG(DBG_info,"buffer_put() line color = %d (0=R, 1=G, 2=B, 3=I)\n",i);
    
    /* Check if we'll be writing into unread data */
    if (buffer->complete[buffer->iWrite[i]] == buffer->nColors) {
        DBG(DBG_error, "buffer_put(): attempt to write into unread data, line %d is complete\n",buffer->iWrite[i]);
        return 0;
    }
    
    /* Check line size (for a line with a single color) */
    if (buffer->nSingleColorLineWidth != (size-2)) {
        DBG(DBG_error, "buffer_put(): incorrect line size, expecting %d, got %d\n", buffer->nSingleColorLineWidth, size);
        return 0;
    }
    
    /* Store in buffer, distinguish four situations */
    p = buffer->buffer + buffer->nColors*buffer->nSingleColorLineWidth*buffer->iWrite[i]; /* first byte of line to write to */
    if (buffer->nDepth == 1) {
        /* Packed */
        p += i; /* color offset */
        for (k=2; k<size; k++) {
            *p = *(line+k);
            p += buffer->nColors;
        }
    } else if (buffer->nDepth <= 8) {
        /* Regular single byte color data */
        p += i; /* color offset */
        for (k=2; k<size; k++) {
            *p = *(line+k);
            p += buffer->nColors;
        }
    } else if (buffer->bigendian) {
        /* Big-endian two-byte color data */
        p += 2*i; /* color offset */
        for (k=2; k<size; k+=2) {
            /* Reverse bytes */
            *(p+1) = *(line+k);
            *p = *(line+k+1);
            p += 2*buffer->nColors;
        }
    } else {
        /* Little-endian two-byte color data */
        p += 2*i; /* color offset */
        for (k=2; k<size; k+=2) {
            /* Keep the scanner order */
            *p = *(line+k);
            *(p+1) = *(line+k+1);
            p += 2*buffer->nColors;
        }
    }
    
    /* Update state & statistics; necessary if we do not want to get lost */
    buffer->complete[buffer->iWrite[i]]++;
    if (buffer->complete[buffer->iWrite[i]] == buffer->nColors) buffer->nData += buffer->nColors * buffer->nSingleColorLineWidth;
    buffer->iWrite[i]++;
    if (buffer->iWrite[i] == buffer->nHeight) buffer->iWrite[i] = 0;
    buffer->nWritten += (size-2);

    /* Output current buffer state */
    buffer_output_state(buffer);
    
    return 1;
}

/**
 * Return bytes from the buffer. Do not mind pixel boundaries.
 * 
 * @param buffer Buffer to return bytes from.
 * @param data Byte array to return bytes in
 * @param max_len Maximum number of bytes returned 
 * @param len Actual number of bytes returned
 */
static void buffer_get(struct Reflecta_Read_Buffer* buffer, SANE_Byte* data, SANE_Int max_len, SANE_Int* len)
{
    SANE_Int line_size;
    SANE_Byte *read_limit, *pdata;
    SANE_Int k, n, i;
    
    DBG(DBG_info,"buffer_get() entered\n");

    /* Line size & read limit */
    line_size = buffer->nSingleColorLineWidth * buffer->nColors;
    n = buffer->iRead;
    while (buffer->complete[n] == buffer->nColors) {
        n++;
        if (n==buffer->nHeight) n=0;
    }
    read_limit = buffer->buffer + line_size*n;
    
    /* Start reading at pRead */
    pdata = data;
    n = 0;
    while (n < max_len && buffer->pRead != read_limit) {
        *pdata++ = *buffer->pRead++;
        if (buffer->pRead == buffer->buffer + buffer->size) buffer->pRead = buffer->buffer;
        n++;
    }
    
    /* Update state */
    *len = n;
    i = (buffer->pRead-buffer->buffer)/line_size; /* line index of read pointer */
    /* Set all previous lines to free */
    while (buffer->iRead != i) {
        buffer->complete[buffer->iRead] = 0;
        buffer->iRead++;
        if (buffer->iRead == buffer->nHeight) buffer->iRead = 0;
    }
    
    /* Update statistics */
    buffer->nRead += n;
    buffer->nData -= n;

    /* Output current buffer state */
    buffer_output_state(buffer);    
}

static void buffer_output_state(struct Reflecta_Read_Buffer* buffer)
{
    SANE_Int line_size;
    SANE_Int k;
    
    line_size = buffer->nSingleColorLineWidth * buffer->nColors; /* Full line size in bytes */
    
    DBG(DBG_info,"Buffer data\n");
    DBG(DBG_info,"  width/height/colors/depth = %d %d %d %d (buffer size %d)\n",
        buffer->nWidth, buffer->nHeight, buffer->nColors, buffer->nDepth, buffer->size);

    /* Complete lines */
    SANE_Int c = -1; /* 0 = free, 1 = incomplete, nColors = complete */
    SANE_Int sb, se;
    for (k=0; k<buffer->nHeight; k++) {
        if (c == -1) {
            /* Initial */
            c = buffer->complete[k];
            sb = k;
            se = k;
        } else if ((c==0 && buffer->complete[k]!=0) ||
                   (c==buffer->nColors && buffer->complete[k]!=buffer->nColors) ||
                   (c>0 && c<buffer->nColors && (buffer->complete[k]==0 || buffer->complete[k]==buffer->nColors))) {
            /* New section*/
            if (c == 0) {
                DBG(DBG_info,"  free:   %4d-%4d (%d lines)\n", sb, se, se-sb+1);
            } else if (c == buffer->nColors) {
                DBG(DBG_info,"  compl:  %4d-%4d (%d lines)\n", sb, se, se-sb+1);
            } else {
                DBG(DBG_info,"  incmpl: %4d-%4d (%d lines)\n", sb, se, se-sb+1);
            }
            c = buffer->complete[k];
            sb = k;
            se = k;
        } else {
            /* Extend series */
            se = k;
        }
    }
    /* Final section */
    if (c == 0) {
        DBG(DBG_info,"  free:   %4d-%4d (%d lines)\n", sb, se, se-sb+1);
    } else if (c == buffer->nColors) {
        DBG(DBG_info,"  compl:  %4d-%4d (%d lines)\n", sb, se, se-sb+1);
    } else {
        DBG(DBG_info,"  incmpl: %4d-%4d (%d lines)\n", sb, se, se-sb+1);
    }
    
    /* Summary */
    if (buffer->iRead == -1) { 
        DBG(DBG_info,"  reading at: not reading yet\n");
    } else {
        DBG(DBG_info,"  reading at: line = %d, offset = %d\n",
            buffer->iRead, buffer->pRead-(buffer->buffer+line_size*buffer->iRead));
    }
    DBG(DBG_info,"  writing at: lines = %d:%d:%d:%d\n",
        buffer->iWrite[0], buffer->iWrite[1], buffer->iWrite[2], buffer->iWrite[3]);
    
    /* Progress */
    double fdata = (double)buffer->nData/buffer->sizeImage*100;
    double fread = (double)buffer->nRead/buffer->sizeImage*100;
    double fwritten = (double)buffer->nWritten/buffer->sizeImage*100;
    DBG(DBG_info,"  byte counts: image = %d, data = %d (%.0f%%), read = %d (%.0f%%), written = %d (%.0f%%)\n",
        buffer->sizeImage, buffer->nData, fdata, buffer->nRead, fread, buffer->nWritten, fwritten);
    DBG(DBG_info,"  line counts: image = %.1f, data = %.1f, read = %.1f, written = %.1f\n",
        (double)buffer->sizeImage/line_size, (double)buffer->nData/line_size, (double)buffer->nRead/line_size, (double)buffer->nWritten/line_size);
    
}