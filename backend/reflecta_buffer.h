/* 
 * File:   reflecta_buffer.h
 * Author: jan
 *
 * Created on September 1, 2012, 3:30 PM
 */

/*
 * Read buffer
 * 
 * Data obtained from the scanner cannot be presented to the frontend immediately.
 * The scanner returns data in the 'index' color format, which means it returns
 * data in batches which contain a single color of a scan line.
 * These must be converted into the SANE data format (data for a single pixel
 * in consecutive bytes). Apart from that, sane_read() must be able to return
 * any amount of data bytes.
 * 
 * The read buffer is constructed by a call to buffer_create(), which initializes
 * the buffer based on width, height, number of colors and depth. The buffer
 * contains bytes organized in lines, where each line consists of a fixed number
 * of pixels, each pixel of a fixed number of colors, and each color of a fixed
 * number of bits (or bytes). 
 * 
 * Reading from the buffer only requires incrementing a byte pointer. Reading
 * should check that data is returned from complete lines. The buffer maintains
 * a read pointer and a current read line index.
 * 
 * Writing data into the buffer is somewhat more complex since the data must be
 * converted. The buffer maintains current write line indices for each color in
 * the buffer, and derives a free line index and a incomplete line index from
 * these. The free line index indicates the first line which contains no data
 * yet, the incomplete line index indicates the first line which data is incomplete
 * (at least one color has been written).
 * 
 * Multi-color data with a bit depth of 1 are packed in single color bytes, so
 * the data obtained from the scanner does not need conversion.
 */

#ifndef REFLECTA_BUFFER_H
#define	REFLECTA_BUFFER_H

struct Reflecta_Read_Buffer
{
    SANE_Byte* buffer;
    /* Buffer parameters */
    SANE_Int nWidth; /* number of pixels on a line */
    SANE_Int nHeight; /* number of lines in buffer */
    SANE_Int nColors; /* number of colors in a pixel */
    SANE_Int nDepth; /* number of bits of a color */
    SANE_Bool bigendian; /* multi-byte value endianess */
    /* Derived quantities */
    SANE_Int nBits; /* number of bits in a color specification */
    SANE_Int size; /* total number of bytes in the buffer (= nWidth * nHeight * nBits) */
    SANE_Byte colors[4]; /* color codes for colors 0 - nColors-1*/
    SANE_Int nSingleColorLineWidth; /* number of bytes in a line for a single color */
    /* State */
    SANE_Byte* pRead; /* pointer to next byte to read */
    SANE_Int iRead; /* index of line to read next, -1 initially */
    SANE_Int iWrite[4]; /* index of lines where to write for a specific color */
    SANE_Byte* complete; /* array of bytes, one byte per line, specifying the amount of colors read for a line */
    /* SANE_Int iIncomplete; index of first incomplete line; should not be read yet */
    /* SANE_Int nFree;  number of free lines in buffer */
    /* Statistics */
    SANE_Int sizeImage; /* number of bytes in full image */
    SANE_Int nRead; /* number of bytes read from the buffer */
    SANE_Int nWritten; /* number of bytes written to the buffer */
    /* SANE_Int linesWritten; number of lines written to the buffer */
    SANE_Int nData; /* number of bytes available for reading (in complete lines) */
};

static void buffer_create(struct Reflecta_Read_Buffer* buffer, SANE_Int width, SANE_Int height, SANE_Byte colors, SANE_Byte depth, SANE_Bool bigendian, SANE_Int maximum_size);
static void buffer_delete(struct Reflecta_Read_Buffer* buffer);
static SANE_Int buffer_put(struct Reflecta_Read_Buffer* buffer, SANE_Byte* line, SANE_Int size);
static void buffer_get(struct Reflecta_Read_Buffer* buffer, SANE_Byte* data, SANE_Int max_len, SANE_Int* len);
static void buffer_output_state(struct Reflecta_Read_Buffer* buffer);

#endif	/* REFLECTA_BUFFER_H */

