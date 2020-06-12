/****************************************************************************
 * examples/jpeg_decode/jpeg_decode_main.c
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * This example is based on read_JPEG_file() function by IJG.
 *  base: example.c in http://www.ijg.org/files/jpegsrc.v9c.tar.gz 
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h> /* For Spresense Kconfig */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

/*
 * Include file for users of JPEG library.
 * You will need to have included system headers that define at least
 * the typedefs FILE and size_t before you can include jpeglib.h.
 * (stdio.h is sufficient on ANSI-conforming systems.)
 * You may also wish to include "jerror.h".
 */

#include "jpeglib.h"

/*
 * In example.c by IJG, application use setjmp() and  longjmp().
 * Because NuttX OS do not support these functions, delete.
 */

/* #include <setjmp.h> */

/* For output to Spresense LCD.
 * imageproc has the color converter(YUV422 -> RGB565) function.
 */

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
#include <sys/boardctl.h>
#include <nuttx/board.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#  ifdef CONFIG_IMAGEPROC
#    include <imageproc/imageproc.h>
#  endif
#endif

#include "jpeg_decode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define APP_FILENAME_LEN  128

#define APP_BYTES_PER_PIXEL 2  /* YUV4:2:2 has 2 bytes/pixel */

#define APP_QVGA_WIDTH    320
#define APP_QVGA_HEIGHT   240

/* For output to Spresense LCD */

#ifndef CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO
#  define CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO 0
#endif

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* For output to Spresense LCD */

struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/
/* In Spresense, the input file can be specified by file descriptor */

static int  infile;   /* file descriptor of input file */
static char infile_name[APP_FILENAME_LEN] = "/mnt/spif/SAMPLE.JPG";

#ifndef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
static FILE *outfile;  /* file pointer of output file */
static char outfile_name[APP_FILENAME_LEN];
#endif  /* CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

struct jpeg_decompress_struct cinfo;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
struct nximage_data_s g_jpeg_decode_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  false,         /* connected */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/******************** JPEG DECOMPRESSION SAMPLE INTERFACE *******************/

/* This half of the example shows how to read data from the JPEG decompressor.
 * It's a bit more refined than the above, in that we show:
 *   (a) how to modify the JPEG library's standard error-reporting behavior;
 *   (b) how to allocate workspace using the library's memory manager.
 *
 * Just to make this example a little different from the first one, we'll
 * assume that we do not intend to put the whole image into an in-memory
 * buffer, but to send it line-by-line someplace else.  We need a one-
 * scanline-high JSAMPLE array as a work buffer, and we will let the JPEG
 * memory manager allocate it for us.  This approach is actually quite useful
 * because we don't need to remember to deallocate the buffer separately: it
 * will go away automatically when the JPEG object is cleaned up.
 */

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
static inline int nximage_initialize(void)
{
  nxgl_mxpixel_t color;
  pthread_t thread;
  int ret;

  /* Start the NX server kernel thread */

  ret = boardctl(BOARDIOC_NX_START, 0);
  if (ret < 0)
    {
      printf("nximage_initialize: Failed to start the NX server: %d\n", errno);
      return ERROR;
    }

  /* Connect to the server */

  g_jpeg_decode_nximage.hnx = nx_connect();
  if (!g_jpeg_decode_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Start a separate thread to listen for server events.
     For simplicity, use defaul thread attribute.
   */

  ret = pthread_create(&thread, NULL, nximage_listener, NULL);
  if (ret != 0)
    {
       printf("nximage_initialize: pthread_create failed: %d\n", ret);
       return ERROR;
    }

  /* Don't return until we are connected to the server */

  while (!g_jpeg_decode_nximage.connected)
    {
      /* Wait for the listener thread to wake us up when we really
       * are connected.
       */

      (void)sem_wait(&g_jpeg_decode_nximage.sem);
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_jpeg_decode_nximage.hnx, &color);
  ret = nx_requestbkgd(g_jpeg_decode_nximage.hnx,
                       &g_jpeg_decode_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_disconnect(g_jpeg_decode_nximage.hnx);
      return ERROR;
    }

  while (!g_jpeg_decode_nximage.havepos)
    {
      (void) sem_wait(&g_jpeg_decode_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_jpeg_decode_nximage.xres, g_jpeg_decode_nximage.yres);

  return 0;
}


#  ifndef CONFIG_IMAGEPROC
static inline void ycbcr2rgb(uint8_t y,  uint8_t cb, uint8_t cr,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = itou8(_r);
  *g = itou8(_g);
  *b = itou8(_b);
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}
#  endif /* !CONFIG_IMAGEPROC */

/* init_output_to_lcd(), output_to_lcd() and fin_output_to_lcd()
 * are specific for this example.
 * These are for displaying to LCD.
 */

static int init_output_to_lcd(void)
{
  int ret = OK;

  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return ERROR;
    }
#  ifdef CONFIG_IMAGEPROC
  imageproc_initialize();
#  endif /* CONFIG_IMAGEPROC */
  return ret;
}

static void output_to_lcd(JSAMPARRAY buffer,
                          JDIMENSION position,
                          JDIMENSION width,
                          JDIMENSION height)
{
  int y_cnt;
  /* Convert YUV4:2:2 to RGB565 */

  for (y_cnt = 0; y_cnt < height; y_cnt++)
    {
#  ifdef CONFIG_IMAGEPROC
      imageproc_convert_yuv2rgb((void *)buffer[y_cnt],
                                width,
                                1);
#  else
      yuv2rgb(buffer[y_cnt], width * APP_BYTES_PER_PIXEL);
#  endif /* CONFIG_IMAGEPROC */
    }

  /* Display RGB565 */

  nximage_image(g_jpeg_decode_nximage.hbkgd,
                buffer, position, width, height);
  return;
}

static void fin_output_to_lcd(void)
{
#  ifdef CONFIG_IMAGEPROC
  imageproc_finalize();
#  endif
  (void)nx_releasebkgd(g_jpeg_decode_nximage.hbkgd);
  nx_disconnect(g_jpeg_decode_nximage.hnx);

  return;
}
#else /* !CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

/* init_output_to_file(), output_to_file() and fin_output_to_file()
 * are specific for this example.
 * These are for saving to file.
 */

static int init_output_to_file(void)
{
  int ret = OK;

  /* Delete old file name */

  memset(outfile_name, 0, sizeof(outfile_name));

  /* Output file name = Input file name without extension + .YUV" */

  strncpy(outfile_name,
          infile_name,
          strlen(infile_name) - 3 /* 3 is extension length */);
  strncat(outfile_name, "YUV", 3);

  outfile = fopen(outfile_name, "wb");

  /* Initialize with the size of created YUV4:2:2 data */

  fseek(outfile,
        APP_QVGA_WIDTH * APP_QVGA_HEIGHT * APP_BYTES_PER_PIXEL,
        SEEK_SET);
  return ret;
}

static void output_to_file(JSAMPARRAY buffer,
                           JDIMENSION position,
                           JDIMENSION width,
                           JDIMENSION height)
{
  int y_cnt;

  fseek(outfile, position * APP_BYTES_PER_PIXEL, SEEK_SET);
  for (y_cnt = 0; y_cnt < height - 1; y_cnt++)
    {
      fwrite(buffer[y_cnt],
             APP_BYTES_PER_PIXEL,
             width,
             outfile);

      /* Go to next line */
      fseek(outfile,
            (APP_QVGA_WIDTH - width) * APP_BYTES_PER_PIXEL,
            SEEK_CUR);
    }

  /* Write last line */

  fwrite(buffer[height - 1],
         APP_BYTES_PER_PIXEL,
         width,
         outfile);
  return;
}

static void fin_output_to_file(void)
{
  fclose(outfile);
  return;
}
#endif /* CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*
 * Sample routine for JPEG decompression to YUV4:2:2.
 * Assume that the source file name is passed in.
 */

int main(int argc, FAR char *argv[])
{
  /* This struct contains the JPEG decompression parameters and pointers to
   * working space (which is allocated as needed by the JPEG library).
   */

  /* For reduction of stack size, globalize */
  /* struct jpeg_decompress_struct cinfo; */

  /* Because Spresense do not support setjmp/longjmp,
   *  use default error handling function for now.
   */
  struct jpeg_error_mgr jerr;
  /* More stuff */

  JSAMPARRAY buffer;            /* Output row buffer */
  JDIMENSION output_position;   /* start position of output */
  JDIMENSION output_width_by_one_decode;
  JDIMENSION output_height_by_one_decode;
  bool       mcu = false;       /* True means "decode by the MCU" */

  /* Command parameter mean input filename in this example. */

  if (argc > 1)
    {
      strncpy(infile_name, argv[1], APP_FILENAME_LEN);
    }

  if (argc > 2)
    {
      /* 2nd parameter setting means "decode by the MCU". */

      mcu = true;
    }

  /* Original libjpeg use file pointer to specify JPEG file.
   * In Spresense, application can use file descripter, which enables to get
   * JPEG from any readable descripter.
   */

  if ((infile = open(infile_name, O_RDONLY)) < 0) {
    fprintf(stderr, "can't open %s\n", infile_name);
    return 0;
  }

  /* Step 1: allocate and initialize JPEG decompression object */

  /* We set up the normal JPEG error routines. */
  cinfo.err = jpeg_std_error(&jerr);
  /* jerr.error_exit = my_error_exit; */
  /* Because Spresense do not support setjmp, delete setjmp */
  /* if (setjmp(jerr.setjmp_buffer)) { */
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    /* jpeg_destroy_decompress(&cinfo); */
    /* fclose(infile); */
    /* return 0;       */
  /* } */
  /* Now we can initialize the JPEG decompression object. */

  jpeg_create_decompress(&cinfo);

  /* Step 2: specify data source (eg, a file)
   * Spresense support file descriptor.
   * In using file descriptor, use jpeg_fd_src() API.
   */

  /* jpeg_stdio_src(&cinfo, infile); This is file pointer    API */
  jpeg_fd_src(&cinfo, infile);    /* This is file descriptor API */

  /* Step 3: read file parameters with jpeg_read_header() */

  (void) jpeg_read_header(&cinfo, TRUE);
  /* We can ignore the return value from jpeg_read_header since
   *   (a) suspension is not possible with the stdio data source, and
   *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
   * See libjpeg.txt for more info.
   */

  /* Step 3-1: Get image information */

  fprintf(stdout, "image width  = %d\n", cinfo.image_width);
  fprintf(stdout, "image height = %d\n", cinfo.image_height);

  /* Step 4: set parameters for decompression */

  /* Spresense support CbYCrY output, which is Spresense-specific.
   * But, Spresense do not support YCbCr(YUV4:4:4) and 4-component formats
   *  (CMYK and YCCK) currently.
   */

  cinfo.out_color_space = JCS_CbYCrY;

  /* In this example, output to QVGA(320*240) display */
  /* For such purpose, set downscaling in large input image case. */

  cinfo.scale_num = APP_QVGA_WIDTH;
  cinfo.scale_denom = cinfo.image_width;

  /* Step 5: Start decompressor */

  (void) jpeg_start_decompress(&cinfo);
  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* We may need to do some setup of our own at this point before reading
   * the data.  After jpeg_start_decompress() we have the correct scaled
   * output image dimensions available, as well as the output colormap
   * if we asked for color quantization.
   * In this example, we need to make an output work buffer of the right size.
   */

  /* Spresense JPEG decoder support the two decode methods.
   *  One is the original libjpeg method: jpeg_read_scanlines()
   *  The other is the Spresense-specific method: jpeg_read_mcus()
   *
   * [examples of decoded data order in LINE UNIT(libjpeg original) case
   *  (jpeg_read_scanlines)]
   * +--MCU1--+ +--MCU2--+                  +--MCU20-+
   * |(1)-----|-|--------|------------------|------->|
   * |(2)-----|-|--------|------------------|------->|
   * |  ...   | |        |                  |        |
   * |(8)-----|-|--------|------------------|------->|
   * +--------+ +--------+                  +--------+
   *   ......
   *
   * +-MCU581-+   +-MCU582-+                  +-MCU600-+
   * |(233)---|-|--------|------------------|------->|
   * |(234)---|-|--------|------------------|------->|
   * |  ...   | |        |                  |        |
   * |(240)---|-|--------|------------------|------->|
   * +--------+ +--------+                  +--------+
   *
   * [examples of decoded data order in MCU UNIT case
   *  (jpeg_read_mcus)]
   * +--MCU1--+ +--MCU2--+                  +--MCU20-+
   * |(1)---->| |(2)---->|       ...        |(20)--->|
   * |------->| |------->|                  |------->|
   * |  ...   | | ...    |                  |  ...   |
   * |------->| |------->|                  |------->|
   * +--------+ +--------+                  +--------+
   *   ......
   *
   * +-MCU581-+ +-MCU582-+                  +-MCU600-+
   * |(581)-->| |(582)-->|       ...        |(600)-->|
   * |------->| |------->|                  |------->|
   * |  ...   | | ...    |                  |  ...   |
   * |------->| |------->|                  |------->|
   * +--------+ +--------+                  +--------+
   */

  if (mcu)
    {
      /* Output size of 1 decode is the size of 1 MCU */

      output_width_by_one_decode  = (cinfo.output_width  / cinfo.MCUs_per_row);
      output_height_by_one_decode = (cinfo.output_height / cinfo.MCU_rows_in_scan);
    }
  else
    {
      /* Output size of 1 decode is the size of 1 line */

      output_width_by_one_decode  = cinfo.output_width;
      output_height_by_one_decode = 1;
    }

  /* Make a multi-rows-high sample array that will go away when done with image.
   * Please allocate g_jpeg_decode_output.youtsize lines. */

  buffer = (*cinfo.mem->alloc_sarray)
                ((j_common_ptr) &cinfo,
                 JPOOL_IMAGE,
                 output_width_by_one_decode * APP_BYTES_PER_PIXEL,
                 output_height_by_one_decode);
  /* For examples, if output_height_by_one_decode = 8,
   * buffer has the following structure:
   *
   *                                 +---------------------------+
   *     buffer[0] points to ---->   | 1st line of decode result |
   *                                 +---------------------------+
   *     buffer[1] points to ---->   | 2nd line of decode result |
   *                                 +---------------------------+
   *      ...                        |  ...                      |
   *                                 +---------------------------+
   *     buffer[7] points to ---->   | 8th line of decode result |
   *                                 +---------------------------+
   */
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
  init_output_to_lcd();
#else
  init_output_to_file();
#endif

  /* Step 6: while (MCU remain to be read) */
  /*           jpeg_read_mcus(...); */

  if (mcu)
    {
      while (cinfo.output_offset < (cinfo.output_width * cinfo.output_height))
        {
          /* jpeg_read_mcus output lines of decode result to each buffer[line],
           *  and notify the position(offset from top-left) which their lines
           *  are written.
           */

          jpeg_read_mcus(&cinfo,
                         buffer,
                         output_height_by_one_decode,
                         &output_position);
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
          output_to_lcd(buffer, output_position,
                        output_width_by_one_decode, output_height_by_one_decode);
#else
          output_to_file(buffer, output_position,
                         output_width_by_one_decode, output_height_by_one_decode); 
#endif
        }
    }
  else
    {
      /* Same as original libjpeg examples in using jpeg_read_scanlines */

      /* Here we use the library's state variable cinfo.output_scanline as the
       * loop counter, so that we don't have to keep track ourselves.
       */
      while (cinfo.output_scanline < cinfo.output_height)
        {
          /* jpeg_read_scanlines expects an array of pointers to scanlines.
           * Here the array is only one element long, but you could ask for
           * more than one scanline at a time if that's more convenient.
           */

          jpeg_read_scanlines(&cinfo, buffer, 1);
          /* Assume output wants a pointer and writing position. */
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
          output_to_lcd(buffer, (cinfo.output_scanline - 1) * cinfo.output_width,
                        output_width_by_one_decode, output_height_by_one_decode);
#else
          output_to_file(buffer, (cinfo.output_scanline - 1) * cinfo.output_width,
                         output_width_by_one_decode, output_height_by_one_decode);
#endif
        }
    }

  /* Step 7: Finish decompression */

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
  fin_output_to_lcd();
#else
  fin_output_to_file();
#endif
  (void) jpeg_finish_decompress(&cinfo);

  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* Step 8: Release JPEG decompression object */

  /* This is an important step since it will release a good deal of memory. */
  jpeg_destroy_decompress(&cinfo);

  /* After finish_decompress, we can close the input file.
   * Here we postpone it until after no more JPEG errors are possible,
   * so as to simplify the setjmp error logic above.  (Actually, I don't
   * think that jpeg_destroy can do an error exit, but why assume anything...)
   */
  /* fclose(infile); */
  close(infile);

  /* At this point you may want to check to see whether any corrupt-data
   * warnings occurred (test whether jerr.pub.num_warnings is nonzero).
   */

  /* And we're done! */
  return 1;
}


/*
 * SOME FINE POINTS:
 *
 * In the above code, we ignored the return value of jpeg_read_scanlines,
 * which is the number of scanlines actually read.  We could get away with
 * this because we asked for only one line at a time and we weren't using
 * a suspending data source.  See libjpeg.txt for more info.
 *
 * We cheated a bit by calling alloc_sarray() after jpeg_start_decompress();
 * we should have done it beforehand to ensure that the space would be
 * counted against the JPEG max_memory setting.  In some systems the above
 * code would risk an out-of-memory error.  However, in general we don't
 * know the output image dimensions before jpeg_start_decompress(), unless we
 * call jpeg_calc_output_dimensions().  See libjpeg.txt for more about this.
 *
 * Scanlines are returned in the same order as they appear in the JPEG file,
 * which is standardly top-to-bottom.  If you must emit data bottom-to-top,
 * you can use one of the virtual arrays provided by the JPEG memory manager
 * to invert the data.  See wrbmp.c for an example.
 *
 * As with compression, some operating modes may require temporary files.
 * On some systems you may need to set up a signal handler to ensure that
 * temporary files are deleted if the program is interrupted.  See libjpeg.txt.
 */
