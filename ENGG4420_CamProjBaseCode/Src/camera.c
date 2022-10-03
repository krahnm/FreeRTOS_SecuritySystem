/*
 * camera.c
 *
 *  Created on: Nov 3, 2018
 *      Author: Graham Thoms
 */

#include "camera.h"
#include "arducam.h"
#include "nanojpeg.h"
#include "picojpeg.h"
#include "stdarg.h"

#include "jpg2tga.c"

#include "malloc.h"
#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"



static uint32_t captureStart;
uint8_t fifoBuffer[BURST_READ_LENGTH];

uint8_t *ptr_picture;

static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);

void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			printf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			printf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		printf("camera: setup failed\n\r");
		cameraReady = pdTRUE;
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	printf("camera: initiate capture\n\r");

	if (!cameraReady) {
		printf("camera: set up camera before capture\n\r");
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		printf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			printf("camera: capture timeout\n\r");
			return;
		}
	}

	printf("camera: capture complete\n\r");

	camera_get_image();

	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		printf("camera: captured jpeg image -> %lu bytes\n\r", length);
		write_fifo_to_buffer(length);
	} else {
		printf("camera: get fifo length failed\n\r");
	}

	return;
}

static void write8(FILE *f, int x) { uint8 z = (uint8) x; fwrite(&z,1,1,f); }

static void writefv(FILE *f, char *fmt, va_list v) {
   while (*fmt) {
      switch (*fmt++) {
         case ' ': break;
         case '1': { uint8 x = va_arg(v, int); write8(f,x); break; }
         case '2': { int16 x = va_arg(v, int); write8(f,x); write8(f,x>>8); break; }
         case '4': { int32 x = va_arg(v, int); write8(f,x); write8(f,x>>8); write8(f,x>>16); write8(f,x>>24); break; }
         default:
            assert(0);
            va_end(v);
            return;
      }
   }
}

static void writef(FILE *f, char *fmt, ...)
{
   va_list v;
   va_start(v, fmt);
   writefv(f,fmt,v);
   va_end(v);
}

static void write_pixels(FILE *f, int rgb_dir, int vdir, int x, int y, int comp, void *data, int write_alpha, int scanline_pad)
{
   uint8 bg[3] = { 255, 0, 255}, px[3];
   uint32 zero = 0;
   int i,j,k, j_end;

   if (vdir < 0)
      j_end = -1, j = y-1;
   else
      j_end =  y, j = 0;

   for (; j != j_end; j += vdir) {
      for (i=0; i < x; ++i) {
         uint8 *d = (uint8 *) data + (j*x+i)*comp;
         if (write_alpha < 0)
            fwrite(&d[comp-1], 1, 1, f);
         switch (comp) {
            case 1:
            case 2: writef(f, "111", d[0],d[0],d[0]);
                    break;
            case 4:
               if (!write_alpha) {
                  for (k=0; k < 3; ++k)
                     px[k] = bg[k] + ((d[k] - bg[k]) * d[3])/255;
                  writef(f, "111", px[1-rgb_dir],px[1],px[1+rgb_dir]);
                  break;
               }
               /* FALLTHROUGH */
            case 3:
               writef(f, "111", d[1-rgb_dir],d[1],d[1+rgb_dir]);
               break;
         }
         if (write_alpha > 0)
            fwrite(&d[comp-1], 1, 1, f);
      }
      fwrite(&zero,scanline_pad,1,f);
   }
}

static int outfile(uint8_t *filename, int size, int rgb_dir, int vdir, int x, int y, int comp, void *data, int alpha, int pad, char *fmt, ...)
{
   FILE *f = fmemopen(filename, size, "w");
   if (f) {
      va_list v;
      va_start(v, fmt);
      writefv(f, fmt, v);
      va_end(v);
      write_pixels(f,rgb_dir,vdir,x,y,comp,data,alpha,pad);
      fclose(f);
   }
   return f != NULL;
}

int stbi_write_bmp(uint8_t *filename, int size, int x, int y, int comp, void *data)
{
   int pad = (-x*3) & 3;
  // printf ("outfile size: %d\n\r", 14+40+(x*3+pad)*y);
   return outfile(filename, size,-1,-1,x,y,comp,data,0,pad,
           "11 4 22 4" "4 44 22 444444",
           'B', 'M', size/*14+40+(x*3+pad)*y*/, 0,0, 14+40,  // file header
            40, x,y, 1,24, 0,0,0,0,0,0);             // bitmap header
}

BaseType_t
write_fifo_to_buffer(uint32_t length) {
	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;
	int curSize = 0;
	int x,y,comp, reduce = 1;
	const char* bmpfile;
	uint8_t* bmp, *ptr_bmp;
	pjpeg_image_info_t image_info;
	pjpeg_scan_type_t scanType;

	// jpeg pic size
	unsigned int jpeg_size = length*sizeof(uint8_t);
	// allocate memory to store jpeg picture
	if((ptr_picture = malloc(jpeg_size)) == NULL){
		printf("camera: ran out of memory\n\r");
	}else{
		printf("camera: allocated %d bytes of memory for picture\n\r", malloc_usable_size(ptr_picture));
	}


	for (uint16_t i = 0; length > 0; ++i) {
		//printf("%d ", i);
		fflush(stdout);
		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);
		length -= chunk;
		memcpy(ptr_picture+curSize, fifoBuffer, chunk);
		curSize += chunk;
		// maybe send the fifo buffer to LabVIEW for displaying ....
	}
	printf("Writing to Memory Complete \n\r");
	bmp = pjpeg_load_from_file(ptr_picture, curSize, &x, &y, &comp, &scanType, reduce);
	free(ptr_picture);
	printf ("Size of the decoded image %d\n\r", malloc_usable_size(bmp));
	if ((ptr_bmp = malloc(malloc_usable_size(bmp))) == NULL){
		printf("out of memory for picture");
	} else {
		stbi_write_bmp(ptr_bmp, malloc_usable_size(bmp), x,y,comp, bmp);
		BSP_LCD_DrawBitmap(80, 100, ptr_bmp);
		free(ptr_bmp);
	}
	free(bmp);
	// test image: make sure to build the project with -Og to show this static .bmp image
	// 		project properties -> C/C++ Build -> Settings -> Optimization | Optimize for debugging (-Og)

    //osDelay(500);

	return pdTRUE;
}


