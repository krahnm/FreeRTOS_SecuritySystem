#include <arducam.h>
#include <spi.h>
#include <i2c.h>

#define ARDUCAM_CS_SELECT  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define ARDUCAM_CS_RELEASE HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

/* Arduchip ----------------------------------------------------------------- */

/**
 * Read an 8-bit register value from the ArduCAM.
 */
uint8_t
arduchip_read_reg(uint8_t reg, uint8_t *ptr) {
	ARDUCAM_CS_SELECT;
	if (spi_read8(&hspi4, reg, ptr, 1) != DEVICES_OK) {
		return 0;
	}
	ARDUCAM_CS_RELEASE;
	return 1;
}

/**
 * Write an 8-bit register value to the ArduCAM.
 */
uint8_t
arduchip_write_reg(uint8_t reg, uint8_t value) {
	ARDUCAM_CS_SELECT;
	if (spi_write8_8(&hspi4, reg, value) != DEVICES_OK) {
		return 0;
	}
	ARDUCAM_CS_RELEASE;
	return 1;
}

/**
 * Initiate a burst read operation.
 */
uint8_t
arduchip_burst_read(uint8_t* buffer, uint16_t length) {
	/**
	 * Send a command byte addressing the burst read register.
	 * The response contains the first byte of data.
	 */
	ARDUCAM_CS_SELECT;
	if (spi_read8(&hspi4, ARDUCHIP_BURST_READ, buffer, 1) != DEVICES_OK) {
		ARDUCAM_CS_RELEASE;
		return 0;
	}

	/* Read the remaining bytes of data. */
	if (HAL_SPI_Receive(&hspi4, buffer + 1, length - 1, 1024) != HAL_OK) {
		ARDUCAM_CS_RELEASE;
		return 0;
	}

	ARDUCAM_CS_RELEASE;
	return 1;
}


/**
 * Read the ArduChip version.
 */
uint8_t
arduchip_chip(uint8_t* chipId) {
	/* Emit a few clock cycles so the ArduChip can get ready. */
	ARDUCAM_CS_RELEASE;
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);
	spi_read8(&hspi4, 0x0, chipId, 1);

	/* Read the ArduChip version register. */
	*chipId = 0;
	if (!arduchip_read_reg(ARDUCHIP_VERSION, chipId)) {
		return 0;
	}
	return 1;
}

/**
 * Initialize the Arduchip.
 */
uint8_t
arduchip_detect() {
	/**
	 * Read the chip ID if present.
	 * Expect a supported chip ID.
	 */
	uint8_t chipId = 0;
	if (   !arduchip_chip(&chipId)
		|| (chipId != ARDUCHIP_5MP && chipId != ARDUCHIP_5MP_PLUS && chipId != ARDUCHIP_5MP_PLUS_REV )
	) {
		printf("camera/arduchip: not present\n");
		return 0;
	}

	/* Explicitly enable FIFO mode on the 5MP. */
	uint8_t flags = 0;
	if (chipId == ARDUCHIP_5MP || chipId == ARDUCHIP_5MP_PLUS_REV) {
		flags |= SITR_FIFO_MASK;
	}

	/**
	 * Configure the chip to use active-low VSYNC.
	 * Configure the chip to capture one frame.
	 */
	flags |= SITR_VSYNC_MASK;
	if (!arduchip_write_reg(ARDUCHIP_SITR, flags) ||
		!arduchip_write_reg(ARDUCHIP_CCR, CCR_FRAMES(1))
	) {
		printf("camera/arduchip: not ready\n\r");
		return 0;
	}

	printf("camera/arduchip: ready (%x)\n\r", chipId);
	return 1;
}

/**
 * Initiate a frame capture.
 */
uint8_t
arduchip_start_capture() {
	/**
	 * Clear the FIFO write done flag.
	 * Reset the FIFO read pointer.
	 * Reset the FIFO write pointer.
	 * Initiate a capture.
	 */
	if (   !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_CLEAR_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_RDPTR_RST_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_WRPTR_RST_MASK)
		|| !arduchip_write_reg(ARDUCHIP_FIFO_CR, FIFO_START_MASK)
	) {
		return 0;
	}

	return 1;
}

/**
 * Poll the capture complete flag.
 */
uint8_t
arduchip_capture_done(uint8_t* done) {
	uint8_t status = 0;
	if (arduchip_read_reg(ARDUCHIP_STATUS, &status)) {
		*done = (status & STATUS_FIFO_DONE_MASK) > 0;
		return 1;
	}
	return 0;
}

/**
 * Get the FIFO buffer length.
 */
BaseType_t
arduchip_fifo_length(uint32_t* length) {
	uint8_t a = 0, b = 0, c = 0;
	if (   !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_0, &a)
		|| !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_1, &b)
		|| !arduchip_read_reg(ARDUCHIP_FIFO_WRITE_2, &c)
	) {
		return pdFALSE;
	}
	*length = a | (b << 8) | ((c & 0x7) << 16);
	return pdTRUE;
}

/* OV5642 Sensor ------------------------------------------------------------ */

/**
 * Initialize the OV5642.
 */
uint8_t
ov5642_detect() {
	/**
	 * Read the chip ID from the sensor.
	 * Expect the OV5642.
	 */
	uint8_t high_byte, low_byte;
	if (   i2c_read16(OV5642_ADDRESS_W, OV5642_CHIP_ID_HIGH_BYTE, &high_byte, 1) != DEVICES_OK
		&& i2c_read16(OV5642_ADDRESS_W, OV5642_CHIP_ID_LOW_BYTE, &low_byte, 1)   != DEVICES_OK
		&& ((high_byte << 8) | low_byte) != OV5642_CHIP_ID
	) {
		printf("camera/ov5642: not present\n\r");
		return 0;
	}

	printf("camera/ov5642: ready\n\r");
	return 1;
}

/**
 * Configure the OV5642 for JPEG capture.
 */
uint8_t
ov5642_configure() {
	/* Perform a software reset of the camera sensor. */
	//if (i2c_write16_8(OV5642_ADDRESS_W, 0x3008, 0x80) != DEVICES_OK) {
	//	goto error;
	//}

	/* This delay appears to be important. */
	//vTaskDelay(100);

	/**
	 * TODO Writing the same register twice in a short period seems to have no
	 * affect. It would be better to merge the arrays into one array with all
	 * desired values.
	 */

	/* Initialize the registers of the camera sensor. */
	if (i2c_array16_8(OV5642_ADDRESS_W, ov5642_dvp_fmt_global_init) != DEVICES_OK) {
		goto error;
	}

	/* This delay appears to be important. */
	vTaskDelay(50);

	if (   i2c_array16_8(OV5642_ADDRESS_W, ov5642_dvp_fmt_jpeg_qvga) != DEVICES_OK) {
		goto error;
	}



	/* This delay appears to be important. */
	vTaskDelay(50);

	/* Configure the format and resolution. */
	if (   i2c_array16_8(OV5642_ADDRESS_W, ov5642_res_480P)          != DEVICES_OK
		&& i2c_write16_8(OV5642_ADDRESS_W, 0x4407, 0x0C)             != DEVICES_OK
	) {
		goto error;
	}

	return 1;

error:
	printf("camera/ov5642: configure failed\n\r");
	return 0;
}

/* Arducam module ----------------------------------------------------------- */

/**
 * Have the Arducam power down the OV5642.
 * This will reset all registers in the OV5642.
 */
uint8_t
arducam_enter_standby() {
	uint8_t gpio;
	if (   !arduchip_read_reg(ARDUCHIP_GPIO, &gpio)
		&& !arduchip_write_reg(ARDUCHIP_GPIO, gpio | GPIO_PWDN_MASK)
	) {
		return 0;
	}

	return 1;
}

/**
 * Have the Arducam power up the OV5642. Note that the sensor requires a few
 * milliseconds to stabilize, and the auto-exposure algorithm also requires
 * a few seconds as well before images are acceptable.
 */
uint8_t
arducam_exit_standby() {
	uint8_t gpio;
	if (   !arduchip_read_reg(ARDUCHIP_GPIO, &gpio)
		&& !arduchip_write_reg(ARDUCHIP_GPIO, gpio & ~GPIO_PWDN_MASK)
	) {
		return 0;
	}

	return 1;
}

////////////////////////////////////////////// IGNORE ///////////////////////////////////////////////////////////
//
//static Msg delayedMsg;
//
///**
// * Buffer for copying from Arducam FIFO to SD card.
// */
//uint8_t fifoBuffer[BURST_READ_LENGTH];
//
///* Function prototypes ------------------------------------------------------ */
////void camera_task_message_loop(void * pvParameters);
////void camera_timer_elapsed(TimerHandle_t xTimer);
////void camera_timer_photo(TimerHandle_t xTimer);
//void camera_tell_delay(uint16_t message, uint32_t, TickType_t delay);
//
////static void filesystem_mounted();
////void camera_setup(Msg* msg);
//void camera_enter_standby();
//void camera_exit_standby(Msg* msg);
//void capture_timer_start();
//void capture_timer_stop();
////void capture_initiate();
////void capture_await_done(Msg* msg);
//void capture_write_disk();
//
//void capture_workflow_done();
//void capture_workflow_error();
//
///**
// * Set when the camera is installed and configured.
// */
////static uint8_t cameraReady = pdFALSE;
//
///**
// * Task healthy status flag.
// * The watchdog will reboot the MCU if this becomes false.
// */
//static BaseType_t taskIsHealthy = pdTRUE;
//
///**
// * Number of consecutive failed captures.
// * Clears the taskIsHealthy flag if it exceeds CAPTURE_FAILED_THRESHOLD.
// */
//static uint8_t workflowFailedCount = 0;
//
//#if 0
//BaseType_t find_dcim_filename(char* buffer, uint8_t length);
//F_FILE* open_dcim_handle(char* buffer, uint8_t length);
//BaseType_t write_fifo_to_disk(F_FILE* pxFile, uint32_t length);
//#endif
///* Task setup and message loop ---------------------------------------------- */
//#if 0
///**
// * Create the camera task.
// */
//uint8_t
//camera_task_create() {
//	BaseType_t retVal;
//
//	/* Create a message loop for this task. */
//	xCameraQueue = xQueueCreate(10, sizeof(Msg));
//	ERROR_ON_NULL(xCameraQueue)
//
//	/* Timer used to schedule delayed state transitions. */
//	xCameraTimer = xTimerCreate(
//			CAMERA_TASK_NAME "STATE",
//			1000,
//			pdFALSE,
//			(void*)0,
//			camera_timer_elapsed);
//	ERROR_ON_NULL(xCameraTimer)
//
//	/* Timer used to schedule camera photo capture. */
//	xCameraTimerPhoto = xTimerCreate(
//			CAMERA_TASK_NAME "CAP",
//			CAMERA_TIMER_INTERVAL,
//			pdTRUE,
//			(void*)0,
//			camera_timer_photo);
//	ERROR_ON_NULL(xCameraTimerPhoto)
//
//	/* Create the task. */
//	retVal = xTaskCreate(
//			camera_task_message_loop,
//			CAMERA_TASK_NAME,
//			CAMERA_TASK_STACK_SIZE,
//			(void *)NULL,
//			tskIDLE_PRIORITY,
//			NULL);
//	ERROR_ON_FAIL(retVal)
//
//	trace_printf("camera_task: created task\n");
//	return 1;
//
//error:
//	trace_printf("camera_task: setup failed\n");
//	return 0;
//}
//#endif
///**
// * Message loop for the camera task.
// */
//void
////camera_task_message_loop(void * pvParameters __attribute__((unused))) {
//camera_task_message_loop() {
//	/* Setup the sensors. */
//	camera_tell(MSG_CAMERA_SETUP, 0);
//
//	for (;;) {
//		Msg msg;
//		/* Block until messages are received. */
//		if (xQueueReceive(xCameraQueue, &msg, portMAX_DELAY) != pdTRUE) {
//			continue;
//		}
//		switch (msg.message) {
//#if 0
//		case MSG_IWDG_PING:
//			/* Respond to the ping from the watchdog task. */
//			if (taskIsHealthy == pdTRUE) {
//				msg.message = MSG_IWDG_PONG;
//				xQueueSend(xWatchdogQueue, &msg, 0);
//			}
//			break;
//		case MSG_SDCARD_MOUNTED:
//			filesystem_mounted();
//			break;
//#endif
//		case MSG_CAMERA_SETUP:
//			camera_setup(&msg);
//			break;
//		case MSG_CAMERA_ENTER_STANDBY:
//			camera_enter_standby();
//			break;
//		case MSG_CAMERA_EXIT_STANDBY:
//			camera_exit_standby(&msg);
//			break;
//		case MSG_CAMERA_CAPTURE_TIMER_START:
//			capture_timer_start();
//			break;
//		case MSG_CAMERA_CAPTURE_TIMER_STOP:
//			capture_timer_stop();
//			break;
//		case MSG_CAMERA_CAPTURE_INITIATE:
//			capture_initiate();
//			break;
//		case MSG_CAMERA_CAPTURE_AWAIT_DONE:
//			capture_await_done(&msg);
//			break;
//#if 0
//		case MSG_CAMERA_CAPTURE_WRITE_DISK:
//			capture_write_disk();
//			break;
//#endif
//		}
//	}
//}
//
///* Timer callbacks ---------------------------------------------------------- */
//
///**
// * Timer callback to initiate state transitions.
// */
//void
////camera_timer_elapsed(TimerHandle_t xTimer __attribute__((unused))) {
//camera_timer_elapsed() {
//	xQueueSend(xCameraQueue, &delayedMsg, 0);
//}
//
///**
// * Timer callback to initiate a capture.
// */
//void
////camera_timer_photo(TimerHandle_t xTimer __attribute__((unused))) {
//camera_timer_photo() {
//	camera_tell(MSG_CAMERA_EXIT_STANDBY, 0);
//}
//
///* Message sending ---------------------------------------------------------- */
//
///**
// * Send a message to the camera task queue.
// */
//void
//camera_tell(uint16_t message, uint32_t param1) {
//	Msg msg = { message, param1 };
//	xQueueSend(xCameraQueue, &msg, 0);
//}
//
///**
// * Send a message to the camera task queue after a delay.
// */
//void
//camera_tell_delay(uint16_t message, uint32_t param1, TickType_t delay) {
//	delayedMsg.message = message;
//	delayedMsg.param1 = param1;
//	xTimerChangePeriod(xCameraTimer, delay, 0);
//	xTimerStart(xCameraTimer, 0);
//}
//
///* Message handlers --------------------------------------------------------- */
//#if 0
///**
// * Invoked when the filesystem is mounted.
// */
//void
//filesystem_mounted() {
//	/* Start the capture timer if the camera is ready. */
//	fsIsMounted = pdTRUE;
//	if (cameraReady == pdTRUE) {
//		camera_tell(MSG_CAMERA_CAPTURE_TIMER_START, 0);
//	}
//}
//#endif
//
/////**
//// * Initialize the peripherals for this task.
//// */
////void
////camera_setup(Msg* msg) {
////	/**
////	 * Detect and initialize the Arduchip interface.
////	 * Ensure that the OV5642 is powered on.
////	 * Detect and initialize the OV5642 sensor chip.
////	 */
////
////	osDelay(50);
////	if (   arduchip_detect()
////		&& arducam_exit_standby()
////		&& ov5642_detect()
////	) {
//////		trace_printf("camera_task: setup complete\n");
////		printf("camera_task: setup complete\n");
////
////		/* Start the capture timer if the filesystem is mounted. */
////		cameraReady = pdTRUE;
////
////
////		camera_tell(MSG_CAMERA_CAPTURE_TIMER_START, 0);
////
////
////#if 0
////		if (fsIsMounted == pdTRUE) {
////			camera_tell(MSG_CAMERA_ENTER_STANDBY, 0);
////			camera_tell(MSG_CAMERA_CAPTURE_TIMER_START, 0);
////		}
////#endif
////	} else {
//////		trace_printf("camera_task: setup failed\n");
////		printf("camera_task: setup failed\n");
////		if (msg->param1++ > 10) {
////			/* Failed to setup the sensors after 10 attempts. */
////			taskIsHealthy = pdFALSE;
////			camera_tell(MSG_CAMERA_ENTER_STANDBY, 0);
////		} else {
////			/* Wait a few seconds and try again. */
////			camera_tell_delay(msg->message, msg->param1++, 5000);
////		}
////	}
////}
//
///**
// * Have the camera enter standby mode.
// */
//void
//camera_enter_standby() {
//	if (!arducam_enter_standby()) {
//		trace_printf("camera_task: enter standby failed\n");
//	}
//}
//
///**
// * Have the camera exit standby mode.
// *
// * Recovery from standby requires a short recovery time for sensor power
// * stabilization and register programming. The value in param1 describes the
// * recovery phase each time this message is handled.
// */
//void
//camera_exit_standby(Msg* msg) {
//	switch (msg->param1) {
//	case RECOVER_POWER:
//		trace_printf("camera_task: exit standby\n");
//
//		/* Power on the sensor. */
//		if (!arducam_exit_standby()) {
//			trace_printf("camera_task: exit standby failed\n");
//			capture_workflow_error();
//			return;
//		}
//
//		/* Allow sensor power and registers to stabilize. */
//		camera_tell_delay(MSG_CAMERA_EXIT_STANDBY, RECOVER_CONFIG, STANDBY_POWER_STABILIZE);
//		break;
//	case RECOVER_CONFIG:
//		/* Configure the sensor. */
//		if (!ov5642_configure()) {
//			trace_printf("camera_task: exit standby failed\n");
//			capture_workflow_error();
//			return;
//		}
//
//		/* Allow sensor auto-exposure algorithm to stabilize. */
//		camera_tell_delay(MSG_CAMERA_EXIT_STANDBY, RECOVER_READY, STANDBY_EXPOSURE_STABILIZE);
//		break;
//	case RECOVER_READY:
//		/* Camera should be ready. */
//		camera_tell(MSG_CAMERA_CAPTURE_INITIATE, 0);
//		break;
//	}
//}
//
///**
// * Start the capture timer.
// * Does nothing if the timer is already started.
// */
//void
//capture_timer_start() {
//	if (xTimerIsTimerActive(xCameraTimerPhoto) == pdFALSE) {
//		trace_printf("camera_task: timer start\n");
//		xTimerStart(xCameraTimerPhoto, 0);
//	}
//}
//
///**
// * Stop the capture timer.
// * Does nothing if the timer is already stopped.
// */
//void
//capture_timer_stop() {
//	if (xTimerIsTimerActive(xCameraTimerPhoto) == pdTRUE) {
//		trace_printf("camera_task: timer stop\n");
//		xTimerStop(xCameraTimerPhoto, 0);
//	}
//}
//
///**
// * Capture an image from the camera to the SD card.
// */
////void
////capture_initiate() {
//////	trace_printf("camera_task: initiate capture\n");
////	printf("camera_task: initiate capture\n");
////
////	/* Initiate an image capture. */
////	if (!arduchip_start_capture()) {
////		printf("camera_task: initiate capture failed\n");
////		capture_workflow_error();
////		return;
////	}
////
////	/**
////	 * Begin polling for capture complete.
////	 * Record the time when polling begins in param1.
////	 */
////	uint32_t captureStart = (uint32_t)xTaskGetTickCount();
////	printf("camera_task: initiate capture command sent, tick_start = %d\n", (int)captureStart);
////	camera_tell(MSG_CAMERA_CAPTURE_AWAIT_DONE, captureStart);
////}
//
///**
// * Check if the image capture is complete.
// */
////void
////capture_await_done(Msg* msg) {
////	/* Check the capture complete flag. */
////	static uint8_t done = 0;
////	if (!arduchip_capture_done(&done) || !done) {
////		/* Failed to read status from Arduchip or not done. */
////		if ((xTaskGetTickCount() - msg->param1) < CAPTURE_TIMEOUT) {
////			camera_tell(MSG_CAMERA_CAPTURE_AWAIT_DONE, msg->param1);
////		} else {
////			/* Timeout elapsed and without capture done. */
//////			trace_printf("camera_task: capture timeout\n");
////			printf("camera_task: capture timeout, tick_cur = %d, tick_prev = %d\n", (int)xTaskGetTickCount(), (int)(msg->param1));
////			capture_workflow_error();
////			camera_tell(MSG_CAMERA_ENTER_STANDBY, 0);
////		}
////		return;
////	};
////	printf("camera_task: capture done!\n");
////	/* Write the FIFO buffer to the SD card. */
//////	camera_tell(MSG_CAMERA_CAPTURE_WRITE_DISK, 0);
////}
//
//#if 0
///**
// * Write the captured image to the filesystem.
// */
//void
//capture_write_disk(Msg* msg) {
//	/* Obtain exclusive filesystem access. */
//	if (fs_access_take() == pdTRUE) {
//		/* Determine the FIFO buffer length. */
//		uint32_t length = 0;
//		if (arduchip_fifo_length(&length) == pdTRUE) {
//			/* Open a file in which to write the captured image. */
//			char filename[32];
//			F_FILE* pxFile = create_file_for_upload(filename, 32, "JPG");
//			if (pxFile != NULL) {
//				/* Copy bytes from the FIFO to the filesystem. */
//				trace_printf("camera_task: write %d bytes to %s\n", length, filename);
//				if (write_fifo_to_disk(pxFile, length) == pdTRUE) {
//					trace_printf("camera_task: write complete\n");
//					capture_workflow_done();
//				} else {
//					trace_printf("camera_task: failed to write file\n");
//					capture_workflow_error();
//				}
//
//				/* write iso timestamp to end of jpeg */
//				char buffer[64] = "no_ts";
//				rtc_get_iso_timestamp(buffer, 64);
//				if (f_write(buffer, 1, 64, pxFile) != 64){
//					trace_printf("camera_task: write timestamp to pic error\n");
//				}
//
//				/* Close the file handle. */
//				f_close(pxFile);
//			} else {
//				trace_printf("camera_task: open file failed\n");
//				capture_workflow_error();
//			}
//		} else {
//			trace_printf("camera_task: get fifo length failed\n");
//			capture_workflow_error();
//		}
//
//		/* Release exclusive filesystem access. */
//		fs_access_give();
//	} else
//	if (msg->param1 < (FS_ACCESS_TIMEOUT / FS_POLL_ACCESS)) {
//		/* Try again to get filesystem access. */
//		camera_tell_delay(MSG_CAMERA_CAPTURE_WRITE_DISK, msg->param1 + 1, FS_POLL_ACCESS);
//		return;
//	} else {
//		/* Failed to get exclusive filesystem access. */
//		trace_printf("camera_task: fs lock failed\n");
//		capture_workflow_error();
//	}
//
//	/* Have the camera enter standby mode. */
//	camera_tell(MSG_CAMERA_ENTER_STANDBY, 0);
//	return;
//}
//#endif
//
///* Other functions ---------------------------------------------------------- */
//
///**
// * Capture workflow completed successfully.
// */
//void
//capture_workflow_done() {
//	/* Reset the consecutive failed workflow count. */
//	workflowFailedCount = 0;
//}
//
///**
// * Capture workflow encountered an error.
// */
//void
//capture_workflow_error() {
//	/* Clear the taskIshealthy flag if there are enough errors. */
//	if (workflowFailedCount++ > CAPTURE_FAILED_THRESHOLD) {
//		taskIsHealthy = pdFALSE;
//	}
//}
//
//#if 0
///**
// * Find the next image name for a JPG capture.
// */
//BaseType_t
//find_dcim_filename(char* buffer, uint8_t length) {
//	F_FIND xFindStruct;
//	for (int i = dcimIndex; i < (dcimIndex + 100); ++i) {
//		snprintf(buffer, length, "upload/dcim%d.jpg", i);
//		if (f_findfirst(buffer, &xFindStruct) == F_ERR_NOTFOUND) {
//			dcimIndex = i + 1;
//			return pdTRUE;
//		}
//	}
//	return pdFALSE;
//}
//
///**
// * Open a file handle for writing the capture.
// */
//F_FILE*
//open_dcim_handle(char* buffer, uint8_t length) {
//	/* Choose a file name for this image. */
//	if (find_dcim_filename(buffer, length) == pdFALSE) {
//		return NULL;
//	}
//	return f_open(buffer, "w");
//}

/**
 * Write the FIFO buffer contents to a file.
 */
//BaseType_t
//write_fifo_to_disk(F_FILE* pxFile, uint32_t length) {
//	/* Write the FIFO contents to disk. */
//	uint16_t chunk = 0;
//	for (uint16_t i = 0; length > 0; ++i) {
//		chunk = MIN(length, BURST_READ_LENGTH);
//		arduchip_burst_read(fifoBuffer, chunk);
//		length -= chunk;
//		if (f_write(fifoBuffer, 1, chunk, pxFile) != chunk) {
//			return pdFALSE;
//		}
//	}
//	return pdTRUE;
//}


