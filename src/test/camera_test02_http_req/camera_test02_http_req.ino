/*
 *  camera.ino - Simple camera example sketch
 *  Copyright 2018, 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  This is a test app for the camera library.
 *  This library can only be used on the Spresense with the FCBGA chip package.
 */

#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "config.h"
#include <Camera.h>
#include <SDHCI.h>

SDClass  theSD;

#define BAUDRATE                115200
#define TOTAL_PICTURE_COUNT     10

int take_picture_count = 0;

typedef enum{
	POST=0,
	GET
} DEMO_STATUS_E;

DEMO_STATUS_E httpStat;
char sendData[100];

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = {0};

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams hostParams;

void parse_httpresponse(char *message)
{
	char *p;
	
	if ((p=strstr(message, "200 OK\r\n")) != NULL) {
		ConsolePrintf("Response : %s\r\n", p+8);
	}
}




/**
 * Print error message
 */
void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
  {
    case CAM_ERR_NO_DEVICE:
      Serial.println("No Device");
      break;
    case CAM_ERR_ILLEGAL_DEVERR:
      Serial.println("Illegal device error");
      break;
    case CAM_ERR_ALREADY_INITIALIZED:
      Serial.println("Already initialized");
      break;
    case CAM_ERR_NOT_INITIALIZED:
      Serial.println("Not initialized");
      break;
    case CAM_ERR_NOT_STILL_INITIALIZED:
      Serial.println("Still picture not initialized");
      break;
    case CAM_ERR_CANT_CREATE_THREAD:
      Serial.println("Failed to create thread");
      break;
    case CAM_ERR_INVALID_PARAM:
      Serial.println("Invalid parameter");
      break;
    case CAM_ERR_NO_MEMORY:
      Serial.println("No memory");
      break;
    case CAM_ERR_USR_INUSED:
      Serial.println("Buffer already in use");
      break;
    case CAM_ERR_NOT_PERMITTED:
      Serial.println("Operation not permitted");
      break;
    default:
      break;
  }
}

/**
 * Callback from Camera library when video frame is captured.
 */
void CamCB(CamImage img)
{
  /* Check the img instance is available or not. */
  if (img.isAvailable())
  {
    /* If you want RGB565 data, convert image data format to RGB565 */
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);

    /* You can use image data directly by using getImgSize() and getImgBuff(). */
    Serial.print("Image data size = ");
    Serial.print(img.getImgSize(), DEC);
    Serial.print(" , ");
    Serial.print("buff addr = ");
    Serial.print((unsigned long)img.getImgBuff(), HEX);
    Serial.println("");
  }
  else
  {
    Serial.println("Failed to get video stream image");
  }
}

/**
 * @brief Initialize camera
 */
void setup()
{
  CamErr err;

  /* Open serial communications and wait for port to open */
  Serial.begin(BAUDRATE);
  while (!Serial)
  {
    ; /* wait for serial port to connect. Needed for native USB port only */
  }

  /* Prepare camera */
  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Start video stream. */
  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Auto white balance configuration */
  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

  /* Set parameters about still picture. */
  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_VGA_H,
    CAM_IMGSIZE_VGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }


  /* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite(LED0, LOW);   // turn the LED off (LOW is the voltage level)

/* Initialize SPI access of GS2200 */
	Init_GS2200_SPI_type(iS110B_TypeA);

	/* Initialize AT Command Library Buffer */
	gsparams.mode = ATCMD_MODE_STATION;
	gsparams.psave = ATCMD_PSAVE_DEFAULT;
	if (gs2200.begin(gsparams)) {
		ConsoleLog("GS2200 Initilization Fails");
		while (1);
	}

	/* GS2200 Association to AP */
	if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
		ConsoleLog("Association Fails");
		while (1);
	}

	hostParams.host = (char *)HTTP_SRVR_IP;
	hostParams.port = (char *)HTTP_PORT;
	theHttpGs2200.begin(&hostParams);

	ConsoleLog("Start HTTP Client");

	/* Set HTTP Headers */
	theHttpGs2200.config(HTTP_HEADER_AUTHORIZATION, "Basic dGVzdDp0ZXN0MTIz");
	theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");
	theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/x-www-form-urlencoded");
	theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);

	digitalWrite(LED0, HIGH); // turn on LED

}

/**
 * @brief Take picture without saving it to SD card
 */
void loop()
{
  sleep(1); /* wait for one second to take still picture. */

  if (take_picture_count < TOTAL_PICTURE_COUNT)
  {
    /* Take still picture. */
    Serial.println("call takePicture()");
    CamImage img = theCamera.takePicture();

    /* Check availability of the img instance. */
    if (img.isAvailable())
    {
      Serial.print("Image captured successfully. Size: ");
      Serial.print(img.getImgSize());
      Serial.println(" bytes");

      char filename[16] = {0};
      sprintf(filename, "PICT_%03d.JPG", take_picture_count);

      Serial.print("Save taken picture as ");
      Serial.print(filename);
      Serial.println("");

      theSD.remove(filename);
      File myFile = theSD.open(filename, FILE_WRITE);
      myFile.write(img.getImgBuff(), img.getImgSize());
      myFile.close();
    }
    else
    {
      Serial.println("Failed to take picture");
    }

  }
  else if (take_picture_count == TOTAL_PICTURE_COUNT)
  {
    Serial.println("End.");
    theCamera.end();
  }

  take_picture_count++;
}
