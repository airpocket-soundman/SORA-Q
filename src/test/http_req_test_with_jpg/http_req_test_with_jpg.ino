/*
 *  HTTPClient.ino - GainSpan WiFi Module Control Program
 *  Copyright 2019 Norikazu Goto
 *
 *  This work is free software; you can redistribute it and/or modify it under the terms 
 *  of the GNU Lesser General Public License as published by the Free Software Foundation; 
 *  either version 2.1 of the License, or (at your option) any later version.
 *
 *  This work is distributed in the hope that it will be useful, but without any warranty; 
 *  without even the implied warranty of merchantability or fitness for a particular 
 *  purpose. See the GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License along with 
 *  this work; if not, write to the Free Software Foundation, 
 *  Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "config.h"
#include <Camera.h>

#define  CONSOLE_BAUDRATE  115200

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


void parse_httpresponse(char *message)
{
	char *p;
	
	if ((p=strstr(message, "200 OK\r\n")) != NULL) {
		ConsolePrintf("Response : %s\r\n", p+8);
	}
}

void setup() {

  CamErr err;

	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite(LED0, LOW);   // turn the LED off (LOW is the voltage level)
	Serial.begin(CONSOLE_BAUDRATE); // talk to PC

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


  //canera setup
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
    CAM_IMGSIZE_QUADVGA_H,
    CAM_IMGSIZE_QUADVGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS)
  {
    printError(err);
  }

}

// the loop function runs over and over again forever
void loop() {
	httpStat = POST;
	bool result = false;
	static int count = 0;

	while (1) {
    Serial.println("check while===============");
    Serial.println(httpStat);
		switch (httpStat) {
		case POST:
      Serial.println("POST method");
			theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");
			//create post data.
			snprintf(sendData, sizeof(sendData), "data=%d", count);
			result = theHttpGs2200.post(HTTP_POST_PATH, sendData);
			if (false == result) {
				break;
			}
      Serial.println("check00");

			do {
				result = theHttpGs2200.receive(5000);
				if (result) {
					theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
          Serial.println("rec start");
					ConsolePrintf("%s", (char *)(Receive_Data));
          Serial.println("rec end");
				} else {
					// AT+HTTPSEND command is done
					ConsolePrintf( "\r\n");
				}
			} while (result);

			result = theHttpGs2200.end();

			delay(3000);
			count+=100;
			httpStat = GET;
			break;
			
		case GET:
      Serial.println("GET method");
			theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "identity");

      Serial.println("check01");
			result = theHttpGs2200.get(HTTP_GET_PATH);
			if (true == result) {
				theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
				parse_httpresponse((char *)(Receive_Data));
        ConsolePrintf(Receive_Data);
			} else {
				ConsoleLog( "?? Unexpected HTTP Response ??" );
			}
      Serial.println("check02");
 			do {
				result = theHttpGs2200.receive(2000);
        Serial.println("check do");
				if (result) {
					theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
					ConsolePrintf("recieve data = %s\n", (char *)(Receive_Data));
				} else {
					// AT+HTTPSEND command is done
					ConsolePrintf( "\r\n");
				}
			} while (result);
      Serial.println("check03");
			result = theHttpGs2200.end();

			delay(3000);
			httpStat = POST;
			break;
		default:
			break;
		}
	}
}