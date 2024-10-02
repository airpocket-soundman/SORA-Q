/* SORA-Q projeckt                                  
 * ver.0.0.1
 *
 *
 *
 *
 *
 */



#include <HttpGs2200.h>
#include <GS2200Hal.h>
#include <SDHCI.h>
#include <TelitWiFi.h>
#include <stdio.h> /* for sprintf */
#include <Camera.h>
#include "config.h"

#define CONSOLE_BAUDRATE 115200
#define TOTAL_PICTURE_COUNT 3
#define PICTURE_INTERVAL 1
#define FIRST_INTERVAL 3

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams hostParams;
SDClass theSD;
int take_picture_count = 0;

void parse_httpresponse(char *message) {
  char *p;

  if ((p = strstr(message, "200 OK\r\n")) != NULL) {
    ConsolePrintf("Response : %s\r\n", p + 8);
  }
}

/* ---------------------------------------------------------------------
 * Function to print error message from the camera
* ----------------------------------------------------------------------
*/
void printError(enum CamErr err) {
  Serial.print("Error: ");
  switch (err) {
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

/* ---------------------------------------------------------------------
* Function to send a file in the SD card to the HTTP server
* ----------------------------------------------------------------------
*/
void uploadImage(CamImage img) {
 
  //CamImage img = theCamera.takePicture();
  Serial.print("img.isAvailable() = ");
  Serial.println(img.isAvailable());
  size_t imageSize = img.getImgSize();
  Serial.print("Image size: ");
  Serial.println(imageSize);

  bool result = custom_post(HTTP_POST_PATH, img.getImgBuff(), img.getImgSize());
  if (false == result) {
    Serial.println("Post Failed");
  }
  //free(body);

  result = false;
  do {
    result = theHttpGs2200.receive(5000);
    if (result) {
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
      ConsolePrintf("%s", (char *)(Receive_Data));
    } else {
      // AT+HTTPSEND command is done
      Serial.println("\r\n");
    }
  } while (result);

  result = theHttpGs2200.end();
}


/* ---------------------------------------------------------------------
* Setup Function
* ----------------------------------------------------------------------
*/
void setup() {
  /* Open serial communications and wait for port to open */

  Serial.begin(CONSOLE_BAUDRATE);
  while (!Serial) {
    ; /* wait for serial port to connect. Needed for native USB port only */
  }

  /* Initialize SD */
  while (!theSD.begin()) {
    /* wait until SD card is mounted. */
    Serial.println("Insert SD card.");
  }

  /* -----------------------------------
   * GS2200-WiFi Setup
   * -----------------------------------
   */

  /* initialize digital pin LED_BUILTIN as an output. */
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)
  Serial.begin(CONSOLE_BAUDRATE);  // talk to PC

  /* Initialize SPI access of GS2200 */
  Init_GS2200_SPI_type(iS110B_TypeA);

  /* Initialize AT Command Library Buffer */
  gsparams.mode = ATCMD_MODE_STATION;
  gsparams.psave = ATCMD_PSAVE_DEFAULT;
  if (gs2200.begin(gsparams)) {
    Serial.println("GS2200 Initilization Fails");
    while (1)
      ;
  }

  /* GS2200 Association to AP */
  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
    Serial.println("Association Fails");
    while (1)
      ;
  }

  hostParams.host = (char *)HTTP_SRVR_IP;
  hostParams.port = (char *)HTTP_PORT;
  theHttpGs2200.begin(&hostParams);

  Serial.println("Start HTTP Client");
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/octet-stream");

  /* -----------------------------------
   * Camera Setup
   * -----------------------------------
   */

  CamErr err;

  /* begin() without parameters means that
   * number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */

  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  /* Auto white balance configuration */

  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }


  /* Set parameters about still picture.
   * In the following case, QUADVGA and JPEG.
   */

  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    //CAM_IMGSIZE_QUADVGA_H,      //1280
    //CAM_IMGSIZE_QUADVGA_V,      //960
    CAM_IMGSIZE_VGA_H,          //640
    CAM_IMGSIZE_VGA_V,          //480
    //CAM_IMGSIZE_QVGA_H,         //320
    //CAM_IMGSIZE_QVGA_V,         //240
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  digitalWrite(LED0, HIGH);  // turn on LED
}

/* ---------------------------------------------------------------------
* Function to send byte data to the HTTP server
* ----------------------------------------------------------------------
*/
bool custom_post(const char *url_path, const char *body, uint32_t size) {
  char size_string[10];
  snprintf(size_string, sizeof(size_string), "%d", size);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_LENGTH, size_string);
  Serial.println("Size");
  Serial.println(size_string);

  bool result = false;
  result = theHttpGs2200.connect();
  WiFi_InitESCBuffer();
  result = theHttpGs2200.send(HTTP_METHOD_POST, 10, url_path, body, size);
  return result;
}


bool first = false;
// the loop function runs over and over again forever
void loop() {

  sleep(FIRST_INTERVAL); /* wait for predefined seconds to take still picture. */
  if (!first) {
    // first = true;

    if (take_picture_count < TOTAL_PICTURE_COUNT) {

      Serial.println("call takePicture()");
      CamImage img = theCamera.takePicture();

      if (img.isAvailable()) {
        uploadImage(img);
      } else {
        Serial.println("Failed to take picture");
      }

    } else if (take_picture_count == TOTAL_PICTURE_COUNT) {
      Serial.println("End.");
      theCamera.end();
    }

    take_picture_count++;
  }
}