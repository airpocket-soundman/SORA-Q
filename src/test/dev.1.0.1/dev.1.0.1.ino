/*
✔ Ver.1.0.0 MQTTサブスクライブ関数化
✔ Ver.1.0.1 MQTTでストリーミングをスイッチンぐしつつCamCBで撮影してpost txtもpost
Ver.1.0.2 dnnモデル保存を追加

*/
char version[] = "Ver.1.0.1";

#include <GS2200Hal.h>
#include <HttpGs2200.h>
#include <MqttGs2200.h>
#include <SDHCI.h>
#include <TelitWiFi.h>
#include <stdio.h> /* for sprintf */
#include <Camera.h>
#include "config.h"
#include <Flash.h>

#define  CONSOLE_BAUDRATE  115200
#define  SUBSCRIBE_TIMEOUT 60000	//ms

//Camera
CamErr err;

//wifi
TelitWiFi gs2200;
TWIFI_Params gsparams;
MqttGs2200 theMqttGs2200(&gs2200);
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams httpHostParams; // HTTPサーバ接続用のホストパラメータ

char server_cid = 0;
bool served = false;
uint16_t len, count=0;
MQTTGS2200_Mqtt mqtt;

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };


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

bool custom_post(const char *url_path, const char *body, uint32_t size) {
  char size_string[10];
  snprintf(size_string, sizeof(size_string), "%d", size);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_LENGTH, size_string);
  bool result = false;
  result = theHttpGs2200.connect();
  WiFi_InitESCBuffer();
  result = theHttpGs2200.send(HTTP_METHOD_POST, 10, url_path, body, size);
  return result;
}

void uploadImage(uint16_t* imgBuffer, size_t imageSize) {
  bool result = custom_post(HTTP_POST_PATH, (const uint8_t*)imgBuffer, imageSize);
  if (false == result) {
    Serial.println("Post Failed");
  }
  result = false;
  do {
    result = theHttpGs2200.receive(5000);
    if (result) {
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
      ConsolePrintf("%s", (char *)(Receive_Data));
      //Serial.printf("http receive %s", (char *)(Receive_Data));
    } else {
      Serial.println("\r\n");
    }
  } while (result);
  result = theHttpGs2200.end();
}

bool uploadString(const String &body) {
  // 送信するデータを格納するバッファ
  char sendData[100];
  
  // Stringからchar配列にコピー（bodyの長さが100を超えない前提で）
  body.toCharArray(sendData, sizeof(sendData));  // 文字列をsendDataにコピー

  size_t size = strlen(sendData);  // 送信する文字列の長さを取得

  // custom_post関数を呼び出してPOSTリクエストを送信
  bool result = custom_post(HTTP_POST_TEXT_PATH, (const uint8_t*)sendData, size);
  if (false == result) {
    Serial.println("Post Failed");
    return false;
  }

  // 受信処理
  result = false;
  do {
    result = theHttpGs2200.receive(5000);  // タイムアウトは5000ms
    if (result) {
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);
      ConsolePrintf("%s", (char *)(Receive_Data));  // 受信データを表示
    } else {
      Serial.println("\r\nNo response from server");
    }
  } while (result);

  // 終了処理
  theHttpGs2200.end();
  return true;
}


void camImagePost(){
  CamImage imgStill = theCamera.takePicture();

  if (imgStill.isAvailable()) {

    String status = "post txt";
    uploadString(status);

    uint16_t* imgBuffer = (uint16_t*)imgStill.getImgBuff();
    if (imgBuffer == nullptr) {
      Serial.println("Image buffer is null");
      return;
    }
    size_t imageSize = imgStill.getImgSize();
    uploadImage(imgBuffer, imageSize);

  } else {
    Serial.println("Failed to take picture");
  }
}

void subscribeMQTT() {
	if (!served) {
		// Start a MQTT client
		ConsoleLog("Start MQTT Client");
		if (false == theMqttGs2200.connect()) {
			return;
		}

		ConsoleLog("Start to receive MQTT Message");
		// Prepare for the next chunk of incoming data
		WiFi_InitESCBuffer();

		// Start the loop to receive the data
		strncpy(mqtt.params.topic, MQTT_TOPIC, sizeof(mqtt.params.topic));
		mqtt.params.QoS = 0;
		mqtt.params.retain = 0;
		if (true == theMqttGs2200.subscribe(&mqtt)) {
			ConsolePrintf("Subscribed! \n");
		}

		served = true;
	} else {
		uint64_t start = millis();
		while (served) {
			if (msDelta(start) < SUBSCRIBE_TIMEOUT) {
				String data;
				/* just in case something from GS2200 */
				while (gs2200.available()) {
					if (false == theMqttGs2200.receive(data)) {
						served = false; // quit the loop
						break;
					}

					Serial.println("Receive data: " + data);
          if (data == "streamOFF"){
            Serial.println("command camera off");
            err = theCamera.startStreaming(false, CamCB);
            if (err != CAM_ERR_SUCCESS) {
              printError(err);
            }
            camImagePost();


          } else if (data == "streamON"){
            Serial.println("command camera on");
            err = theCamera.startStreaming(true, CamCB);
            if (err != CAM_ERR_SUCCESS) {
              printError(err);
            }
          }
				}
				start = millis();
			} else {
				ConsolePrintf("Subscribed over for %d ms! \n", SUBSCRIBE_TIMEOUT);
				theMqttGs2200.stop();
				served = false;
				exit(0);
			}
		}
	}
}

void CamCB(CamImage img){
  Serial.println("->CamCB Call back");
}



// the setup function runs once when you press reset or power the board
void setup() {
	MQTTGS2200_HostParams mqttHostParams;
	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite( LED0, LOW );   // turn the LED off (LOW is the voltage level)
	Serial.begin( CONSOLE_BAUDRATE ); // talk to PC
  Serial.println(version);

	/* Initialize SPI access of GS2200 */
  #if defined(iS110_TYPEA)
    Init_GS2200_SPI_type(iS110B_TypeA);
  #elif defined(iS110_TYPEB)
    Init_GS2200_SPI_type(iS110B_TypeB);
  #elif defined(iS110_TYPEC)
    Init_GS2200_SPI_type(iS110B_TypeC);
  #else
    #error "No valid device type defined. Please define iS110_TYPEA, iS110_TYPEB, or iS110_TYPEC."
  #endif

	/* Initialize AT Command Library Buffer */
	gsparams.mode = ATCMD_MODE_STATION;
	gsparams.psave = ATCMD_PSAVE_DEFAULT;
	if (gs2200.begin(gsparams)) {
		ConsoleLog("GS2200 Initilization Fails");
		while(1);
	}

	/* GS2200 Association to AP */
	if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
		ConsoleLog("Association Fails");
		while(1);
	}
 
  httpHostParams.host = (char *)HTTP_SRVR_IP;
  httpHostParams.port = (char *)HTTP_PORT;
  theHttpGs2200.begin(&httpHostParams);

  Serial.println("Start HTTP Client");
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/octet-stream");


	mqttHostParams.host = (char *)MQTT_SRVR;
	mqttHostParams.port = (char *)MQTT_PORT;
	mqttHostParams.clientID = (char *)MQTT_CLI_ID;
	mqttHostParams.userName = NULL;
	mqttHostParams.password = NULL;
	theMqttGs2200.begin(&mqttHostParams);
	digitalWrite( LED0, HIGH ); // turn on LED


  // Camera setup ========================================================
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

  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H,         //320
    CAM_IMGSIZE_QVGA_V,         //240
    CAM_IMAGE_PIX_FMT_JPG
    );
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
}



// the loop function runs over and over again forever
void loop() {
  Serial.println("loop");
  subscribeMQTT();
}
