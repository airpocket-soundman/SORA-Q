#include <MqttGs2200.h>
#include <TelitWiFi.h>
#include "config.h"
#include <SDHCI.h>
#include <File.h>


#define  CONSOLE_BAUDRATE  115200

/*-------------------------------------------------------------------------*
 * Globals:
 *-------------------------------------------------------------------------*/
TelitWiFi gs2200;
TWIFI_Params gsparams;
MqttGs2200 theMqttGs2200(&gs2200);

// ファイル保存管理用
SDClass SD;
File imgFile;  // Fileオブジェクト
// JPEG画像を保存するバッファサイズ
#define JPEG_BUFFER_SIZE 512
// JPEGデータの格納用バッファ
uint8_t jpgBuffer[JPEG_BUFFER_SIZE];


bool served = false;
uint16_t len, count=0;
MQTTGS2200_Mqtt mqtt;


// the setup function runs once when you press reset or power the board
void setup() {
	MQTTGS2200_HostParams hostParams;
	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite( LED0, LOW );   // turn the LED off (LOW is the voltage level)
	Serial.begin( CONSOLE_BAUDRATE ); // talk to PC

	/* Initialize SPI access of GS2200 */
	Init_GS2200_SPI_type(iS110B_TypeA);

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
	hostParams.host = (char *)MQTT_SRVR;
	hostParams.port = (char *)MQTT_PORT;
	hostParams.clientID = (char *)MQTT_CLI_ID;
	hostParams.userName = NULL;
	hostParams.password = NULL;

	theMqttGs2200.begin(&hostParams);

	digitalWrite( LED0, HIGH ); // turn on LED

  if (!SD.begin()) {
    Serial.println("SDカードの初期化に失敗しました");
    return;
  }
  // SDカード上のJPEGファイルを開く
  File jpgFile = SD.open("/airpocket_newlogo.jpg", FILE_READ);
  if (!jpgFile) {
    Serial.println("JPEGファイルを開けませんでした");
    return;
  }
  Serial.println("JPEGファイルを開きました");

  // JPEGファイルからデータを読み込み、シリアル出力
  int bytesRead;
  while ((bytesRead = jpgFile.read(jpgBuffer, JPEG_BUFFER_SIZE)) > 0) {
    // 読み込んだデータを処理（ここではシリアル出力）
    //Serial.write(jpgBuffer, bytesRead);
  }

  // ファイルを閉じる
  jpgFile.close();

  sendMQTT();
  sendJPG();
}



// the loop function runs over and over again forever
void loop() {
}

void sendMQTT(){
  if (!served) {
    // Start a MQTT client
    ConsoleLog( "Start MQTT Client");
    if (false == theMqttGs2200.connect()) {
      return;
    }
    served = true;
  }	
  
  
  ConsoleLog( "Start to send MQTT Message");
  // Prepare for the next chunck of incoming data
  WiFi_InitESCBuffer();
  // Start the loop to send the data
  strncpy(mqtt.params.topic, MQTT_TOPIC , sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;
  snprintf(mqtt.params.message, sizeof(mqtt.params.message), "%d", count++ );
  mqtt.params.len = strlen(mqtt.params.message);
  if (true == theMqttGs2200.publish(&mqtt)) {
    ConsolePrintf( "%d was sent\r\n", count-1 );

	}
}

void sendJPG(){
  if (!served) {
    // Start a MQTT client
    ConsoleLog( "Start MQTT Client");
    if (false == theMqttGs2200.connect()) {
      return;
    }
    served = true;
  }	
  
  
  ConsoleLog( "Start to send MQTT Message");
  // Prepare for the next chunck of incoming data
  WiFi_InitESCBuffer();
  // Start the loop to send the data
  strncpy(mqtt.params.topic, MQTT_TOPIC , sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;

  if (!SD.begin()) {
    Serial.println("SDカードの初期化に失敗しました");
    return;
  }
  // SDカード上のJPEGファイルを開く
  File jpgFile = SD.open("/airpocket_newlogo.jpg", FILE_READ);
  if (!jpgFile) {
    Serial.println("JPEGファイルを開けませんでした");
    return;
  }
  Serial.println("JPEGファイルを開きました");

  // JPEGファイルからデータを読み込み、シリアル出力
  int bytesRead;
  while ((bytesRead = jpgFile.read(jpgBuffer, JPEG_BUFFER_SIZE)) > 0) {
    // 読み込んだデータを処理（ここではシリアル出力）
    //Serial.write(jpgBuffer, bytesRead);
    //snprintf(mqtt.params.message, bytesRead, "%d", \ );
    Serial.println(sizeof(mqtt.params.message));
    snprintf(mqtt.params.message, sizeof(mqtt.params.message), "%d", 1234567890 );
    mqtt.params.len = strlen(mqtt.params.message);
    //mqtt.params.message = (char*)jpgBuffer; // バッファをそのままセット
    //mqtt.params.len = bytesRead; // 送信するデータの長さを指
    if (true == theMqttGs2200.publish(&mqtt)) {
      ConsolePrintf( "%d was sent\r\n", count-1 );
  	}
  }

  // ファイルを閉じる
  jpgFile.close();
}
