#include "MqttGs2200.h"
#include <TelitWiFi.h>
#include "config.h"
#include <Camera.h>

#define  CONSOLE_BAUDRATE  115200
#define  SUBSCRIBE_TIMEOUT 60000	// サブスクリプションのタイムアウト（ミリ秒）

/*-------------------------------------------------------------------------*
 * グローバル変数:
 *-------------------------------------------------------------------------*/
TelitWiFi gs2200;                // WiFiモジュールのインスタンス
TWIFI_Params gsparams;           // WiFiのパラメータ
MqttGs2200 theMqttGs2200(&gs2200); // MQTTクライアントインスタンス

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

// setup関数は、リセットボタンを押すかボードに電源を入れたときに一度実行される
void setup() {
  Serial.begin(CONSOLE_BAUDRATE);   // PCとのシリアル通信開始
  CamErr err;

  
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

  CamImage img = theCamera.takePicture();
  Serial.println("image size = ");
  Serial.println(img.getImgSize());
  theCamera.end();

  MQTTGS2200_HostParams hostParams; // MQTT接続のホストパラメータ
  pinMode(LED0, OUTPUT);            // LEDピンを出力に設定
  digitalWrite(LED0, LOW);          // LEDを消灯（LOWは電圧レベル）


  /* GS2200のSPIアクセスを初期化 */
  Init_GS2200_SPI_type(iS110B_TypeA);

  /* ATコマンドライブラリバッファを初期化 */
  gsparams.mode = ATCMD_MODE_STATION;
  gsparams.psave = ATCMD_PSAVE_DEFAULT;
  if (gs2200.begin(gsparams)) {
    ConsoleLog("GS2200の初期化に失敗しました");
    while (1); // 初期化失敗時は無限ループ
  }

  /* GS2200をアクセスポイントに接続 */
  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
    ConsoleLog("接続に失敗しました");
    while (1); // 接続失敗時は無限ループ
  }

  hostParams.host = (char *)MQTT_SRVR;    // MQTTサーバのホスト名
  hostParams.port = (char *)MQTT_PORT;    // MQTTサーバのポート番号
  hostParams.clientID = (char *)MQTT_CLI_ID; // MQTTクライアントID
  hostParams.userName = NULL;              // ユーザー名（使用しない場合はNULL）
  hostParams.password = NULL;               // パスワード（使用しない場合はNULL）
  theMqttGs2200.begin(&hostParams);        // MQTTクライアントを初期化
  digitalWrite(LED0, HIGH);                // LEDを点灯

  // setup内で一度メッセージをパブリッシュ
  MQTTGS2200_Mqtt mqtt;
  strncpy(mqtt.params.topic, MQTT_TOPIC_2, sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;

  // メッセージを準備
  snprintf(mqtt.params.message, sizeof(mqtt.params.message), "%d", 0); // デモ用に'0'を送信
  mqtt.params.len = strlen(mqtt.params.message);
  
  // MQTTクライアントに接続を試みる
  ConsoleLog("MQTTクライアントを開始");
  if (theMqttGs2200.connect()) {
    // メッセージをパブリッシュ
    if (theMqttGs2200.publish(&mqtt)) {
        ConsolePrintf("送信されたメッセージ: %d\r\n", 0);
    }
  } else {
    ConsoleLog("MQTT接続に失敗しました");
  }
}

//char server_cid = 0;                      // サーバーのCID
//bool served = false;                      // サブスクリプション状態
//uint16_t len, count = 0;                  // データ長とカウント
//MQTTGS2200_Mqtt mqtt1;                    // 最初のトピック用のMQTTオブジェクト
//MQTTGS2200_Mqtt mqtt2;                    // 2番目のトピック用のMQTTオブジェクト

// loop関数は、永遠に繰り返し実行される
void loop() {
}

