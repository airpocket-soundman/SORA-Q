/* SORA-Q projeckt                                  
 * ver.0.1.0 SDがある場合はFlashにファイルをコピーする
 * ver.0.1.1 SDが無い、NNBファイルが無い時の処理追加。フラッシュ保存先を変更したり処理が遅いのを調べたり<readbyteが遅い。起動時バージョン表示を追加
 * ver.0.1.2 D18,19.20,21のDout、A2,3のAin　テスト追加
 * Ver.0.2.0 MQTT受信追加
 *
 */

char version[] = "Ver.0.2.0";

#include <GS2200Hal.h>
#include <HttpGs2200.h>
#include <MqttGs2200.h>
#include <SDHCI.h>
#include <TelitWiFi.h>
#include <stdio.h> /* for sprintf */
#include <Camera.h>
#include "config.h"
#include <Flash.h>
#include <Arduino_JSON.h>

#define CONSOLE_BAUDRATE 115200
#define TOTAL_PICTURE_COUNT 3
#define PICTURE_INTERVAL 1000
#define FIRST_INTERVAL 3000
#define SUBSCRIBE_TIMEOUT 60000	//ms

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };
bool nnb_copy = true;

char nnbFile[] = "airpocket_newlogo.jpg";
char flashPath[] = "data/slim.nnb";
char flashFolder[] = "data/";
//char nnbFile[] = "slim.nnb";

String param1, param2, param3, param4, param5, param6;

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams httpHostParams; // HTTPサーバ接続用のホストパラメータ
SDClass theSD;
MqttGs2200 theMqttGs2200(&gs2200);
MQTTGS2200_HostParams mqttHostParams; // MQTT接続のホストパラメータ
bool served = false;
MQTTGS2200_Mqtt mqtt;

// test mode on/off
bool imgPostTest    = true;    //イメージ撮影とhttp post requestのテスト
bool nnbFilePost    = false;    //NNBファイルをhttp postしてチェック
bool digitalOutTest = false;    //
bool analogReadTest = false;

void splitString(String input, String &var1, String &var2, String &var3, String &var4, String &var5, String &var6) {
    int startIndex = 0;
    int commaIndex;
    int varCount = 0;

    while ((commaIndex = input.indexOf(',', startIndex)) != -1 && varCount < 5) {
        if (varCount == 0) {
            var1 = input.substring(startIndex, commaIndex);
        } else if (varCount == 1) {
            var2 = input.substring(startIndex, commaIndex);
        } else if (varCount == 2) {
            var3 = input.substring(startIndex, commaIndex);
        } else if (varCount == 3) {
            var4 = input.substring(startIndex, commaIndex);
        } else if (varCount == 4) {
            var5 = input.substring(startIndex, commaIndex);
        }
        startIndex = commaIndex + 1;
        varCount++;
    }
    
    // 最後の変数に残りの部分を格納
    if (varCount < 6) {
        var6 = input.substring(startIndex);
    } else {
        var6 = ""; // 余った部分がない場合
    }
}

void checkMQTTtopic(){ 
  String data;
  /* just in case something from GS2200 */
  while (gs2200.available()) {
    if (false == theMqttGs2200.receive(data)) {
      served = false; // quite the loop
      break;
    }

    Serial.println("Recieve data: " + data);
    
    // dataをパース
    splitString(data, param1, param2, param3, param4, param5, param6);
    
    Serial.println("param 1: " + param1);
    Serial.println("param 2: " + param2);
    Serial.println("param 3: " + param3);
    Serial.println("param 4: " + param4);
    Serial.println("param 5: " + param5);
    Serial.println("param 6: " + param6);

  }


}

void checkDigitalOut(){
  if (digitalOutTest){
    int counter = 10;
    char buffer[10];
    Serial.println("==== Digital out Test D18/D19/D20/D21");
    while (counter > 0){
      sprintf(buffer, "ON, %02d/10",counter);
      Serial.println(buffer);
      digitalWrite(18,HIGH);
      digitalWrite(19,HIGH);
      digitalWrite(20,HIGH);
      digitalWrite(21,HIGH);

      delay(1000);
      Serial.println("OFF");
      digitalWrite(18,LOW);
      digitalWrite(19,LOW);
      digitalWrite(20,LOW);
      digitalWrite(21,LOW);
      delay(1000);
      counter--;
    }
    Serial.println("==== digital out test is finished.\n");
  } else {
    Serial.println("==== digital out test was skipped.\n");
  }
}

void checkAnalogRead(){
  if (analogReadTest){
    int counter = 300;
    int valueA = 0;
    int valueB = 0;
    char buffer[30];
    Serial.println("==== Analog Read test A2/A3");
    while (counter > 0){
      valueA = analogRead(A2);
      valueB = analogRead(A3);
      sprintf(buffer, "valueA = %d / valueB = %d /    %03d/300",valueA, valueB, counter);
      Serial.println(buffer);
      delay(100);
      counter--;
    }
    Serial.println("==== the analog read test is finished.\n");
  } else {
    Serial.println("==== analog read test was skipped.\n");
  }
}

void move_nnbFile(){
  if (nnb_copy){
    File readFile = theSD.open(nnbFile, FILE_READ);
    uint32_t file_size = readFile.size();
    Serial.print("SD data file size = ");
    Serial.println(file_size);
    char *body = (char *)malloc(file_size);  // +1 is null char
    if (body == NULL) {
      Serial.println("No free memory");
    }
    int index = 0;
    while (readFile.available()) {
      body[index++] = readFile.read();
    }
    readFile.close();
    //Flash.format();
    Flash.mkdir(flashFolder);

    if (Flash.exists(flashPath)) {
      Flash.remove(flashPath);  // ファイルを削除
    }
    
    File writeFile = Flash.open(flashPath, FILE_WRITE);
    if (writeFile) {
      Serial.println("nnb fileをフラッシュメモリ に書き込み中...");

      // body のデータをフラッシュメモリに書き込む
      writeFile.write((uint8_t*)body, file_size);

      writeFile.close(); // ファイルを閉じる
      Serial.println("nnb fileをフラッシュメモリへ書き込み完了");
    } else {
      // ファイルのオープンに失敗した場合
      Serial.println("フラッシュメモリのファイルオープンに失敗しました");
    }
    free(body);
  } else {
    Serial.println("move nnb file passed.\n");
  }

}

void parse_httpresponse(char *message) {
  char *p;

  if ((p = strstr(message, "200 OK\r\n")) != NULL) {
    ConsolePrintf("Response : %s\r\n", p + 8);
  }
}

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
  Serial.println("Size");
  Serial.println(size_string);

  bool result = false;
  result = theHttpGs2200.connect();
  WiFi_InitESCBuffer();
  result = theHttpGs2200.send(HTTP_METHOD_POST, 10, url_path, body, size);
  return result;

}

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

/* カメラ撮影とhttp request postのテスト*/
void camImagePost(){
  if (imgPostTest){
    Serial.println("==== start Camera Image Post Test");
    int take_picture_count = 0;
    while (take_picture_count < TOTAL_PICTURE_COUNT) {
      Serial.println("call takePicture()");
      CamImage img = theCamera.takePicture();

      if (img.isAvailable()) {
        uploadImage(img);
      } else {
        Serial.println("Failed to take picture");
      }
    take_picture_count++;
    }
    theCamera.end();
    Serial.println("==== cam Image Post test is finished.\n");
  } else {
    Serial.println("==== cam Image Post test was passed.\n");
  }
}

void uploadNNB() {
  if (nnbFilePost){
    Serial.println("===== start NNB file post");
    File file = Flash.open(flashPath, FILE_READ);
    Serial.println("flash opened");
    if (file){
      // Calculate size in byte
      uint32_t file_size = file.size();
      Serial.println(file_size);
      // Define_a body pointer having the continuous memory space with size `file_size`
      char *body = (char *)malloc(file_size);  // +1 is null char
      if (body == NULL) {
        Serial.println("No free memory");
      }
      Serial.println("nnb read byte");
      // Read byte of the file iteratively and put it in the address where each member of body pointer points out
      int index = 0;
      while (file.available()) {
        body[index++] = file.read();
      }
      file.close();
      Serial.println("nnb read finished");
      // Send the body data to the server
      bool result = custom_post(HTTP_POST_PATH, body, file_size);
      if (false == result) {
        Serial.println("Post Failed");
      }
      free(body);
      Serial.println("custom_post finished");
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
      Serial.println("NNBファイル送信完了======================= \n");
    } else {
      Serial.println("nnbファイルがありません \n");
    }

  } else {
  Serial.println("==== nnb file post is skipped.\n");
  }
}

/* wifi Setup */
void GS2200wifiSetup(){
  /* initialize digital pin LED_BUILTIN as an output. */
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)

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

  httpHostParams.host = (char *)HTTP_SRVR_IP;
  httpHostParams.port = (char *)HTTP_PORT;
  theHttpGs2200.begin(&httpHostParams);

  Serial.println("Start HTTP Client");
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/octet-stream");

  mqttHostParams.host     = (char *)MQTT_SRVR;        // MQTTサーバのホスト名
  mqttHostParams.port     = (char *)MQTT_PORT;        // MQTTサーバのポート番号
  mqttHostParams.clientID = (char *)MQTT_CLI_ID;      // MQTTクライアントID
  mqttHostParams.userName = NULL;                     // ユーザー名（使用しない場合はNULL）
  mqttHostParams.password = NULL;                     // パスワード（使用しない場合はNULL）
  theMqttGs2200.begin(&mqttHostParams);               // MQTTクライアントを初期化

  ConsoleLog( "Start MQTT Client");
  if (false == theMqttGs2200.connect()) {
    return;
  }

  ConsoleLog( "Start to receive MQTT Message");
  // Prepare for the next chunck of incoming data
  WiFi_InitESCBuffer();

  // Start the loop to receive the data
  strncpy(mqtt.params.topic, MQTT_TOPIC1 , sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;
  if (true == theMqttGs2200.subscribe(&mqtt)) {
    ConsolePrintf( "Subscribed! \n" );
  } 


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
  Serial.println(version);

  /* Initialize SD */
  Serial.println("theSD.begin()");
  if(!theSD.begin()) {
    Serial.println("SD card mount failed");
    nnb_copy = false;
  }

    /* Initialize SD */
  if(!Flash.begin()) {
    Serial.println("Flash card mount failed");
  }

  //Pin initialize
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)

  // GS2200 WiFi setup

  GS2200wifiSetup();
  
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

  move_nnbFile();
  uploadNNB();
  camImagePost();
  checkDigitalOut();
  checkAnalogRead();
}




void loop() {
  delay(FIRST_INTERVAL); /* wait for predefined seconds to take still picture. */
  
  checkMQTTtopic();


				
}
