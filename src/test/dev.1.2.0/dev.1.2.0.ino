/* SORA-Q projeckt                                  
 * ver.0.1.0 SDがある場合はFlashにファイルをコピーする
 * ver.0.1.1 SDが無い、NNBファイルが無い時の処理追加。フラッシュ保存先を変更したり処理が遅いのを調べたり<readbyteが遅い。起動時バージョン表示を追加
 * ver.0.1.2 D18,19.20,21のDout、A2,3のAin　テスト追加
 * Ver.0.2.0 MQTT受信追加
 *
 */

char version[] = "Ver.0.2.0";
#include "config.h"

#include <GS2200Hal.h>
#include <HttpGs2200.h>
#include <MqttGs2200.h>
#include <TelitWiFi.h>

#include <stdio.h> /* for sprintf */

#include <Camera.h>
#include <DNNRT.h>
#include <Flash.h>
#include <SDHCI.h>


#define CONSOLE_BAUDRATE 115200
#define PICTURE_INTERVAL 1000
#define FIRST_INTERVAL 3000
#define SUBSCRIBE_TIMEOUT 60000	//ms

#define AIN1 A2       //左車輪エンコーダ
#define AIN2 A3       //右車輪エンコーダ
#define IN1_LEFT  14//19       //左車輪出力
#define IN1_RIGHT 15//21       //右車輪出力
#define IN2_LEFT  18       //左車輪方向
#define IN2_RIGHT 20       //右車輪方向
#define PHOTO_REFLECTOR_THRETHOLD_LEFT  100
#define PHOTO_REFLECTOR_THRETHOLD_RIGHT 100

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };
bool nnb_copy = true;

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams httpHostParams; // HTTPサーバ接続用のホストパラメータ

MqttGs2200 theMqttGs2200(&gs2200);
MQTTGS2200_HostParams mqttHostParams; // MQTT接続のホストパラメータ
bool served = false;
MQTTGS2200_Mqtt mqtt;

SDClass theSD;
char nnbFile[] = "model.nnb";
char flashPath[] = "data/slim.nnb";
char flashFolder[] = "data/";

// test mode on/off
bool imgPostTest    = true;    //イメージ撮影とhttp post requestのテスト
bool nnbFilePost    = false;    //NNBファイルをhttp postしてチェック
bool analogReadTest = false;
bool driveTest      = true;


// イメージ設定
#define DNN_IMG_W 28
#define DNN_IMG_H 28
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 96//96
#define CAM_CLIP_Y 0//64
#define CAM_CLIP_W 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define CAM_CLIP_H 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)

//DNN
float threshold = 0.2;
DNNRT dnnrt;
DNNVariable input(DNN_IMG_W*DNN_IMG_H);


//serch slim
bool autoSerch          = false;
bool waitInferrence     = true;    //推論を待つ
bool imagePost         = false;
bool detectedSLIM      = false;
bool doInferrence      = false;
bool selectedImageOnly = true;
String gStrResult = "?";
String maxLabel   = "?";
int targetArea    = 0;
int maxIndex      = 24;
float maxOutput   = 0.0;


bool doLockWheels = false;
bool doUnLockWheels = false;

//推論時間計測用
unsigned long startMicros; // 処理開始時間を記録する変数
unsigned long endMicros;   // 処理終了時間を記録する変数
unsigned long elapsedTime;    // 処理時間を記録する変数

//カメライメージ操作用
volatile bool newImageAvailable = false;  // 新しい画像があるかどうかのフラグ
String messageStr = "";

//フォトリフレクタの値を格納すする構造体
struct PhotoReflectorState {
    bool left;
    bool right;
};

//static uint8_t const label[2]= {0,1};
static String const label[25]= {"Back_R0",    "Back_R1",    "Back_R2",    "Back_R3", 
                                "Bottom_R0",  "Bottom_R1",  "Bottom_R2",  "Bottom_R3", 
                                "Front_R0",   "Front_R1",   "Front_R2",   "Front_R3",
                                "Left_R0",    "Left_R1",    "Left_R2",    "Left_R3",
                                "Right_R0",   "Right_R1",   "Right_R2",   "Right_R3",
                                "Top_R0",     "Top_R1",     "Top_R2",     "Top_R3",
                                "empty"};


//画像クロップ領域指定
struct ClipRect {
    int x;
    int y;
    int width;
    int height;
};

struct ClipRectSet {
    ClipRect clips[17];  // 5つのクロップ領域を格納する配列
};

ClipRectSet clipSet = {
    {
        {  0,   0, 224, 224}, //  0:large 1
        { 96,   0, 224, 224}, //  1:large 2
        {  0,   0, 112, 112}, //  2:small 1
        {  0,  56, 112, 112}, //  3:small 2
        {  0, 112, 112, 112}, //  4:small 3
        { 56,   0, 112, 112}, //  5:small 4
        { 56,  56, 112, 112}, //  6:small 5
        { 56, 112, 112, 112}, //  7:small 6
        {112,   0, 112, 112}, //  8:small 7
        {112,  56, 112, 112}, //  9:small 8
        {112, 112, 112, 112}, // 10:small 9
        {168,   0, 112, 112}, // 11:small 10
        {168,  56, 112, 112}, // 12:small 11
        {168, 112, 112, 112}, // 13:small 12
        {208,   0, 112, 112}, // 14:small 13
        {208,  56, 112, 112}, // 15:small 14
        {208, 112, 112, 112}, // 16:small 15

    }
};

void checkDrive(){

  if (driveTest){
    Serial.println("==== drive test start");
    int counter = 1;
    while(counter > 0){
      Serial.print("counter = ");
      Serial.println(counter);
      Serial.println("PWM:255  ENB:HIGH");
      analogWrite(IN1_LEFT,255);
      analogWrite(IN1_RIGHT,255);
      digitalWrite(IN2_LEFT,HIGH);
      digitalWrite(IN2_RIGHT,HIGH);
      delay(1000);
      Serial.println("PWM:128  ENB:HIGH");
      analogWrite(IN1_LEFT,128);
      analogWrite(IN1_RIGHT,128);
      digitalWrite(IN2_LEFT,HIGH);
      digitalWrite(IN2_RIGHT,HIGH);
      delay(1000);
      Serial.println("PWM: 64  ENB:HIGH");
      analogWrite(IN1_LEFT,64);
      analogWrite(IN1_RIGHT,64);
      digitalWrite(IN2_LEFT,HIGH);
      digitalWrite(IN2_RIGHT,HIGH);
      delay(1000);
      Serial.println("PWM:  0  ENB:HIGH");
      analogWrite(IN1_LEFT,32);
      analogWrite(IN1_RIGHT,32);
      digitalWrite(IN2_LEFT,HIGH);
      digitalWrite(IN2_RIGHT,HIGH);
      delay(1000);
      Serial.println("PWM:255  ENB:LOW");
      analogWrite(IN1_LEFT,255);
      analogWrite(IN1_RIGHT,255);
      digitalWrite(IN2_LEFT,LOW);
      digitalWrite(IN2_RIGHT,LOW);
      delay(1000);
      Serial.println("PWM:128  ENB:LOW");
      analogWrite(IN1_LEFT,128);
      analogWrite(IN1_RIGHT,128);
      digitalWrite(IN2_LEFT,LOW);
      digitalWrite(IN2_RIGHT,LOW);
      delay(1000);
      Serial.println("PWM: 64  ENB:LOW");
      analogWrite(IN1_LEFT,64);
      analogWrite(IN1_RIGHT,64);
      digitalWrite(IN2_LEFT,LOW);
      digitalWrite(IN2_RIGHT,LOW);
      delay(1000);
      Serial.println("PWM:  0  ENB:LOW");
      analogWrite(IN1_LEFT,32);
      analogWrite(IN1_RIGHT,32);
      digitalWrite(IN2_LEFT,LOW);
      digitalWrite(IN2_RIGHT,LOW);
      delay(1000);
      counter--;
    }
    Serial.println("==== drive test is finished.\n");
  } else {
    Serial.println("==== drive test was skipped.\n");
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

void motor_handler(int left_speed, int right_speed){
  char buffer[30];  // 十分なサイズのバッファを用意
  snprintf(buffer, sizeof(buffer), "SET left: %3d, Right: %3d", left_speed, right_speed);
  Serial.println(buffer);
  if (left_speed == 0){
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, LOW);
  } else if (left_speed < 0) {
    //analogWrite(IN1_LEFT,   map(abs(left_speed),  0, 100, 0, 255));
    digitalWrite(IN2_LEFT, LOW);
  } else if (left_speed > 0) {
    //analogWrite(IN2_LEFT,   map(abs(left_speed),  0, 100, 0, 255));
    digitalWrite(IN1_LEFT, LOW);
  }

  if (right_speed == 0){
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, LOW);
  } else if (right_speed > 0) {
    //analogWrite(IN1_RIGHT,   map(abs(right_speed),  0, 100, 0, 255));
    digitalWrite(IN2_RIGHT, LOW);
  } else if (right_speed < 0) {
    //analogWrite(IN2_RIGHT,   map(abs(right_speed),  0, 100, 0, 255));
    digitalWrite(IN1_RIGHT, LOW);
  }
}

void lockWheels() {
  Serial.println("lock wheels");
  motor_handler(-75, -75);
  delay(400);
  motor_handler(0, 0);
}

void unLockWheels(){
  Serial.println("unlock wheels");
  motor_handler(  75,   75);
  delay(400);
  motor_handler(   0,    0);
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
  Serial.println("call takePicture()");
  CamImage img = theCamera.takePicture();

  if (img.isAvailable()) {
    uploadImage(img);
  } else {
    Serial.println("Failed to take picture");
  }
  //theCamera.end();
}

/* wifi Setup */
void GS2200wifiSetup(){
  /* initialize digital pin LED_BUILTIN as an output. */
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)

  // Wi-Fi setup =====================================================
	/* iS110BのRevに合わせて初期化*/
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
  strncpy(mqtt.params.topic, MQTT_TOPIC , sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;
  if (true == theMqttGs2200.subscribe(&mqtt)) {
    ConsolePrintf( "Subscribed! \n" );
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
    if (data == "doInferrence"){
      Serial.println("command do inferrence");
      //startInferrence();
    }

    else if (data == "sendImage"){
      Serial.println("command send Image");
      camImagePost();
    }

    else if (data == "lockWheels"){
      Serial.println("command lock wheels");
      doLockWheels = true;
      Serial.println("command locked");
    }

    else if (data == "unLockWheels"){
      Serial.println("command unlock wheels");
      doLockWheels = true;
      Serial.println("command unlocked");
    }

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
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);


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

  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H,         //320
    CAM_IMGSIZE_QVGA_V,         //240
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  digitalWrite(LED0, HIGH);  // turn on LED

  //move_nnbFile();
  //camImagePost();
  checkAnalogRead();
  //checkDrive();
}




void loop() {
  delay(FIRST_INTERVAL); /* wait for predefined seconds to take still picture. */
  Serial.println("loop");
  checkMQTTtopic();
  camImagePost();

}
