/* SORA-Q projeckt                                  
✔ ver.0.1.0 SDがある場合はFlashにファイルをコピーする
✔ ver.0.1.1 SDが無い、NNBファイルが無い時の処理追加。フラッシュ保存先を変更したり処理が遅いのを調べたり<readbyteが遅い。起動時バージョン表示を追加
✔ ver.0.1.2 D18,19.20,21のDout、A2,3のAin　テスト追加
✔ Ver.0.2.0 MQTT受信追加
✔ Ver.0.3.0 motor_handler追加
✔ Ver.0.3.1 モータードライバmode 0化
✔ Ver.0.3.2 開閉ロック制御関数追加
✔           MQTTによる車輪ロックON/OFF機能
✔ Ver.0.3.4 電源ONからタイマーによる車輪ロックOFF及び自動走行開始
✔ Ver.0.3.5 is110のリビジョン違いの設定変更をconfigに追加
✔ Ver.0.4.0 roop内から定期的に画像送信
✔ Ver.0.4.1 CamCBの中でinferenceする
✔ Ver.0.4.2 inferenceの有無をスイッチングする
✔ Ver.0.4.3 inferenceの結果をMQTTで送りつつ、画像をpostする。オブジェクト検出ロジックのバグ修正
✔ Ver.0.5.0 NNBファイルのフラッシュ書き込み機能およびフラッシュからのNNBファイル読み込み実行
 Ver.0.6.0 ラジコン走行
 Ver.0.6.1 自動走行追加
 */

char version[] = "toragi_Ver.1.0.0";

#include <GS2200Hal.h>
#include <HttpGs2200.h>
#include <MqttGs2200.h>
#include <SDHCI.h>
#include <TelitWiFi.h>
#include <stdio.h> /* for sprintf */
#include <Camera.h>
#include "config.h"
#include <Flash.h>
#include <DNNRT.h>

#define CONSOLE_BAUDRATE    115200
#define TOTAL_PICTURE_COUNT 1
#define PICTURE_INTERVAL    1000
#define FIRST_INTERVAL      3000
#define SUBSCRIBE_TIMEOUT   60000	//ms
#define START_TIMER         10000 //ms

#define AIN1        A2       //左車輪エンコーダ
#define AIN2        A3       //右車輪エンコーダ
#define IN1_LEFT    19       //左車輪出力
#define IN1_RIGHT   21       //右車輪出力
#define IN2_LEFT    18       //左車輪方向
#define IN2_RIGHT   20       //右車輪方向
#define PHOTO_REFLECTOR_THRETHOLD_LEFT  100
#define PHOTO_REFLECTOR_THRETHOLD_RIGHT 100

// イメージ設定
#define DNN_IMG_W 28
#define DNN_IMG_H 28
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 96//96
#define CAM_CLIP_Y 0//64
#define CAM_CLIP_W 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define CAM_CLIP_H 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)

#define LINE_THICKNESS 5

#define RGB565(r, g, b) (((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F))


float threshold = 0.2;

DNNRT dnnrt;
DNNVariable input(DNN_IMG_W*DNN_IMG_H);
CamErr err;

String gStrResult = "?";
String maxLabel   = "?";
int targetArea    = 0;
int maxIndex      = 24;
float maxOutput   = 0.0;

//推論時間計測用
unsigned long startMicros; // 処理開始時間を記録する変数
unsigned long endMicros;   // 処理終了時間を記録する変数
unsigned long elapsedTime;    // 処理時間を記録する変数

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

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };
bool nnb_copy = true;

//char nnbFile[] = "airpocket_newlogo.jpg";
char flashPath[] = "data/slim.nnb";
char flashFolder[] = "data/";
char nnbFile[] = "model.nnb";
bool doInference   = false;



// auto start on/off
bool autoStart      = false;

// test mode on/off
bool nnbFilePost       = false;    //NNBファイルをhttp postしてチェック
bool analogReadTest    = false;
bool driveTest         = false;
bool selectedImageOnly = true;
bool imagePost         = false;
bool detectedSLIM      = false;
bool autoSerch         = true;    //SLIM探索を行う
bool waitInference    = true;    //推論を待つ

// out put mode on/off
bool photo_reflector_out    = false;
bool photo_reflector_left   = false;
bool photo_reflector_right  = false;

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
String messageStr;


void listFiles(File dir) {
  if (!dir || !dir.isDirectory()) {
    Serial.println("Failed to open directory");
    return;
  }

  Serial.println("Listing files:");
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break; // ファイルがなくなったら終了
    }

    if (entry.isDirectory()) {
      Serial.print("DIR : ");
      Serial.println(entry.name());
    } else {
      Serial.print("FILE: ");
      Serial.print(entry.name());
      Serial.print("  SIZE: ");
      Serial.println(entry.size());
    }
    entry.close();
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
    analogWrite(IN1_LEFT,   map(abs(left_speed),  0, 100, 0, 255));
    digitalWrite(IN2_LEFT, LOW);
  } else if (left_speed > 0) {
    analogWrite(IN2_LEFT,   map(abs(left_speed),  0, 100, 0, 255));
    digitalWrite(IN1_LEFT, LOW);
  }

  if (right_speed == 0){
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, LOW);
  } else if (left_speed > 0) {
    analogWrite(IN1_RIGHT,   map(abs(right_speed),  0, 100, 0, 255));
    digitalWrite(IN2_RIGHT, LOW);
  } else if (left_speed < 0) {
    analogWrite(IN2_RIGHT,   map(abs(right_speed),  0, 100, 0, 255));
    digitalWrite(IN1_RIGHT, LOW);
  }
}

void read_photo_reflector(){
  photo_reflector_left  = (analogRead(A2) >= PHOTO_REFLECTOR_THRETHOLD_LEFT);
  photo_reflector_right = (analogRead(A3) >= PHOTO_REFLECTOR_THRETHOLD_RIGHT);
  //Serial.println("read photo reflector");
  if (photo_reflector_out){
    char buffer[50];
    sprintf(buffer, "photo reflector value  LEFT = %d / RIGHT = %d", photo_reflector_left, photo_reflector_right);
    Serial.println(buffer);
  }
}

void lockWheels(){
  Serial.println("lock wheels");
  motor_handler( -75,  -75);
  delay(300);
  motor_handler(   0,    0);
}

void unlockWheels(){
  Serial.println("unlock wheels");
  motor_handler(  75,   75);
  delay(300);
  motor_handler(   0,    0);
}

void checkDrive(){

  if (driveTest){
    Serial.println("==== drive test start");
    int counter = 10;
    while(counter > 0){
      Serial.print("counter = ");
      Serial.println(counter);
      Serial.println("PWM:255  ENB:HIGH");
      motor_handler( 100,  100);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      Serial.println("PWM:128  ENB:HIGH");
      motor_handler(  75,   75);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      Serial.println("PWM: 64  ENB:HIGH");
      motor_handler(  0,   0);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      Serial.println("PWM:255  ENB:LOW");
      motor_handler(-100, -100);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      Serial.println("PWM:128  ENB:LOW");
      motor_handler( -75,  -75);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      Serial.println("PWM: 64  ENB:LOW");
      motor_handler( -0,  -0);
      for(int i = 0; i < 100; i++){
        delay(100);
        read_photo_reflector();
      }
      counter--;
    }
    Serial.println("==== drive test is finished.\n");
  } else {
    Serial.println("==== drive test was skipped.\n");
  }
}

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

    Serial.println("===========================================Recieve data: " + data);
    
    // dataをパース
    splitString(data, param1, param2, param3, param4, param5, param6);
    
    Serial.println("param 1: " + param1);
    Serial.println("param 2: " + param2);
    Serial.println("param 3: " + param3);
    Serial.println("param 4: " + param4);
    Serial.println("param 5: " + param5);
    Serial.println("param 6: " + param6);
    // param1の値によってlockwheelsまたはunlockWheelsを実行
    if (param1 == "lockWheels") {
      lockWheels();
    } else if (param1 == "unlockWheels") {
      unlockWheels();
    } else if (param1 == "autoON"){
      autoSerch = true;
    } else if (param1 == "autoOFF"){
      autoSerch = false;
    }
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

//microSDに model.nnb が存在するとき、推論用モデルをflashメモリにコピーする
void move_nnbFile() {
  Serial.println("\n////////////////// nnb file /////////////////");

  if (nnb_copy) {
    File readFile = theSD.open(nnbFile, FILE_READ);
    uint32_t file_size = readFile.size();
    Serial.print("SD data file size = ");
    Serial.println(file_size);
    char *body = (char *)malloc(file_size);

    if (body == NULL) {
      Serial.println("メモリ確保に失敗しました");
      return;
    }

    int index = 0;
    while (readFile.available()) {
      body[index++] = readFile.read();
    }
    readFile.close();

    Flash.mkdir(flashFolder);

    if (Flash.exists(flashPath)) {
      Flash.remove(flashPath);  // ファイルを削除
    }

    File writeFile = Flash.open(flashPath, FILE_WRITE);
    if (writeFile) {
      Serial.println("nnb fileをフラッシュメモリに書き込み中...");

      size_t written = writeFile.write((uint8_t*)body, file_size);
      if (written != file_size) {
        Serial.println("ファイルの書き込みに失敗しました");
      } else {
        Serial.println("nnb fileをフラッシュメモリへ書き込み完了");
      }

      writeFile.close();

      // フラッシュメモリに保存されたファイルのサイズを表示
      File flashFile = Flash.open(flashPath, FILE_READ);
      if (flashFile) {
        uint32_t flashFileSize = flashFile.size();
        Serial.print("フラッシュメモリのファイルサイズ = ");
        Serial.println(flashFileSize);
        flashFile.close();
      } else {
        Serial.println("フラッシュメモリのファイルオープンに失敗しました");
      }
    } else {
      Serial.println("フラッシュメモリのファイルオープンに失敗しました");
    }

    free(body);
  } else {
    Serial.println("move nnb file passed.\n");
  }

Serial.println("\n////////////////// nnb file end //////////////////////");

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
  //Serial.println("Size");
  //Serial.println(size_string);

  bool result = false;
  result = theHttpGs2200.connect();
  WiFi_InitESCBuffer();
  result = theHttpGs2200.send(HTTP_METHOD_POST, 10, url_path, body, size);
  return result;

}

bool uploadString(const String &body) {
  // 送信するデータを格納するバッファ
  char sendData[100];
  body.toCharArray(sendData, sizeof(sendData));

  size_t size = strlen(sendData);  // 送信する文字列の長さを取得

  // custom_post関数を呼び出してPOSTリクエストを送信
  bool result = custom_post(HTTP_POST_TEXT_PATH, (const uint8_t*)sendData, size);
  if (!result) {
    Serial.println("Post Failed");
    return false;
  }

  // 受信処理
  result = false;
  String responseStr = "";  // 受信データをStringに格納する変数
  do {
    result = theHttpGs2200.receive(5000);  // タイムアウトは5000ms
    Serial.print("receive() returned: ");
    Serial.println(result ? "true" : "false");

    if (result) {
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);

      // 受信データを String に変換
      responseStr = String((char*)Receive_Data);

      // 💡 受信データのデバッグ表示
      Serial.println("Raw Received Data:");
      Serial.println(responseStr);

      // 💡 決め打ちで最初の n 文字を削除 (例えば 13 文字)
      int fixedOffset = 8;  // 削除する文字数（適宜調整）
      if (responseStr.length() > fixedOffset) {
          responseStr = responseStr.substring(fixedOffset);  // 先頭の fixedOffset 文字を削除
      } else {
          Serial.println("Warning: Response too short to trim");
      }

      // 💡 余分な改行やスペースを除去
      responseStr.trim();

      Serial.println("Processed Response: '" + responseStr + "'");

      // 💡 "lockWheel" だけを比較
      if (responseStr == "lockWheel") {
        Serial.println("✅ do lock Wheel");
        lockWheels();
      } else {
        Serial.println("❌ Comparison failed: '" + responseStr + "'");
      }
    } else {
      Serial.println("\r\nNo response from server");
    }
  } while (result);

  // 終了処理
  theHttpGs2200.end();
  return true;
}

void uploadImage(uint16_t* imgBuffer, size_t imageSize) {
 
  Serial.print("imgBuffer is available: ");
  Serial.println(imgBuffer != nullptr);  // imgBufferがnullでないか確認
  
  Serial.print("Image size: ");
  Serial.println(imageSize);

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
    } else {
      // AT+HTTPSEND command is done
      Serial.println("\r\n");
    }
  } while (result);

  result = theHttpGs2200.end();
}

/* カメラ撮影とhttp request postのテスト*/
void camImagePost(){

  messageStr = "{\"status\":\"" + String("sending an image") + "\"}";
  uploadString(messageStr);

  Serial.println("==== start Camera Image Post Test");
  int take_picture_count = 0;
  while (take_picture_count < TOTAL_PICTURE_COUNT) {
    Serial.println("call takePicture()");
    CamImage img = theCamera.takePicture();
    Serial.println("======image info =========== @ cam Image post");
    //printPixInfomation(img);

    if (img.isAvailable()) {
      uint16_t* imgBuffer = (uint16_t*)img.getImgBuff();
      size_t imageSize = img.getImgSize();
      uploadImage(imgBuffer, imageSize);
    } else {
      Serial.println("Failed to take picture");
    }
  take_picture_count++;
  }

  Serial.println("==== cam Image Post test is finished.\n");

  imagePost = false;
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


void preprocessImage(CamImage& img, DNNVariable& input, const ClipRect& clip) {
  // 画像をクロップしてリサイズ

  /*
  Serial.println("\n==preprocessImage ===============");

  Serial.print("Clip Rect: ");
  Serial.print("x: "); Serial.print(CAM_CLIP_X);
  Serial.print(", y: "); Serial.print(CAM_CLIP_Y);
  Serial.print(", width: "); Serial.print(CAM_CLIP_W);
  Serial.print(", height: "); Serial.println(CAM_CLIP_H);
  */

  if (CAM_CLIP_X + CAM_CLIP_W > CAM_IMG_W || CAM_CLIP_Y + CAM_CLIP_H > CAM_IMG_H) {
      Serial.println("Error: Clip region exceeds image boundaries.");
  }

  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, 
                                           clip.x, clip.y, 
                                           clip.x + clip.width - 1, 
                                           clip.y + clip.height - 1, 
                                           DNN_IMG_W, DNN_IMG_H);

  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  if (!small.isAvailable()) {
    Serial.println("Error: Clip and Resize failed.");
    return;
  }
  //Serial.println("small infmation=====");
  //printPixInfomation(small);

  // 画像フォーマットの変換
  //Serial.println("comvert SMALL");
  err = small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  //Serial.println(err);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
  //Serial.println("small infmation @after convert to RGB565=====");
  //printPixInfomation(small);

  uint16_t* tmp = (uint16_t*)small.getImgBuff();

  // DNNに入力する輝度データの計算
  float* dnnbuf = input.data();
  float f_max = 0.0;
  for (int n = 0; n < DNN_IMG_H * DNN_IMG_W; ++n) {
    uint16_t pixel = tmp[n];
    
    // RGB成分を抽出
    float red   = (float)((pixel & 0xF800) >> 11) * (255.0 / 31.0); // 5ビット赤
    float green = (float)((pixel & 0x07E0) >> 5) * (255.0 / 63.0);  // 6ビット緑
    float blue  = (float)(pixel & 0x001F) * (255.0 / 31.0);         // 5ビット青
    
    // 輝度を計算
    dnnbuf[n] = 0.299 * red + 0.587 * green + 0.114 * blue;

    // 最大値を記録（正規化に使用）
    if (dnnbuf[n] > f_max) f_max = dnnbuf[n];
  }

  // 正規化処理
  if (f_max == 0) {
    Serial.println("Error: Max value is zero, normalization failed.");
    return;
  }
  
  // 正規化
  for (int n = 0; n < DNN_IMG_W * DNN_IMG_H; ++n) {
    dnnbuf[n] /= f_max;
  }
}

void printPixInfomation(CamImage& img) {
    // ピクセルフォーマットを取得して文字列に変換
    int pixFormat = img.getPixFormat(); // 画像オブジェクトからピクセルフォーマットを取得
    char format[5]; // 4文字 + 終端文字('\0')
    format[0] = pixFormat & 0xFF;        // 最下位バイト
    format[1] = (pixFormat >> 8) & 0xFF;
    format[2] = (pixFormat >> 16) & 0xFF;
    format[3] = (pixFormat >> 24) & 0xFF; // 最上位バイト
    format[4] = '\0';                     // 終端文字

    // 画像幅、高さ、サイズ、バッファサイズを取得
    int width = img.getWidth();
    int height = img.getHeight();
    size_t imgSize = img.getImgSize();
    size_t bufferSize = img.getImgBuffSize();

    // 情報をシリアル出力
    Serial.println("==== Image Info ====");
    Serial.print("Pixel format: ");
    Serial.println(format);

    Serial.print("Image width: ");
    Serial.println(width);

    Serial.print("Image height: ");
    Serial.println(height);

    Serial.print("Image size (bytes): ");
    Serial.println(imgSize);

    Serial.print("Buffer size (bytes): ");
    Serial.println(bufferSize);
    Serial.println("====================");
}

/* wifi Setup */
void GS2200wifiSetup(){
  /* initialize digital pin LED_BUILTIN as an output. */
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)

  /* Initialize SPI access of GS2200 */
  //Init_GS2200_SPI_type(iS110B_TypeA);

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
  strncpy(mqtt.params.topic, MQTT_TOPIC1 , sizeof(mqtt.params.topic));
  mqtt.params.QoS = 0;
  mqtt.params.retain = 0;
  if (true == theMqttGs2200.subscribe(&mqtt)) {
    ConsolePrintf( "Subscribed! \n" );
  }
}

// 推論結果送信
void sendResult(const char* label, float probability, int targetArea, int maxIndex) {
  String messageStr;
  if (maxIndex == 24){
    messageStr = "{\"result\":\"" + String("not detected.") + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);
  }
  else {
    String messageStr = "{";
    messageStr += "\"result\":\"Detected SLIM!!\",";
    messageStr += "\"area\":\"" + String(targetArea) + "\",";
    messageStr += "\"label\":\"" + String(label) + "\",";
    messageStr += "\"probability\":" + String(probability, 6);
    messageStr += "}";

    uploadString(messageStr);
    Serial.println(messageStr);
  }
}

void CamCB(CamImage img){
  //Serial.println("->CamCB Call back");

  if (!doInference) {
    return;
  }
  
  waitInference = true;

  messageStr = "{\"status\":\"" + String("Take a picture") + "\"}";
  uploadString(messageStr);


  Serial.println("\nCamCB Start ++++++++++++++++++ <<@CamCB>>");
  //printPixInfomation(img);

  //変数初期化
  gStrResult = "?";
  maxLabel   = "?";
  targetArea = 0;
  maxIndex   = 24;
  maxOutput  = 0.0;

  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  messageStr = "{\"status\":\"" + String("Inference") + "\"}";
  uploadString(messageStr);
  for (int i = 0; i < 17; i++) {
    delay(1000);


    preprocessImage(img, input, clipSet.clips[i]);
  
    startMicros = millis();
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    int size = dnnrt.outputSize(0);

    DNNVariable output = dnnrt.outputVariable(0);

    endMicros = millis();
    elapsedTime = endMicros - startMicros;


    //Serial.print("elapsed time: ");
    //Serial.print(elapsedTime);
    //Serial.println(" ms");

    // text描画
    int index = output.maxIndex();

    if (i == 0){
      targetArea = i;
      maxIndex   = index;
      maxOutput  = output[index];
      maxLabel   = String(label[index]);
    } 
    else{
      if (maxIndex == 24){
        if (index != 24){
          targetArea = i;
          maxIndex   = index;
          maxOutput  = output[index];
          maxLabel   = String(label[index]);
        }
      }
      else {
        if (output[index] > maxOutput && index != 24){
          targetArea = i;
          maxIndex   = index;
          maxOutput  = output[index];
          maxLabel   = String(label[index]);
        }
      }
    }

    if (output[index] >= threshold) {
      gStrResult = "index:" + String(index) + " " + String(label[index]) + ":" + String(output[index]) + " / targetArea:" + String(targetArea) +" / maxIndex:" + String(maxIndex) + " / maxOutput:" + String(maxOutput);
    } else {
      gStrResult = "not identify";
    }
    
    Serial.print("index:");
    Serial.print(index);
    Serial.print("  /  ");
    Serial.print("-->area:");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(gStrResult);
  }
  

  Serial.println("\n****total score======");
  Serial.print(" targetArea:");
  Serial.println(targetArea);
  Serial.print(" maxIndex  :");
  Serial.println(maxIndex);
  Serial.println(" label     :" + String(maxLabel));
  Serial.println(" prbability:" + String(maxOutput));
  Serial.println("****=================");
  //Serial.println("CamCB finished+++++++++++++++++++++++\n");


  sendResult(maxLabel.c_str(), maxOutput, targetArea, maxIndex);
  if (selectedImageOnly){
    if (maxIndex != 24){
      imagePost = true;
    }
  }
  else {
    imagePost = true;
  }
  doInference = false;
  waitInference = false;
  delay(2000);
}

// Setup Function 
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

  // ディレクトリを開く
  File root = Flash.open("/data");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open directory /data");
    return;
  }

  // ファイル一覧を表示
  listFiles(root);

  //SDカード接続時はnnbFile更新
  move_nnbFile();
  File nnbfile = Flash.open(flashPath);
  //File nnbfile = theSD.open("model.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
    return;
  }

  //Pin initialize
  pinMode(IN2_LEFT,  OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN1_LEFT,    OUTPUT);
  pinMode(IN1_RIGHT,   OUTPUT);


  digitalWrite(LED0, LOW);         // turn the LED off (LOW is the voltage level)

  // GS2200 WiFi setup

  GS2200wifiSetup();

  //Camera Setup

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

  digitalWrite(LED0, HIGH);  // turn on LED
  //uploadNNB();
  checkAnalogRead();
  checkDrive();

  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  delay(3000);
  lockWheels();
  delay(3000);
  unlockWheels();

  messageStr = "{\"status\":\"" + String("SLIM ready.") + "\"}";
  uploadString(messageStr);
  delay(3000);
  doInference = true;

}

void loop() {

  delay(FIRST_INTERVAL); /* wait for predefined seconds to take still picture. */
  Serial.println("");
  Serial.println("loop");
  Serial.println("");
  checkMQTTtopic();

  if (autoSerch){
    if (imagePost == true){
      camImagePost();
    }
    //Serial.println(maxLabel);
    //Serial.println(maxOutput);
   

    if (!detectedSLIM && !waitInference ){

    messageStr = "{\"status\":\"" + String("Moving") + "\"}";
    uploadString(messageStr);
    motor_handler(   25,   50);
    delay(1000);
    motor_handler(   0,    0);
    messageStr = "{\"status\":\"" + String("finished Moving") + "\"}";
    uploadString(messageStr);

    doInference = true;
    }
  }
  //read_photo_reflector();
}
