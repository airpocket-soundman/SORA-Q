char version[] = "toragi_Ver.2.0.0";

#include <GS2200Hal.h>
#include <HttpGs2200.h>
#include <SDHCI.h>
#include <TelitWiFi.h>
#include <stdio.h>
#include <Camera.h>
#include "config.h"
#include <Flash.h>
#include <DNNRT.h>

#define CONSOLE_BAUDRATE    115200  //USBシリアル通信速度
#define LOOP_INTERVAL       1000    //ms　メインループのインターバル

// GPIO設定
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
#define CAM_CLIP_X 96
#define CAM_CLIP_Y 0
#define CAM_CLIP_W 224//DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define CAM_CLIP_H 224//DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define RGB565(r, g, b) (((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F))

//物体検出関連
DNNRT dnnrt;
DNNVariable input(DNN_IMG_W*DNN_IMG_H);
CamErr err;
SDClass theSD;

String gStrResult = "?";
String maxLabel   = "?";
int targetArea    = 0;
int maxIndex      = 24;
float maxOutput   = 0.0;

// 検出するクラスリスト24種類+空クラス
static String const label[25]= {"Back_R0",    "Back_R1",    "Back_R2",    "Back_R3", 
                                "Bottom_R0",  "Bottom_R1",  "Bottom_R2",  "Bottom_R3", 
                                "Front_R0",   "Front_R1",   "Front_R2",   "Front_R3",
                                "Left_R0",    "Left_R1",    "Left_R2",    "Left_R3",
                                "Right_R0",   "Right_R1",   "Right_R2",   "Right_R3",
                                "Top_R0",     "Top_R1",     "Top_R2",     "Top_R3",
                                "empty"
};

//推論用画像クロップ領域格納用
struct ClipRect {
    int x;
    int y;
    int width;
    int height;
};

// 5つのクロップ領域を格納する構造体
struct ClipRectSet {
    ClipRect clips[17];  
};

// 画像内のオブジェクト探索領域
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

//内部処理用フラグ
bool doInference            = false;              // 内部flag trueで推論実行
bool waitInference          = false;              // 内部flag 推論実行中はtrue
bool photo_reflector_left   = false;              // photo_reflectorの値をバイナリ変換した値
bool photo_reflector_right  = false;

//Node-REDから制御可能
bool autoSerch              = false;              // 自動探索する場合はtrue
bool imagePost              = false;              // trueで画像を送信する
bool doLockWheels           = false;              // trueでホイールをロック
bool doUnLockWheels         = false;              // trueでホイールを案魯一句
bool fullAuto               = false;              // trueで全自動開始
bool doSendImage            = false;              // trueで画像送信

//http request関連
TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams httpHostParams; // HTTPサーバ接続用のホストパラメータ
String messageStr;
const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = { 0 };

//モータ操作関数
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

//フォトリフレクタの値を読む
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

//wheelをロックする
void lockWheels(){
  Serial.println("lock wheels");
  motor_handler( -75,  -75);
  delay(400);
  motor_handler(   0,    0);
}

//wheelをアンロックする
void unLockWheels(){
  Serial.println("unlock wheels");
  motor_handler(  75,   75);
  delay(400);
  motor_handler(   0,    0);
}

//microSDに model.nnb が存在するとき、推論用モデルをflashメモリにコピーする
void move_nnbFile() {
  Serial.println("\nmove NNB File.");

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
}

//カメラ制御エラー表示
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

// HTTP post
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

// テキスト情報をHTTP postする
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

    // HTTP リクエストに対するレスポンスの処理
    if (result) {
      theHttpGs2200.read_data(Receive_Data, RECEIVE_PACKET_SIZE);

      // 受信データを String に変換
      responseStr = String((char*)Receive_Data);

      // 受信データのデバッグ表示
      Serial.println("Raw Received Data:");
      Serial.println(responseStr);

      // ステータスラインを削除
      int fixedOffset = 8;  // 削除する文字数
      if (responseStr.length() > fixedOffset) {
          responseStr = responseStr.substring(fixedOffset);  // 先頭の fixedOffset 文字を削除
      } else {
          Serial.println("Warning: Response too short to trim");
      }
      // 余分な改行やスペースを除去
      responseStr.trim();
      Serial.println("Processed Response: '" + responseStr + "'");

      //コマンドに対応したフラグを書き換え
      if (responseStr == "lockWheels") {
        Serial.println("do lock Wheel");
        doLockWheels = true;
      } 
      
      else if (responseStr == "unLockWheels") {
        Serial.println("do unlock Wheel");
        doUnLockWheels = true;
      }

      else if (responseStr == "startAutoSerch") {
        Serial.println("start Auto Serch");
        autoSerch = true;
      }

      else if (responseStr == "stopAutoSerch") {
        Serial.println("stop Auto Serch");
        autoSerch = false;
      }

      else if (responseStr == "fullAuto") {
        Serial.println("full auto mode enable.");
        fullAuto = true;
      }

      else if (responseStr == "sendImage") {
        Serial.println("send Image");
        doSendImage = true;
      }
      
      else {
        Serial.println("Comparison failed: '" + responseStr + "'");
      }
    } else {
      Serial.println("\r\nNo response from server");
    }
  } while (result);

  // 終了処理
  theHttpGs2200.end();
  return true;
}

// イメージをHTTP postする
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

// 画像を撮影してHTTP postする
void camImagePost(){
  messageStr = "{\"status\":\"" + String("sending an image") + "\"}";
  uploadString(messageStr);
  Serial.println(messageStr);

  CamImage img = theCamera.takePicture();

  if (img.isAvailable()) {
    uint16_t* imgBuffer = (uint16_t*)img.getImgBuff();
    size_t imageSize = img.getImgSize();
    uploadImage(imgBuffer, imageSize);
  } else {
    Serial.println("Failed to take picture");
  }
  imagePost = false;
}

// 推論用画像の前処理を行い画像をセットする
void preprocessImage(CamImage& img, DNNVariable& input, const ClipRect& clip) {
  // 画像をクロップしてリサイズ
  if (CAM_CLIP_X + CAM_CLIP_W > CAM_IMG_W || CAM_CLIP_Y + CAM_CLIP_H > CAM_IMG_H) {
      Serial.println("Error: Clip region exceeds image boundaries.");
  }

  //推論用に画像を縮小する。
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

  // 画像フォーマットを変換する
  err = small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

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
    
    // 輝度を計算して推論用バッファに保存
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

//Wifiのセットアップ
void GS2200wifiSetup(){
  //インジケータ用LED消灯
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);

  //is110にはrev A,B,Cがあるためconfig.hに従って設定
  #if defined(iS110_TYPEA)
    Init_GS2200_SPI_type(iS110B_TypeA);
  #elif defined(iS110_TYPEB)
    Init_GS2200_SPI_type(iS110B_TypeB);
  #elif defined(iS110_TYPEC)
    Init_GS2200_SPI_type(iS110B_TypeC);
  #else
    #error "No valid device type defined. Please define iS110_TYPEA, iS110_TYPEB, or iS110_TYPEC."
  #endif

  // AT Command Library Buffer を初期化
  gsparams.mode = ATCMD_MODE_STATION;
  gsparams.psave = ATCMD_PSAVE_DEFAULT;
  if (gs2200.begin(gsparams)) {
    Serial.println("GS2200 Initilization Fails");
    while (1)
      ;
  }

  // アクセスポイントに接続
  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
    Serial.println("Association Fails");
    while (1)
      ;
  }

  //HTTPの設定
  httpHostParams.host = (char *)HTTP_SRVR_IP;
  httpHostParams.port = (char *)HTTP_PORT;
  theHttpGs2200.begin(&httpHostParams);

  Serial.println("Start HTTP Client");
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/octet-stream");


  WiFi_InitESCBuffer();
}

// 推論結果をサーバに送信
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

// Camera streaming のコールバック関数
void CamCB(CamImage img){

  // doInferenceがfalseの場合は終了
  if (!doInference) {
    return;
  }
  
  waitInference = true;   //推論中は他の動作を停止するためのフラグ

  messageStr = "{\"status\":\"" + String("Take a picture") + "\"}";
  uploadString(messageStr);
  Serial.println(messageStr);

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

  // 取得画像を17エリアに分割し、順番に推論を実行
  messageStr = "{\"status\":\"" + String("Inference") + "\"}";
  uploadString(messageStr);
  Serial.println(messageStr);
  for (int i = 0; i < 17; i++) {

    //画像を前処理する
    preprocessImage(img, input, clipSet.clips[i]);
  
    //推論実行
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    int size = dnnrt.outputSize(0);
    DNNVariable output = dnnrt.outputVariable(0);

    int index = output.maxIndex();

    //最も確率の高いエリアをSLIM検出エリアとして抽出
    if (i == 0){
      targetArea = i;
      maxIndex   = index;
      maxOutput  = output[index];
      maxLabel   = String(label[index]);
    } 
    else {
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

    //結果出力
    if (output[index] >= threshold) {
      gStrResult = "index:" + String(index) + " " 
                            + String(label[index]) + ":" 
                            + String(output[index]) 
                            + " / targetArea:" + String(targetArea) 
                            + " / maxIndex:"   + String(maxIndex) 
                            + " / maxOutput:"  + String(maxOutput);
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

  //Node-REDに結果送信
  sendResult(maxLabel.c_str(), maxOutput, targetArea, maxIndex);

  // 画像送信可否の判断
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
}

// Setup
void setup() {
  //USB シリアル通信開始
  Serial.begin(CONSOLE_BAUDRATE);
  while (!Serial) {;}

  Serial.println(version);

  //SDcard初期化
  Serial.println("theSD.begin()");
  if(!theSD.begin()) {
    Serial.println("SD card mount failed");
    nnb_copy = false;
  }

  //内蔵Flashメモリ初期化
  if(!Flash.begin()) {
    Serial.println("Flash card mount failed");
  }

  File root = Flash.open("/data");
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open directory /data");
    return;
  }

  //SDカード接続時はnnbFile更新
  move_nnbFile();

  //NNBファイルを内蔵Flashから読み込む
  File nnbfile = Flash.open(flashPath);
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
    return;
  }

  //wheel制御用GPIOを出力モードで初期化
  pinMode(IN2_LEFT,   OUTPUT);
  pinMode(IN2_RIGHT,  OUTPUT);
  pinMode(IN1_LEFT,   OUTPUT);
  pinMode(IN1_RIGHT,  OUTPUT);

  // GS2200 WiFi セットアップ
  GS2200wifiSetup();

  //Camera Setup
  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  //自動ホワイトバランス
  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  //撮影画像サイズ設定
  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H,         //320
    CAM_IMGSIZE_QVGA_V,         //240
    CAM_IMAGE_PIX_FMT_JPG
    );
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  //カメラストリーミング開始（送信画像用）
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
  messageStr = "{\"status\":\"" + String("SLIM ready.") + "\"}";
  uploadString(messageStr);
  Serial.println(messageStr);
}

void loop() {

  delay(LOOP_INTERVAL); //loop処理のインターバル
  Serial.println("\n================= loop ==================\n");

  // full autoコマンド受信時：実行5秒後にホイールを展開、展開3秒後にsuto serch開始
  if (fullAuto){
    fullAuto = false;
    delay(5000);
    unLockWheels();
    delay(3000);
    autoSerch = true;
  }

  // unlock wheels コマンド受信時
  if (doUnLockWheels){
    unLockWheels();
    doUnLockWheels = false;
  }

  // lock wheel コマンド受信時
  if (doLockWheels){
    lockWheels();
    doLockWheels = false;
  }

  // send Image コマンド受信時
  if (doSendImage){
    camImagePost();
    doSendImage = false;
  }

  // auto Serch コマンド受信時
  if (autoSerch){
    if (imagePost == true){
      camImagePost();
    }

    if (!waitInference ){

    messageStr = "{\"status\":\"" + String("Moving") + "\"}";
    Serial.println(messageStr);
    uploadString(messageStr);

    //探索移動(右車輪出力25,左車輪出力50で1秒走行)
    motor_handler(   25,   50);
    delay(1000);
    motor_handler(   0,    0);
    
    messageStr = "{\"status\":\"" + String("finished Moving") + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);

    doInference = true;
    }
  }
    else {
    messageStr = "{\"status\":\"" + String("waiting") + "\"}";
    Serial.println(messageStr);
    uploadString(messageStr);
  }
 
  //フォトリフレクタの値を読み込む場合は有効化
  //read_photo_reflector();
}
