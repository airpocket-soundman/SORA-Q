/*
✔ Ver.1.0.0 MQTTサブスクライブ関数化
✔ Ver.1.0.1 MQTTでストリーミングをスイッチンぐしつつCamCBで撮影してpost txtもpost
✔ Ver.1.0.2 MQTTにsendImageコマンド追加dnnモデル保存を追加
☑ Ver.1.0.3 autoSerchモード実装 streamONボタンでstreamワンショットできたきがするので整理が必要

*/
char version[] = "Ver.1.0.2";

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


//SD
SDClass theSD;
char flashPath[] = "data/slim.nnb";
char flashFolder[] = "data/";
char nnbFile[] = "model.nnb";


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


//推論時間計測用
unsigned long startMicros; // 処理開始時間を記録する変数
unsigned long endMicros;   // 処理終了時間を記録する変数
unsigned long elapsedTime;    // 処理時間を記録する変数

//カメライメージ操作用
volatile bool newImageAvailable = false;  // 新しい画像があるかどうかのフラグ
CamImage globalImg;


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
  //ストリーミングを停止
  err = theCamera.startStreaming(false, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
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
      if (autoSerch){
        doInferrence = true;
      }

      if (newImageAvailable) {
        Serial.println("CamCB captured");

        newImageAvailable = false;
        err = theCamera.startStreaming(false, CamCB);
        if (err != CAM_ERR_SUCCESS) {
          Serial.println("Failed to stop streaming.");
          return;
        }
      }

			if (msDelta(start) < SUBSCRIBE_TIMEOUT) {
				String data;
				/* just in case something from GS2200 */
				while (gs2200.available()) {
          Serial.println("mqtt loop");
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


          } else if (data == "streamON"){
            Serial.println("command camera on");
            err = theCamera.startStreaming(true, CamCB);
            if (err != CAM_ERR_SUCCESS) {
              printError(err);
            }

          } else if (data == "sendImage"){
            Serial.println("command send Image");
            camImagePost();

          } else if (data == "doInferrence"){
            Serial.println("command do Inferrence");
            doInferrence = true;
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

// MQTTメッセージ送信
void sendMqttMessage(const char* label, float probability, int targetArea, int maxIndex) {
  String messageStr;
  if (maxIndex == 24){
    messageStr = "{\"result\":\"" + String("not detected.") + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);
  }
  else {
    messageStr = "{\"result\":\"" + String("Detected SLIM!!") + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);
    messageStr = "{\"area\":\"" + String(targetArea) + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);
    messageStr = "{\"label\":\"" + String(label) + "\"}";
    uploadString(messageStr);
    Serial.println(messageStr);
    messageStr = "{\"probability\":" + String(probability, 6) + "}";
    uploadString(messageStr);
    Serial.println(messageStr);
  }
}


void CamCB(CamImage img) {
    Serial.println("->CamCB Call back");
    Serial.println(doInferrence);
    if (!doInferrence) {
        return;
    }

    // グローバル変数に画像を保存
    //globalImg = img;
    newImageAvailable = true;  // フラグを立てる
}

void CamCB2(CamImage img){
  String messageStr;
  Serial.println("->CamCB Call back");
    if (!doInferrence) {
    return;
  }

  err = theCamera.startStreaming(false, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    Serial.println("Failed to stop streaming.");
    return;
  }
  Serial.println("delay");
  delay(2000);
  waitInferrence = true;

  messageStr = "{\"status\":\"" + String("Taking photos.") + "\"}";
  //Serial.println("post = " + String(uploadString(messageStr)));
  uploadString("Taking a picture");
  Serial.println(messageStr);
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
  //uploadString(messageStr);
  Serial.println(messageStr);

 
  //snprintf(mqtt.params.message, sizeof(mqtt.params.message), "{\"status\":\"%s\"}", "Inference");
  //printf("Sending JSON: %s\n", mqtt.params.message);
  //mqtt.params.len = strlen(mqtt.params.message);
  //theMqttGs2200.publish(&mqtt);
  for (int i = 0; i < 17; i++) {
    delay(1000);


    preprocessImage(img, input, clipSet.clips[i]);
  
    startMicros = millis();
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    //Serial.print("dnnrt output Size:");
    int size = dnnrt.outputSize(0);
    //Serial.println(size);

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


  sendMqttMessage(maxLabel.c_str(), maxOutput, targetArea, maxIndex);

  if (selectedImageOnly){
    if (maxIndex != 24){
      imagePost = true;
    }
  }
  else {
    imagePost = false;
  }
  
  doInferrence = false;
  waitInferrence = false;
  delay(2000);
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

void NNBModelSetup(){
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
  //SDカード接続時はnnbFile更新
  move_nnbFile();

  File nnbfile = Flash.open(flashPath);
  //File nnbfile = theSD.open("model.nnb");  //microSDからnnbファイルを読み取る場合ｆ
}

// the setup function runs once when you press reset or power the board
void setup() {
	MQTTGS2200_HostParams mqttHostParams;
	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite( LED0, LOW );   // turn the LED off (LOW is the voltage level)
	Serial.begin( CONSOLE_BAUDRATE ); // talk to PC
  Serial.println(version);

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
  /* カメラ画像サイズ設定 */
  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QVGA_H,         //320
    CAM_IMGSIZE_QVGA_V,         //240
    CAM_IMAGE_PIX_FMT_JPG
    );
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  //camera streaming開始
  //err = theCamera.startStreaming(true, CamCB);
  //if (err != CAM_ERR_SUCCESS) {
  //  printError(err);
  //}

  // NNB model setup
  NNBModelSetup();
  File nnbfile = Flash.open(flashPath);
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
    return;
  }
}

// the loop function runs over and over again forever
void loop() {
  Serial.println("loop");
  subscribeMQTT();
}
