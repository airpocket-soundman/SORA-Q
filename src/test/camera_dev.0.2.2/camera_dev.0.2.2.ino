/*
 * Ver.0.2.1 camera rotation 1 -> 3
 *
 *
 *
 *
 *
 */


#include <DNNRT.h>
#include <SDHCI.h>
#include <stdio.h>  /* for sprintf */

#include <Camera.h>

#define BAUDRATE                (115200)
#define TOTAL_PICTURE_COUNT     (1)

SDClass  theSD;
int take_picture_count = 0;
int value=0;
bool CamReady=true;
bool CamDone=false;
const int DIN_PIN = 7;


//TFT設定
#include <SPI.h>
#include "Adafruit_ILI9341.h"

/* LCD Settings */
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST); 


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

char nnbFile[] = "airpocket_newlogo.jpg";
char flashPath[] = "data/slim.nnb";
char flashFolder[] = "data/";
//char nnbFile[] = "slim.nnb";




void putStringOnLcd(String str, int color) {
  int len = str.length();
  tft.fillRect(0,224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160
  - len/2*12;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

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

void preprocessImage(CamImage& img, DNNVariable& input, const ClipRect& clip) {
  // 画像をクロップしてリサイズ

  Serial.print("Clip Rect: ");
  Serial.print("x: "); Serial.print(CAM_CLIP_X);
  Serial.print(", y: "); Serial.print(CAM_CLIP_Y);
  Serial.print(", width: "); Serial.print(CAM_CLIP_W);
  Serial.print(", height: "); Serial.println(CAM_CLIP_H);

  if (CAM_CLIP_X + CAM_CLIP_W > CAM_IMG_W || CAM_CLIP_Y + CAM_CLIP_H > CAM_IMG_H) {
      Serial.println("Error: Clip region exceeds image boundaries.");
  }

  Serial.println("======image info =========== @ before clipAndResizeImageByHW");
  printPixInfomation(img);
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, 
                                           clip.x, clip.y, 
                                           clip.x + clip.width - 1, 
                                           clip.y + clip.height - 1, 
                                           DNN_IMG_W, DNN_IMG_H);

  Serial.print("Clip and Resize failed with error code: ");
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  if (!small.isAvailable()) {
    Serial.println("Error: Clip and Resize failed.");
    return;
  }

  // 画像フォーマットの変換
  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
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

/**
 * Callback from Camera library when video frame is captured.
 */

void CamCB(CamImage img)
{

  /* Check the img instance is available or not. */

  Serial.println("check pix format++++++++++++++++++ @CamCB");
  printPixInfomation(img);

  preprocessImage(img, input, clipSet.clips[0]);

  if (img.isAvailable()){

    /* If you want RGB565 data, convert image data format to RGB565 */

    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);

    Serial.print("Image data size = ");
    Serial.print(img.getImgSize(), DEC);
    Serial.print(" , ");

    Serial.print("buff addr = ");
    Serial.print((unsigned long)img.getImgBuff(), HEX);
    Serial.println("");

  uint16_t* imgBuf = (uint16_t*)img.getImgBuff(); 

  String gStrResult = "Ready";
  if(CamDone){
    gStrResult = "PIC=OK";
    CamDone=false;
  }
  //TFT描画
  tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 224); 
  putStringOnLcd(gStrResult, ILI9341_YELLOW);
  }
  else{
    Serial.println("Failed to get video stream image");
  }
}

/**
 * @brief Initialize camera
 */
void setup()
{
  //SW用
  pinMode( DIN_PIN, INPUT_PULLUP );

  //TFT
  tft.begin();
  tft.setRotation(3);
  
  CamErr err;

  /* Open serial communications and wait for port to open */

  Serial.begin(BAUDRATE);
  while (!Serial)
    {
      ; /* wait for serial port to connect. Needed for native USB port only */
    }

  /* Initialize SD */
  while (!theSD.begin()) 
    {
      Serial.println("Insert SD card.");
    }

  /* begin() without parameters means that
   * number of buffers = 1, 30FPS, QVGA, YUV 4:2:2 format */

  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }

  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS){
    printError(err);
  }

  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS){
    printError(err);
  }
 
  Serial.println("Set still picture format");
  err = theCamera.setStillPictureImageFormat(
     CAM_IMGSIZE_QVGA_H,
     CAM_IMGSIZE_QVGA_V,
     CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS){
    printError(err);
  }
  theCamera.startStreaming(true, CamCB);
}

void CheckSW(){

  value = digitalRead( DIN_PIN );
  //Serial.println( value );
  if(value == 0){
    CamReady=true;
    CamDone=true;
  }
}

void loop()
{
  sleep(0.5);
  CheckSW();

  if(CamReady)
  {
  
      Serial.println("call takePicture()");
      CamImage img = theCamera.takePicture();
      Serial.println("check pix format++++++++++++++++++ @take picture:");
      int pixFormat = img.getPixFormat();
      printPixInfomation(img);

      if (img.isAvailable())
        {    
          char filename[16] = {0};
          sprintf(filename, "PICT_%03d.JPG", take_picture_count);
    
          Serial.print("Save taken picture as ");
          Serial.print(filename);
          Serial.println("");

          theSD.remove(filename);
          File myFile = theSD.open(filename, FILE_WRITE);
          myFile.write(img.getImgBuff(), img.getImgSize());
          myFile.close();

          take_picture_count++;
        }
      else
        {
          Serial.println("Failed to take picture");
        }
        CamReady=false;

    }
}