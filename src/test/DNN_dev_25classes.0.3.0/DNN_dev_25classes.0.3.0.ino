/*
✔ Ver.0.0.0 推論実行
✔ Ver.0.0.1 推論時間計測、表示
✔ Ver.0.0.2 preprocessを関数化、大サイズ、小サイズ推論
✔ Ver.0.1.0 複数エリアをループで推論
✔ Ver.0.1.1 スコア比較して表示
✔ Ver.0.2.0 全箇所推論
 Ver.0.3.0 roop処理化
*/

char version[] = "DNN_dev_25classes Ver.0.1.1";

#include <Camera.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DNNRT.h>
#include "Adafruit_ILI9341.h"

int before_value=0;
bool ChangeModeFlag=false;

#include <SDHCI.h>
SDClass theSD;

/* Audio Setting*/
#include <Audio.h>


/* LCD Settings */
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10

#define DNN_IMG_W 28
#define DNN_IMG_H 28
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 96//96
#define CAM_CLIP_Y 0//64
#define CAM_CLIP_W 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define CAM_CLIP_H 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)

#define LINE_THICKNESS 5

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

float threshold = 0.2;

uint8_t buf[DNN_IMG_W*DNN_IMG_H];

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

void putStringOnLcd(String str, int color) {
  int len = str.length();
  tft.fillRect(0,224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - len/2*12;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

void drawBox(uint16_t* imgBuf, const ClipRect& clip) {
  /* Draw target line */
  for (int x = clip.x; x < clip.x+clip.width; ++x) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*(clip.y+n) + x)              = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*(clip.y+clip.height-1-n) + x) = ILI9341_RED;
    }
  }
  for (int y = clip.y; y < clip.y+clip.height; ++y) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*y + clip.x+n)                = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*y + clip.x + clip.width-1-n) = ILI9341_RED;
    }
  }  
}


void preprocessImage(CamImage& img, DNNVariable& input, const ClipRect& clip) {
  // 画像をクロップしてリサイズ
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, 
                                           clip.x, clip.y, 
                                           clip.x + clip.width - 1, 
                                           clip.y + clip.height - 1, 
                                           DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()) {
    Serial.println("Error: Clip and Resize failed.");
    putStringOnLcd("Clip and Resize Error", ILI9341_RED);
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
    putStringOnLcd("Normalization Error", ILI9341_RED);
    return;
  }
  
  // 正規化
  for (int n = 0; n < DNN_IMG_W * DNN_IMG_H; ++n) {
    dnnbuf[n] /= f_max;
  }

  Serial.println("Preprocessing successful.");
  // 正常終了後、次の処理に進む
}



void CamCB(CamImage img) {
  String gStrResult = "?";
  String maxLabel   = "?";
  int targetArea    = 0;
  int maxIndex      = 24;
  float maxOutput   = 0.0;
  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  
  for (int i = 0; i < 17; i++) {
    Serial.println(i);

    preprocessImage(img, input, clipSet.clips[i]);

    
    startMicros = millis();
    dnnrt.inputVariable(input, 0);
    dnnrt.forward();
    DNNVariable output = dnnrt.outputVariable(0);
    endMicros = millis();
    elapsedTime = endMicros - startMicros;

    Serial.print("Loop time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");

    // text描画
    int index = output.maxIndex();
    Serial.print("index:");
    Serial.println(index);

    for(int j = 0;j < 25; j++){
      Serial.print(j);
      Serial.print(":");
      Serial.println(output[j]);
    }

    if (i == 0){
      targetArea = 0;
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
        if (output[index] > maxOutput){
          targetArea = i;
          maxIndex   = index;
          maxOutput  = output[index];
          maxLabel   = String(label[index]);
        }
      }
    }




    if (output[index] >= threshold) {
      gStrResult = String(label[index]) + String(":") + String(output[index]);
    } else {
      gStrResult = "not identify";
    }
    Serial.println(gStrResult);

  }



  Serial.println("total score======");
  Serial.print("targetArea:");
  Serial.println(targetArea);
  Serial.print("maxIndex:");
  Serial.println(maxIndex);
  Serial.println(String(maxLabel) + String(":") + String(maxOutput));
  Serial.println("=======================");

  gStrResult = String(String(maxLabel) + String(":") + String(maxOutput));
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff(); 

  // Box描画
  if (maxIndex != 24){
    drawBox(imgBuf, clipSet.clips[targetArea]);
  }
  // TFT描画
  tft.drawRGBBitmap(0, 0, imgBuf, 320, 224);

  putStringOnLcd(gStrResult, ILI9341_YELLOW);
 

  // モード変更有無をチェック
  if (index != before_value) {
    ChangeModeFlag = true;
  }

  // 更新
  before_value = index;
  // flag初期化
  ChangeModeFlag = false;
}

void setup() {   
  Serial.begin(115200);
  Serial.println("tft.begin");
  tft.begin();
  tft.setRotation(3);
  Serial.println(version);
  Serial.println("SD.begin");
  while (!theSD.begin()) { putStringOnLcd("Insert SD card", ILI9341_RED); }
  
  File nnbfile = theSD.open("model.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    Serial.println("dnnrt.begin failed" + String(ret));
    putStringOnLcd("dnnrt.begin failed" + String(ret), ILI9341_RED);
    return;
  }

    
  //start Camera
  Serial.println("camera.begin");
  theCamera.begin();
  
  Serial.println("camera.streaming");
  
  theCamera.startStreaming(true, CamCB);
}

void loop() { 

}