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

#define DNN_IMG_W 32
#define DNN_IMG_H 32
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 0//104
#define CAM_CLIP_Y 0//64
#define CAM_CLIP_W 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)
#define CAM_CLIP_H 224//112 //DNN_IMGのn倍であること(clipAndResizeImageByHWの制約)

#define LINE_THICKNESS 5

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

float threshold = 0.2;

uint8_t buf[DNN_IMG_W*DNN_IMG_H];

DNNRT dnnrt;
DNNVariable input(DNN_IMG_W*DNN_IMG_H);
  
//static uint8_t const label[2]= {0,1};
static String const label[25]= {"Back_R0",    "Back_R1",    "Back_R2",    "Back_R3", 
                                "Bottom_R0",  "Bottom_R1",  "Bottom_R2",  "Bottom_R3", 
                                "Front_R0",   "Front_R1",   "Front_R2",   "Front_R3",
                                "Left_R0",    "Left_R1",    "Left_R2",    "Left_R3",
                                "Right_R0",   "Right_R1",   "Right_R2",   "Right_R3",
                                "Top_R0",     "Top_R1",     "Top_R2",     "Top_R3",
                                "empty"};

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

void drawBox(uint16_t* imgBuf) {
  /* Draw target line */
  for (int x = CAM_CLIP_X; x < CAM_CLIP_X+CAM_CLIP_W; ++x) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*(CAM_CLIP_Y+n) + x)              = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*(CAM_CLIP_Y+CAM_CLIP_H-1-n) + x) = ILI9341_RED;
    }
  }
  for (int y = CAM_CLIP_Y; y < CAM_CLIP_Y+CAM_CLIP_H; ++y) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*y + CAM_CLIP_X+n)                = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*y + CAM_CLIP_X + CAM_CLIP_W-1-n) = ILI9341_RED;
    }
  }  
}


void CamCB(CamImage img) {
  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, 
                                          CAM_CLIP_X, CAM_CLIP_Y, 
                                          CAM_CLIP_X + CAM_CLIP_W - 1, 
                                          CAM_CLIP_Y + CAM_CLIP_H - 1, 
                                          DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()) {
    putStringOnLcd("Clip and Resize Error:" + String(err), ILI9341_RED);
    return;
  }

  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* tmp = (uint16_t*)small.getImgBuff();

  // RGBデータから輝度データを計算してDNNに入力
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

  // 正規化
  for (int n = 0; n < DNN_IMG_W * DNN_IMG_H; ++n) {
    dnnbuf[n] /= f_max;
  }
  
  String gStrResult = "?";
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff(); 

  // Box描画
  drawBox(imgBuf); 
  // TFT描画
  tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 224);
  // text描画
  int index = output.maxIndex();

  if (output[index] >= threshold){
    gStrResult = String(label[index]) + String(":") + String(output[index]);
  } else {
    gStrResult = "not identify";
  }
  Serial.println(gStrResult);
  putStringOnLcd(gStrResult, ILI9341_YELLOW);

  // モード変更有無をチェック
  if(index != before_value) {
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