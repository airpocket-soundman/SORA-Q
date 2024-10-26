/*
 *  Spresense_camera_take_picture.ino - Simple digital camera application
 *  Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include <Camera.h>
#include <SPI.h>
#include <SDHCI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS ,TFT_DC ,TFT_RST);

SDClass theSD;
bool bButtonPressed = false;
int gCounter = 0;

void changeState () {
  bButtonPressed = true;
}

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

void CamCB(CamImage img){
  if (img.isAvailable()) {
    img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
    tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 240);
    Serial.print("Image data size = ");
    Serial.print(img.getImgSize(), DEC);
    Serial.print(" , ");
    Serial.print("buff addr = ");
    Serial.print((unsigned long)img.getImgBuff(), HEX);
    Serial.println("");
    if (bButtonPressed) {
      tft.setTextSize(2); 
      tft.setCursor(100, 200);
      tft.setTextColor(ILI9341_RED);
      //tft.println("Shooting");
      char filename[16] = {0};
      sprintf(filename, "PICT%03d.JPG", gCounter);
      tft.println(filename);
    }
  } else {
    Serial.println("Failed to get video stream image");
  }

}

int intPin = 4;
void setup() {
  CamErr err;
  pinMode(LED0, OUTPUT);
  pinMode(intPin, INPUT_PULLUP);

  theSD.begin();
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  Serial.println("Prepare camera");
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
  {
      printError(err);
  }
  Serial.println("Start streaming");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
  {
      printError(err);
  }

  theCamera.setStillPictureImageFormat(
    //CAM_IMGSIZE_QVGA_H ,CAM_IMGSIZE_QVGA_V
    CAM_IMGSIZE_QVGA_H ,CAM_IMGSIZE_QVGA_H
   ,CAM_IMAGE_PIX_FMT_JPG);

  attachInterrupt(digitalPinToInterrupt(intPin) ,changeState ,FALLING);
}

void loop() {
  if (!bButtonPressed) return;
  
  Serial.println("Button pressed");
  digitalWrite(LED0, HIGH);
  
  // カメラのストリーミングをオフ
  theCamera.startStreaming(false, CamCB);
  Serial.println("Streaming stopped for capture");

  CamImage img = theCamera.takePicture();

  if (img.isAvailable()) {
    char filename[16] = {0};
    sprintf(filename, "PICT%03d.JPG", gCounter);

    // 古いファイルが存在する場合は削除
    if (theSD.exists(filename)) {
      if (theSD.remove(filename)) {
        Serial.println("Previous file removed successfully");
      } else {
        Serial.println("Failed to remove previous file");
      }
    }

    // 新しいファイルを作成して画像データを書き込み
    File myFile = theSD.open(filename, FILE_WRITE | O_TRUNC);
    if (myFile) {
      Serial.print("Saving image to ");
      Serial.println(filename);

      myFile.write(img.getImgBuff(), img.getImgSize());
      myFile.flush();  // バッファをフラッシュして書き込みを確定
      myFile.close();  // ファイルを閉じる
      delay(100);      // 書き込み後のディレイ

      Serial.println("Image saved successfully");
      ++gCounter;  // 次の画像のためにカウンターをインクリメント
    } else {
      Serial.println("Failed to open file for writing");
    }
  } else {
    Serial.println("Failed to capture image");
  }

  // 状態をリセット
  bButtonPressed = false;
  
  // カメラのストリーミングを再開
  theCamera.startStreaming(true, CamCB);
  Serial.println("Streaming restarted");
  
  digitalWrite(LED0, LOW);
}
