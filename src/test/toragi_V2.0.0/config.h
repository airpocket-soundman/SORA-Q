#ifndef _CONFIG_H_
#define _CONFIG_H_

/*-------------------------------------------------------------------------*
 * Configration
 *-------------------------------------------------------------------------*/
// is110のRev設定
#define iS110_TYPEA   //is110_TYPEA, is110_TYPEB, is110_TYPEC, 

// Wi-Fi アクセスポイント設定
#define  AP_SSID              "airpocket"
#define  PASSPHRASE           "openpass"
#define  HTTP_SRVR_IP         "192.168.137.1"

#define  HTTP_PORT            "1880"
#define  HTTP_GET_PATH        "/getdata"
#define  HTTP_POST_PATH       "/postdata"
#define  HTTP_POST_TEXT_PATH  "/posttxt"

#endif /*_CONFIG_H_*/

//初期設定
bool nnb_copy               = true;               // trueでMicroSD挿入時にnnbファイルをコピー
bool photo_reflector_out    = false;              // flag trueで読み取った値をSerial.print
char flashPath[]            = "data/slim.nnb";    // flashメモリ内のCNNモデルのPath
char flashFolder[]          = "data/";            // flashメモリ内のCNNモデルの保存フォルダ
char nnbFile[]              = "model.nnb";        // MicroSDから読み取るCNNモデル名
bool selectedImageOnly      = true;               // 推論後、SLIM検出時のみ画像を送信する場合はtrue
float threshold             = 0.2;                // 物体認識閾値