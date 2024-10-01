#include <SDHCI.h>
#include <Flash.h>
#include <MqttGs2200.h>
#include <TelitWiFi.h>
#include "config.h"

#define CONSOLE_BAUDRATE  115200
#define SUBSCRIBE_TIMEOUT 60000 // ms
#define MAX_DATA_SIZE 100000 // 各受信メッセージの最大サイズ

/*-------------------------------------------------------------------------*
 * Globals:
 *-------------------------------------------------------------------------*/
TelitWiFi gs2200;
TWIFI_Params gsparams;
MqttGs2200 theMqttGs2200(&gs2200);

// ファイル保存管理用
SDClass SD;
File imgFile;  // Fileオブジェクト

// バイナリデータを格納するバッファを宣言
uint8_t binaryData[MAX_DATA_SIZE];
size_t binaryDataSize = 0;

// the setup function runs once when you press reset or power the board
void setup() {
    MQTTGS2200_HostParams hostParams;
    pinMode(LED0, OUTPUT);
    digitalWrite(LED0, LOW);   // turn the LED off (LOW is the voltage level)
    Serial.begin(CONSOLE_BAUDRATE); // talk to PC

    /* Initialize SPI access of GS2200 */
    Init_GS2200_SPI_type(iS110B_TypeA);

    /* Initialize AT Command Library Buffer */
    gsparams.mode = ATCMD_MODE_STATION;
    gsparams.psave = ATCMD_PSAVE_DEFAULT;
    if (gs2200.begin(gsparams)) {
        ConsoleLog("GS2200 Initialization Fails");
        while(1);
    }

    /* GS2200 Association to AP */
    if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
        ConsoleLog("Association Fails");
        while(1);
    }

    hostParams.host = (char *)MQTT_SRVR;
    hostParams.port = (char *)MQTT_PORT;
    hostParams.clientID = (char *)MQTT_CLI_ID;
    hostParams.userName = NULL;
    hostParams.password = NULL;
    theMqttGs2200.begin(&hostParams);
    digitalWrite(LED0, HIGH); // turn on LED
}

bool served = false;
MQTTGS2200_Mqtt mqtt;

// the loop function runs over and over again forever
void loop() {
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
            ConsolePrintf("Subscribed!\n");
        } 

        served = true;
    } else {
        uint64_t start = millis();


        if (imgFile) {  // servedがfalseになった時点でファイルを閉じる
          imgFile.close();
          ConsolePrintf("SDカードを閉じました。\n");
        }


        // SDカードを初期化し、ファイルを開く
        SD.begin();
        imgFile = SD.open("image.jpg", FILE_WRITE);  // SD.open()をwhileの外に移動
        ConsolePrintf("SD open");



        while (served) {
            if (msDelta(start) < SUBSCRIBE_TIMEOUT) {

                if (imgFile) {
                    while (gs2200.available()) {
                        // 受信メソッドを修正してバイナリデータを返すことを確認
                        String data;
                        ConsolePrintf("check receive false\n");
                        if (false == theMqttGs2200.receive(data)) {
                            served = false; // ループを終了
                            break;
                            ConsolePrintf("receeve false\n");
                        }

                        // Stringをバイト配列に変換
                        binaryDataSize = data.length();
                        ConsolePrintf("chacke binaryDataSize\n");
                        if (binaryDataSize > MAX_DATA_SIZE) {
                            ConsolePrintf("受信したデータのサイズがバッファサイズを超えています。\n");
                            served = false;
                            break;
                        }

                        // Stringをバイナリに変換
                        ConsolePrintf("transrate str to binary\n");
                        for (size_t i = 0; i < binaryDataSize; i++) {
                            binaryData[i] = (uint8_t)data[i]; // エンコーディングに応じて調整が必要な場合があります
                        }

                        // バイナリデータをSDカードに書き込む
                        ConsolePrintf("write to SD\n");
                        imgFile.write(binaryData, binaryDataSize);
                        Serial.println("バイナリデータを受信し、書き込みました。");
                        ConsolePrintf("in the while loop");
                    }


                } else {
                    ConsolePrintf("SDカードを開けませんでした。");
                    served = false; // SDカードが開けなかった場合、ループを終了
                }
                start = millis();
            } else {
                ConsolePrintf("サブスクリプションタイムアウト: %d ms!\n", SUBSCRIBE_TIMEOUT);
                theMqttGs2200.stop();
                served = false;
                exit(0);
            }
        }

        if (served == false && imgFile) { // servedがfalseになったときにもファイルを閉じる
            imgFile.close();
            ConsolePrintf("受信後にSDカードを閉じました。\n");
        }
    }
}


