#include <MqttGs2200.h>
#include <TelitWiFi.h>
#include "config.h"

#define  CONSOLE_BAUDRATE  115200
#define  SUBSCRIBE_TIMEOUT 60000	// サブスクリプションのタイムアウト（ミリ秒）

/*-------------------------------------------------------------------------*
 * グローバル変数:
 *-------------------------------------------------------------------------*/
TelitWiFi gs2200;                // WiFiモジュールのインスタンス
TWIFI_Params gsparams;           // WiFiのパラメータ
MqttGs2200 theMqttGs2200(&gs2200); // MQTTクライアントインスタンス

// setup関数は、リセットボタンを押すかボードに電源を入れたときに一度実行される
void setup() {
    MQTTGS2200_HostParams hostParams; // MQTT接続のホストパラメータ
    pinMode(LED0, OUTPUT);            // LEDピンを出力に設定
    digitalWrite(LED0, LOW);          // LEDを消灯（LOWは電圧レベル）
    Serial.begin(CONSOLE_BAUDRATE);   // PCとのシリアル通信開始

    /* GS2200のSPIアクセスを初期化 */
    Init_GS2200_SPI_type(iS110B_TypeA);

    /* ATコマンドライブラリバッファを初期化 */
    gsparams.mode = ATCMD_MODE_STATION;
    gsparams.psave = ATCMD_PSAVE_DEFAULT;
    if (gs2200.begin(gsparams)) {
        ConsoleLog("GS2200の初期化に失敗しました");
        while (1); // 初期化失敗時は無限ループ
    }

    /* GS2200をアクセスポイントに接続 */
    if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
        ConsoleLog("接続に失敗しました");
        while (1); // 接続失敗時は無限ループ
    }

    hostParams.host = (char *)MQTT_SRVR;    // MQTTサーバのホスト名
    hostParams.port = (char *)MQTT_PORT;    // MQTTサーバのポート番号
    hostParams.clientID = (char *)MQTT_CLI_ID; // MQTTクライアントID
    hostParams.userName = NULL;              // ユーザー名（使用しない場合はNULL）
    hostParams.password = NULL;               // パスワード（使用しない場合はNULL）
    theMqttGs2200.begin(&hostParams);        // MQTTクライアントを初期化
    digitalWrite(LED0, HIGH);                // LEDを点灯
}

char server_cid = 0;                      // サーバーのCID
bool served = false;                       // サブスクリプション状態
uint16_t len, count = 0;                  // データ長とカウント
MQTTGS2200_Mqtt mqtt1;                    // 最初のトピック用のMQTTオブジェクト
MQTTGS2200_Mqtt mqtt2;                    // 2番目のトピック用のMQTTオブジェクト

// loop関数は、永遠に繰り返し実行される
void loop() {
    if (!served) {
        // MQTTクライアントを開始
        ConsoleLog("MQTTクライアントを開始します");
        if (false == theMqttGs2200.connect()) {
            return; // 接続失敗時は何もしない
        }

        ConsoleLog("MQTTメッセージの受信を開始します");
        // 次の受信データのためにバッファを初期化
        WiFi_InitESCBuffer();

        // 最初のトピックにサブスクライブ
        strncpy(mqtt1.params.topic, MQTT_TOPIC_1, sizeof(mqtt1.params.topic)); // MQTT_TOPIC_1をconfig.hで定義
        mqtt1.params.QoS = 0;  // QoSレベルを0に設定
        mqtt1.params.retain = 0; // リテインフラグを無効に設定

        if (true == theMqttGs2200.subscribe(&mqtt1)) {
            ConsolePrintf("最初のトピックにサブスクライブしました!\n");
        }

        // 2番目のトピックにサブスクライブ
        strncpy(mqtt2.params.topic, MQTT_TOPIC_2, sizeof(mqtt2.params.topic)); // MQTT_TOPIC_2をconfig.hで定義
        mqtt2.params.QoS = 0;  // QoSレベルを0に設定
        mqtt2.params.retain = 0; // リテインフラグを無効に設定

        if (true == theMqttGs2200.subscribe(&mqtt2)) {
            ConsolePrintf("2番目のトピックにサブスクライブしました!\n");
        }

        served = true; // サブスクリプション完了
    } else {
        uint64_t start = millis(); // 現在の時間を取得
        while (served) {
            if (msDelta(start) < SUBSCRIBE_TIMEOUT) { // タイムアウトチェック
                String data; // 受信データ用の文字列
                // 両方のトピックからのデータを処理
                while (gs2200.available()) {
                    // データの受信を確認
                    if (false == theMqttGs2200.receive(data)) {
                        served = false; // ループを終了
                        break;
                    }
                    Serial.println("最初のトピックから受信したデータ: " + data); // 受信したデータを表示

                    // 2番目のトピックからのデータに対するロジックを追加することも可能
                }
                start = millis(); // 時間を更新
            } else {
                ConsolePrintf("サブスクリプションがタイムアウトしました（%d ms）!\n", SUBSCRIBE_TIMEOUT);
                theMqttGs2200.stop(); // MQTTクライアントを停止
                served = false; // サブスクリプション状態をリセット
                exit(0); // プログラムを終了
            }
        }
    }
}

