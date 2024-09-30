#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "config.h"

#define  CONSOLE_BAUDRATE  115200
#define HTTP_GET_PATH "/data/get"
#define HTTP_POST_PATH "/data/post"

typedef enum{
    POST=0,
    GET
} DEMO_STATUS_E;

DEMO_STATUS_E httpStat;
char sendData[100];

const uint16_t RECEIVE_PACKET_SIZE = 1500;
uint8_t Receive_Data[RECEIVE_PACKET_SIZE] = {0};

TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams hostParams;

void setup() {
    pinMode(LED0, OUTPUT);
    digitalWrite(LED0, LOW);   
    Serial.begin(CONSOLE_BAUDRATE);

    Init_GS2200_SPI_type(iS110B_TypeA);
    gsparams.mode = ATCMD_MODE_STATION;
    gsparams.psave = ATCMD_PSAVE_DEFAULT;
    if (gs2200.begin(gsparams)) {
        ConsoleLog("GS2200 Initialization Failed");
        while (1);
    }

    if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
        ConsoleLog("Association Failed");
        while (1);
    }

    hostParams.host = (char *)HTTP_SRVR_IP;
    hostParams.port = (char *)HTTP_PORT;
    theHttpGs2200.begin(&hostParams);

    ConsoleLog("Start HTTP Client");
    theHttpGs2200.config(HTTP_HEADER_AUTHORIZATION, "Basic dGVzdDp0ZXN0MTIz");
    theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "chunked");
    theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "application/x-www-form-urlencoded");
    theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
    digitalWrite(LED0, HIGH); 
}

void loop() {
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 10000) { // 10秒ごとにリクエスト
        lastTime = millis();
        httpStat = GET;
    }

    if (httpStat == GET) {
        bool result = theHttpGs2200.get(HTTP_GET_PATH);
        if (result) {
            uint8_t receivedData[RECEIVE_PACKET_SIZE];
            theHttpGs2200.read_data(receivedData, RECEIVE_PACKET_SIZE); // void関数のため代入しない

            ConsoleLog("Data received successfully");
            theHttpGs2200.post(HTTP_POST_PATH, (const char*)receivedData); // char*にキャストして送信
            ConsoleLog("Sending data back to Node-RED");
        } else {
            ConsoleLog("GET request failed");
        }
    }
}
