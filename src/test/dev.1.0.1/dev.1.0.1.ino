/*
✔ Ver.1.0.0 MQTTサブスクライブ関数化
Ver.1.0.1 CamCBでディスプレイ表示
Ver.1.0.2 CamCBからストリーミング止めてstill撮影呼び出し
Ver.1.0.3 still撮影をhttpPost

*/


#include <MqttGs2200.h>
#include <TelitWiFi.h>
#include "config.h"

#define  CONSOLE_BAUDRATE  115200
#define  SUBSCRIBE_TIMEOUT 60000	//ms
/*-------------------------------------------------------------------------*
 * Globals:
 *-------------------------------------------------------------------------*/
TelitWiFi gs2200;
TWIFI_Params gsparams;
MqttGs2200 theMqttGs2200(&gs2200);

char server_cid = 0;
bool served = false;
uint16_t len, count=0;
MQTTGS2200_Mqtt mqtt;

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
			if (msDelta(start) < SUBSCRIBE_TIMEOUT) {
				String data;
				/* just in case something from GS2200 */
				while (gs2200.available()) {
					if (false == theMqttGs2200.receive(data)) {
						served = false; // quit the loop
						break;
					}

					Serial.println("Receive data: " + data);
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


// the setup function runs once when you press reset or power the board
void setup() {
	MQTTGS2200_HostParams hostParams;
	/* initialize digital pin LED_BUILTIN as an output. */
	pinMode(LED0, OUTPUT);
	digitalWrite( LED0, LOW );   // turn the LED off (LOW is the voltage level)
	Serial.begin( CONSOLE_BAUDRATE ); // talk to PC

	/* Initialize SPI access of GS2200 */
	Init_GS2200_SPI_type(iS110B_TypeC);

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
 
	hostParams.host = (char *)MQTT_SRVR;
	hostParams.port = (char *)MQTT_PORT;
	hostParams.clientID = (char *)MQTT_CLI_ID;
	hostParams.userName = NULL;
	hostParams.password = NULL;
	theMqttGs2200.begin(&hostParams);
	digitalWrite( LED0, HIGH ); // turn on LED
}







// the loop function runs over and over again forever
void loop() {
  Serial.println("loop");
  subscribeMQTT();
}
