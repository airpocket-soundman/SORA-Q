/*
 *  config.h - WiFi Configration Header
 *
 *  This work is free software; you can redistribute it and/or modify it under the terms 
 *  of the GNU Lesser General Public License as published by the Free Software Foundation; 
 *  either version 2.1 of the License, or (at your option) any later version.
 *
 *  This work is distributed in the hope that it will be useful, but without any warranty; 
 *  without even the implied warranty of merchantability or fitness for a particular 
 *  purpose. See the GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License along with 
 *  this work; if not, write to the Free Software Foundation, 
 *  Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef _CONFIG_H_
#define _CONFIG_H_

/*-------------------------------------------------------------------------*
 * Configration
 *-------------------------------------------------------------------------*/
// is110のRev設定
#define iS110_TYPEA   //is110_TYPEA, is110_TYPEB, is110_TYPEC, 


#define BASE //接続先のパターンを選択

#ifdef BASE
#define  AP_SSID        "your SSID"
#define  PASSPHRASE     "your PASS"
#define  HTTP_SRVR_IP   "HTTP server IP"
#define  MQTT_SRVR      "MQTT server IP"

#else
#error "No configuration defined. Please define HOME, WORK, or NOTE."
#endif

#define  HTTP_PORT      "1880"
#define  HTTP_GET_PATH  "/getdata"
#define  HTTP_POST_PATH "/postdata"

#define  MQTT_PORT     "1883"
#define  MQTT_CLI_ID   "Telit_Device_pub"
#define  MQTT_TOPIC1   "fromNodeRed"
#define  MQTT_TOPIC2   "fromSORAQ"

#endif /*_CONFIG_H_*/