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
#define NOTE //HOME, HOME3A. WORK, WORK3A or NOTE3A

#ifdef HOME
#define  AP_SSID        "kumakero2.4"
#define  PASSPHRASE     "4roses6126"
#define  HTTP_SRVR_IP   "192.168.50.104"
#define  MQTT_SRVR      "192.168.50.104"

#elif defined(HOME3A)
#define  AP_SSID        "airpocket"
#define  PASSPHRASE     "openpass"
#define  HTTP_SRVR_IP   "192.168.50.105"
#define  MQTT_SRVR      "192.168.50.105"

#elif defined(WORK)
#define  AP_SSID        "J00WLN1305A"
#define  PASSPHRASE     "m1n0ru0869553434@LAN"
#define  HTTP_SRVR_IP   "192.168.1.65"
#define  MQTT_SRVR      "192.168.1.65"

#elif defined(WORK3A)
#define  AP_SSID        "airpocket"
#define  PASSPHRASE     "openpass"
#define  HTTP_SRVR_IP   "192.168.101.56"
#define  MQTT_SRVR      "192.168.191.56"

#elif defined(NOTE)
#define  AP_SSID        "airpocket"
#define  PASSPHRASE     "openpass"
#define  HTTP_SRVR_IP   "192.168.137.1"
#define  MQTT_SRVR      "192.168.137.1"

#elif defined(NOTE3A)
#define  AP_SSID        "airpocket"
#define  PASSPHRASE     "openpass"
#define  HTTP_SRVR_IP   "192.168.137.234"
#define  MQTT_SRVR      "192.168.137.234"

#else
#error "No configuration defined. Please define HOME, WORK, or NOTE."
#endif

#define  HTTP_PORT      "1880"
#define  HTTP_GET_PATH  "/getdata"
#define  HTTP_POST_PATH "/postdata"

#define  MQTT_PORT     "1883"
#define  MQTT_CLI_ID   "Telit_Device_pub"
#define  MQTT_TOPIC1   "fromNodeRed"

#endif /*_CONFIG_H_*/