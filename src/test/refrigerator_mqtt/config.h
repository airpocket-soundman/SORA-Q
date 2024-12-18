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
#define  AP_SSID        "J00WLN1305A"
#define  PASSPHRASE     "m1n0ru0869553434@LAN"

//#define  AP_SSID        "SSID:J00WLN2206A_EXT2.4_2"
//#define  PASSPHRASE     "o86q556264@CP"

//#define  AP_SSID        "kumakero2.4"
//#define  PASSPHRASE     "4roses6126"


//#define  HTTP_SRVR_IP   "192.168.50.104"
//#define  HTTP_PORT      "1880"

#define  HTTP_SRVR_IP  "192.168.1.65"
#define  HTTP_PORT     "1880"


#define  HTTP_GET_PATH  "/getdata"
#define  HTTP_POST_PATH "/postdata"

#define  MQTT_SRVR     "192.168.1.65"
#define  MQTT_PORT     "1880"
#define  MQTT_CLI_ID   "Telit_Device_pub"
#define  MQTT_TOPIC    "fromSpresense"

#endif /*_CONFIG_H_*/