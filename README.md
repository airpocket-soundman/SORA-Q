# SORA-Q
SORA-Q改造プロジェクト

# 無線モジュール

GS2200リポジトリ
https://github.com/jittermaster/GS2200-WiFi

Cameraリポジトリ
https://github.com/sonydevworld/spresense-arduino-compatible/tree/master/Arduino15/packages/SPRESENSE/hardware/spresense/1.0.0/libraries/Camera

Cameraリファレンス
https://developer.sony.com/spresense/spresense-api-references-arduino/Camera_8h.html

http post参考
https://elchika.com/article/13a1da3a-d410-4d05-8087-ba78b855d7db/

ILI9341 ドライバ
https://github.com/kzhioki/Adafruit_ILI9341

学習用画像取得プログラム
https://github.com/TE-YoshinoriOota/Spresense-Tech-Seminar-Basic/blob/master/Sketches/Spresense_camera_take_picture/Spresense_camera_take_picture.ino

# 受信局

Raspberry Pi 3A+





## Node-RED
```
sudo apt update
sudo apt upgrade
```
node-red install
```
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
node-red-pi --max-old-space-size=256
sudo systemctl enable nodered.service
```
added node
```
node-red-contrib-aedes
node-red-contrib-image-tools

```

## Wi-Fi AP化

```
sudo apt install hostapd
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo apt install dnsmasq
sudo DEBIAN_FRONTEND=noninteractive apt install -y netfilter-persistent iptables-persistent
```

```
sudo nano /etc/dhcpcd.conf
```

```
interface wlan0
static ip_address=192.168.4.1/24
nohook wpa_supplicant
```
```
sudo nano /etc/sysctl.d/routed-ap.conf
```
```
# Enable IPv4 routing
net.ipv4.ip_forward=1
```
※
```
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE
sudo netfilter-persistent save
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
```
※

```
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
sudo nano /etc/dnsmasq.conf
```
```
interface=wlan0 # Listening interface
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
                # Pool of IP addresses served via DHCP
domain=wlan     # Local wireless DNS domain
address=/gw.wlan/192.168.4.1
                # Alias for this router
```
```
sudo rfkill unblock wlan
sudo nano /etc/hostapd/hostapd.conf
```
```
interface=wlan0
driver=nl80211
hw_mode=b
channel=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
ieee80211ac=0
wmm_enabled=1
ieee80211d=1
country_code=JP
ieee80211h=1
local_pwr_constraint=3
spectrum_mgmt_required=1
wpa=3
wpa_key_mgmt=WPA-PSK
ssid=****
wpa_passphrase=*********   #パスワードは8文字以64文字以下
```
```
sudo nano /etc/sysctl.conf
```
コメントアウトを有効化
```
net.ipv4.ip_forward=1
```
```
sudo nano /etc/default/hostapd
```
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```
```
sudo nano /etc/rc.local
```
「exit 0」とある行の前に、以下の内容を追記します。
```
iptables-restore < /etc/iptables.ipv4.nat
```


sudo systemctl reboot