```
sudo nano /etc/hostapd/hostapd-2ghz.conf
```

```
# 2.4GHz точка доступа
interface=wlan0-ap
driver=nl80211

# SSID с именем хоста
ssid=junkybot-26082005a-2G
country_code=RU
hw_mode=g
channel=6
ht_capab=[HT20][HT40+][SHORT-GI-20][SHORT-GI-40][DSSS_CCK-40]
wmm_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0

# Безопасность
wpa=2
wpa_passphrase=0713281097
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP

# Настройки производительности
max_num_sta=10
wpa_group_rekey=3600
```

```
sudo nano /etc/hostapd/hostapd-5ghz.conf
```

```
# 5GHz точка доступа
interface=wlan1-ap
driver=nl80211

# SSID с именем хоста
ssid=junkybot-26082005a-5G
country_code=RU
hw_mode=a
channel=36
ht_capab=[HT40+][SHORT-GI-20][SHORT-GI-40][DSSS_CCK-40]
wmm_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0

# Безопасность
wpa=2
wpa_passphrase=0713281097
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP

# Настройки производительности
max_num_sta=10
wpa_group_rekey=3600
ieee80211ac=1
vht_oper_chwidth=1
vht_oper_centr_freq_seg0_idx=42
```

```
sudo nano /usr/local/bin/setup_ap.sh
```

```
#!/bin/bash

# Получаем имя хоста
HOSTNAME=$(hostname)

echo "Setting up dual-band AP for $HOSTNAME..."

# Создаем виртуальные интерфейсы
echo "Creating virtual interfaces..."
iw dev wlan0 interface add wlan0-ap type __ap
ip link set wlan0-ap up
ip addr add 192.168.10.1/24 dev wlan0-ap

# Создаем интерфейс для 5GHz
iw dev wlan0 interface add wlan1-ap type __ap
ip link set wlan1-ap up
ip addr add 192.168.11.1/24 dev wlan1-ap

# Обновляем SSID в конфигах
sed -i "s/^ssid=.*/ssid=${HOSTNAME}-2G/" /etc/hostapd/hostapd-2ghz.conf
sed -i "s/^ssid=.*/ssid=${HOSTNAME}-5G/" /etc/hostapd/hostapd-5ghz.conf

# Устанавливаем пароль
sed -i "s/^wpa_passphrase=.*/wpa_passphrase=0713281097/" /etc/hostapd/hostapd-2ghz.conf
sed -i "s/^wpa_passphrase=.*/wpa_passphrase=0713281097/" /etc/hostapd/hostapd-5ghz.conf

echo "Dual-band AP setup complete!"
echo "- 2.4GHz: ${HOSTNAME}-2G on 192.168.10.1/24"
echo "- 5GHz:   ${HOSTNAME}-5G on 192.168.11.1/24"
echo "- Password: 0713281097 for both"
```

```
sudo chmod +x /usr/local/bin/setup_ap.sh
```

```
sudo nano /etc/dnsmasq.conf
```

```
# Для 2.4GHz сети (192.168.10.0/24)
interface=wlan0-ap
dhcp-range=wlan0-ap,192.168.10.100,192.168.10.200,255.255.255.0,24h
dhcp-option=wlan0-ap,3,192.168.10.1
dhcp-option=wlan0-ap,6,192.168.10.1

# Для 5GHz сети (192.168.11.0/24)
interface=wlan1-ap
dhcp-range=wlan1-ap,192.168.11.100,192.168.11.200,255.255.255.0,24h
dhcp-option=wlan1-ap,3,192.168.11.1
dhcp-option=wlan1-ap,6,192.168.11.1

# Дополнительные настройки
log-dhcp
log-queries
```

```
sudo nano /etc/systemd/system/orangepi-ap.service
```

```
[Unit]
Description=Orange Pi 5 Dual-Band AP Setup
After=network.target
Wants=hostapd-2ghz.service hostapd-5ghz.service
Before=hostapd-2ghz.service hostapd-5ghz.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_ap.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```
```
sudo nano /etc/systemd/system/hostapd-2ghz.service
```
```
[Unit]
Description=HostAPD for 2.4GHz AP
After=orangepi-ap.service
Wants=orangepi-ap.service

[Service]
Type=forking
ExecStart=/usr/sbin/hostapd -B /etc/hostapd/hostapd-2ghz.conf
ExecReload=/bin/kill -HUP $MAINPID
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```
```
sudo nano /etc/systemd/system/hostapd-5ghz.service
```
```
[Unit]
Description=HostAPD for 5GHz AP
After=orangepi-ap.service
Wants=orangepi-ap.service

[Service]
Type=forking
ExecStart=/usr/sbin/hostapd -B /etc/hostapd/hostapd-5ghz.conf
ExecReload=/bin/kill -HUP $MAINPID
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```
sudo nano /etc/sysctl.conf
```
```
net.ipv4.ip_forward=1
```
```
sudo sysctl -p
```
```
sudo iptables -t nat -A POSTROUTING -s 192.168.10.0/24 -o wlan0 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -s 192.168.11.0/24 -o wlan0 -j MASQUERADE
```
```
sudo apt install iptables-persistent -y
sudo netfilter-persistent save
```

```
# Активируем сервисы
sudo systemctl enable orangepi-ap.service
sudo systemctl enable hostapd-2ghz.service
sudo systemctl enable hostapd-5ghz.service
sudo systemctl enable dnsmasq

# Запускаем
sudo systemctl start orangepi-ap.service
sudo systemctl start hostapd-2ghz.service
sudo systemctl start hostapd-5ghz.service
sudo systemctl start dnsmasq
```