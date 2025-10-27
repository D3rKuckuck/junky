# Создаем сервис для автозапуска
## Создайте файл сервиса /etc/systemd/system/junky-web.service:
```
[Unit]
Description=Junky Robot Web Interface
After=network.target
Wants=network.target

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi
ExecStart=/usr/bin/python3 /home/orangepi/web_interface.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```
## WiFi скрипт для автоподключения
Создайте скрипт /home/orangepi/connect_wifi.sh:
```
#!/bin/bash

CONFIG_FILE="/home/orangepi/wifi_config.json"

if [ -f "$CONFIG_FILE" ]; then
    SSID=$(python3 -c "import json; f=open('$CONFIG_FILE'); d=json.load(f); print(d['ssid']); f.close()")
    PASSWORD=$(python3 -c "import json; f=open('$CONFIG_FILE'); d=json.load(f); print(d['password']); f.close()")
    
    echo "Подключаемся к WiFi: $SSID"
    
    # Создаем конфиг для wpa_supplicant
    sudo tee /etc/wpa_supplicant/wpa_supplicant.conf > /dev/null <<EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=RU

network={
    ssid="$SSID"
    psk="$PASSWORD"
}
EOF
    
    # Перезапускаем сеть
    sudo systemctl restart networking
    sudo wpa_cli -i wlan0 reconfigure
fi
```

## Настройка автозапуска
```
# Делаем скрипты исполняемыми
chmod +x /home/orangepi/web_interface.py
chmod +x /home/orangepi/connect_wifi.sh

# Добавляем WiFi подключение в автозагрузку
sudo crontab -e
# Добавьте строку:
@reboot /home/orangepi/connect_wifi.sh

# Включаем веб-сервис
sudo systemctl enable junky-web.service
sudo systemctl start junky-web.service
```
