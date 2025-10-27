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
