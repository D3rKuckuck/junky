# junky
Repository for experiments with AI and NPU on my robot
For working with harware
```
sudo usermod -a -G rdma,docker,video $USER
```
junky_server/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/junky_server
├── launch/
│   └── server_launch.py
├── junky_server/
│   ├── __init__.py
│   ├── web_server.py
│   ├── control_node.py
│   ├── camera_node.py
│   ├── static/
│   |   ├── css/
│   |   │   └── style.css
│   |   └── js/
│   |       └── script.js
│   └── templates/
│       ├── base.html
│       ├── index.html
│       ├── control_pc.html
│       ├── control_phone.html
│       └── sensors.html
└── test/
