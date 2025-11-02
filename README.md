# junky
Repository for experiments with AI and NPU on my robot
For working with harware
```
sudo usermod -a -G rdma,docker,video $USER
```
## Structure of packages
```
control/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/control
├── launch/
│   └── controls_launch.py
├── control/
│   ├── __init__.py
│   └── manual_control_node.py
└── test/
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
│   ├── camera_node.py
│   └── templates/
│       ├── base.html
│       ├── index.html
│       ├── control_pc.html
│       ├── control_phone.html
│       └── sensors.html
└── test/
```
