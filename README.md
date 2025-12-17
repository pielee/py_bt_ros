# py_bt_ros

A delivery classification system for multi-robot.


Launch Limo sensor
```
ros2 launch wego teleop_launch.py
```

Nav2-limo
```
ros2 launch wego navigation_diff_launch.py
```

Launch qr detecter
```
ros2 launch limo_qr_system qr_system.launch.py
```

Launch button_serial
```
ros2 run limo_button_serial button_serial_node
```

BT-Nav2
```
python3 scenarios/finalproject/limo_nav_action_server.py
```

BT-Limo
'''
python3 scenarios/finalproject/limo_action_server.py
'''

Run BT
'''
python3 main.py
'''
