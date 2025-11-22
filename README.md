# py_bt_ros

A minimal example showing how to use the Behaviour Tree runtime with ROS 2 through a nav_senario.


### Execution nav_senario

Execution Nav2
```source install/local_setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

Execution Webot's camera
```
python3 scenarios/nav_scenario/camera_service_node.py
```

Run Behaviour Tree Controller
```
python3 main.py
```
