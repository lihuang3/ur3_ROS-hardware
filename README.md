#### 1. Connecting Universal Robot (UR3) to PC (Ubuntu 16.04)
##### 1.1 Configure your hardware
Follow the steps on the official website [`"Getting Started with a Universal Robot and ROS-Industrial"`](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) to set up network using a router.

Download the repo [`ur_modern_driver`](https://github.com/ThomasTimm/ur_modern_driver) to `src` in your Universal Robot workspace, and `catkin_make`. If there is hardware interface error during catkin make, replace [`ur_hardware_interface.cpp`](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) in `ur_modern_driver`

To connect your PC and an UR3, run the following launch file, (you need to `source devel/setup.bash` first, and go to each folder that includes the launch file to launch it)
```
  roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=ROBOT_IP_ADDRESS
  
  roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch
  
  roslaunch ur3_moveit_config moveit_rviz.launch config:=true
```
