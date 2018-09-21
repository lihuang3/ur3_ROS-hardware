Universal Robot (UR3) Pick and Place Hardware Implementation with ROS using a USB Cam and an Electromagnetic Gripper

[`Simulation video`](https://youtu.be/Yj5DEocFa48)
[`Hardware video`](https://youtu.be/FAYPbAhYoXw)

- How to cite this repository:
  ```
    Huang, L., Zhao, H., Universal Robot (UR3) Pick and Place Hardware Implementation with ROS using a USB Cam and an Electromagnetic Gripper, (2018), GitHub repository, https://github.com/lihuang3/ur3_ROS-hardware
  ```
  or BibTex
  ```
    @misc{Huang2018,
      author = {Huang, L., Zhao, H.},
      title = {Universal Robot (UR3) Pick and Place Hardware Implementation with ROS using a USB Cam and an Electromagnetic Gripper},
      year = {2018},
      publisher = {GitHub},
      journal = {GitHub repository},
      howpublished = {\url{https://github.com/lihuang3/ur3_ROS-hardware}}
    }
  ```

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

You may need to change your joint limit file before you run any motion planning program on the hardware. ([`Example`](https://github.com/lihuang3/ur3_ROS-hardware/issues/1#issuecomment-422070509))


#### 2. Image Processing

#### 3. ROS-Arduino
##### 3.1
[`official tutorial`](http://wiki.ros.org/rosserial_arduino/Tutorials)
[`Generating Message Header File`](http://wiki.ros.org/rosserial_client/Tutorials/Generating%20Message%20Header%20Files)

After you download rosserial, to generate custom messages you should follow the following step:
```
  cd <your workspace>
  source devel/setup.bash
  rosrun rosserial_arduino make_libraries.py /home/<username>/arduino-1.8.5/libraries
```
you can check you custom message folder at /home/<username>/arduino-1.8.5/libraries/ros_lib/ur5_notebook (or ur3_hardware)

To use arduino subscribe ROS topic:
[`Template for a ROS Subscriber Using rosserial on Arduino`](https://www.intorobotics.com/template-for-a-ros-subscriber-using-rosserial-on-arduino/)
1. compile /ur3_hardware/arduino_gripper/arduino_gripper.ino in Arduino IDE, and uplaod to board
2. Open a new Terminal and run initialize.roslaunch
3. Open another Terminal and start the subscriber node by typing the following command:
```
cd <your workspace>
source devel/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyACM1
```
4. In Arduino IDE open serial monitor
