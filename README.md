# VX300s

The repo is robot repo for control VX300s on melodic.

UR5 repo including:
* interbotix_ros_core
* interbotix_ros_mainpulators
* interbotix_ros_toolboxes
* vx300s_bringup

## ROS Distro Support

|         | Melodic | Noetic  |
|:-------:|:-------:|:-------:|
| branch | [`melodic`](https://github.com/kuolunwang/VX300s/tree/melodic) | [`noetic`](https://github.com/kuolunwang/VX300s/tree/noetic) |
| Status | supported | supported |

## Clone repo

```
    git clone --recursive git@github.com:kuolunwang/VX300s.git
```

## How to use VX300s

### Hardware Setup

1. Plug the 12V power cable into an outlet and insert the barrel plug into the barrel jack on the X-series power hub (located under the see-through acrylic on the base of the robot). You should briefly see the LEDs on the Dynamixel motors flash red.
2. Plug in the micro-usb cable into the U2D2 (located under the see-through acrylic on the robot's base) and your computer.
3. Make sure your computer can detect /dev/USB0 or other port. 
### Software Setup

1. First you need to launch vx300s_connect.launch to connect vx300s.
\
This launch file have three parameters can control.
    * robot_name : robot name. 
    * port : you need see your computer detect which port, then fix this.
    * use_rviz : show rviz or not.
    ```
        roslaunch vx300s_bringup vx300s_connect.launch
    ```

2. Then, rosrun node to control vx300s.
\
    You can rename node name using --node_name [your name] type command behide, or use default name vx300s_control_node.
    ```
        rosrun vx300s_bringup vx300s_control.py
    ``` 

## Service List
\
**The defualt robot_name is vx300s.**

---

| Service Name | Service Type | Service Description |
|:--------:|:--------:|:--------:|
| /vx300s/go_home | [Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Set vx300s joints to initial value(0) |
|/vx300s/go_sleep| [Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Set vx300s go back to initial pose |
| /vx300s/go_pose | [ee_pose.srv](https://github.com/kuolunwang/VX300s/blob/main/vx300s_bringup/srv/ee_pose.srv) | Set vx300s go to specific pose |
| /vx300s/gripper_open| [Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Set vx300s end-effector gripper open |
| /vx300s/gripper_close| [Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Set vx300s end-effector gripper close |
| /vx300s/check_grasped| [Trigger](http://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html) | Check if the end-effector gripper is grasped object