# mw_ahrs_ros
[![Licence](https://img.shields.io/badge/License-BSD--3-green.svg)](https://opensource.org/license/bsd-3-clause/)
[![ubuntu20](https://img.shields.io/badge/-UBUNTU_20.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/)
[![noetic](https://img.shields.io/badge/-NOETIC-blue?style=flat-square&logo=ros)](https://wiki.ros.org/noetic)

## Overview
ROS driver package for MW-AHRSv1 and MW-AHRSv2U

## ROS Interface

| Topic Name   | Type                             | Description           |
|--------------|----------------------------------|-----------------------|
| ``imu/data`` | ``sensor_msgs/Imu``              | IMU data              |
| ``imu/rpy``  | ``geometry_msgs/Vector3Stamped`` | roll, pitch, yaw data |
| ``imu/mag``  | ``sensor_msgs/MagneticField``    | magnetic field data   |

| Service Name  | Type             | Description      |
|---------------|------------------|------------------|
| ``imu/reset`` | ``std_srvs/Trigger`` | Reset the sensor |

## Installation
```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/mw_ahrs_ros.git

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

## Usage
```
sudo cp rules/99-mwahrs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

roslaunch mw_ahrs_ros mw_ahrs.launch
```
