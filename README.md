# Mobile_Manipulator
This repository is created for build ROS mobile manipulator and for graduation project.

# Start Guide
You have to install ROS noetic on Ubuntu 20.04 LTS and Create catkin_ws before use this repository.
## Step 1
### Setup environment
$ sudo apt-get update

$ sudo apt-get upgrade

$ cd catkin_ws/src/

$ git clone http://github.com/luppyfox/Mobile_Manipulator

$ roscd;cd ..;catkin_make;rospack profile

## Step 2
###Install Sensor Package
$ cd catkin_ws/src/Mobile_Manipulator/sensor_pkg/

### Lidar LDS-02
$ git clone https://github.com/ROBOTIS-GIT/ld08_driver.git
### RPLidar A1
$ git clone https://github.com/Slamtec/rplidar_ros.git
### laser_filters
$ sudo apt-get install ros-noetic-laser-filters
### Astra Orbbec 3D
https://shop.orbbec3d.com/Astra

$ git clone https://github.com/orbbec/ros_astra_camera.git

$ sudo apt install ros-noetic-rgbd-launch libuvc-dev
### Motor with encoder

## Step 3
### Update all package just installed
$ sudo apt-get update

$ sudo apt-get upgrade

$ roscd;cd ..;catkin_make;rospack profile

## Step 4
$ roslaunch navigation full_launch.launch
