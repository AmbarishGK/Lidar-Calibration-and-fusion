# Lidar-Calibration-and-fusion
Trying to calibrate multiple lidar in our test bed for an indoor autonomous vehicle. Currently two, 1. Base station 2. Autonomous Vehicle for i2v. Below instructions are temp and will be moved to relevant places once we have a setup ready.

## MOLA SLAM Docker Setup
This repository contains a Dockerfile for setting up MOLA SLAM with ROS 2 Humble. This README provides instructions for building and running the Docker container with proper display configuration for GUI applications only for linux.

### Prerequisites
- Docker installed on your system
- X11 server

### Building the Docker Image
Clone this repository and Build the Docker image:

```bash
docker build -t mola_slam:humble .
```
Running the Container with Display Support (Linux)
Grant access to your X server:

```bash
xhost +local:docker
```
Run the container with display configuration:
```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/dri \
  mola_slam:humble
```
For better performance with 3D applications, you can add the --privileged flag:
```
bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/dri \
  --privileged \
  mola_slam:humble
```
When you're done, revoke X server access:
```bash
xhost -local:docker
```
Testing MOLA Installation
Once inside the container, you can test if MOLA is properly installed:

```bash
# Test the mm map viewer
mm-viewer

# Test MOLA LO CLI
mola-lidar-odometry-cli --help

# Run the MOLA LO tutorial
ros2 launch mvsim demo_warehouse.launch.py do_fake_localization:=False use_rviz:=False
```
In another terminal, connect to the running container:

```bash
# First, get the container ID
docker ps

# Then connect to it
docker exec -it CONTAINER_ID bash

# Run MOLA LO
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py lidar_topic_name:=/lidar1_points generate_simplemap:=True
```