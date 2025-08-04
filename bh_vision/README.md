# BH Vision

This is a **ROS 2 wrapper around [darknet](https://pjreddie.com/darknet)**, an open source neural network framework.  The code has been taken from [Open Robotics Darknet ROS](https://github.com/ros2/openrobotics_darknet_ros) and modified to fit our needs.

This is not a repository for training new weights, to do this follow the instrcutions in [darknet](https://pjreddie.com/darknet).  This repository soley uses pre-trained networks to make detections on images and publish these detections.

This node has the following paramets:
- detector_parameters - This sets the parameter file which points to the weights files for the network
- rgb_image - This sets the topic the raw image will come from default = /camera/image_raw
- detections - This sets the topic to publish the detections to default = /camera/detections

To build this package you need to have [darknet](https://pjreddie.com/darknet) in your workspace.  This can be in your src/ folder or outside of it, this does not matter.  To build use the following command

```bash
colcon build --packages-up-to bh_vision --cmake-args -DENABLE_CUDA=OFF -DCMAKE_DISABLE_FIND_PACKAGE_OpenMP=TRUE
```

or

```bash
colcon build --cmake-args -DENABLE_CUDA=OFF -DCMAKE_DISABLE_FIND_PACKAGE_OpenMP=TRUE
```

To run the node there is a launch file which can be run as follows

```bash
ros2 launch bh_vision detector_launch.py
```

Currently YOLO-V7 and YOLO-V7-tiny are configured but this configuration can be added to by following the stes below:

1. Create a new folder in the config folder with a custom name
2. In this folder add the following four files:
   1. .names file - this defines the names of the detections
   2. .yaml file - this is used by ros2 to find the necessary files to run the detector using the custom network
   3. .cfg file - this sets up the network
   4. .weights file - this is the network weights
3. Run the launch file providing the path to the .yaml file as a parameter
```bash
ros2 launch bh_vision detector_launch.py detector_parameters:=install/bh_vision/share/bh_vision/config/your_folder_name/params.yaml
```
