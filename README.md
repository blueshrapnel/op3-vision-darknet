# BH Vision

This repository has three packages

## bh_detection_visualizer

This repository is for debugging purposes.  It subscribes to both the image topic and detections topic and outputs an image with this overlayed.  This can then be viewed in something like rqt

## bh_vision

This is a modifified version of [Open Robotics Darknet ROS](https://github.com/ros2/openrobotics_darknet_ros), which is a ros2 wrapper for darknet.  This takes in the raw image and outputs detections in the image

## bh_vision_msgs

These are our custom vision msgs to be used for detections.  The main difference between these and vision_msgs is the addition of the bearing variable for each detection.
