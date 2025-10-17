# Darknet Vision

This repository has three packages


## vision_darknet

This is a modifified version of [Open Robotics Darknet ROS](https://github.com/ros2/openrobotics_darknet_ros), which is a ros2 wrapper for darknet.  This takes in the raw image and outputs detections in the image.

The latest weights trained on the Torso database are stored in the `reality` folder.

For simulation, `op3_webots` publishes the image data to the default `/camera/image_raw`. 

```bash
 ros2 launch op3_vision_darknet detector.launch.py detector_parameters:=$(ros2 pkg prefix op3_vision_darknet)/share/op3_vision_darknet/config/reality-v0.1/params.yaml rgb_image:=/camera/image_raw

```
The physical OP3 publishes the camera stream to `/usb_cam_node/image_raw`.  Supply the topic using the parameter `rgb_image`.

```bash
 ros2 launch op3_vision_darknet detector.launch.py detector_parameters:=$(ros2 pkg prefix op3_vision_darknet)/share/op3_vision_darknet/config/reality-v0.1/params.yaml rgb_image:=/usb_cam_node/image_raw

```

## op3_vision_msgs

These are custom vision msgs to be used for detections.  The main difference between these and vision_msgs is the addition of the bearing variable for each detection.


## detection_visualizer

This repository is for debugging purposes.  It subscribes to both the image topic and detections topic and outputs an image with this overlayed.  This can then be viewed in something like rqt.
This is a modified version of [ROS2 Detection Visualizer](https://github.com/ros2/detection_visualizer).

```bash
ros2 run detection_visualizer detection_visualizer
```
View predictions using ROS2 rqt image view plugin.
```bash
ros2 run rqt_image_view rqt_image_view

```