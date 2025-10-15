# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import cv2
import cv_bridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from op3_vision_msgs.msg import Vision
import message_filters


class DetectionVisualizerNode(Node):

    def __init__(self):
        super().__init__('detection_visualizer')

        self._bridge = cv_bridge.CvBridge()
        self.image_message = None  # store the most recent image

        # configurable topic names
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/camera/detections')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value


        # publisher with reliable QoS
        output_image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1)

        self._image_pub = self.create_publisher(Image, '/dbg_images', output_image_qos)

        # subscribers with message_filters to sync image + detections
        image_sub = message_filters.Subscriber(self, Image, image_topic,
                                               qos_profile=qos_profile_sensor_data)
        detections_sub = message_filters.Subscriber(self, Vision,
                                                    detections_topic,
                                                    qos_profile=qos_profile_sensor_data)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub], queue_size=5, slop=0.3
        )
        ts.registerCallback(self.synced_callback)



    def synced_callback(self, image_msg, detections_msg):
        # called when both image and detections with close timestamps are received
        self.image_message = image_msg

        cv_image = self._bridge.imgmsg_to_cv2(image_msg)

        for detection in detections_msg.detections:
            max_class = None
            max_score = 0.0
            for result in detection.results:
                hypothesis = result.hypothesis
                if hypothesis.score > max_score:
                    max_score = hypothesis.score
                    max_class = hypothesis.class_id

            if max_class is None:
                self.get_logger().warn("No valid class found")
                continue

            cx, cy = detection.bbox.center.position.x, detection.bbox.center.position.y
            sx, sy = detection.bbox.size_x, detection.bbox.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))
            cv2.rectangle(cv_image, min_pt, max_pt, (0, 255, 0), 1)

            label = f'{max_class} {max_score:.3f}'
            pos = (min_pt[0], max_pt[1])
            cv2.putText(cv_image, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (0, 255, 0), 1, cv2.LINE_AA)

        dbg_image_msg = self._bridge.cv2_to_imgmsg(cv_image, encoding=image_msg.encoding)
        dbg_image_msg.header = image_msg.header
        self._image_pub.publish(dbg_image_msg)

 
def main():
    rclpy.init()
    node = DetectionVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

