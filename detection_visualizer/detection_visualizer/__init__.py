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

import copy
import sys

import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter('sync_slop', 0.3)
        self.declare_parameter('allow_headerless', False)
        self.declare_parameter('sync_mode', 'approx')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        sync_slop = float(self.get_parameter('sync_slop').value)
        allow_headerless = bool(self.get_parameter('allow_headerless').value)
        sync_mode = str(self.get_parameter('sync_mode').value).lower()
        if sync_mode not in ('approx', 'latest'):
            self.get_logger().warning(
                f"Unsupported sync_mode '{sync_mode}', defaulting to 'approx'"
            )
            sync_mode = 'approx'

        self._last_image_log_ns = 0
        self._last_detection_log_ns = 0
        self._image_count = 0
        self._detections_count = 0
        self._sync_count = 0
        self._last_image_stamp = None
        self._last_detection_stamp = None
        self._allow_headerless = allow_headerless
        self._sync_slop = sync_slop
        self._sync_mode = sync_mode
        self._latest_detection_msg = None
        self._latest_delta_warned_stamp = None
        self._latest_headerless_warned = False

        self.get_logger().info(
            f"Waiting for image on '{image_topic}' and detections on '{detections_topic}' "
            f"(sync slop {sync_slop:.3f}s, allow_headerless={allow_headerless}, mode={sync_mode})"
        )

        self._status_timer = self.create_timer(5.0, self._log_status)

        # publish debug image with sensor data QoS so tooling like rqt_image_view can subscribe
        self._image_pub = self.create_publisher(Image, '/dbg_images', qos_profile_sensor_data)

        if self._sync_mode == 'approx':
            # subscribers with message_filters to sync image + detections
            image_sub = message_filters.Subscriber(
                self, Image, image_topic, qos_profile=qos_profile_sensor_data
            )
            detections_sub = message_filters.Subscriber(
                self, Vision, detections_topic, qos_profile=qos_profile_sensor_data
            )
            image_sub.registerCallback(self._image_debug_callback)
            detections_sub.registerCallback(self._detections_debug_callback)

            ts = message_filters.ApproximateTimeSynchronizer(
                [image_sub, detections_sub],
                queue_size=5,
                slop=sync_slop,
                allow_headerless=allow_headerless,
            )
            ts.registerCallback(self.synced_callback)
            self.get_logger().info("Using ApproximateTimeSynchronizer.")
        else:
            self.get_logger().info("Using 'latest' synchronization mode.")
            self._image_subscription = self.create_subscription(
                Image,
                image_topic,
                self._image_latest_callback,
                qos_profile_sensor_data,
            )
            self._detections_subscription = self.create_subscription(
                Vision,
                detections_topic,
                self._detections_latest_callback,
                qos_profile_sensor_data,
            )

    @staticmethod
    def _stamp_to_float(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def synced_callback(self, image_msg, detections_msg):
        # called when both image and detections with close timestamps are received
        self.image_message = image_msg

        image_stamp = image_msg.header.stamp
        detection_stamp = detections_msg.header.stamp
        self._sync_count += 1
        if detection_stamp.sec == 0 and detection_stamp.nanosec == 0:
            self.get_logger().warn(
                "Received detections without timestamp; consider setting detections header stamp."
            )
        else:
            delta = self._stamp_to_float(image_stamp) - self._stamp_to_float(detection_stamp)
            self.get_logger().debug(
                f"Synced image {image_stamp.sec}.{image_stamp.nanosec:09d}s with detections "
                f"{detection_stamp.sec}.{detection_stamp.nanosec:09d}s (dt={delta:.3f}s, "
                f"{len(detections_msg.detections)} detections)"
            )

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

    def _image_debug_callback(self, image_msg):
        self._image_count += 1
        self._last_image_stamp = image_msg.header.stamp
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_image_log_ns >= 1_000_000_000:
            stamp = image_msg.header.stamp
            self.get_logger().info(
                f"Image received #{self._image_count} stamp {stamp.sec}.{stamp.nanosec:09d} "
                f"frame '{image_msg.header.frame_id}'"
            )
            self._last_image_log_ns = now_ns

    def _detections_debug_callback(self, detections_msg):
        self._detections_count += 1
        self._last_detection_stamp = detections_msg.header.stamp
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_detection_log_ns >= 1_000_000_000:
            stamp = detections_msg.header.stamp
            self.get_logger().info(
                f"Detections received #{self._detections_count} stamp {stamp.sec}.{stamp.nanosec:09d} "
                f"count {len(detections_msg.detections)}"
            )
            self._last_detection_log_ns = now_ns

    def _log_status(self):
        image_stamp = (f"{self._last_image_stamp.sec}.{self._last_image_stamp.nanosec:09d}"
                       if self._last_image_stamp else "n/a")
        detection_stamp = (f"{self._last_detection_stamp.sec}.{self._last_detection_stamp.nanosec:09d}"
                           if self._last_detection_stamp else "n/a")
        self.get_logger().info(
            f"Status: images received={self._image_count} (last {image_stamp}), "
            f"detections received={self._detections_count} (last {detection_stamp}), "
            f"sync callbacks={self._sync_count}"
        )

    def _image_latest_callback(self, image_msg):
        self._image_debug_callback(image_msg)
        if self._latest_detection_msg is None:
            return
        detection_msg = self._latest_detection_msg
        detection_stamp = detection_msg.header.stamp
        if (not self._allow_headerless and
                detection_stamp.sec == 0 and detection_stamp.nanosec == 0):
            if not self._latest_headerless_warned:
                self.get_logger().warn(
                    "Latest sync mode received detections without timestamps; skipping overlay. "
                    "Set allow_headerless:=true to bypass."
                )
                self._latest_headerless_warned = True
            return

        if detection_stamp.sec != 0 or detection_stamp.nanosec != 0:
            delta = self._stamp_to_float(image_msg.header.stamp) - self._stamp_to_float(detection_stamp)
            stamp_tuple = (detection_stamp.sec, detection_stamp.nanosec)
            if abs(delta) > self._sync_slop and self._latest_delta_warned_stamp != stamp_tuple:
                self.get_logger().warn(
                    f"Latest sync mode: image/detection delta {delta:.3f}s exceeds sync_slop "
                    f"{self._sync_slop:.3f}s; overlaying anyway."
                )
                self._latest_delta_warned_stamp = stamp_tuple

        self.synced_callback(image_msg, detection_msg)

    def _detections_latest_callback(self, detections_msg):
        self._detections_debug_callback(detections_msg)
        self._latest_detection_msg = copy.deepcopy(detections_msg)
        self._latest_delta_warned_stamp = None
        self._latest_headerless_warned = False

def main():
    rclpy.init()
    node = DetectionVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
