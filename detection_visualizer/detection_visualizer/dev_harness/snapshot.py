import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py import message_to_yaml

from op3_vision_msgs.msg import Vision
from sensor_msgs.msg import Image


class SnapshotNode(Node):
    def __init__(
        self,
        image_topic: str,
        detection_topic: str | None,
        image_path: Path,
        detection_path: Path | None,
        timeout: float,
    ):
        super().__init__("detection_visualizer_snapshot")
        self._bridge = CvBridge()
        self._image_path = image_path
        self._detection_path = detection_path
        self._timeout = timeout

        self._image_saved = False
        self._detections_saved = detection_path is None

        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )
        if detection_topic:
            self._detection_sub = self.create_subscription(
                Vision, detection_topic, self._detections_callback, qos_profile_sensor_data
            )
        else:
            self._detection_sub = None

        if timeout > 0.0:
            self.create_timer(timeout, self._handle_timeout)

        detections_text = f" and detections on '{detection_topic}'" if detection_topic else ""
        self.get_logger().info(
            f"Waiting for image on '{image_topic}'{detections_text}"
        )

    def _image_callback(self, msg: Image) -> None:
        if self._image_saved:
            return
        cv_image = self._bridge.imgmsg_to_cv2(msg)
        self._image_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self._image_path), cv_image)
        self._image_saved = True
        stamp = msg.header.stamp
        self.get_logger().info(
            f"Saved image ({msg.height}x{msg.width} {msg.encoding}) to {self._image_path} "
            f"at stamp {stamp.sec}.{stamp.nanosec:09d}"
        )
        self._check_completion()

    def _detections_callback(self, msg: Vision) -> None:
        if self._detections_saved:
            return
        if self._detection_path is None:
            self._detections_saved = True
            return
        yaml_str = message_to_yaml(msg)
        self._detection_path.parent.mkdir(parents=True, exist_ok=True)
        self._detection_path.write_text(yaml_str)
        stamp = msg.header.stamp
        self.get_logger().info(
            f"Saved detections ({len(msg.detections)} entries) to {self._detection_path} "
            f"at stamp {stamp.sec}.{stamp.nanosec:09d}"
        )
        self._detections_saved = True
        self._check_completion()

    def _handle_timeout(self) -> None:
        if not (self._image_saved and self._detections_saved):
            self.get_logger().error(
                f"Timeout {self._timeout:.1f}s reached waiting for data. "
                f"Image saved={self._image_saved}, detections saved={self._detections_saved}"
            )
            rclpy.shutdown()

    def _check_completion(self) -> None:
        if self._image_saved and self._detections_saved:
            self.get_logger().info("Snapshot complete, shutting down.")
            rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Capture one debug image and detections to disk.")
    parser.add_argument(
        "--image-topic",
        default="/dbg_images",
        help="Image topic to subscribe to (default: /dbg_images).",
    )
    parser.add_argument(
        "--detections-topic",
        default="/camera/detections",
        help="Detections topic to subscribe to (default: /camera/detections).",
    )
    parser.add_argument(
        "--image-path",
        type=Path,
        default=Path("dbg_image.png"),
        help="Output path for the captured image (default: dbg_image.png).",
    )
    parser.add_argument(
        "--detections-path",
        type=Path,
        default=Path("dbg_detections.yaml"),
        help="Output path for the captured detections (default: dbg_detections.yaml).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait before aborting (default: 10.0). Use 0 for no timeout.",
    )
    parser.add_argument(
        "--skip-detections",
        action="store_true",
        help="Do not record detections; only save the image.",
    )

    args = parser.parse_args(argv)
    detection_topic = None if args.skip_detections else args.detections_topic
    detection_path = None if args.skip_detections else args.detections_path

    rclpy.init()
    node = SnapshotNode(
        image_topic=args.image_topic,
        detection_topic=detection_topic,
        image_path=args.image_path,
        detection_path=detection_path,
        timeout=max(0.0, args.timeout),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
