import argparse
import copy
import json
from array import array
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion
from op3_vision_msgs.msg import Bearing, Detection, Vision
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D, ObjectHypothesisWithPose, Point2D, Pose2D


def _make_time(stamp_dict) -> Time:
    msg = Time()
    if stamp_dict is None:
        return msg
    msg.sec = int(stamp_dict.get("sec", 0))
    msg.nanosec = int(stamp_dict.get("nanosec", 0))
    return msg


def _make_header(data: dict) -> Header:
    msg = Header()
    if data is None:
        return msg
    msg.stamp = _make_time(data.get("stamp", {}))
    msg.frame_id = data.get("frame_id", "")
    return msg


def _make_pose(data: dict) -> Pose:
    msg = Pose()
    if data is None:
        return msg
    position = data.get("position", {})
    orientation = data.get("orientation", {})
    msg.position = Point(
        x=float(position.get("x", 0.0)),
        y=float(position.get("y", 0.0)),
        z=float(position.get("z", 0.0)),
    )
    msg.orientation = Quaternion(
        x=float(orientation.get("x", 0.0)),
        y=float(orientation.get("y", 0.0)),
        z=float(orientation.get("z", 0.0)),
        w=float(orientation.get("w", 1.0)),
    )
    return msg


def _make_pose_with_covariance(data: dict) -> PoseWithCovariance:
    msg = PoseWithCovariance()
    if data is None:
        return msg
    msg.pose = _make_pose(data.get("pose", {}))
    covariance = data.get("covariance", [])
    msg.covariance = [float(value) for value in covariance]
    return msg


def _make_bbox(data: dict) -> BoundingBox2D:
    msg = BoundingBox2D()
    if data is None:
        return msg
    center = data.get("center", {})
    position = center.get("position", {})
    msg.center = Pose2D(
        position=Point2D(
            x=float(position.get("x", 0.0)),
            y=float(position.get("y", 0.0)),
        ),
        theta=float(center.get("theta", 0.0)),
    )
    msg.size_x = float(data.get("size_x", 0.0))
    msg.size_y = float(data.get("size_y", 0.0))
    return msg


def _make_detection(data: dict) -> Detection:
    msg = Detection()
    msg.header = _make_header(data.get("header", {}))
    for result_data in data.get("results", []):
        result = ObjectHypothesisWithPose()
        hypothesis = result_data.get("hypothesis", {})
        result.hypothesis.class_id = str(hypothesis.get("class_id", ""))
        result.hypothesis.score = float(hypothesis.get("score", 0.0))
        result.pose = _make_pose_with_covariance(result_data.get("pose", {}))
        msg.results.append(result)
    msg.bbox = _make_bbox(data.get("bbox", {}))
    bearing_data = data.get("bearing", {})
    msg.bearing = Bearing(
        x=float(bearing_data.get("x", 0.0)),
        y=float(bearing_data.get("y", 0.0)),
    )
    return msg


def load_image(path: Path) -> Image:
    payload = json.loads(path.read_text())
    msg = Image()
    msg.header = _make_header(payload.get("header", {}))
    msg.height = int(payload.get("height", 0))
    msg.width = int(payload.get("width", 0))
    msg.encoding = payload.get("encoding", "rgb8")
    msg.is_bigendian = int(payload.get("is_bigendian", 0))
    msg.step = int(payload.get("step", msg.width * 3))
    msg.data = array("B", payload.get("data", []))
    return msg


def load_detections(path: Path) -> Vision:
    payload = json.loads(path.read_text())
    msg = Vision()
    msg.header = _make_header(payload.get("header", {}))
    for detection_data in payload.get("detections", []):
        msg.detections.append(_make_detection(detection_data))
    return msg


class DetectionVisualizerHarness(Node):
    def __init__(self, image_msg: Image, detections_msg: Vision, frequency_hz: float):
        super().__init__("detection_visualizer_harness")
        self._image_template = image_msg
        self._detections_template = detections_msg
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self._detections_pub = self.create_publisher(Vision, "/camera/detections", qos_profile_sensor_data)
        period = 1.0 / frequency_hz if frequency_hz > 0.0 else 0.5
        effective_freq = frequency_hz if frequency_hz > 0.0 else 1.0 / period
        self.create_timer(period, self._publish_once)
        self.get_logger().info(
            f"Publishing sample image ({self._image_template.height}x{self._image_template.width}) "
            f"and {len(self._detections_template.detections)} detections at {effective_freq:.2f} Hz"
        )

    def _publish_once(self) -> None:
        now = self.get_clock().now().to_msg()
        image_msg = copy.deepcopy(self._image_template)
        detections_msg = copy.deepcopy(self._detections_template)

        image_msg.header.stamp = now
        detections_msg.header.stamp = now
        for detection in detections_msg.detections:
            detection.header.stamp = now

        self._image_pub.publish(image_msg)
        self._detections_pub.publish(detections_msg)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publish canned image and detection test data.")
    default_base = Path(__file__).resolve().parent / "messages"
    parser.add_argument(
        "--image-file",
        type=Path,
        default=default_base / "image.json",
        help="Path to JSON file containing a serialized sensor_msgs/Image payload.",
    )
    parser.add_argument(
        "--detections-file",
        type=Path,
        default=default_base / "detections.json",
        help="Path to JSON file containing a serialized op3_vision_msgs/Vision payload.",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=2.0,
        help="Publish frequency in Hz (default: 2.0).",
    )
    args = parser.parse_args(argv)

    image_path: Path = args.image_file
    detections_path: Path = args.detections_file
    if not image_path.exists():
        raise FileNotFoundError(f"Image message file not found: {image_path}")
    if not detections_path.exists():
        raise FileNotFoundError(f"Detections message file not found: {detections_path}")

    image_msg = load_image(image_path)
    detections_msg = load_detections(detections_path)

    rclpy.init()
    node = DetectionVisualizerHarness(image_msg, detections_msg, args.frequency)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
