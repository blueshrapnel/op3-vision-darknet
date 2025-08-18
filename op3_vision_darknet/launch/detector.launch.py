import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    detector_parameters_file = LaunchConfiguration('detector_parameters')
    rgb_image_topic = LaunchConfiguration('rgb_image')
    detections_topic = LaunchConfiguration('detections')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_detector_parameter_file = os.path.join(
        get_package_share_directory('op3_vision_darknet'),
        'yolo-v7-tiny',
        'params.yaml'
    )
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description='Set if sim time should be used'
    )
    detector_parameters_cmd = DeclareLaunchArgument(
        'detector_parameters',
        default_value=default_detector_parameter_file,
        description='YOLO object detection configuration file'
    )
    rgb_image_cmd = DeclareLaunchArgument(
        'rgb_image',
        default_value="/camera/image_raw",
        description='RGB image that should be used for object detection with YOLO'
    )
    detections_topic_cmd = DeclareLaunchArgument(
        'detections',
        default_value='/camera/detections',
        description='Detections containing the bounding boxes of objects to be considered'
    )

    yolo_detector_node = Node(
        package='op3_vision_darknet',
        executable='detector_node',
        name='detector_node',
        remappings=[
            ('~/images', rgb_image_topic),
            ('~/detections', detections_topic)
        ],
        parameters = [ParameterFile(detector_parameters_file, allow_substs=True),{"use_sim_time": use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(detector_parameters_cmd)
    ld.add_action(rgb_image_cmd)
    ld.add_action(detections_topic_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(yolo_detector_node)
    return ld
