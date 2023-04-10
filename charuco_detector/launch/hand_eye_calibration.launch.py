from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    charuco_detector_share_dir = get_package_share_directory('charuco_detector')
    
    charuco_detector_ros_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([charuco_detector_share_dir + '/launch/charuco_detector.launch.py']),
        # launch_arguments={'image_topic': "/camera/color/image_raw",
        #                   'camera_info_topic': "/camera/color/camera_info"}.items()
    )
    return LaunchDescription([
    charuco_detector_ros_launch,
    # if you want to disable camera node, remove the following line.
    # camera,
  ])