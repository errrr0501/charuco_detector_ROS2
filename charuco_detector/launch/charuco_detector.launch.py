import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    charuco_detector_share_dir = get_package_share_directory('charuco_detector')
    ros_param_file = LaunchConfiguration('ros_param_file', default = charuco_detector_share_dir + '/config/ros.yaml')
    charuco_param_file = LaunchConfiguration('charuco_param_file', default = charuco_detector_share_dir + '/yaml/charuco.yaml')


    declare_ros_param_file_cmd = DeclareLaunchArgument(
      'ros_param_file',
      default_value = charuco_detector_share_dir + '/config/ros.yaml',
      description = 'Path to file with ROS related config')  

    declare_charuco_param_file_cmd = DeclareLaunchArgument(
      'charuco_param_file',
      default_value = charuco_detector_share_dir + '/yaml/charuco.yaml',
      description = 'Path to file with charuco related config')  
    

    charuco_node = Node(
        package='charuco_detector',
        executable='charuco_detector_node',
        name='charuco_detector',
        output='screen',
        parameters=[ros_param_file, charuco_param_file,
    ])               
    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(declare_ros_param_file_cmd)
    ld.add_action(declare_charuco_param_file_cmd)
    ld.add_action(charuco_node)

    return ld
