from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Change the following parameters according to the pattern in usage
    squares_sides_size_in_meters = DeclareLaunchArgument('squares_sides_size_in_meters', default_value='0.0200')
    markers_sides_size_in_meters = DeclareLaunchArgument('markers_sides_size_in_meters', default_value='0.0150')
    # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default_value='0.0248')
    # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default_value='0.0155')
    number_of_squares_in_x = DeclareLaunchArgument('number_of_squares_in_x', default_value='10')
    number_of_squares_in_y = DeclareLaunchArgument('number_of_squares_in_y', default_value='14')
    number_of_markers = DeclareLaunchArgument('number_of_markers', default_value='70')
    number_of_bits_for_markers_sides = DeclareLaunchArgument('number_of_bits_for_markers_sides', default_value='4')
    dictionary_id = DeclareLaunchArgument('dictionary_id', default_value='3')
    # Change the following 2 topics to match the ones published by your camera sensor
    image_topic = DeclareLaunchArgument('image_topic', default_value='/rgb/image_raw')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='/rgb/camera_info')
    image_analysis_publish_topic = DeclareLaunchArgument('image_analysis_publish_topic', default_value='image_topic' + '_charuco_detection')
    charuco_pose_publish_topic = DeclareLaunchArgument('charuco_pose_publish_topic', default_value='image_topic' + '_charuco_pose')
    # Change the charuco TF frame if you need poses to be estimated from several charuco boards
    charuco_tf_frame = DeclareLaunchArgument('charuco_tf_frame', default_value='calib_charuco')
    # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
    # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
    sensor_frame_override = DeclareLaunchArgument('sensor_frame_override', default_value='calib_camera')

    # Fine tune the yaml and parameters below if needed
    config_file = DeclareLaunchArgument('config_file', default_value='$(find charuco_detector)/yaml/charuco.yaml')
    use_median_blur = DeclareLaunchArgument('use_median_blur', default_value='false')
    median_blur_k_size = DeclareLaunchArgument('median_blur_k_size', default_value='3')  # must be odd number
    use_dynamic_range = DeclareLaunchArgument('use_dynamic_range', default_value='false')
    use_bilateral_filter = DeclareLaunchArgument('use_bilateral_filter', default_value='false')
    bilateral_filter_pixel_neighborhood = DeclareLaunchArgument('bilateral_filter_pixel_neighborhood', default_value='5')
    bilateral_filter_sigma_color = DeclareLaunchArgument('bilateral_filter_sigma_color', default_value='100.0')
    bilateral_filter_sigma_space = DeclareLaunchArgument('bilateral_filter_sigma_space', default_value='100.0')
    bilateral_filter_border_type = DeclareLaunchArgument('bilateral_filter_border_type', default_value='4')  # cv::BORDER_DEFAUL
    
    use_clahe = DeclareLaunchArgument('use_clahe', default_value='true')
    clahe_clip_limit = DeclareLaunchArgument('clahe_clip_limit', default_value="1.0")
    clahe_size_x = DeclareLaunchArgument('clahe_size_x', default_value="2")
    clahe_size_y = DeclareLaunchArgument('clahe_size_y', default_value="2")

    use_adaptive_threshold = DeclareLaunchArgument('use_adaptive_threshold', default_value="false")
    adaptive_threshold_max_value = DeclareLaunchArgument('adaptive_threshold_max_value', default_value="255.0")
    adaptive_threshold_method = DeclareLaunchArgument('adaptive_threshold_method', default_value="1"),#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
    adaptive_threshold_type = DeclareLaunchArgument('adaptive_threshold_type', default_value="0"),#cv::THRESH_BINARY
    adaptive_threshold_block_size = DeclareLaunchArgument('adaptive_threshold_block_size', default_value="65")
    adaptive_threshold_constant_offset_from_mean = DeclareLaunchArgument('adaptive_threshold_constant_offset_from_mean', default_value="0.0")

    use_static_tf_broadcaster = DeclareLaunchArgument('adaptive_threshold_constant_offset_from_mean', default_value="0.0")
    tf_broadcaster_republish_rate = DeclareLaunchArgument('tf_broadcaster_republish_rate', default_value="10.0")#only used if > 
    

    charuco_node_params = {
        # Change the following parameters according to the pattern in usage
        'squares_sides_size_in_meters' : LaunchConfiguration('squares_sides_size_in_meters'),
        'markers_sides_size_in_meters' : LaunchConfiguration('markers_sides_size_in_meters'),
        # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default_value='0.0248')
        # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default_value='0.0155')
        'number_of_squares_in_x' : LaunchConfiguration('number_of_squares_in_x'),
        'number_of_squares_in_y' : LaunchConfiguration('number_of_squares_in_y'),
        'number_of_markers' : LaunchConfiguration('number_of_markers'),
        'number_of_bits_for_markers_sides' : LaunchConfiguration('number_of_bits_for_markers_sides'),
        'dictionary_id' : LaunchConfiguration('dictionary_id'),
        # Change the following 2 topics to match the ones published by your camera sensor
        'image_topic' : LaunchConfiguration('image_topic'),
        'camera_info_topic' : LaunchConfiguration('camera_info_topic'),
        'image_analysis_publish_topic' : LaunchConfiguration('image_analysis_publish_topic'),
        'charuco_pose_publish_topic' : LaunchConfiguration('charuco_pose_publish_topic'),
        # Change the charuco TF frame if you need poses to be estimated from several charuco boards
        'charuco_tf_frame' : LaunchConfiguration('charuco_tf_frame'),
        # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
        # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
        'sensor_frame_override' : LaunchConfiguration('sensor_frame_override'),

        # Fine tune the yaml and parameters below if needed
        'config_file' : LaunchConfiguration('config_file'),
        'use_median_blur' : LaunchConfiguration('use_median_blur'),
        'median_blur_k_size' : LaunchConfiguration('median_blur_k_size'),  # must be odd number
        'use_dynamic_range' : LaunchConfiguration('use_dynamic_range'),
        'use_bilateral_filter' : LaunchConfiguration('use_bilateral_filter'),
        'bilateral_filter_pixel_neighborhood' : LaunchConfiguration('bilateral_filter_pixel_neighborhood'),
        'bilateral_filter_sigma_color' : LaunchConfiguration('bilateral_filter_sigma_color'),
        'bilateral_filter_sigma_space' : LaunchConfiguration('bilateral_filter_sigma_space'),
        'bilateral_filter_border_type' : LaunchConfiguration('bilateral_filter_border_type'),  # cv::BORDER_default_value

        'use_clahe' : LaunchConfiguration('use_clahe'),
        'clahe_clip_limit' : LaunchConfiguration('clahe_clip_limit'),
        'clahe_size_x' : LaunchConfiguration('clahe_size_x'),
        'clahe_size_y' : LaunchConfiguration('clahe_size_y'),

        'use_adaptive_threshold' : LaunchConfiguration('use_adaptive_threshold'),
        'adaptive_threshold_max_value' : LaunchConfiguration('adaptive_threshold_max_value'),
        'adaptive_threshold_method' : LaunchConfiguration('adaptive_threshold_method'),#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
        'adaptive_threshold_type' : LaunchConfiguration('adaptive_threshold_type'),#cv::THRESH_BINARY
        'adaptive_threshold_block_size' : LaunchConfiguration('adaptive_threshold_block_size'),
        'adaptive_threshold_constant_offset_from_mean' : LaunchConfiguration('adaptive_threshold_constant_offset_from_mean'),

        'use_static_tf_broadcaster' : LaunchConfiguration('use_static_tf_broadcaster'),
        'tf_broadcaster_republish_rate' : LaunchConfiguration('tf_broadcaster_republish_rate')#only used if > 0
    
    }

    # Create the node
    # charuco_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'charuco_detector', 'charuco_detector_node'],
    #     output='screen',
    #     parameters=[charuco_node_params]
    # )


    charuco_node = Node(
        package='charuco_detector',
        executable='charuco_detector_node',
        name='charuco_detector',
        output='screen',
        parameters=[charuco_node_params]
        # parameters=[ros_param_file, network_param_file,
        #   {
        #     "config_path": yolo_config_path, 
        #     "weights_path": yolo_weights_path,
        #   },]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(squares_sides_size_in_meters)
    ld.add_action(markers_sides_size_in_meters)
    ld.add_action(number_of_squares_in_x)
    ld.add_action(number_of_squares_in_y)
    ld.add_action(number_of_markers)
    ld.add_action(number_of_bits_for_markers_sides)
    ld.add_action(dictionary_id)
    ld.add_action(image_topic)
    ld.add_action(camera_info_topic)
    ld.add_action(image_analysis_publish_topic)
    ld.add_action(charuco_pose_publish_topic)
    ld.add_action(charuco_tf_frame)
    ld.add_action(sensor_frame_override)
    ld.add_action(config_file)
    ld.add_action(use_median_blur)
    ld.add_action(median_blur_k_size)
    ld.add_action(use_dynamic_range)
    ld.add_action(use_bilateral_filter)
    ld.add_action(bilateral_filter_pixel_neighborhood)
    ld.add_action(bilateral_filter_sigma_color)
    ld.add_action(bilateral_filter_sigma_space)
    ld.add_action(bilateral_filter_border_type)
    ld.add_action(use_clahe)
    ld.add_action(clahe_clip_limit)
    ld.add_action(clahe_size_x)
    ld.add_action(clahe_size_y)
    ld.add_action(use_adaptive_threshold)
    ld.add_action(adaptive_threshold_max_value)
    ld.add_action(adaptive_threshold_method)
    ld.add_action(adaptive_threshold_type)
    ld.add_action(adaptive_threshold_block_size)
    ld.add_action(adaptive_threshold_constant_offset_from_mean)
    ld.add_action(use_static_tf_broadcaster)
    ld.add_action(tf_broadcaster_republish_rate)

    # Add the charuco_node to the launch description
    ld.add_action(charuco_node)

    return ld

    # return LaunchDescription([

    #     Node(
    #         package='charuco_detector',
    #         executable='charuco_detector_node',
    #         name='charuco_detector',
    #     )

    # ])