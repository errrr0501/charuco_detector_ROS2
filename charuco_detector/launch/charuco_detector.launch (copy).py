from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Change the following parameters according to the pattern in usage
    squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default="0.0200")
    markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default='0.0150')
    # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default='0.0248')
    # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default='0.0155')
    number_of_squares_in_x = LaunchConfiguration('number_of_squares_in_x', default='10')
    number_of_squares_in_y = LaunchConfiguration('number_of_squares_in_y', default='14')
    number_of_markers = LaunchConfiguration('number_of_markers', default='70')
    number_of_bits_for_markers_sides = LaunchConfiguration('number_of_bits_for_markers_sides', default='4')
    dictionary_id = LaunchConfiguration('dictionary_id', default='3')
    # Change the following 2 topics to match the ones published by your camera sensor
    image_topic = LaunchConfiguration('image_topic', default='/rgb/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/rgb/camera_info')
    image_analysis_publish_topic = LaunchConfiguration('image_analysis_publish_topic', default='/rgb/image_raw' + '_charuco_detection')
    charuco_pose_publish_topic = LaunchConfiguration('charuco_pose_publish_topic', default='/rgb/image_raw'+ '_charuco_pose')
    # Change the charuco TF frame if you need poses to be estimated from several charuco boards
    charuco_tf_frame = LaunchConfiguration('charuco_tf_frame', default='calib_charuco')
    # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
    # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
    sensor_frame_override = LaunchConfiguration('sensor_frame_override', default='calib_camera')

    # Fine tune the yaml and parameters below if needed
    config_file = LaunchConfiguration('config_file', default='$(find charuco_detector)/yaml/charuco.yaml')
    use_median_blur = LaunchConfiguration('use_median_blur', default='false')
    median_blur_k_size = LaunchConfiguration('median_blur_k_size', default='3')  # must be odd number
    use_dynamic_range = LaunchConfiguration('use_dynamic_range', default='false')
    use_bilateral_filter = LaunchConfiguration('use_bilateral_filter', default='false')
    bilateral_filter_pixel_neighborhood = LaunchConfiguration('bilateral_filter_pixel_neighborhood', default='5')
    bilateral_filter_sigma_color = LaunchConfiguration('bilateral_filter_sigma_color', default='100.0')
    bilateral_filter_sigma_space = LaunchConfiguration('bilateral_filter_sigma_space', default='100.0')
    bilateral_filter_border_type = LaunchConfiguration('bilateral_filter_border_type', default='4')  # cv::BORDER_DEFAULT
    
    use_clahe = LaunchConfiguration('use_clahe', default='true')
    clahe_clip_limit = LaunchConfiguration('clahe_clip_limit', default="1.0")
    clahe_size_x = LaunchConfiguration('clahe_size_x', default="2")
    clahe_size_y = LaunchConfiguration('clahe_size_y', default="2")

    use_adaptive_threshold = LaunchConfiguration('use_adaptive_threshold', default="False")
    adaptive_threshold_max_value = LaunchConfiguration('adaptive_threshold_max_value', default="255.0")
    adaptive_threshold_method = LaunchConfiguration('adaptive_threshold_method', default="1")#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
    adaptive_threshold_type = LaunchConfiguration('adaptive_threshold_type', default="0")#cv::THRESH_BINARY
    adaptive_threshold_block_size = LaunchConfiguration('adaptive_threshold_block_size', default="65")
    adaptive_threshold_constant_offset_from_mean = LaunchConfiguration('adaptive_threshold_constant_offset_from_mean', default="0.0")

    use_static_tf_broadcaster = LaunchConfiguration('use_static_tf_broadcaster', default="True")
    tf_broadcaster_republish_rate = LaunchConfiguration('tf_broadcaster_republish_rate', default="10.0")#only used if > 0
    

    # charuco_node_params = {
    #     # Change the following parameters according to the pattern in usage
    #     'squares_sides_size_in_meters' : LaunchConfiguration('squares_sides_size_in_meters'),
    #     'markers_sides_size_in_meters' : LaunchConfiguration('markers_sides_size_in_meters'),
    #     # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default='0.0248')
    #     # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default='0.0155')
    #     'number_of_squares_in_x' : LaunchConfiguration('number_of_squares_in_x'),
    #     'number_of_squares_in_y' : LaunchConfiguration('number_of_squares_in_y'),
    #     'number_of_markers' : LaunchConfiguration('number_of_markers'),
    #     'number_of_bits_for_markers_sides' : LaunchConfiguration('number_of_bits_for_markers_sides'),
    #     'dictionary_id' : LaunchConfiguration('dictionary_id'),
    #     # Change the following 2 topics to match the ones published by your camera sensor
    #     'image_topic' : LaunchConfiguration('image_topic'),
    #     'camera_info_topic' : LaunchConfiguration('camera_info_topic'),
    #     'image_analysis_publish_topic' : LaunchConfiguration('image_analysis_publish_topic'),
    #     'charuco_pose_publish_topic' : LaunchConfiguration('charuco_pose_publish_topic'),
    #     # Change the charuco TF frame if you need poses to be estimated from several charuco boards
    #     'charuco_tf_frame' : LaunchConfiguration('charuco_tf_frame'),
    #     # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
    #     # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
    #     'sensor_frame_override' : LaunchConfiguration('sensor_frame_override'),

    #     # Fine tune the yaml and parameters below if needed
    #     'config_file' : LaunchConfiguration('config_file'),
    #     'use_median_blur' : LaunchConfiguration('use_median_blur'),
    #     'median_blur_k_size' : LaunchConfiguration('median_blur_k_size'),  # must be odd number
    #     'use_dynamic_range' : LaunchConfiguration('use_dynamic_range'),
    #     'use_bilateral_filter' : LaunchConfiguration('use_bilateral_filter', default='false'),
    #     'bilateral_filter_pixel_neighborhood' : LaunchConfiguration('bilateral_filter_pixel_neighborhood'),
    #     'bilateral_filter_sigma_color' : LaunchConfiguration('bilateral_filter_sigma_color'),
    #     'bilateral_filter_sigma_space' : LaunchConfiguration('bilateral_filter_sigma_space'),
    #     'bilateral_filter_border_type' : LaunchConfiguration('bilateral_filter_border_type'),  # cv::BORDER_DEFAULT

    #     'use_clahe' : LaunchConfiguration('use_clahe'),
    #     'clahe_clip_limit' : LaunchConfiguration('clahe_clip_limit'),
    #     'clahe_size_x' : LaunchConfiguration('clahe_size_x'),
    #     'clahe_size_y' : LaunchConfiguration('clahe_size_y'),

    #     'use_adaptive_threshold' : LaunchConfiguration('use_adaptive_threshold'),
    #     'adaptive_threshold_max_value' : LaunchConfiguration('adaptive_threshold_max_value'),
    #     'adaptive_threshold_method' : LaunchConfiguration('adaptive_threshold_method'),#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
    #     'adaptive_threshold_type' : LaunchConfiguration('adaptive_threshold_type'),#cv::THRESH_BINARY
    #     'adaptive_threshold_block_size' : LaunchConfiguration('adaptive_threshold_block_size'),
    #     'adaptive_threshold_constant_offset_from_mean' : LaunchConfiguration('adaptive_threshold_constant_offset_from_mean'),

    #     'use_static_tf_broadcaster' : LaunchConfiguration('use_static_tf_broadcaster'),
    #     'tf_broadcaster_republish_rate' : LaunchConfiguration('tf_broadcaster_republish_rate')#only used if > 0
    
    # }

    declare_squares_sides_size_in_meters = DeclareLaunchArgument('squares_sides_size_in_meters', default_value=squares_sides_size_in_meters),
    declare_markers_sides_size_in_meters = DeclareLaunchArgument('markers_sides_size_in_meters', default_value=markers_sides_size_in_meters),
    # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default='0.0248')
    # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default='0.0155')
    declare_number_of_squares_in_x = DeclareLaunchArgument('number_of_squares_in_x', default_value=number_of_squares_in_x),
    declare_number_of_squares_in_y = DeclareLaunchArgument('number_of_squares_in_y', default_value=number_of_squares_in_y),
    declare_number_of_markers = DeclareLaunchArgument('number_of_markers', default_value=number_of_markers),
    declare_number_of_bits_for_markers_sides = DeclareLaunchArgument('number_of_bits_for_markers_sides', default_value=number_of_bits_for_markers_sides),
    declare_dictionary_id = DeclareLaunchArgument('dictionary_id', default_value=dictionary_id),
    # Change the following 2 topics to match the ones published by your camera sensor
    declare_image_topic = DeclareLaunchArgument('image_topic', default_value=image_topic),
    declare_camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic),
    declare_image_analysis_publish_topic = DeclareLaunchArgument('image_analysis_publish_topic', default_value=image_analysis_publish_topic),
    declare_charuco_pose_publish_topic = DeclareLaunchArgument('charuco_pose_publish_topic', default_value=charuco_pose_publish_topic),
    # Change the charuco TF frame if you need poses to be estimated from several charuco boards
    declare_charuco_tf_frame = DeclareLaunchArgument('charuco_tf_frame', default_value=charuco_tf_frame),
    # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
    # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
    declare_sensor_frame_override = DeclareLaunchArgument('sensor_frame_override', default_value=sensor_frame_override),
    # Fine tune the yaml and parameters below if needed
    declare_config_file = DeclareLaunchArgument('config_file', default_value=config_file),
    declare_use_median_blur = DeclareLaunchArgument('use_median_blur', default_value=use_median_blur),
    declare_median_blur_k_size = DeclareLaunchArgument('median_blur_k_size', default_value=median_blur_k_size),  # must be odd number
    declare_use_dynamic_range = DeclareLaunchArgument('use_dynamic_range', default_value=use_dynamic_range),
    declare_use_bilateral_filter = DeclareLaunchArgument('use_bilateral_filter', default_value=use_bilateral_filter),
    declare_bilateral_filter_pixel_neighborhood = DeclareLaunchArgument('bilateral_filter_pixel_neighborhood', default_value=bilateral_filter_pixel_neighborhood),
    declare_bilateral_filter_sigma_color = DeclareLaunchArgument('bilateral_filter_sigma_color', default_value=bilateral_filter_sigma_color),
    declare_bilateral_filter_sigma_space = DeclareLaunchArgument('bilateral_filter_sigma_space', default_value=bilateral_filter_sigma_space),
    declare_bilateral_filter_border_type = DeclareLaunchArgument('bilateral_filter_border_type', default_value=bilateral_filter_border_type),  # cv::BORDER_DEFAUL
    declare_use_clahe = DeclareLaunchArgument('use_clahe', default_value=use_clahe),
    declare_clahe_clip_limit = DeclareLaunchArgument('clahe_clip_limit', default_value=clahe_clip_limit),
    declare_clahe_size_x = DeclareLaunchArgument('clahe_size_x', default_value=clahe_size_x),
    declare_clahe_size_y = DeclareLaunchArgument('clahe_size_y', default_value=clahe_size_y),
    declare_use_adaptive_threshold = DeclareLaunchArgument('use_adaptive_threshold', default_value=use_adaptive_threshold),
    declare_adaptive_threshold_max_value = DeclareLaunchArgument('adaptive_threshold_max_value', default_value=adaptive_threshold_max_value),
    declare_adaptive_threshold_method = DeclareLaunchArgument('adaptive_threshold_method', default_value=adaptive_threshold_method),#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
    declare_adaptive_threshold_type = DeclareLaunchArgument('adaptive_threshold_type', default_value=adaptive_threshold_type),#cv::THRESH_BINARY
    declare_adaptive_threshold_block_size = DeclareLaunchArgument('adaptive_threshold_block_size', default_value=adaptive_threshold_block_size),
    declare_adaptive_threshold_constant_offset_from_mean = DeclareLaunchArgument('adaptive_threshold_constant_offset_from_mean', default_value=adaptive_threshold_constant_offset_from_mean),
    declare_use_static_tf_broadcaster = DeclareLaunchArgument('use_static_tf_broadcaster', default_value=use_static_tf_broadcaster),
    declare_tf_broadcaster_republish_rate = DeclareLaunchArgument('tf_broadcaster_republish_rate', default_value=tf_broadcaster_republish_rate)#only used if > 0
    
    charuco_node = Node(
        package='charuco_detector',
        executable='charuco_detector_node',
        name='charuco_detector',
        output='screen'
        # parameters=[squares_sides_size_in_meters,
        #             markers_sides_size_in_meters,
        #             number_of_squares_in_x,
        #             number_of_squares_in_y,
        #             number_of_markers,
        #             number_of_bits_for_markers_sides,
        #             dictionary_id,
        #             image_topic,
        #             camera_info_topic,
        #             image_analysis_publish_topic,
        #             charuco_pose_publish_topic,
        #             charuco_tf_frame,
        #             sensor_frame_override,
        #             config_file,
        #             use_median_blur,
        #             median_blur_k_size,
        #             use_dynamic_range,
        #             use_bilateral_filter,
        #             bilateral_filter_pixel_neighborhood,
        #             bilateral_filter_sigma_color,
        #             bilateral_filter_sigma_space,
        #             bilateral_filter_border_type,
        #             use_clahe,
        #             clahe_clip_limit,
        #             clahe_size_x,
        #             clahe_size_y,
        #             use_adaptive_threshold,
        #             adaptive_threshold_max_value,
        #             adaptive_threshold_method,
        #             adaptive_threshold_type,
        #             adaptive_threshold_block_size,
        #             adaptive_threshold_constant_offset_from_mean,
        #             use_static_tf_broadcaster,
        #             tf_broadcaster_republish_rate]
        # parameters=[ros_param_file, network_param_file,
        #   {
        #     "config_path": yolo_config_path, 
        #     "weights_path": yolo_weights_path,
        #   },]
    )               
    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    # ld.add_action(declare_squares_sides_size_in_meters)
    # ld.add_action(declare_markers_sides_size_in_meters)
    # ld.add_action(declare_number_of_squares_in_x)
    # ld.add_action(declare_number_of_squares_in_y)
    # ld.add_action(declare_number_of_markers)
    # ld.add_action(declare_number_of_bits_for_markers_sides)
    # ld.add_action(declare_dictionary_id)
    # ld.add_action(declare_image_topic)
    # ld.add_action(declare_camera_info_topic)
    # ld.add_action(declare_image_analysis_publish_topic)
    # ld.add_action(declare_charuco_pose_publish_topic)
    # ld.add_action(declare_charuco_tf_frame)
    # ld.add_action(declare_sensor_frame_override)
    # ld.add_action(declare_config_file)
    # ld.add_action(declare_use_median_blur)
    # ld.add_action(declare_median_blur_k_size)
    # ld.add_action(declare_use_dynamic_range)
    # ld.add_action(declare_use_bilateral_filter)
    # ld.add_action(declare_bilateral_filter_pixel_neighborhood)
    # ld.add_action(declare_bilateral_filter_sigma_color)
    # ld.add_action(declare_bilateral_filter_sigma_space)
    # ld.add_action(declare_bilateral_filter_border_type)
    # ld.add_action(declare_use_clahe)
    # ld.add_action(declare_clahe_clip_limit)
    # ld.add_action(declare_clahe_size_x)
    # ld.add_action(declare_clahe_size_y)
    # ld.add_action(declare_use_adaptive_threshold)
    # ld.add_action(declare_adaptive_threshold_max_value)
    # ld.add_action(declare_adaptive_threshold_method)
    # ld.add_action(declare_adaptive_threshold_type)
    # ld.add_action(declare_adaptive_threshold_block_size)
    # ld.add_action(declare_adaptive_threshold_constant_offset_from_mean)
    # ld.add_action(declare_use_static_tf_broadcaster)
    # ld.add_action(declare_tf_broadcaster_republish_rate)

    # Add the charuco_node to the launch description
    ld.add_action(charuco_node)

    return ld
    # return LaunchDescription([
    #     DeclareLaunchArgument('squares_sides_size_in_meters', default_value='0.0200'),
    #     DeclareLaunchArgument('markers_sides_size_in_meters', default_value='0.0150'),
    #     # squares_sides_size_in_meters = LaunchConfiguration('squares_sides_size_in_meters', default='0.0248')
    #     # markers_sides_size_in_meters = LaunchConfiguration('markers_sides_size_in_meters', default='0.0155')
    #     DeclareLaunchArgument('number_of_squares_in_x', default_value='10'),
    #     DeclareLaunchArgument('number_of_squares_in_y', default_value='14'),
    #     DeclareLaunchArgument('number_of_markers', default_value='70'),
    #     DeclareLaunchArgument('number_of_bits_for_markers_sides', default_value='4'),
    #     DeclareLaunchArgument('dictionary_id', default_value='3'),
    #     # Change the following 2 topics to match the ones published by your camera sensor
    #     DeclareLaunchArgument('image_topic', default_value='/rgb/image_raw'),
    #     DeclareLaunchArgument('camera_info_topic', default_value='/rgb/camera_info'),
    #     DeclareLaunchArgument('image_analysis_publish_topic', default_value=str(image_topic) + '_charuco_detection'),
    #     DeclareLaunchArgument('charuco_pose_publish_topic', default_value=str(image_topic) + '_charuco_pose'),
    #     # Change the charuco TF frame if you need poses to be estimated from several charuco boards
    #     DeclareLaunchArgument('charuco_tf_frame', default_value='calib_charuco'),
    #     # If necessary, the TF publisher of the charuco_detector can use a custom frame_id instead of using the one present in the header of the sensor_msgs/Image messages
    #     # If [sensor_frame_override] is empty, the TF publisher will use [ image_msg.header.frame_id -> charuco_tf_frame ], otherwise it will use [ sensor_frame_override -> charuco_tf_frame ]
    #     DeclareLaunchArgument('sensor_frame_override', default_value='calib_camera'),
    #     # Fine tune the yaml and parameters below if needed
    #     DeclareLaunchArgument('config_file', default_value='$(find charuco_detector)/yaml/charuco.yaml'),
    #     DeclareLaunchArgument('use_median_blur', default_value='false'),
    #     DeclareLaunchArgument('median_blur_k_size', default_value='3'),  # must be odd number
    #     DeclareLaunchArgument('use_dynamic_range', default_value='false'),
    #     DeclareLaunchArgument('use_bilateral_filter', default_value='false'),
    #     DeclareLaunchArgument('bilateral_filter_pixel_neighborhood', default_value='5'),
    #     DeclareLaunchArgument('bilateral_filter_sigma_color', default_value='100.0'),
    #     DeclareLaunchArgument('bilateral_filter_sigma_space', default_value='100.0'),
    #     DeclareLaunchArgument('bilateral_filter_border_type', default_value='4'),  # cv::BORDER_DEFAUL
    #     DeclareLaunchArgument('use_clahe', default_value='true'),
    #     DeclareLaunchArgument('clahe_clip_limit', default_value="1.0"),
    #     DeclareLaunchArgument('clahe_size_x', default_value="2"),
    #     DeclareLaunchArgument('clahe_size_y', default_value="2"),
    #     DeclareLaunchArgument('use_adaptive_threshold', default_value="false"),
    #     DeclareLaunchArgument('adaptive_threshold_max_value', default_value="255.0"),
    #     DeclareLaunchArgument('adaptive_threshold_method', default_value="1"),#cv::ADAPTIVE_THRESH_GAUSSIAN_C 
    #     DeclareLaunchArgument('adaptive_threshold_type', default_value="0"),#cv::THRESH_BINARY
    #     DeclareLaunchArgument('adaptive_threshold_block_size', default_value="65"),
    #     DeclareLaunchArgument('adaptive_threshold_constant_offset_from_mean', default_value="0.0"),
    #     DeclareLaunchArgument('use_static_tf_broadcaster', default_value="true"),
        
    #     Node(
    #         package='charuco_detector',
    #         executable='charuco_detector_node',
    #         name='charuco_detector',
    #         output="screen",
    #     ),
    # ])