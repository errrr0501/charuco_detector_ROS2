/**\file charuco_detector.cpp
 * \brief Detector of ChArUco patterns
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "charuco_detector/charuco_detector.hpp"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// namespace charuco_detector {
ChArUcoDetector::ChArUcoDetector() 
	: Node("charuco_detector")
	{
  		RCLCPP_INFO(get_logger(), "[ChArUcoDetector] Node started.");
	static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	file_path_ = __FILE__;
	file_path_.erase(file_path_.size() - 24);  // find package location
	file_path_ = file_path_ + std::string("config/camera_calibration/camera_calibration.ini");

	declare_parameter("charuco_parameters.adaptiveThreshWinSizeMin", 3);
	declare_parameter("charuco_parameters.adaptiveThreshWinSizeMax", 23);
	declare_parameter("charuco_parameters.adaptiveThreshWinSizeStep", 10);
	declare_parameter("charuco_parameters.adaptiveThreshConstant", 7.0);
	declare_parameter("charuco_parameters.minMarkerPerimeterRate", 0.03);
	declare_parameter("charuco_parameters.maxMarkerPerimeterRate", 4.0);
	declare_parameter("charuco_parameters.polygonalApproxAccuracyRate", 0.03);
	declare_parameter("charuco_parameters.minCornerDistanceRate", 0.05);
	declare_parameter("charuco_parameters.minDistanceToBorder", 3);
	declare_parameter("charuco_parameters.minMarkerDistanceRate", 0.05);
	//private_node_handle_->param("charuco/cornerRefinementMethod", detector_parameters_->cornerRefinementMethod, 0);
	declare_parameter("charuco_parameters.cornerRefinementWinSize", 5);
	declare_parameter("charuco_parameters.cornerRefinementMaxIterations", 30);
	declare_parameter("charuco_parameters.cornerRefinementMinAccuracy", 0.1);
	declare_parameter("charuco_parameters.markerBorderBits", 1);
	declare_parameter("charuco_parameters.perspectiveRemovePixelPerCell", 4);
	declare_parameter("charuco_parameters.perspectiveRemoveIgnoredMarginPerCell", 0.13);
	declare_parameter("charuco_parameters.maxErroneousBitsInBorderRate", 0.35);
	declare_parameter("charuco_parameters.minOtsuStdDev", 5.0);
	declare_parameter("charuco_parameters.errorCorrectionRate", 0.6);

	declare_parameter("charuco_parameters.squaresSidesSizeM", 0.0200);
	declare_parameter("charuco_parameters.markersSidesSizeM", 0.0150);
	declare_parameter("charuco_parameters.numberOfBitsForMarkersSides", 6);
	declare_parameter("charuco_parameters.numberOfMarkers", 70);
	declare_parameter("charuco_parameters.numberOfSquaresInX", 10);
	declare_parameter("charuco_parameters.numberOfSquaresInY", 14);
	declare_parameter("charuco_parameters.dictionaryId", 10);

	declare_parameter("function_parameters.use_median_blur", false);
	declare_parameter("function_parameters.median_blur_k_size", 3);

	declare_parameter("function_parameters.use_dynamic_range", true);

	declare_parameter("function_parameters.use_bilateral_filter", false);
	declare_parameter("function_parameters.bilateral_filter_pixel_neighborhood", 5);
	declare_parameter("function_parameters.bilateral_filter_sigma_color", 100.0);
	declare_parameter("function_parameters.bilateral_filter_sigma_space", 100.0);
	declare_parameter("function_parameters.bilateral_filter_border_type", (int)cv::BORDER_DEFAULT);

	declare_parameter("function_parameters.use_clahe", true);
	declare_parameter("function_parameters.clahe_clip_limit", 4.0);
	declare_parameter("function_parameters.clahe_size_x", 2);
	declare_parameter("function_parameters.clahe_size_y", 2);

	declare_parameter("function_parameters.use_adaptive_threshold", false);
	declare_parameter("function_parameters.adaptive_threshold_max_value", 255.0);
	declare_parameter("function_parameters.adaptive_threshold_method", (int)cv::ADAPTIVE_THRESH_GAUSSIAN_C);
	declare_parameter("function_parameters.adaptive_threshold_type", (int)cv::THRESH_BINARY);
	declare_parameter("function_parameters.adaptive_threshold_block_size", 65);
	declare_parameter("function_parameters.adaptive_threshold_constant_offset_from_mean", 0.0);

	declare_parameter("tf_parameters.use_static_tf_broadcaster", false);
	declare_parameter("tf_parameters.tf_broadcaster_republish_rate", 10.0);
	transform_stamped_valid_ = false;

	declare_parameter("tf_parameters.sensor_frame_override", std::string(""));
	declare_parameter("tf_parameters.charuco_tf_frame", std::string("charuco"));
	declare_parameter("subscribers.image_topic.topic", std::string("image_raw"));
	declare_parameter("subscribers.camera_info.topic", std::string("camera_info"));
	declare_parameter("publishers.image_results_publisher.queue_size", 1);
	declare_parameter("publishers.image_results_publisher.latch", false);
	}

ChArUcoDetector::~ChArUcoDetector() {}
void ChArUcoDetector::setupConfigurationFromParameterServer() {

	detector_parameters_ = cv::aruco::DetectorParameters::create();
	get_parameter("charuco_parameters.adaptiveThreshWinSizeMin", detector_parameters_->adaptiveThreshWinSizeMin);
	get_parameter("charuco_parameters.adaptiveThreshWinSizeMax", detector_parameters_->adaptiveThreshWinSizeMax);
	get_parameter("charuco_parameters.adaptiveThreshWinSizeStep", detector_parameters_->adaptiveThreshWinSizeStep);
	get_parameter("charuco_parameters.adaptiveThreshConstant", detector_parameters_->adaptiveThreshConstant);
	get_parameter("charuco_parameters.minMarkerPerimeterRate", detector_parameters_->minMarkerPerimeterRate);
	get_parameter("charuco_parameters.maxMarkerPerimeterRate", detector_parameters_->maxMarkerPerimeterRate);
	get_parameter("charuco_parameters.polygonalApproxAccuracyRate", detector_parameters_->polygonalApproxAccuracyRate);
	get_parameter("charuco_parameters.minCornerDistanceRate", detector_parameters_->minCornerDistanceRate);
	get_parameter("charuco_parameters.minDistanceToBorder", detector_parameters_->minDistanceToBorder);
	get_parameter("charuco_parameters.minMarkerDistanceRate", detector_parameters_->minMarkerDistanceRate);
	//private_node_handle_->param("charuco/cornerRefinementMethod", detector_parameters_->cornerRefinementMethod, 0);
	get_parameter("charuco_parameters.cornerRefinementWinSize", detector_parameters_->cornerRefinementWinSize);
	get_parameter("charuco_parameters.cornerRefinementMaxIterations", detector_parameters_->cornerRefinementMaxIterations);
	get_parameter("charuco_parameters.cornerRefinementMinAccuracy", detector_parameters_->cornerRefinementMinAccuracy);
	get_parameter("charuco_parameters.markerBorderBits", detector_parameters_->markerBorderBits);
	get_parameter("charuco_parameters.perspectiveRemovePixelPerCell", detector_parameters_->perspectiveRemovePixelPerCell);
	get_parameter("charuco_parameters.perspectiveRemoveIgnoredMarginPerCell", detector_parameters_->perspectiveRemoveIgnoredMarginPerCell);
	get_parameter("charuco_parameters.maxErroneousBitsInBorderRate", detector_parameters_->maxErroneousBitsInBorderRate);
	get_parameter("charuco_parameters.minOtsuStdDev", detector_parameters_->minOtsuStdDev);
	get_parameter("charuco_parameters.errorCorrectionRate", detector_parameters_->errorCorrectionRate);

	get_parameter("charuco_parameters.squaresSidesSizeM", squares_sides_size_m_);
	get_parameter("charuco_parameters.markersSidesSizeM", markers_sides_size_m_);
	get_parameter("charuco_parameters.numberOfBitsForMarkersSides", number_of_bits_for_markers_sides_);
	get_parameter("charuco_parameters.numberOfMarkers", number_of_markers_);
	get_parameter("charuco_parameters.numberOfSquaresInX", number_of_squares_in_x_);
	get_parameter("charuco_parameters.numberOfSquaresInY", number_of_squares_in_y_);
	get_parameter("charuco_parameters.dictionaryId", dictionary_id_);

	get_parameter("function_parameters.use_median_blur",  use_median_blur_);
	get_parameter("function_parameters.median_blur_k_size",  median_blur_k_size_);

	get_parameter("function_parameters.use_dynamic_range",  use_dynamic_range_);

	get_parameter("function_parameters.use_bilateral_filter",  use_bilateral_filter_);
	get_parameter("function_parameters.bilateral_filter_pixel_neighborhood",  bilateral_filter_pixel_neighborhood_);
	get_parameter("function_parameters.bilateral_filter_sigma_color", bilateral_filter_sigma_color_);
	get_parameter("function_parameters.bilateral_filter_sigma_space", bilateral_filter_sigma_space_);
	get_parameter("function_parameters.bilateral_filter_border_type", bilateral_filter_border_type_);

	get_parameter("function_parameters.use_clahe",  use_clahe_);
	get_parameter("function_parameters.clahe_clip_limit",  clahe_clip_limit_);
	get_parameter("function_parameters.clahe_size_x", clahe_size_x_);
	get_parameter("function_parameters.clahe_size_y", clahe_size_y_);

	get_parameter("function_parameters.use_adaptive_threshold", use_adaptive_threshold_);
	get_parameter("function_parameters.adaptive_threshold_max_value", adaptive_threshold_max_value_);
	get_parameter("function_parameters.adaptive_threshold_method", adaptive_threshold_method_);
	get_parameter("function_parameters.adaptive_threshold_type", adaptive_threshold_type_);
	get_parameter("function_parameters.adaptive_threshold_block_size", adaptive_threshold_block_size_);
	get_parameter("function_parameters.adaptive_threshold_constant_offset_from_mean", adaptive_threshold_constant_offset_from_mean_);
	
	get_parameter("tf_parameters.use_static_tf_broadcaster", use_static_tf_broadcaster_);
	get_parameter("tf_parameters.tf_broadcaster_republish_rate", tf_broadcaster_republish_rate_);
	transform_stamped_valid_ = false;

	get_parameter("tf_parameters.sensor_frame_override", sensor_frame_override_);
	get_parameter("tf_parameters.charuco_tf_frame", charuco_tf_frame_);
	get_parameter("subscribers.image_topic.topic", image_topic_);
	get_parameter("subscribers.camera_info.topic", camera_info_topic_);
	get_parameter("publishers.image_results_publisher.queue_size", imageQueueSize);
	get_parameter("publishers.image_results_publisher.latch", imageLatch);
	image_results_publish_topic_ = image_topic_ + std::string("_charuco_detection");
	charuco_pose_publish_topic_ = image_topic_ + std::string("_charuco_pose");
	if (dictionary_id_ > 0)
		dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
	else
		dictionary_ = cv::aruco::generateCustomDictionary(number_of_markers_, number_of_bits_for_markers_sides_);
	board_ = cv::aruco::CharucoBoard::create(number_of_squares_in_x_, number_of_squares_in_y_,
											 static_cast<float>(squares_sides_size_m_), static_cast<float>(markers_sides_size_m_), dictionary_);
}
void ChArUcoDetector::startDetection() {

  	rclcpp::QoS image_publisher_qos(imageQueueSize);
  	if (imageLatch) {
  	  image_publisher_qos.transient_local();
  	}
	image_transport_ptr_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
	image_subscriber_ = image_transport_ptr_->subscribe(image_topic_, 10, std::bind(&ChArUcoDetector::imageCallback, 
					this, std::placeholders::_1));

	camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, image_publisher_qos, 
							std::bind(&ChArUcoDetector::cameraInfoCallback, this, std::placeholders::_1));
	image_transport_results_ptr_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

	image_results_publisher_ = image_transport_results_ptr_->advertise(image_results_publish_topic_,1);
	charuco_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(charuco_pose_publish_topic_, image_publisher_qos);
	if (tf_broadcaster_republish_rate_ > 0.0 && !use_static_tf_broadcaster_) {
		rclcpp::Rate tf_broadcaster_republish_rate(tf_broadcaster_republish_rate_);
		while (rclcpp::ok()) {
			if (transform_stamped_valid_) {
				tf_broadcaster_->sendTransform(transform_stamped_);
			}
			// rclcpp::spin_some(node_handle_);
			tf_broadcaster_republish_rate.sleep();
		}
	} else {
		return;
		// rclcpp::spin(node_handle_);
	}
}
void ChArUcoDetector::getCameraCalibrationCoefficient() {

	inih::INIReader Camera_KD{file_path_};

	if (Camera_KD.ParseError() != 0) {
    	std::cout << "Can't load 'camera_calibration.ini'\n";
    	return;
	}
	camera_intrinsics_matrix = cv::Mat::zeros(3, 3, CV_64F);
	camera_distortion_coefficients_matrix = cv::Mat::zeros(1, 5, CV_64F);

	distortion.push_back(std::stod(Camera_KD.Get("Distortion", "k1")));
	distortion.push_back(std::stod(Camera_KD.Get("Distortion", "k2")));
	distortion.push_back(std::stod(Camera_KD.Get("Distortion", "t1")));
	distortion.push_back(std::stod(Camera_KD.Get("Distortion", "t2")));
	distortion.push_back(std::stod(Camera_KD.Get("Distortion", "k3")));
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "0_0"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "0_1"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "0_2"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "1_0"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "1_1"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "1_2"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "2_0"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "2_1"))); 
	intrinsic.push_back(std::stod(Camera_KD.Get("Intrinsic", "2_2"))); 
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			camera_intrinsics_matrix.at<double>(i, j) = intrinsic[i * 3 + j];
		}
	}
	for (int i = 0; i < 5; i++) {
		camera_distortion_coefficients_matrix.at<double>(0, i) = distortion[i];
	}
}
void ChArUcoDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &_msg) {
	bool valid_camera_info = false;
	for (size_t i = 0; i < _msg->k.size(); ++i) {
		if (_msg->k[i] != 0.0) {
			valid_camera_info = true;
			break;
		}
	}
	if (valid_camera_info) {
		getCameraCalibrationCoefficient();
		camera_info_ = _msg;
		// camera_intrinsics_matrix = cv::Mat::zeros(3, 3, CV_64F);
		// camera_distortion_coefficients_matrix = cv::Mat::zeros(1, 5, CV_64F);
		// for (int i = 0; i < 3; i++) {
		// 	for (int j = 0; j < 3; j++) {
		// 		camera_intrinsics_matrix.at<double>(i, j) = _msg->K[i * 3 + j];
		// 	}
		// }
		// for (int i = 0; i < 5; i++) {
		// 	camera_distortion_coefficients_matrix.at<double>(0, i) = _msg->D[i];
		// }
	} else {
		RCLCPP_WARN(get_logger(),"Received invalid camera intrinsics (K all zeros)");
	}
}
void ChArUcoDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &_msg) {

	if (camera_info_) {
		if (_msg->data.empty() || _msg->step == 0) {
			RCLCPP_WARN(get_logger(),"Discarded empty image");
			return;
		}
		cv::Mat image_grayscale;
		bool dynamic_range_applied = false;
		if ((_msg->encoding == sensor_msgs::image_encodings::MONO16 || use_dynamic_range_)) {
			try {
				image_grayscale = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO16)->image;
				if (use_median_blur_) applyMedianBlur(image_grayscale);
				applyDynamicRange(image_grayscale);
				dynamic_range_applied = true;
			} catch (...) {}
		}
		if (!dynamic_range_applied) {
			try {
				image_grayscale = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::MONO8)->image;
			} catch (cv_bridge::Exception& e) {
				try {
					cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(_msg);
					cv_bridge::CvtColorForDisplayOptions options;
					options.do_dynamic_scaling = true;
					options.colormap = -1;
					image_grayscale = cv_bridge::cvtColorForDisplay(image, sensor_msgs::image_encodings::MONO8, options)->image;
				} catch (cv_bridge::Exception& e) {
					RCLCPP_WARN_STREAM(get_logger(),"Caught exception when analyzing image: " << e.what());
					return;
				}
			}
		}
		if (!dynamic_range_applied && use_median_blur_) applyMedianBlur(image_grayscale);
		if (use_bilateral_filter_) applyBilateralFilter(image_grayscale);
		if (use_clahe_) applyCLAHE(image_grayscale);
		if (use_adaptive_threshold_) applyAdaptiveThreshold(image_grayscale);
		cv::Vec3d camera_rotation, camera_translation;
		cv::Mat image_results;
		if (detectChArUcoBoard(image_grayscale, camera_intrinsics_matrix, camera_distortion_coefficients_matrix, camera_rotation, camera_translation, image_results, true)) {
			geometry_msgs::msg::PoseStamped charuco_pose;
			charuco_pose.header = _msg->header;
			fillPose(camera_rotation, camera_translation, charuco_pose);
			charuco_pose_publisher_->publish(charuco_pose);
			transform_stamped_.header = _msg->header;
			if (!sensor_frame_override_.empty())
				transform_stamped_.header.frame_id = sensor_frame_override_;
			transform_stamped_.child_frame_id = charuco_tf_frame_;
			transform_stamped_.transform.translation.x = charuco_pose.pose.position.x;
			transform_stamped_.transform.translation.y = charuco_pose.pose.position.y;
			transform_stamped_.transform.translation.z = charuco_pose.pose.position.z;
			transform_stamped_.transform.rotation = charuco_pose.pose.orientation;
			transform_stamped_valid_ = true;
			if (use_static_tf_broadcaster_)
				static_tf_broadcaster_->sendTransform(transform_stamped_);
			else
				tf_broadcaster_->sendTransform(transform_stamped_);
			sensor_msgs::msg::Image::Ptr image_results_msg = cv_bridge::CvImage(_msg->header, "bgr8", image_results).toImageMsg();
			image_results_publisher_.publish(image_results_msg);
		} else {
			sensor_msgs::msg::Image::Ptr image_filtered_msg = cv_bridge::CvImage(_msg->header, "mono8", image_grayscale).toImageMsg();
			image_results_publisher_.publish(image_filtered_msg);
		}
	} 
	else {
		RCLCPP_WARN(get_logger(),"Discarded image because a valid CameraInfo was not received yet");
	}
}
void ChArUcoDetector::applyMedianBlur(cv::Mat &image_in_out_) {
	cv::Mat temp;
	cv::medianBlur(image_in_out_, temp, median_blur_k_size_);
	image_in_out_ = temp;
}
void ChArUcoDetector::applyDynamicRange(cv::Mat& image_in_out_) {
	double min = 0;
	double max = 65535;
	cv::minMaxLoc(image_in_out_, &min, &max);
	cv::Mat temp = cv::Mat(image_in_out_ - min);
	temp.convertTo(image_in_out_, CV_8UC1, 255 / (max - min));
}
void ChArUcoDetector::applyBilateralFilter(cv::Mat &image_in_out_) {
	cv::Mat temp;
	cv::bilateralFilter(image_in_out_, temp, bilateral_filter_pixel_neighborhood_, bilateral_filter_sigma_color_, bilateral_filter_sigma_space_, bilateral_filter_border_type_);
	image_in_out_ = temp;
}
void ChArUcoDetector::applyCLAHE(cv::Mat &image_in_out_) {
	auto clahe = cv::createCLAHE(clahe_clip_limit_, cv::Size(clahe_size_x_, clahe_size_y_));
	cv::Mat temp;
	clahe->apply(image_in_out_, temp);
	image_in_out_ = temp;
}
void ChArUcoDetector::applyAdaptiveThreshold(cv::Mat &image_in_out_) {
	cv::Mat temp;
	cv::adaptiveThreshold(image_in_out_, temp, adaptive_threshold_max_value_, adaptive_threshold_method_, adaptive_threshold_type_, adaptive_threshold_block_size_, adaptive_threshold_constant_offset_from_mean_);
	image_in_out_ = temp;
}
bool ChArUcoDetector::detectChArUcoBoard(const cv::Mat &_image_grayscale, const cv::Mat &_camera_intrinsics, const cv::Mat &_camera_distortion_coefficients,
										 cv::Vec3d &_camera_rotation_out, cv::Vec3d &_camera_translation_out,
										 cv::InputOutputArray _image_with_detection_results, bool _show_rejected_markers) {
	std::vector<int> _marker_ids, charuco_ids;
	std::vector<std::vector<cv::Point2f> > marker_corners, rejected_markers;
	std::vector<cv::Point2f> charuco_corners;
	cv::aruco::detectMarkers(_image_grayscale, dictionary_, marker_corners, _marker_ids, detector_parameters_, rejected_markers);
	cv::aruco::refineDetectedMarkers(_image_grayscale, board_, marker_corners, _marker_ids, rejected_markers, _camera_intrinsics, _camera_distortion_coefficients);
	int interpolatedCorners = 0;
	if (!_marker_ids.empty())
		interpolatedCorners = cv::aruco::interpolateCornersCharuco(marker_corners, _marker_ids, _image_grayscale, board_, charuco_corners, charuco_ids,
																   _camera_intrinsics, _camera_distortion_coefficients);

	bool valid_pose = false;
	if (_camera_intrinsics.total() != 0)
		valid_pose = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, board_, _camera_intrinsics, _camera_distortion_coefficients,
														 _camera_rotation_out, _camera_translation_out);
	if (_image_with_detection_results.needed()) {
		cv::cvtColor(_image_grayscale, _image_with_detection_results, cv::COLOR_GRAY2BGR);
		if (!_marker_ids.empty()) {
			cv::aruco::drawDetectedMarkers(_image_with_detection_results, marker_corners);
		}
		if (_show_rejected_markers && !rejected_markers.empty())
			cv::aruco::drawDetectedMarkers(_image_with_detection_results, rejected_markers, cv::noArray(), cv::Scalar(100, 0, 255));
		if (interpolatedCorners > 0) {
			cv::Scalar color(255, 0, 0);
			cv::aruco::drawDetectedCornersCharuco(_image_with_detection_results, charuco_corners, charuco_ids, color);
		}
	}
	if (valid_pose) {
		if (_image_with_detection_results.needed()) {
			float axisLength = 0.5f * (static_cast<float>(std::min(number_of_squares_in_x_, number_of_squares_in_y_) * (squares_sides_size_m_)));
			cv::aruco::drawAxis(_image_with_detection_results, _camera_intrinsics, _camera_distortion_coefficients, _camera_rotation_out, _camera_translation_out, axisLength);
		}
		return true;
	}
	return false;
}
void ChArUcoDetector::fillPose(const cv::Vec3d &_camera_rotation, const cv::Vec3d &_camera_translation, geometry_msgs::msg::PoseStamped &_pose_in_out) {
	cv::Mat rotation_matrix;
	cv::Rodrigues(_camera_rotation, rotation_matrix);
	Eigen::Matrix3d eigen_rotation_matrix;
	cv::cv2eigen(rotation_matrix, eigen_rotation_matrix);
	Eigen::Quaterniond q(eigen_rotation_matrix);
	_pose_in_out.pose.position.x = _camera_translation(0);
	_pose_in_out.pose.position.y = _camera_translation(1);
	_pose_in_out.pose.position.z = _camera_translation(2);
	_pose_in_out.pose.orientation.x = q.x();
	_pose_in_out.pose.orientation.y = q.y();
	_pose_in_out.pose.orientation.z = q.z();
	_pose_in_out.pose.orientation.w = q.w();
}

// }
