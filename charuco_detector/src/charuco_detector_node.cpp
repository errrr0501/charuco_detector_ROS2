/**\file charuco_detector_node.cpp
 * \brief Main for the detector of ChArUco patterns
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "rclcpp/rclcpp.hpp"
#include "charuco_detector/charuco_detector.hpp"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
    // auto node_handle = std::make_shared<rclcpp::Node>("charuco_detector");
    // // auto private_node_handle = std::make_shared<rclcpp::Node>("charuco_detector", "~");
	// auto private_node_handle = std::make_shared<rclcpp::Node>("charuco_detector");

	// ChArUcoDetector chArUcoDetector;
	auto charuco_detector = std::make_shared<ChArUcoDetector>();
	// charuco_detector->init();
	charuco_detector->setupConfigurationFromParameterServer();
	charuco_detector->startDetection();
	// auto on_init = [&]() {
    // 	charuco_detector->startDetection();
	// };
	rclcpp::spin(charuco_detector);
    rclcpp::shutdown();
	return 0;
}
// ###################################################################################   </main>   #############################################################################
