cmake_minimum_required(VERSION 3.8)
project(charuco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

find_package(charuco_detector_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)


set(executable_name charuco_detector)
set(library_name ${executable_name}_core)

# Add the include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)


# add_executable(charuco_detector_node 
#                 src/charuco_detector_node.cpp
#                 include/charuco_detector/charuco_detector.h)
add_library(${library_name} SHARED
  src/charuco_detector.cpp
  include/charuco_detector/charuco_detector.hpp
)
target_compile_features(${library_name} 
  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${LIBMODBUS_PATH}/src>
  $<BUILD_INTERFACE:${LIBMODBUS_PATH}>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS})


ament_target_dependencies(${library_name}
rclcpp 
Eigen3 
OpenCV 
cv_bridge 
std_msgs
geometry_msgs
sensor_msgs
image_transport
tf2_ros)


ament_export_dependencies(rclcpp)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
# add_library(charuco_detector_lib src/charuco_detector.cpp)


add_executable(charuco_detector_node 
                src/charuco_detector_node.cpp)

target_link_libraries(charuco_detector_node
                # charuco_detector
                ${library_name}
                ${Eigen_LIBS}
                ${OpenCV_LIBS}
                ${cv_bridge_LIBS}
                ${image_transport_LIBS}
                ${tf2_ros_LIBS}
                ${charuco_detector_INCLUDE_DIRS}

)
# Add any necessary dependencies
ament_target_dependencies(charuco_detector_node
${dependencies}
rclcpp 
Eigen3 
OpenCV 
cv_bridge 
std_msgs
geometry_msgs
sensor_msgs
image_transport
tf2_ros)

  

# target_include_directories(charuco_detector_node PUBLIC include)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)
# Install the executable
install(TARGETS charuco_detector_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# Install package.xml
install(FILES package.xml
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config yaml DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rclcpp geometry_msgs)
# ament_export_dependencies(rosidl_default_runtime)

ament_export_include_directories(
  include
)
ament_export_libraries(${library_name})


ament_export_dependencies(${dependencies})

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
ament_package()
