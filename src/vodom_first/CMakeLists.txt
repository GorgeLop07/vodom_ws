
cmake_minimum_required(VERSION 3.8)
project(vodom_first)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Create only the executables we need
add_executable(visual_odo_simple src/visual_odo_simple.cpp)
add_executable(gt_pose_publisher src/gt_pose_publisher.cpp)
add_executable(odometry_to_path src/odometry_to_path.cpp)
add_executable(path_recorder src/path_recorder.cpp)

# Dependencies for visual odometry
ament_target_dependencies(visual_odo_simple 
  rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs 
  cv_bridge tf2 tf2_geometry_msgs tf2_ros)

# Link OpenCV to visual odometry
target_link_libraries(visual_odo_simple ${OpenCV_LIBS})

# Dependencies for GT publisher
ament_target_dependencies(gt_pose_publisher
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# Dependencies for utility nodes
ament_target_dependencies(odometry_to_path 
  rclcpp std_msgs nav_msgs geometry_msgs)

ament_target_dependencies(path_recorder
  rclcpp std_msgs nav_msgs geometry_msgs)

install(TARGETS 
  visual_odo_simple
  gt_pose_publisher
  odometry_to_path
  path_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)



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

ament_package()
