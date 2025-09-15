
cmake_minimum_required(VERSION 3.8)
project(vodom_first)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)



add_executable(vodom_first_node src/vodom_first_node.cpp)
add_executable(listener_node src/listener_node.cpp)
add_executable(visual_odometry_node src/visual_odometry_node.cpp)
add_executable(visual_odo_Uli src/visual_odo_Uli.cpp)
add_executable(ground_truth_path_node src/ground_truth_path_node.cpp)
add_executable(minimal_path_publisher src/minimal_path_publisher.cpp)

ament_target_dependencies(vodom_first_node rclcpp std_msgs)
ament_target_dependencies(listener_node rclcpp std_msgs)
ament_target_dependencies(visual_odometry_node rclcpp std_msgs nav_msgs geometry_msgs)
ament_target_dependencies(visual_odo_Uli rclcpp std_msgs nav_msgs geometry_msgs)
ament_target_dependencies(ground_truth_path_node rclcpp std_msgs nav_msgs geometry_msgs)
ament_target_dependencies(minimal_path_publisher rclcpp std_msgs nav_msgs geometry_msgs)

target_link_libraries(visual_odometry_node ${OpenCV_LIBS})
target_link_libraries(visual_odo_Uli ${OpenCV_LIBS})

install(TARGETS 
  vodom_first_node
  listener_node
  visual_odometry_node
  visual_odo_Uli
  ground_truth_path_node
  minimal_path_publisher
  DESTINATION lib/${PROJECT_NAME}
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
