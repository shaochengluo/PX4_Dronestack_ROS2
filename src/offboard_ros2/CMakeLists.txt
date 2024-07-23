cmake_minimum_required(VERSION 3.8)
project(offboard_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(gazebo_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



# Specify C++17 standard
add_compile_options(-std=c++17)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executables
add_executable(offboard_ctrl_ros2 src/offboard_ctrl_ros2.cpp)
ament_target_dependencies(offboard_ctrl_ros2 rclcpp std_msgs geometry_msgs nav_msgs mavros_msgs tf2 tf2_ros tf2_msgs Eigen3)

# gazebo_msgs

# Install targets
install(TARGETS offboard_ctrl_ros2
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/offboard_ros2/
  DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )







ament_package()
