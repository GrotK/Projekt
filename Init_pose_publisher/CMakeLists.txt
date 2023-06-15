cmake_minimum_required(VERSION 3.8)
project(Init_pose_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(publish_init_pose_exe src/publish_init_pose_main.cpp)
add_executable(publish_transform_exe src/publish_transform.cpp)

target_include_directories(publish_init_pose_exe PRIVATE
  ${geometry_msgs_INCLUDE_DIRS})
  target_include_directories(publish_transform_exe PRIVATE
  ${geometry_msgs_INCLUDE_DIRS})
ament_target_dependencies(publish_init_pose_exe 
  rclcpp
  geometry_msgs)
  ament_target_dependencies(publish_transform_exe
  rclcpp
  geometry_msgs
  tf2_ros)
#install cpp executables
install(TARGETS
publish_init_pose_exe
publish_transform_exe
DESTINATION lib/${PROJECT_NAME})

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
