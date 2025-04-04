cmake_minimum_required(VERSION 3.8)
project(follow_person)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  vision_msgs
  tf2_ros
  tf2_geometry_msgs
  yolo_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/follow_person/Detection2Tf.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(yolo_to_tf src/detection_2_tf_main.cpp)
ament_target_dependencies(yolo_to_tf  ${dependencies})
target_link_libraries(yolo_to_tf  ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} yolo_to_tf
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()