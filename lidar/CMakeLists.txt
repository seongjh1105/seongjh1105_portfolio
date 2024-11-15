cmake_minimum_required(VERSION 3.8)
project(lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)


add_executable(sampling src/sampling.cpp)
ament_target_dependencies(sampling rclcpp sensor_msgs pcl_conversions)
target_link_libraries(sampling ${PCL_LIBRARIES})

add_executable(ransac src/ransac.cpp)
ament_target_dependencies(ransac rclcpp sensor_msgs pcl_conversions)
target_link_libraries(ransac ${PCL_LIBRARIES})

add_executable(roi_straight src/roi_straight.cpp)
ament_target_dependencies(roi_straight rclcpp sensor_msgs PCL pcl_conversions)
target_link_libraries(roi_straight ${PCL_LIBRARIES})

add_executable(roi_curved src/roi_curved.cpp)
ament_target_dependencies(roi_curved rclcpp sensor_msgs PCL pcl_conversions)
target_link_libraries(roi_curved ${PCL_LIBRARIES})

add_executable(dbscan src/dbscan.cpp)
ament_target_dependencies(dbscan rclcpp sensor_msgs visualization_msgs PCL pcl_conversions)
target_link_libraries(dbscan ${PCL_LIBRARIES})

install(TARGETS
  sampling
  ransac
  roi_straight
  roi_curved
  dbscan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
