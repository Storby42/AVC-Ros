cmake_minimum_required(VERSION 3.5)
project(pose_to_pose_with_covariance)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pose_to_pose_with_covariance src/pose_to_pose_with_covariance.cpp)
ament_target_dependencies(pose_to_pose_with_covariance rclcpp geometry_msgs)

install(TARGETS
  pose_to_pose_with_covariance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
