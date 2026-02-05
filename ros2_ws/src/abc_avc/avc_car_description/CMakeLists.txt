cmake_minimum_required(VERSION 3.10)
project(avc_car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY avc_car/urdf avc_car/rviz
  DESTINATION share/${PROJECT_NAME}/avc_car
)


ament_package()
