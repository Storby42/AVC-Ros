cmake_minimum_required(VERSION 3.10)
project(avc_car_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/avc_car_localization
)


ament_package()
