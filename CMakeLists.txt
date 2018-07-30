cmake_minimum_required(VERSION 2.8.3)
project(manos_capabilities)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  manos
  manos_description
  manos_vision
)
catkin_package()
