cmake_minimum_required(VERSION 3.0)

project(MotionControl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(./include)

add_subdirectory(./src)