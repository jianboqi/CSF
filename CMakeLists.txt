cmake_minimum_required(VERSION 3.10)
project(CSF LANGUAGES CXX)

find_package(OpenMP)

add_subdirectory(src)

option (BUILD_DEMO "Build csfdemo" ON)

if (BUILD_DEMO)
  add_subdirectory (CSFDemo)
endif()
