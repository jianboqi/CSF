cmake_minimum_required(VERSION 3.10)
project(CSF LANGUAGES CXX)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    add_compile_definitions("CSF_USE_OPENMP")
endif()

add_subdirectory(src)

option(BUILD_DEMO "Build csfdemo executable" OFF)
set(BUILD_SHARED_LIBS "Build as shared library" OFF)

if(BUILD_DEMO)
  add_subdirectory(CSFDemo)
endif()
