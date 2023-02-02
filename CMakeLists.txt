cmake_minimum_required(VERSION 3.10)
project(csf_seg)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenMP)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  map4_util
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
set(SRCS
  src/csf_seg/c2cdist.cpp
  src/csf_seg/cloth.cpp
  src/csf_seg/constraint.cpp
  src/csf_seg/csf_seg.cpp
  src/csf_seg/particle.cpp
  src/csf_seg/point_cloud.cpp
  src/csf_seg/rasterization.cpp
)

add_library(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(pcd_ground_filter
        src/pcd_ground_filter.cpp
        )
target_link_libraries(pcd_ground_filter 
        csf_seg
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES}
        )
add_dependencies(pcd_ground_filter
        csf_seg
        ${catkin_EXPORTED_TARGETS}
        )