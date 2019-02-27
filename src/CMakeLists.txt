cmake_minimum_required(VERSION 2.8)
project(CSF)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

file(GLOB_RECURSE sources *.cpp)

ADD_LIBRARY(CSF STATIC ${sources})

set_target_properties(CSF PROPERTIES OUTPUT_NAME "CSF")

set(LIBRARY_OUTPUT_PATH ../lib)

install(TARGETS CSF LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

install(FILES CSF.h DESTINATION include)