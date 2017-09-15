cmake_minimum_required(VERSION 2.8)
project(CSF)

file(GLOB_RECURSE sources *.cpp)

ADD_LIBRARY(CSF STATIC ${sources})

set_target_properties(CSF PROPERTIES OUTPUT_NAME "CSF")

set(LIBRARY_OUTPUT_PATH ../lib)

install(TARGETS CSF LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

install(FILES CSF.h DESTINATION include)