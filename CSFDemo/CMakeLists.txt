cmake_minimum_required(VERSION 3.10)

set(EXECUTABLE_OUTPUT_PATH ../bin)

add_executable(csfdemo CSFDemo.cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(csfdemo 
                        PUBLIC CSF OpenMP::OpenMP_CXX)
endif()