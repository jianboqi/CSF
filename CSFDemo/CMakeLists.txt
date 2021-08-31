add_executable(csfdemo CSFDemo.cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(csfdemo PUBLIC CSF OpenMP::OpenMP_CXX)
endif()

install(TARGETS csfdemo DESTINATION bin)
