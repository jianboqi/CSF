project(CSF LANGUAGES CXX)

set (package_name "CSF")

set(LIBRARY_OUTPUT_PATH ../lib)

find_package(OpenMP)

add_library(CSF STATIC
${CMAKE_CURRENT_LIST_DIR}/c2cdist.cpp
${CMAKE_CURRENT_LIST_DIR}/c2cdist.h
${CMAKE_CURRENT_LIST_DIR}/Cloth.cpp
${CMAKE_CURRENT_LIST_DIR}/Cloth.h
${CMAKE_CURRENT_LIST_DIR}/Constraint.cpp
${CMAKE_CURRENT_LIST_DIR}/Constraint.h
${CMAKE_CURRENT_LIST_DIR}/CSF.cpp
${CMAKE_CURRENT_LIST_DIR}/CSF.h
${CMAKE_CURRENT_LIST_DIR}/Particle.cpp
${CMAKE_CURRENT_LIST_DIR}/Particle.h
${CMAKE_CURRENT_LIST_DIR}/point_cloud.cpp
${CMAKE_CURRENT_LIST_DIR}/point_cloud.h
${CMAKE_CURRENT_LIST_DIR}/Rasterization.cpp
${CMAKE_CURRENT_LIST_DIR}/Rasterization.h
${CMAKE_CURRENT_LIST_DIR}/Vec3.h
${CMAKE_CURRENT_LIST_DIR}/XYZReader.cpp
${CMAKE_CURRENT_LIST_DIR}/XYZReader.h
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(CSF PUBLIC OpenMP::OpenMP_CXX)
    target_compile_options(CSF PRIVATE -Werror -Wall -Wextra)
endif()

install(TARGETS CSF
        DESTINATION ../lib
)