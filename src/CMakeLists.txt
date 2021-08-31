set(CSF_SOURCES
    c2cdist.cpp
    Cloth.cpp
    Constraint.cpp
    CSF.cpp
    Particle.cpp
    point_cloud.cpp
    Rasterization.cpp
    XYZReader.cpp
)

set(CSF_HEADERS
    c2cdist.h
    Cloth.h
    Constraint.h
    CSF.h
    Particle.h
    point_cloud.h
    Rasterization.h
    Vec3.h
    XYZReader.h
)

add_library(CSF ${CSF_SOURCES} ${CSF_HEADERS})
target_compile_options(CSF PRIVATE -Werror -Wall -Wextra)

if(OpenMP_CXX_FOUND)
    target_link_libraries(CSF PUBLIC OpenMP::OpenMP_CXX)
endif()

install(TARGETS CSF LIBRARY DESTINATION lib)
install(TARGETS CSF ARCHIVE DESTINATION lib)
install(FILES ${CSF_HEADERS} DESTINATION include)
