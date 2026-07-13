set(CSFCLI_SOURCES
    main.cpp
    ${CMAKE_SOURCE_DIR}/src/PCDReader.cpp
    ${CMAKE_SOURCE_DIR}/src/LASWriter.cpp
)

add_executable(csf_cli ${CSFCLI_SOURCES})

target_link_libraries(csf_cli PRIVATE CSF)

if(OpenMP_CXX_FOUND)
    target_link_libraries(csf_cli PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS csf_cli DESTINATION bin)
