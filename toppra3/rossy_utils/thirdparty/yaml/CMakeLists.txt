cmake_minimum_required(VERSION 3.22.1)
project (yamlwrapper)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

message(${PROJECT_SOURCE_DIR})
message(${PROJECT_INCLUDE_DIR})

file(GLOB_RECURSE sources "*.cpp")
file(GLOB yaml_headers "include/*.h")
file(GLOB yaml_contrib_headers "include/contrib/*.h")
file(GLOB yaml_node_headers "include/yaml/node/*.h")
file(GLOB yaml_node_detail_headers "include/yaml/node/detail/*.h")

add_library(yamlwrapper STATIC ${sources})

target_include_directories(yamlwrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/yaml>)

# Make sure position independent code is used
set_target_properties(yamlwrapper PROPERTIES 
    POSITION_INDEPENDENT_CODE ON)

# Install yaml headers
install(DIRECTORY include/
        DESTINATION include/yaml
        FILES_MATCHING PATTERN "*.h")

# install(TARGETS yaml DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${yaml_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml")
# install(FILES ${yaml_contrib_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/contrib")
# install(FILES ${yaml_node_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/node")
# install(FILES ${yaml_node_detail_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/node/detail")
