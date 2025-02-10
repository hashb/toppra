cmake_minimum_required(VERSION 3.22.1)
project(clpwrapper)

find_package(Eigen3)

FILE(GLOB_RECURSE headers "*.hpp")
FILE(GLOB_RECURSE sources *.cpp)

add_library(clpwrapper STATIC ${sources})

target_include_directories(clpwrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

# Make sure position independent code is used
set_target_properties(clpwrapper PROPERTIES 
    POSITION_INDEPENDENT_CODE ON)

# Install clp headers
install(FILES ${headers}
        DESTINATION include/clp)
