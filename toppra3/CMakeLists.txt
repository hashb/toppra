cmake_minimum_required(VERSION 3.15)
project(toppra3 VERSION 0.1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PYBIND11_FINDPYTHON ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# clp
find_library(libClp
            NAMES libClp.so
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu
            REQUIRED)

find_library(libCoinUtils
            libCoinUtils.so
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu
            REQUIRED)


add_subdirectory(rossy_utils)

##=============================================================================
## Add Subdirectory
##=============================================================================
set(ROSSY_UTILS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rossy_utils/include)
set(YAML_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rossy_utils/thirdparty/yaml/include)
set(GOLDFARB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rossy_utils/thirdparty)
set(SDLP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rossy_utils/thirdparty)
set(CLP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rossy_utils/thirdparty)

include_directories(
    ${ROSSY_UTILS_INCLUDE_DIR}
    ${YAML_INCLUDE_DIR}
    ${GOLDFARB_INCLUDE_DIR}
    ${SDLP_INCLUDE_DIR}
    ${CLP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the main library
add_library(toppra3_lib
    src/toppra/topt_solver.cpp
    src/toppra/trajectory_manager.cpp
)

# Set include directories for the main library
target_include_directories(toppra3_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${ROSSY_UTILS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# Set compile options
target_compile_features(toppra3_lib PUBLIC cxx_std_17)

# Link dependencies
target_link_libraries(toppra3_lib PUBLIC
    Eigen3::Eigen rossy_utils ${libCoinUtils} ${libClp}
)

# Add python bindings
pybind11_add_module(toppra3 
    bindings/python_bindings.cpp
)

# Link against required libraries
target_link_libraries(toppra3 PRIVATE
    toppra3_lib
    Eigen3::Eigen rossy_utils ${libCoinUtils} ${libClp}
)

# Set include directories for Python module
target_include_directories(toppra3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
    ${ROSSY_UTILS_INCLUDE_DIR}
)

# Installation rules
install(TARGETS toppra3 toppra3_lib
        LIBRARY DESTINATION toppra3
        RUNTIME DESTINATION toppra3)
