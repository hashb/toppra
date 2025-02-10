cmake_minimum_required(VERSION 3.15)
project(toppra3 VERSION 0.1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # Add this for shared library support

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)  # Added CONFIG keyword for better compatibility

# Create the main library
add_library(toppra3_lib
    src/toppra/topt_solver.cpp
    src/toppra/trajectory_manager.cpp
)

# Set include directories for the main library
target_include_directories(toppra3_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

# Set compile options
target_compile_features(toppra3_lib PUBLIC cxx_std_17)

# Link dependencies
target_link_libraries(toppra3_lib PUBLIC
    Eigen3::Eigen
)

# Add python bindings
pybind11_add_module(toppra3 
    bindings/python_bindings.cpp
)

# Link against required libraries
target_link_libraries(toppra3 PRIVATE
    toppra3_lib
    Eigen3::Eigen
)

# Set include directories for Python module
target_include_directories(toppra3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
)

# Installation rules
install(TARGETS toppra3
        LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
        RUNTIME DESTINATION ${PYTHON_SITE_PACKAGES})
