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

add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)

include_directories(
    ${ROSSY_UTILS_INCLUDE_DIR}
    ${YAML_INCLUDE_DIR}
    ${GOLDFARB_INCLUDE_DIR}
    ${SDLP_INCLUDE_DIR}
    ${CLP_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create the main library
add_library(libtoppra3
    src/toppra/topt_solver.cpp
    src/toppra/trajectory_manager.cpp
)

# Set include directories for the main library
target_include_directories(libtoppra3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${ROSSY_UTILS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

# Set compile options
target_compile_features(libtoppra3 PUBLIC cxx_std_17)

# Link dependencies
target_link_libraries(libtoppra3 PUBLIC
    Eigen3::Eigen rossy_utils ${libCoinUtils} ${libClp}
)

# Add python bindings
pybind11_add_module(_toppra3 
    bindings/python_bindings.cpp
)

# Link against required libraries
target_link_libraries(_toppra3 PRIVATE
    libtoppra3
    Eigen3::Eigen rossy_utils ${libCoinUtils} ${libClp}
)

# Set include directories for Python module
target_include_directories(_toppra3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
    ${ROSSY_UTILS_INCLUDE_DIR}
)

add_custom_target(generate_stub ALL
    COMMAND ${Python_EXECUTABLE} "-m" "pybind11_stubgen" "-o${CMAKE_CURRENT_BINARY_DIR}" "_toppra3"
    WORKING_DIRECTORY $<TARGET_FILE_DIR:_toppra3>
    DEPENDS _toppra3
)

# rename the generated .pyi file to __init__.pyi
add_custom_target(rename_pyi ALL
    COMMAND mv ${CMAKE_CURRENT_BINARY_DIR}/_toppra3.pyi ${CMAKE_CURRENT_BINARY_DIR}/__init__.pyi
    WORKING_DIRECTORY $<TARGET_FILE_DIR:_toppra3>
    DEPENDS generate_stub
)

# Installation rules
install(TARGETS rossy_utils LIBRARY DESTINATION toppra3/lib)
install(TARGETS _toppra3 LIBRARY DESTINATION toppra3)
install(DIRECTORY
    $<TARGET_FILE_DIR:_toppra3>/
    DESTINATION toppra3
    FILES_MATCHING REGEX "\.pyi$"
)

# Add the test file to your test executable
add_executable(test_parameterization 
    tests/test_parameterization.cpp
)

# Link against the main library
target_link_libraries(test_parameterization PRIVATE
    libtoppra3
)
