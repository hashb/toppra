cmake_minimum_required(VERSION 3.15)
project(toppra3 VERSION 0.1.1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PYBIND11_FINDPYTHON ON)

# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# clp
find_library(libClp
            NAMES libClp.so libClp.dylib
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu /opt/homebrew/lib
            REQUIRED)

find_library(libCoinUtils
            NAMES libCoinUtils.so libCoinUtils.dylib
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu /opt/homebrew/lib
            REQUIRED)

include_directories(
    /opt/homebrew/include/coinutils
    /opt/homebrew/include/coinutils/coin
    /opt/homebrew/include/clp
    /opt/homebrew/include/clp/coin
)


include(FetchContent)

# Also requires one of: libbfd (gnu binutils), libdwarf, libdw (elfutils)
FetchContent_Declare(backward
    GIT_REPOSITORY https://github.com/bombela/backward-cpp
    GIT_TAG master  # or a version tag, such as v1.6
    SYSTEM          # optional, the Backward include directory will be treated as system directory
)
FetchContent_MakeAvailable(backward)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/toppra
    ${CMAKE_CURRENT_SOURCE_DIR}/src/toppra/math
)


# library files
file(GLOB TOPPRA_CORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/toppra/*.cpp")
file(GLOB TOPPRA_MATH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/toppra/math/*.cpp")
set(TOPPRA_LIB_FILES
    ${TOPPRA_CORE_SRC}
    ${TOPPRA_MATH_SRC}
)

# Create the main library
add_library(libtoppra3
    ${TOPPRA_LIB_FILES}
)

# Set include directories for the main library
target_include_directories(libtoppra3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

# Set compile options
target_compile_features(libtoppra3 PUBLIC cxx_std_17)

# Link dependencies
target_link_libraries(libtoppra3 PUBLIC
    Eigen3::Eigen ${libCoinUtils} ${libClp} Backward::Interface
)

# Add python bindings
pybind11_add_module(_toppra3 
    bindings/python_bindings.cpp
)

# Link against required libraries
target_link_libraries(_toppra3 PRIVATE
    libtoppra3
    Eigen3::Eigen ${libCoinUtils} ${libClp}
)

# Set include directories for Python module
target_include_directories(_toppra3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
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

target_include_directories(test_parameterization PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

# Link against the main library
target_link_libraries(test_parameterization PRIVATE
    libtoppra3 Backward::Interface
)
