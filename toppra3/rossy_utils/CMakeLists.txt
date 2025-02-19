cmake_minimum_required(VERSION 3.22.1)
project(toppra)

set(CMAKE_BUILD_TYPE "Release")
set(THIRDPARTY_INCLUDE_DIR "thirdparty")

find_package(Eigen3)
add_subdirectory(thirdparty/clp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB general_headers "general/*.hpp")
FILE(GLOB io_headers "io/*.hpp")
FILE(GLOB math_headers "math/*.hpp" "*.h")

add_library(toppra  ${sources} ${general_headers} ${io_headers} ${math_headers})
target_link_libraries(toppra clpwrapper ${EIGEN_LIBRARIES} ${libCoinUtils} ${libClp})
target_include_directories(toppra PUBLIC   
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/${THIRDPARTY_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

message(status ${CMAKE_SOURCE_DIR})
message(status ${PROJECT_SOURCE_DIR})

# First install the dependencies
install(TARGETS clpwrapper
        EXPORT toppra-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

# Then install toppra
install(TARGETS toppra
        EXPORT toppra-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(EXPORT toppra-targets
        FILE toppra-targets.cmake
        NAMESPACE toppra::
        DESTINATION lib/cmake/toppra)

# Install header files
install(FILES ${general_headers} 
        DESTINATION include/utils/general)
install(FILES ${io_headers} 
        DESTINATION include/utils/io)
install(FILES ${math_headers} 
        DESTINATION include/utils/math)
