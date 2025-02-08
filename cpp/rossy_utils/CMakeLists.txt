cmake_minimum_required(VERSION 3.10)
project(rossy_utils)

set(CMAKE_BUILD_TYPE "Release")
set(THIRDPARTY_INCLUDE_DIR "thirdparty")

find_package(Eigen3)
add_subdirectory(thirdparty/yaml)
add_subdirectory(thirdparty/clp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})


FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB general_headers "general/*.hpp")
FILE(GLOB io_headers "io/*.hpp")
FILE(GLOB math_headers "math/*.hpp" "*.h")

add_library(rossy_utils SHARED  ${sources} ${general_headers} ${io_headers} ${math_headers})
target_link_libraries(rossy_utils yamlwrapper clpwrapper ${EIGEN_LIBRARIES} ${libCoinUtils} ${libClp})
target_include_directories(rossy_utils PUBLIC   
                      ${PROJECT_SOURCE_DIR}
                      ${EIGEN3_INCLUDE_DIR} 
                      ${THIRDPARTY_INCLUDE_DIR} )

message(status ${CMAKE_SOURCE_DIR})
message(status ${PROJECT_SOURCE_DIR})

# Add export configuration
install(TARGETS rossy_utils
        EXPORT rossy_utils-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(EXPORT rossy_utils-targets
        FILE rossy_utils-targets.cmake
        NAMESPACE rossy_utils::
        DESTINATION lib/cmake/rossy_utils)


# install(TARGETS rossy_utils DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${general_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils/General")
# install(FILES ${io_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/IO")
# install(FILES ${math_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/Math")
