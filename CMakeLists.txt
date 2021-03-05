project(ekf_gps_imu)
cmake_minimum_required(VERSION 2.8)

#set(GCC_COMPILATION_FLAGS "-DLINUX")
#set(CMAKE_CXX_FLAGS  "${GCC_COMPILATION_FLAGS}")

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/bin
    )

set(
    CMAKE_LIBRARY_OUTPUT_DIRECTORY 
    ${CMAKE_HOME_DIRECTORY}/lib
    )

set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/modules")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("${CMAKE_MODULE_PATH}")
message("${EIGEN3_INCLUDE_DIR}")

find_package(Boost 1.40 COMPONENTS system REQUIRED)

link_directories(
    /usr/local/lib
    /usr/lib
    /lib
	${CMAKE_HOME_DIRECTORY}/lib
    )

include_directories(
    /usr/include/
    ${CMAKE_HOME_DIRECTORY}/ekf_nav_ins/inc
    ${CMAKE_HOME_DIRECTORY}/Ublox_Linux_Library/inc
    ${CMAKE_HOME_DIRECTORY}/Ublox_Linux_Library/SparkFun_Ublox_Arduino_Library/src
    )

add_subdirectory(Ublox_Linux_Library)
add_subdirectory(ekf_nav_ins)
add_subdirectory(ekf_test)