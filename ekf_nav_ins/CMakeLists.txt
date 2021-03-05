project (ekf_nav_ins)
cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("${CMAKE_MODULE_PATH}")
message("${EIGEN3_INCLUDE_DIR}")

add_definitions(
    -std=c++14 -O3 -Wno-deprecated -v -ftime-report
    )

link_directories(
    /usr/local/lib
    /usr/lib
    /lib
    )

#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
include_directories(
    ${CMAKE_HOME_DIRECTORY}/inc
    /usr/include/
    )

set(
    ekf_nav_ins_src
    src/ekfNavINS.cpp
    )

add_library(
    ekf_nav_ins SHARED ${ekf_nav_ins_src} 
    )

