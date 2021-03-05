project (ekf_test)
cmake_minimum_required(VERSION 2.8)

add_definitions(
    -std=c++11 -O3 -Wno-deprecated -v -ftime-report
    )

set(
    ekf_test_src
    src/ekf_test.cpp
    )

add_executable(
    ekf_test ${ekf_test_src} 
    )

target_link_libraries(ekf_test robotcontrol ekf_nav_ins ublox_linux Boost::system util)
