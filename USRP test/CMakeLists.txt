cmake_minimum_required(VERSION 3.16)
project(usrp_tx_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(UHD REQUIRED)

add_executable(usrp_tx_test usrp_tx_test.cpp)
if(TARGET UHD::uhd)
    target_link_libraries(usrp_tx_test PRIVATE UHD::uhd)
else()
    target_include_directories(usrp_tx_test PRIVATE ${UHD_INCLUDE_DIRS})
    target_link_libraries(usrp_tx_test PRIVATE ${UHD_LIBRARIES})
endif()
