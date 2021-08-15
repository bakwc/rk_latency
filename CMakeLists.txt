cmake_minimum_required(VERSION 3.1)
project(jpgdec)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O2")
set(CMAKE_FLAGS "-O2")

add_executable(jpgdec main.cpp jpeg_decoder.cpp utils.cpp)

include_directories(/usr/local/include/rockchip/)

target_link_libraries(jpgdec pthread rockchip_mpp rockchip_vpu)

