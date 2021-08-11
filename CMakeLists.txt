cmake_minimum_required(VERSION 3.1)
project(jpgdec)

set(CMAKE_CXX_STANDARD 14)

add_executable(jpgdec main.cpp jpeg_decoder.cpp utils.c)

include_directories(/usr/local/include/rockchip/)

target_link_libraries(jpgdec pthread rockchip_mpp rockchip_vpu)

