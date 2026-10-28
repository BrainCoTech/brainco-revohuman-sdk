# 纯 C 工程用二进制包：
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=<解压目录>
#   cmake --build build && ./build/rh_info
cmake_minimum_required(VERSION 3.16)
project(rh_example LANGUAGES C)

find_package(revohuman 0.1 CONFIG REQUIRED)

add_executable(rh_info rh_info.c)
target_link_libraries(rh_info PRIVATE revohuman::revohuman)          # 动态库

add_executable(rh_info_static rh_info.c)
target_link_libraries(rh_info_static PRIVATE revohuman::revohuman_static)  # 静态库，不依赖 .so
