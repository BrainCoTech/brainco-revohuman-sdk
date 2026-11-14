# Using the binary package from a plain C project:
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=<extracted directory>
#   cmake --build build && ./build/rh_info
cmake_minimum_required(VERSION 3.16)
project(rh_example LANGUAGES C)

find_package(revohuman 0.1 CONFIG REQUIRED)

add_executable(rh_info rh_info.c)
target_link_libraries(rh_info PRIVATE revohuman::revohuman)          # shared library

add_executable(rh_info_static rh_info.c)
target_link_libraries(rh_info_static PRIVATE revohuman::revohuman_static)  # static library, no .so needed
