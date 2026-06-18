cmake_minimum_required(VERSION 3.16)
project(ellipse-d-config LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Use the system-wide sbgECom install (/usr/local/lib/cmake/sbgECom).
find_package(sbgECom REQUIRED)

add_executable(ellipse-d-config main.c)
target_link_libraries(ellipse-d-config PRIVATE sbgECom::sbgECom)
