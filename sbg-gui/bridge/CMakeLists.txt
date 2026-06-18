cmake_minimum_required(VERSION 3.16)
project(sbg-bridge LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(sbgECom REQUIRED)

add_executable(sbg-bridge main.c)
target_link_libraries(sbg-bridge PRIVATE sbgECom::sbgECom)
