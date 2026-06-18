cmake_minimum_required(VERSION 3.16)
project(imu-serial-reader LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Locate the system-wide sbgECom installation (/usr/local/lib/cmake/sbgECom).
find_package(sbgECom REQUIRED)

add_executable(imu-serial-reader main.c)

# The imported target carries its include dirs and compile definitions.
target_link_libraries(imu-serial-reader PRIVATE sbgECom::sbgECom)
