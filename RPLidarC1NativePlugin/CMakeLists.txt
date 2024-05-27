cmake_minimum_required(VERSION 3.25)
project(RPLidarC1NativePlugin)

set(CMAKE_CXX_STANDARD 17)

if(MSVC)
    add_compile_options(
            $<$<CONFIG:>:/MT> #---------|
            $<$<CONFIG:Debug>:/MTd> #---|-- Statically link the runtime libraries
            $<$<CONFIG:Release>:/MT> #--|
    )
endif()

set(INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(${INCLUDE_PATH})
link_directories(${LIBRARY_PATH})

add_library(RPLidarC1NativePlugin SHARED library.cpp)

target_link_libraries(RPLidarC1NativePlugin rplidar_driver_x64.lib)
