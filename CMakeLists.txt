cmake_minimum_required(VERSION 3.26)
project(DynaHex)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(DynaHex main.cpp
        include/dynaHex/core.h
        include/dynaHex/precision.h
        include/dynaHex/particle.h
        scr/particle.cpp)
