cmake_minimum_required(VERSION 3.26)
project(DynaHex)

set(CMAKE_CXX_STANDARD 17)

add_executable(DynaHex main.cpp
        include/dynaHex/core.h
        include/dynaHex/precision.h)
