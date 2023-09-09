cmake_minimum_required(VERSION 3.25)
project(DynaHex)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(DynaHex main.cpp
        include/dynaHex/core.h
        include/dynaHex/precision.h
        include/dynaHex/particle.h
        scr/particle.cpp
        scr/demos/Particles/Ballistic/BallisticDemo.cpp
        scr/demos/Particles/Ballistic/BallisticDemo.h
        scr/demos/Application.h
        scr/demos/Particles/Firework/Firework.cpp
        scr/demos/Particles/Firework/FireworkDemo.h
        scr/demos/Particles/Firework/FireworkDemo.cpp
        include/dynaHex/random.h
        scr/random.cpp
        scr/core.cpp
        scr/particle.cpp scr/particle.cpp include/dynaHex/pfgen.h scr/pfgen.cpp include/dynaHex/pcontacts.h scr/pcontacts.cpp include/dynaHex/plinks.h scr/plinks.cpp include/dynaHex/pworld.h scr/pworld.cpp
        include/dynaHex/body.h
        scr/body.cpp
        scr/fgen.cpp
        include/dynaHex/fgen.h)