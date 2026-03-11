cmake_minimum_required(VERSION 3.10)
project(NES-emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(nes_core_sources src/bus.cpp src/cpu.cpp)

add_executable(run main.cpp ${nes_core_sources})
target_include_directories(run PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(test klaus_test.cpp ${nes_core_sources})
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/include)
