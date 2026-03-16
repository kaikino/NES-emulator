cmake_minimum_required(VERSION 3.10)
project(NES-emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(SDL2 REQUIRED)

set(nes_core_sources src/apu.cpp src/bus.cpp src/cpu.cpp src/ines.cpp)
set(nes_run_sources ${nes_core_sources} src/ppu.cpp)

add_executable(run main.cpp ${nes_run_sources})
target_include_directories(run PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(run PRIVATE SDL2::SDL2)

add_executable(test klaus_test.cpp ${nes_run_sources})
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/include)
