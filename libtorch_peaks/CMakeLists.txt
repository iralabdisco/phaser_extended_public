cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(phaser_peaks)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(phaser_peaks phaser_peaks.cpp)
target_link_libraries(phaser_peaks "${TORCH_LIBRARIES}")
set_property(TARGET phaser_peaks PROPERTY CXX_STANDARD 17)

