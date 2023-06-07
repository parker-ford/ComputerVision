cmake_minimum_required(VERSION 3.14)
project(MVE)
add_executable(MVE Main.cpp)

find_package(OpenMVS)
if(OpenMVS_FOUND)
	include_directories(${OpenMVS_INCLUDE_DIRS})
	add_definitions(${OpenMVS_DEFINITIONS})
endif()

add_executable(your_project source_code.cpp)
target_link_libraries(your_project PRIVATE OpenMVS::MVS)