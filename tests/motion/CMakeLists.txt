cmake_minimum_required(VERSION 3.22)

# Standalone harness for the body motion core (src/spaces/box3d_motion_core.cpp): Box3D worlds and a port of Godot's
# CharacterBody3D::move_and_slide, no Godot. Build and run through tests/motion/run.cmd.
project(b3motion_test LANGUAGES C CXX)

set(BOX3D_DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
if(NOT DEFINED B3M_VALIDATE)
	set(B3M_VALIDATE ON)
endif()
set(BOX3D_VALIDATE ${B3M_VALIDATE} CACHE BOOL "" FORCE)

add_subdirectory(../../box3d box3d)

# B3M_CORE_DIR: another copy of the motion core to measure against (a directory holding box3d_motion_core.cpp/.hpp).
if(NOT DEFINED B3M_CORE_DIR)
	set(B3M_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/spaces)
endif()

add_executable(motion_test
	motion_test.cpp
	${B3M_CORE_DIR}/box3d_motion_core.cpp)
target_include_directories(motion_test PRIVATE ${B3M_CORE_DIR})
target_link_libraries(motion_test PRIVATE box3d::box3d)
target_compile_features(motion_test PRIVATE cxx_std_17)
