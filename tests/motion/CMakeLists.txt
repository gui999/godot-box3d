cmake_minimum_required(VERSION 3.22)

# Standalone harness for the body motion core (src/spaces/box3d_motion_core.cpp): Box3D worlds and a port of Godot's
# CharacterBody3D::move_and_slide, no Godot. Build and run through tests/motion/run.cmd.
project(b3motion_test LANGUAGES C CXX)

set(BOX3D_DOUBLE_PRECISION OFF CACHE BOOL "" FORCE)
set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_VALIDATE ON CACHE BOOL "" FORCE)

add_subdirectory(../../box3d box3d)

add_executable(motion_test
	motion_test.cpp
	../../src/spaces/box3d_motion_core.cpp)
target_include_directories(motion_test PRIVATE ../../src/spaces)
target_link_libraries(motion_test PRIVATE box3d::box3d)
target_compile_features(motion_test PRIVATE cxx_std_17)
