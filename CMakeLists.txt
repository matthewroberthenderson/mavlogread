cmake_minimum_required(VERSION 3.31.6)
project(logreader_proj)

file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_subdirectory(raylib)
add_executable(logreader ${SOURCES})
target_link_libraries(logreader raylib)

target_include_directories(logreader PUBLIC ${CMAKE_SOURCE_DIR}/raylib/src)
target_include_directories(logreader PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(logreader PUBLIC ${CMAKE_SOURCE_DIR}/mavlink_headers)