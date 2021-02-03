cmake_minimum_required(VERSION 3.17)
project(JNP1_7)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)

add_executable(JNP1_7 bezier_example.cc)