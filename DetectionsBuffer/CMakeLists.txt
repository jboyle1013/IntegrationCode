cmake_minimum_required(VERSION 3.27)
project(DetectionsBuffer)

set(CMAKE_CXX_STANDARD 17)

add_executable(DetectionsBuffer main.cpp
        DetectionsBuffer.cpp
        DetectionsBuffer.h
        DetectionsBuffer_Tests.c)
