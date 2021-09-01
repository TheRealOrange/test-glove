cmake_minimum_required(VERSION 3.20)
project(test_glove)

set(CMAKE_CXX_STANDARD 14)

add_executable(test_glove main.cpp)

if (UNIX)
    target_link_directories(test_glove PUBLIC lib/linux/Release)
endif()

if (WIN32)
    target_link_directories(test_glove PUBLIC lib/win/Release)
endif()

target_include_directories(test_glove PUBLIC incl)
target_link_libraries(test_glove SGCoreCpp)

