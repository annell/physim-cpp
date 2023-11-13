cmake_minimum_required(VERSION 3.21)
project(physim-cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

file(COPY resources/myfont.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/src)

add_subdirectory("thirdparty")
add_subdirectory("src")

