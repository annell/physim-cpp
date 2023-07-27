cmake_minimum_required(VERSION 3.21)
project(physim-cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

add_subdirectory(thirdParty/ecs)
add_subdirectory("thirdParty/octree-cpp")

add_executable(${PROJECT_NAME} src/main.cpp src/System.h src/System.cpp src/Util.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE sfml-window sfml-graphics)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
if (WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}> COMMAND_EXPAND_LISTS)
endif()

install(TARGETS ${PROJECT_NAME})
