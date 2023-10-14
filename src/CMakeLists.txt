
include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

add_executable(${PROJECT_NAME} main.cpp System.cpp Util.cpp Physics.cpp Components.h Physics.h System.h Util.h)
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-window sfml-graphics)

add_subdirectory(tests)