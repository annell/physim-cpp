
include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

add_executable(${PROJECT_NAME} main.cpp System.cpp Util.cpp Physics.cpp Components.h Physics.h System.h Util.h Controls.cpp
        PhysimCpp.h)
target_link_libraries(${PROJECT_NAME} PRIVATE sfml-window sfml-graphics ecs-cpp octree-cpp SFMLMath)
set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_WARNING_AS_ERROR ON)
target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

add_subdirectory(tests)