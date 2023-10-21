include(FetchContent)
FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

find_package(GTest)
enable_testing()

add_executable(${PROJECT_NAME}_test
        PhysimTests.cpp
        ../System.cpp
        ../Util.cpp
        ../Physics.cpp
        ../Components.h
        ../Physics.h
        ../System.h
        ../Util.h
)
add_executable(${PROJECT_NAME}_Utiltest
        UtilTests.cpp
        ../System.cpp
        ../Util.cpp
        ../Physics.cpp
        ../Components.h
        ../Physics.h
        ../System.h
        ../Util.h
)
target_link_libraries(${PROJECT_NAME}_test GTest::gtest GTest::gtest_main sfml-graphics sfml-window)
target_link_libraries(${PROJECT_NAME}_Utiltest GTest::gtest GTest::gtest_main sfml-graphics sfml-window)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_test)