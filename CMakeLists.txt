cmake_minimum_required(VERSION 3.20)

project(ekf_slam_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_library(ekf_slam_parser
    parser.cpp
)

target_include_directories(ekf_slam_parser
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ekf_slam_parser
    PUBLIC
        Eigen3::Eigen
)

target_compile_options(ekf_slam_parser
    PRIVATE
        -Wall
        -Wextra
        -Werror
)

include(CTest)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(parser_tests
        tests/parser_test.cpp
    )

    target_link_libraries(parser_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(parser_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    include(GoogleTest)
    gtest_discover_tests(parser_tests)
endif()
