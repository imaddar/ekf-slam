cmake_minimum_required(VERSION 3.20)

project(ekf_slam_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen relies on optimization to vectorize and to inline its expression
# templates; an unoptimized build runs the filter and its tests ~50x slower.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

add_library(ekf_slam_parser
    parser_csv.cpp
    parser.cpp
    parser_yaml.cpp
    measurement_model.cpp
    propagation.cpp
    slam_state.cpp
    stereo_geometry.cpp
    synthetic.cpp
)

target_include_directories(ekf_slam_parser
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ekf_slam_parser
    PUBLIC
        Eigen3::Eigen
        Sophus::Sophus
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

    target_compile_definitions(parser_tests
        PRIVATE
            EKF_SLAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    include(GoogleTest)
    gtest_discover_tests(parser_tests)

    add_executable(state_tests
        tests/state_test.cpp
    )

    target_link_libraries(state_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(state_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(state_tests)

    add_executable(slam_state_tests
        tests/slam_state_test.cpp
    )

    target_link_libraries(slam_state_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(slam_state_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(slam_state_tests)

    add_executable(stereo_geometry_tests
        tests/stereo_geometry_test.cpp
    )

    target_link_libraries(stereo_geometry_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(stereo_geometry_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(stereo_geometry_tests)

    add_executable(measurement_model_tests
        tests/measurement_model_test.cpp
    )

    target_link_libraries(measurement_model_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(measurement_model_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(measurement_model_tests)

    add_executable(propagation_tests
        tests/propagation_test.cpp
    )

    target_link_libraries(propagation_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(propagation_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    target_compile_definitions(propagation_tests
        PRIVATE
            EKF_SLAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    gtest_discover_tests(propagation_tests)

    add_executable(synthetic_tests
        tests/synthetic_test.cpp
    )

    target_link_libraries(synthetic_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(synthetic_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(synthetic_tests)
endif()
