cmake_minimum_required(VERSION 3.20)

project(ekf_slam_cpp LANGUAGES CXX)

execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE EKF_SLAM_GIT_REVISION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT EKF_SLAM_GIT_REVISION)
    set(EKF_SLAM_GIT_REVISION "unknown")
endif()

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(ekf_slam_parser
    benchmark_trace.cpp
    parser_csv.cpp
    parser.cpp
    parser_yaml.cpp
    image.cpp
    image_io.cpp
    rectification.cpp
    image_pyramid.cpp
    corner_detector.cpp
    klt_tracker.cpp
    stereo_matcher.cpp
    feature_frontend.cpp
    evaluation.cpp
    augmentation_jacobians.cpp
    landmark_augmentation.cpp
    measurement_model.cpp
    measurement_update.cpp
    propagation.cpp
    slam_state.cpp
    stereo_geometry.cpp
    triangulation.cpp
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
    PRIVATE
        ${OpenCV_LIBS}
)

# OpenCV 5 split calib3d into calib/geometry; OpenCV 4 keeps calib3d.
if(TARGET opencv_calib)
    target_link_libraries(ekf_slam_parser PRIVATE opencv_calib opencv_geometry)
elseif(TARGET opencv_calib3d)
    target_link_libraries(ekf_slam_parser PRIVATE opencv_calib3d)
endif()

target_compile_options(ekf_slam_parser
    PRIVATE
        -Wall
        -Wextra
        -Werror
)

include(CTest)

# Loading OpenCV's runtime can exceed GoogleTest's default five-second
# post-build discovery timeout on a cold machine. Discover just before CTest.
set(CMAKE_GTEST_DISCOVER_TESTS_DISCOVERY_MODE PRE_TEST)

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

    add_executable(measurement_update_tests
        tests/measurement_update_test.cpp
    )

    target_link_libraries(measurement_update_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(measurement_update_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(measurement_update_tests)

    add_executable(augmentation_jacobians_tests
        tests/augmentation_jacobians_test.cpp
    )

    target_link_libraries(augmentation_jacobians_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(augmentation_jacobians_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(augmentation_jacobians_tests)

    add_executable(triangulation_tests
        tests/triangulation_test.cpp
    )

    target_link_libraries(triangulation_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(triangulation_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(triangulation_tests)

    add_executable(landmark_augmentation_tests
        tests/landmark_augmentation_test.cpp
    )

    target_link_libraries(landmark_augmentation_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(landmark_augmentation_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(landmark_augmentation_tests)

    add_executable(slam_integration_tests
        tests/slam_integration_test.cpp
    )

    target_link_libraries(slam_integration_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(slam_integration_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(slam_integration_tests)

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

    add_executable(corner_detector_tests
        tests/corner_detector_test.cpp
    )

    target_link_libraries(corner_detector_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(corner_detector_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    target_compile_definitions(corner_detector_tests
        PRIVATE
            EKF_SLAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    gtest_discover_tests(corner_detector_tests)

    add_executable(euroc_frontend_tests
        tests/euroc_frontend_test.cpp
    )

    target_link_libraries(euroc_frontend_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(euroc_frontend_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    target_compile_definitions(euroc_frontend_tests
        PRIVATE
            EKF_SLAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    gtest_discover_tests(euroc_frontend_tests)

    add_executable(evaluation_tests
        tests/evaluation_test.cpp
    )

    target_link_libraries(evaluation_tests
        PRIVATE
            ekf_slam_parser
            GTest::gtest_main
    )

    target_compile_options(evaluation_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
    )

    gtest_discover_tests(evaluation_tests)

    add_executable(benchmark_trace_tests
        tests/benchmark_trace_test.cpp
    )
    target_link_libraries(benchmark_trace_tests PRIVATE ekf_slam_parser GTest::gtest_main)
    target_compile_options(benchmark_trace_tests PRIVATE -Wall -Wextra -Werror)
    gtest_discover_tests(benchmark_trace_tests)
endif()

add_executable(mh01_benchmark
    mh01_benchmark.cpp
)

target_link_libraries(mh01_benchmark PRIVATE ekf_slam_parser)
target_compile_options(mh01_benchmark PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(mh01_benchmark PRIVATE
    EKF_SLAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    EKF_SLAM_GIT_REVISION="${EKF_SLAM_GIT_REVISION}"
)
