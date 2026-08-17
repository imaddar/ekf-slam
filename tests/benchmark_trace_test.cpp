#include "benchmark_trace.hpp"

#include <gtest/gtest.h>

#include <fstream>

TEST(BenchmarkTraceTest, WritesReproducibleStateAndObservationRows) {
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "ekf_slam_benchmark_trace_test";
    std::filesystem::remove_all(directory);
    NominalState state{.position = {1.0, 2.0, 3.0}, .velocity = {4.0, 5.0, 6.0}, .orientation = Sophus::SO3d{},
        .accelerometer_bias = {0.1, 0.2, 0.3}, .gyroscope_bias = {0.4, 0.5, 0.6}};
    GroundTruthState truth{.timestamp = 100, .position = {7.0, 8.0, 9.0}, .orientation = Eigen::Quaterniond::Identity(),
        .velocity = {10.0, 11.0, 12.0}, .gyroscope_bias = Eigen::Vector3d::Zero(), .accelerometer_bias = Eigen::Vector3d::Zero()};
    const StereoObservation observation{.id = 42, .pixel_cam0 = {100.0, 200.0}, .pixel_cam1 = {90.0, 200.0}};
    const LandmarkUpdateDiagnostics diagnostic{.id = 42, .outcome = ObservationOutcome::kApplied, .mahalanobis_distance = 1.5,
        .innovation = {1.0, 2.0, 3.0, 4.0}};
    {
        auto writer = BenchmarkTraceWriter::create(directory, "test-revision", 100, 0.5);
        ASSERT_TRUE(writer) << writer.error();
        ASSERT_TRUE(writer->write_imu(100, 0.005, state, ImuStateCovariance::Identity(), truth));
        ASSERT_TRUE(writer->write_camera(100, state, ImuStateCovariance::Identity(), state, ImuStateCovariance::Identity(), 3, 1, 2, 1, truth));
        ASSERT_TRUE(writer->write_observations(100, std::span{&observation, 1}, std::span{&diagnostic, 1}));
    }
    std::ifstream metadata(directory / "metadata.json");
    std::ifstream camera(directory / "camera_trace.csv");
    std::ifstream observations(directory / "observation_trace.csv");
    std::string metadata_contents((std::istreambuf_iterator<char>(metadata)), {});
    std::string camera_contents((std::istreambuf_iterator<char>(camera)), {});
    std::string observation_contents((std::istreambuf_iterator<char>(observations)), {});
    EXPECT_NE(metadata_contents.find("test-revision"), std::string::npos);
    EXPECT_NE(camera_contents.find("prior_px"), std::string::npos);
    EXPECT_NE(camera_contents.find("truth_bax"), std::string::npos);
    EXPECT_NE(camera_contents.find("1,2,3"), std::string::npos);
    EXPECT_NE(observation_contents.find("99,198,87,196"), std::string::npos);
    std::filesystem::remove_all(directory);
}
