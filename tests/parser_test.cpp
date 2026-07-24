#include "parser.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace {

std::filesystem::path write_temp_file(std::string_view filename, std::string_view content) {
    const auto path = std::filesystem::temp_directory_path() / filename;
    std::ofstream file(path);
    file << content;
    return path;
}

TEST(CameraYamlParserTest, ParsesCameraCalibration) {
    const auto path = write_temp_file(
        "ekf_slam_camera_sensor.yaml",
        R"(
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
)");

    const auto calibration = parse_camera_yaml(path);

    ASSERT_TRUE(calibration) << calibration.error();
    EXPECT_EQ(calibration->resolution.x(), 752);
    EXPECT_EQ(calibration->resolution.y(), 480);
    EXPECT_DOUBLE_EQ(calibration->rate_hz, 20.0);
    EXPECT_DOUBLE_EQ(calibration->t_bs(0, 3), 0.1);
    EXPECT_DOUBLE_EQ(calibration->t_bs(1, 3), 0.2);
    EXPECT_DOUBLE_EQ(calibration->t_bs(2, 3), 0.3);
    EXPECT_DOUBLE_EQ(calibration->intrinsics[0], 458.654);
    EXPECT_DOUBLE_EQ(calibration->distortion_coefficients[2], 0.00019359);
}

TEST(CameraYamlParserTest, RejectsBadTransformShape) {
    const auto path = write_temp_file(
        "ekf_slam_bad_camera_sensor.yaml",
        R"(
T_BS:
  cols: 3
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
)");

    const auto calibration = parse_camera_yaml(path);

    ASSERT_FALSE(calibration);
    EXPECT_EQ(calibration.error(), "T_BS must be 4x4, got 4x3");
}

TEST(ImuYamlParserTest, ParsesImuCalibration) {
    const auto path = write_temp_file(
        "ekf_slam_imu_sensor.yaml",
        R"(
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 200.0
gyroscope_noise_density: 0.00016968
gyroscope_random_walk: 1.9393e-05
accelerometer_noise_density: 0.002
accelerometer_random_walk: 0.003
)");

    const auto calibration = parse_imu_yaml(path);

    ASSERT_TRUE(calibration) << calibration.error();
    EXPECT_DOUBLE_EQ(calibration->rate_hz, 200.0);
    EXPECT_DOUBLE_EQ(calibration->t_bs(0, 3), 0.1);
    EXPECT_DOUBLE_EQ(calibration->gyroscope_noise_density, 0.00016968);
    EXPECT_DOUBLE_EQ(calibration->gyroscope_random_walk, 1.9393e-05);
    EXPECT_DOUBLE_EQ(calibration->accelerometer_noise_density, 0.002);
    EXPECT_DOUBLE_EQ(calibration->accelerometer_random_walk, 0.003);
}

TEST(ImuCsvParserTest, ParsesImuMeasurements) {
    const auto path = write_temp_file(
        "ekf_slam_imu_data.csv",
        R"(
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1403636579758555392,0.1,0.2,0.3,9.7,9.8,9.9
1403636579763555584,-0.1,-0.2,-0.3,-9.7,-9.8,-9.9
)");

    const auto measurements = parse_imu_measurements_csv(path);

    ASSERT_TRUE(measurements) << measurements.error();
    ASSERT_EQ(measurements->size(), 2);
    EXPECT_EQ((*measurements)[0].timestamp, 1403636579758555392);
    EXPECT_DOUBLE_EQ((*measurements)[0].angular_velocity.x(), 0.1);
    EXPECT_DOUBLE_EQ((*measurements)[0].angular_velocity.y(), 0.2);
    EXPECT_DOUBLE_EQ((*measurements)[0].angular_velocity.z(), 0.3);
    EXPECT_DOUBLE_EQ((*measurements)[0].acceleration.x(), 9.7);
    EXPECT_DOUBLE_EQ((*measurements)[0].acceleration.y(), 9.8);
    EXPECT_DOUBLE_EQ((*measurements)[0].acceleration.z(), 9.9);
    EXPECT_DOUBLE_EQ((*measurements)[1].angular_velocity.x(), -0.1);
}

TEST(ImuCsvParserTest, RejectsWrongFieldCount) {
    const auto path = write_temp_file(
        "ekf_slam_bad_imu_data.csv",
        R"(
# header
1403636579758555392,0.1,0.2
)");

    const auto measurements = parse_imu_measurements_csv(path);

    ASSERT_FALSE(measurements);
    EXPECT_EQ(measurements.error(), "IMU measurement line 3 must contain 7 fields, got 3");
}

TEST(GroundTruthCsvParserTest, ParsesGroundTruthStates) {
    const auto path = write_temp_file(
        "ekf_slam_groundtruth.csv",
        R"(
#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1],b_w_RS_S_x [rad s^-1],b_w_RS_S_y [rad s^-1],b_w_RS_S_z [rad s^-1],b_a_RS_S_x [m s^-2],b_a_RS_S_y [m s^-2],b_a_RS_S_z [m s^-2]
1403636579758555392,1.0,2.0,3.0,1.0,0.0,0.0,0.0,4.0,5.0,6.0,0.01,0.02,0.03,0.11,0.12,0.13
)");

    const auto states = parse_ground_truth_csv(path);

    ASSERT_TRUE(states) << states.error();
    ASSERT_EQ(states->size(), 1);
    EXPECT_EQ((*states)[0].timestamp, 1403636579758555392);
    EXPECT_DOUBLE_EQ((*states)[0].position.x(), 1.0);
    EXPECT_DOUBLE_EQ((*states)[0].position.y(), 2.0);
    EXPECT_DOUBLE_EQ((*states)[0].position.z(), 3.0);
    EXPECT_DOUBLE_EQ((*states)[0].orientation.w(), 1.0);
    EXPECT_DOUBLE_EQ((*states)[0].orientation.x(), 0.0);
    EXPECT_DOUBLE_EQ((*states)[0].velocity.x(), 4.0);
    EXPECT_DOUBLE_EQ((*states)[0].gyroscope_bias.y(), 0.02);
    EXPECT_DOUBLE_EQ((*states)[0].accelerometer_bias.z(), 0.13);
}

TEST(GroundTruthCsvParserTest, RejectsWrongFieldCount) {
    const auto path = write_temp_file(
        "ekf_slam_bad_groundtruth.csv",
        R"(
# header
1403636579758555392,1.0,2.0
)");

    const auto states = parse_ground_truth_csv(path);

    ASSERT_FALSE(states);
    EXPECT_EQ(states.error(), "ground truth state line 3 must contain 17 fields, got 3");
}

TEST(StereoPairCsvParserTest, ParsesStereoPairs) {
    const auto cam0_path = write_temp_file(
        "ekf_slam_cam0_data.csv",
        R"(
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
1403636579813555456,1403636579813555456.png
)");
    const auto cam1_path = write_temp_file(
        "ekf_slam_cam1_data.csv",
        R"(
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
1403636579813555456,1403636579813555456.png
)");

    const auto pairs = parse_stereo_pairs_csv(cam0_path, "mav0/cam0/data", cam1_path, "mav0/cam1/data");

    ASSERT_TRUE(pairs) << pairs.error();
    ASSERT_EQ(pairs->size(), 2);
    EXPECT_EQ((*pairs)[0].timestamp, 1403636579763555584);
    EXPECT_EQ((*pairs)[0].cam0_image_path, std::filesystem::path("mav0/cam0/data/1403636579763555584.png"));
    EXPECT_EQ((*pairs)[0].cam1_image_path, std::filesystem::path("mav0/cam1/data/1403636579763555584.png"));
}

TEST(StereoPairCsvParserTest, RejectsMismatchedTimestamps) {
    const auto cam0_path = write_temp_file(
        "ekf_slam_bad_cam0_data.csv",
        R"(
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
)");
    const auto cam1_path = write_temp_file(
        "ekf_slam_bad_cam1_data.csv",
        R"(
#timestamp [ns],filename
1403636579813555456,1403636579813555456.png
)");

    const auto pairs = parse_stereo_pairs_csv(cam0_path, "mav0/cam0/data", cam1_path, "mav0/cam1/data");

    ASSERT_FALSE(pairs);
    EXPECT_EQ(
        pairs.error(),
        "cam0 timestamp 1403636579763555584 does not match cam1 timestamp 1403636579813555456");
}

}  // namespace
