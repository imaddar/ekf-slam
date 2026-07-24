#include "parser.hpp"

#include <filesystem>
#include <fstream>
#include <string_view>

#include <gtest/gtest.h>

namespace {

void write_file(const std::filesystem::path& path, std::string_view content) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    file << content;
}

void write_euroc_sequence(
    const std::filesystem::path& sequence_root,
    std::string_view cam0_yaml,
    std::string_view cam1_yaml,
    std::string_view imu_yaml,
    std::string_view cam0_csv,
    std::string_view cam1_csv,
    std::string_view imu_csv,
    std::string_view ground_truth_csv) {
    std::filesystem::remove_all(sequence_root);
    write_file(sequence_root / "mav0/cam0/sensor.yaml", cam0_yaml);
    write_file(sequence_root / "mav0/cam1/sensor.yaml", cam1_yaml);
    write_file(sequence_root / "mav0/imu0/sensor.yaml", imu_yaml);
    write_file(sequence_root / "mav0/cam0/data.csv", cam0_csv);
    write_file(sequence_root / "mav0/cam1/data.csv", cam1_csv);
    write_file(sequence_root / "mav0/imu0/data.csv", imu_csv);
    write_file(sequence_root / "mav0/state_groundtruth_estimate0/data.csv", ground_truth_csv);
}

constexpr std::string_view kCameraYaml = R"(
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
)";

constexpr std::string_view kImuYaml = R"(
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 200.0
gyroscope_noise_density: 0.00016968
gyroscope_random_walk: 1.9393e-05
accelerometer_noise_density: 0.002
accelerometer_random_walk: 0.003
)";

constexpr std::string_view kCameraCsv = R"(
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
1403636579813555456,1403636579813555456.png
)";

constexpr std::string_view kImuCsv = R"(
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1403636579758555392,0.1,0.2,0.3,9.7,9.8,9.9
1403636579763555584,-0.1,-0.2,-0.3,-9.7,-9.8,-9.9
)";

constexpr std::string_view kGroundTruthCsv = R"(
#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1],b_w_RS_S_x [rad s^-1],b_w_RS_S_y [rad s^-1],b_w_RS_S_z [rad s^-1],b_a_RS_S_x [m s^-2],b_a_RS_S_y [m s^-2],b_a_RS_S_z [m s^-2]
1403636579758555392,1.0,2.0,3.0,1.0,0.0,0.0,0.0,4.0,5.0,6.0,0.01,0.02,0.03,0.11,0.12,0.13
)";

TEST(DatasetParserTest, ParsesEuRocSequenceDirectory) {
    const auto sequence_root = std::filesystem::temp_directory_path() / "ekf_slam_sequence";
    write_euroc_sequence(
        sequence_root,
        kCameraYaml,
        kCameraYaml,
        kImuYaml,
        kCameraCsv,
        kCameraCsv,
        kImuCsv,
        kGroundTruthCsv);

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_TRUE(dataset) << dataset.error();
    EXPECT_EQ(dataset->sequence_root, sequence_root);
    EXPECT_EQ(dataset->cam0_calibration.resolution.x(), 752);
    EXPECT_EQ(dataset->cam0_calibration.resolution.y(), 480);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.rate_hz, 20.0);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.t_bs(0, 3), 0.1);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.t_bs(1, 3), 0.2);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.t_bs(2, 3), 0.3);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.intrinsics[0], 458.654);
    EXPECT_DOUBLE_EQ(dataset->cam0_calibration.distortion_coefficients[2], 0.00019359);
    EXPECT_DOUBLE_EQ(dataset->cam1_calibration.rate_hz, 20.0);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.rate_hz, 200.0);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.gyroscope_noise_density, 0.00016968);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.gyroscope_random_walk, 1.9393e-05);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.accelerometer_noise_density, 0.002);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.accelerometer_random_walk, 0.003);
    ASSERT_EQ(dataset->stereo_pairs.size(), 2);
    EXPECT_EQ(dataset->stereo_pairs[0].timestamp, 1403636579763555584);
    EXPECT_EQ(dataset->stereo_pairs[0].cam0_image_path, sequence_root / "mav0/cam0/data/1403636579763555584.png");
    EXPECT_EQ(dataset->stereo_pairs[0].cam1_image_path, sequence_root / "mav0/cam1/data/1403636579763555584.png");
    ASSERT_EQ(dataset->imu_measurements.size(), 2);
    EXPECT_EQ(dataset->imu_measurements[0].timestamp, 1403636579758555392);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].angular_velocity.x(), 0.1);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].angular_velocity.y(), 0.2);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].angular_velocity.z(), 0.3);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].acceleration.x(), 9.7);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].acceleration.y(), 9.8);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[0].acceleration.z(), 9.9);
    EXPECT_DOUBLE_EQ(dataset->imu_measurements[1].angular_velocity.x(), -0.1);
    ASSERT_EQ(dataset->ground_truth_states.size(), 1);
    EXPECT_EQ(dataset->ground_truth_states[0].timestamp, 1403636579758555392);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].position.x(), 1.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].position.y(), 2.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].position.z(), 3.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].orientation.w(), 1.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].orientation.x(), 0.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].velocity.x(), 4.0);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].gyroscope_bias.y(), 0.02);
    EXPECT_DOUBLE_EQ(dataset->ground_truth_states[0].accelerometer_bias.z(), 0.13);
}

TEST(DatasetParserTest, RejectsBadCameraTransformShape) {
    const auto sequence_root = std::filesystem::temp_directory_path() / "ekf_slam_bad_camera_sequence";
    write_euroc_sequence(
        sequence_root,
        R"(
T_BS:
  cols: 3
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
)",
        kCameraYaml,
        kImuYaml,
        kCameraCsv,
        kCameraCsv,
        kImuCsv,
        kGroundTruthCsv);

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_FALSE(dataset);
    EXPECT_EQ(dataset.error(), "T_BS must be 4x4, got 4x3");
}

TEST(DatasetParserTest, RejectsWrongImuFieldCount) {
    const auto sequence_root = std::filesystem::temp_directory_path() / "ekf_slam_bad_imu_sequence";
    write_euroc_sequence(
        sequence_root,
        kCameraYaml,
        kCameraYaml,
        kImuYaml,
        kCameraCsv,
        kCameraCsv,
        R"(
# header
1403636579758555392,0.1,0.2
)",
        kGroundTruthCsv);

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_FALSE(dataset);
    EXPECT_EQ(dataset.error(), "IMU measurement line 3 must contain 7 fields, got 3");
}

TEST(DatasetParserTest, RejectsWrongGroundTruthFieldCount) {
    const auto sequence_root = std::filesystem::temp_directory_path() / "ekf_slam_bad_groundtruth_sequence";
    write_euroc_sequence(
        sequence_root,
        kCameraYaml,
        kCameraYaml,
        kImuYaml,
        kCameraCsv,
        kCameraCsv,
        kImuCsv,
        R"(
# header
1403636579758555392,1.0,2.0
)");

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_FALSE(dataset);
    EXPECT_EQ(dataset.error(), "ground truth state line 3 must contain 17 fields, got 3");
}

TEST(DatasetParserTest, RejectsMismatchedStereoTimestamps) {
    const auto sequence_root = std::filesystem::temp_directory_path() / "ekf_slam_bad_stereo_sequence";
    write_euroc_sequence(
        sequence_root,
        kCameraYaml,
        kCameraYaml,
        kImuYaml,
        R"(
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
)",
        R"(
#timestamp [ns],filename
1403636579813555456,1403636579813555456.png
)",
        kImuCsv,
        kGroundTruthCsv);

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_FALSE(dataset);
    EXPECT_EQ(
        dataset.error(),
        "cam0 timestamp 1403636579763555584 does not match cam1 timestamp 1403636579813555456");
}

TEST(DatasetParserTest, SmokeParsesMh01EasyDataset) {
    const auto sequence_root = std::filesystem::path(EKF_SLAM_SOURCE_DIR) / "datasets/machine_hall/MH_01_easy";
    ASSERT_TRUE(std::filesystem::exists(sequence_root))
        << "MH_01_easy dataset fixture is required for this smoke test: " << sequence_root;

    const auto dataset = parse_dataset(sequence_root);

    ASSERT_TRUE(dataset) << dataset.error();
    EXPECT_EQ(dataset->sequence_root, sequence_root);
    EXPECT_EQ(dataset->cam0_calibration.resolution.x(), 752);
    EXPECT_EQ(dataset->cam0_calibration.resolution.y(), 480);
    EXPECT_EQ(dataset->cam1_calibration.resolution.x(), 752);
    EXPECT_EQ(dataset->cam1_calibration.resolution.y(), 480);
    EXPECT_DOUBLE_EQ(dataset->imu_calibration.rate_hz, 200.0);
    EXPECT_FALSE(dataset->stereo_pairs.empty());
    EXPECT_FALSE(dataset->imu_measurements.empty());
    EXPECT_FALSE(dataset->ground_truth_states.empty());
    EXPECT_EQ(dataset->stereo_pairs.front().cam0_image_path.parent_path(), sequence_root / "mav0/cam0/data");
    EXPECT_EQ(dataset->stereo_pairs.front().cam1_image_path.parent_path(), sequence_root / "mav0/cam1/data");
}

}  // namespace
