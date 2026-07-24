use std::fs::read_to_string;
use std::path::{Path, PathBuf};

use nalgebra::{Matrix4, Quaternion, UnitQuaternion, Vector3, Vector4};
use serde::Deserialize;
use serde_yaml::from_str;

#[derive(Debug)]
pub struct Dataset {
    pub cam0_calibration: CameraCalibration,
    pub cam1_calibration: CameraCalibration,
    pub imu_calibration: ImuCalibration,
    pub stereo_pairs: Vec<StereoPair>,
    pub imu_measurements: Vec<ImuMeasurement>,
    pub ground_truth_states: Vec<GroundTruthState>,
}

#[derive(Debug)]
pub struct ImuMeasurement {
    pub timestamp: u64,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

#[derive(Debug)]
pub struct CameraFrame {
    pub timestamp: u64,
    pub image_path: PathBuf,
}

#[derive(Debug)]
pub struct StereoPair {
    pub timestamp: u64,
    pub cam0_image_path: PathBuf,
    pub cam1_image_path: PathBuf,
}

#[derive(Debug)]
pub struct GroundTruthState {
    pub timestamp: u64,
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub velocity: Vector3<f64>,
    pub gyroscope_bias: Vector3<f64>,
    pub accelerometer_bias: Vector3<f64>,
}

#[derive(Debug)]
pub struct CameraCalibration {
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub resolution: (u32, u32),
    pub intrinsics: Vector4<f64>,
    pub distortion_coefficients: Vector4<f64>,
}

#[derive(Debug)]
pub struct ImuCalibration {
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub gyroscope_noise_density: f64,
    pub gyroscope_random_walk: f64,
    pub accelerometer_noise_density: f64,
    pub accelerometer_random_walk: f64,
}

#[derive(Deserialize)]
struct RawTransform {
    cols: usize,
    rows: usize,
    data: Vec<f64>,
}

#[derive(Deserialize)]
struct RawCameraCalibration {
    #[serde(rename = "T_BS")]
    t_bs: RawTransform,
    rate_hz: f64,
    resolution: Vec<u32>,
    intrinsics: Vec<f64>,
    distortion_coefficients: Vec<f64>,
}

#[derive(Deserialize)]
struct RawImuCalibration {
    #[serde(rename = "T_BS")]
    t_bs: RawTransform,
    rate_hz: f64,
    gyroscope_noise_density: f64,
    gyroscope_random_walk: f64,
    accelerometer_noise_density: f64,
    accelerometer_random_walk: f64,
}

#[derive(Debug, PartialEq)]
struct RawImuMeasurement {
    timestamp: u64,
    angular_velocity: [f64; 3],
    acceleration: [f64; 3],
}

#[derive(Debug, PartialEq)]
struct RawGroundTruthState {
    timestamp: u64,
    position: [f64; 3],
    orientation: [f64; 4],
    velocity: [f64; 3],
    gyroscope_bias: [f64; 3],
    accelerometer_bias: [f64; 3],
}

#[derive(Debug, PartialEq)]
struct RawCameraFrame {
    timestamp: u64,
    filename: String,
}

impl TryFrom<RawCameraCalibration> for CameraCalibration {
    type Error = String;

    fn try_from(raw: RawCameraCalibration) -> Result<Self, Self::Error> {
        Ok(Self {
            t_bs: matrix4_from_raw_transform(raw.t_bs)?,
            rate_hz: raw.rate_hz,
            resolution: tuple2_from_vec(raw.resolution, "resolution")?,
            intrinsics: vector4_from_vec(raw.intrinsics, "intrinsics")?,
            distortion_coefficients: vector4_from_vec(
                raw.distortion_coefficients,
                "distortion_coefficients",
            )?,
        })
    }
}

impl TryFrom<RawImuCalibration> for ImuCalibration {
    type Error = String;

    fn try_from(raw: RawImuCalibration) -> Result<Self, Self::Error> {
        Ok(Self {
            t_bs: matrix4_from_raw_transform(raw.t_bs)?,
            rate_hz: raw.rate_hz,
            gyroscope_noise_density: raw.gyroscope_noise_density,
            gyroscope_random_walk: raw.gyroscope_random_walk,
            accelerometer_noise_density: raw.accelerometer_noise_density,
            accelerometer_random_walk: raw.accelerometer_random_walk,
        })
    }
}

impl From<RawImuMeasurement> for ImuMeasurement {
    fn from(raw: RawImuMeasurement) -> Self {
        Self {
            timestamp: raw.timestamp,
            acceleration: Vector3::new(
                raw.acceleration[0],
                raw.acceleration[1],
                raw.acceleration[2],
            ),
            angular_velocity: Vector3::new(
                raw.angular_velocity[0],
                raw.angular_velocity[1],
                raw.angular_velocity[2],
            ),
        }
    }
}

impl From<RawGroundTruthState> for GroundTruthState {
    fn from(raw: RawGroundTruthState) -> Self {
        let [qw, qx, qy, qz] = raw.orientation;

        Self {
            timestamp: raw.timestamp,
            position: Vector3::new(raw.position[0], raw.position[1], raw.position[2]),
            orientation: UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)),
            velocity: Vector3::new(raw.velocity[0], raw.velocity[1], raw.velocity[2]),
            gyroscope_bias: Vector3::new(
                raw.gyroscope_bias[0],
                raw.gyroscope_bias[1],
                raw.gyroscope_bias[2],
            ),
            accelerometer_bias: Vector3::new(
                raw.accelerometer_bias[0],
                raw.accelerometer_bias[1],
                raw.accelerometer_bias[2],
            ),
        }
    }
}

impl RawCameraFrame {
    fn into_camera_frame(self, image_dir: &Path) -> CameraFrame {
        CameraFrame {
            timestamp: self.timestamp,
            image_path: image_dir.join(self.filename),
        }
    }
}

fn matrix4_from_raw_transform(raw: RawTransform) -> Result<Matrix4<f64>, String> {
    if raw.rows != 4 || raw.cols != 4 {
        return Err(format!("T_BS must be 4x4, got {}x{}", raw.rows, raw.cols));
    }

    if raw.data.len() != 16 {
        return Err(format!(
            "T_BS must contain 16 values, got {}",
            raw.data.len()
        ));
    }

    Ok(Matrix4::from_row_slice(&raw.data))
}

fn vector4_from_vec(values: Vec<f64>, field_name: &str) -> Result<Vector4<f64>, String> {
    if values.len() != 4 {
        return Err(format!(
            "{} must contain 4 values, got {}",
            field_name,
            values.len()
        ));
    }

    Ok(Vector4::new(values[0], values[1], values[2], values[3]))
}

fn tuple2_from_vec(values: Vec<u32>, field_name: &str) -> Result<(u32, u32), String> {
    if values.len() != 2 {
        return Err(format!(
            "{} must contain 2 values, got {}",
            field_name,
            values.len()
        ));
    }

    Ok((values[0], values[1]))
}

fn parse_camera_yaml(file_path: PathBuf) -> Result<CameraCalibration, String> {
    let content_yaml =
        read_to_string(file_path).map_err(|e| format!("Failed to read YAML file: {}", e))?;
    let raw: RawCameraCalibration = from_str(&content_yaml)
        .map_err(|e| format!("Failed to deserialize camera YAML string: {}", e))?;
    raw.try_into()
}

fn parse_imu_yaml(file_path: PathBuf) -> Result<ImuCalibration, String> {
    let content_yaml =
        read_to_string(file_path).map_err(|e| format!("Failed to read YAML file: {}", e))?;
    let raw: RawImuCalibration = from_str(&content_yaml)
        .map_err(|e| format!("Failed to deserialize IMU YAML string: {}", e))?;
    raw.try_into()
}

pub fn parse_imu_measurements_csv(
    file_path: impl AsRef<Path>,
) -> Result<Vec<ImuMeasurement>, String> {
    let content_csv =
        read_to_string(file_path).map_err(|e| format!("Failed to read IMU CSV file: {}", e))?;
    parse_imu_measurements_csv_content(&content_csv)
}

pub fn parse_ground_truth_csv(
    file_path: impl AsRef<Path>,
) -> Result<Vec<GroundTruthState>, String> {
    let content_csv = read_to_string(file_path)
        .map_err(|e| format!("Failed to read ground truth CSV file: {}", e))?;
    parse_ground_truth_csv_content(&content_csv)
}

pub fn parse_camera_frames_csv(
    file_path: impl AsRef<Path>,
    image_dir: impl AsRef<Path>,
) -> Result<Vec<CameraFrame>, String> {
    let content_csv =
        read_to_string(file_path).map_err(|e| format!("Failed to read camera CSV file: {}", e))?;
    parse_camera_frames_csv_content(&content_csv, image_dir.as_ref())
}

pub fn parse_stereo_pairs_csv(
    cam0_csv_path: impl AsRef<Path>,
    cam0_image_dir: impl AsRef<Path>,
    cam1_csv_path: impl AsRef<Path>,
    cam1_image_dir: impl AsRef<Path>,
) -> Result<Vec<StereoPair>, String> {
    let cam0_frames = parse_camera_frames_csv(cam0_csv_path, cam0_image_dir)?;
    let cam1_frames = parse_camera_frames_csv(cam1_csv_path, cam1_image_dir)?;
    pair_stereo_frames(cam0_frames, cam1_frames)
}

fn parse_imu_measurements_csv_content(content: &str) -> Result<Vec<ImuMeasurement>, String> {
    csv_data_lines(content)
        .map(|(line_number, line)| parse_raw_imu_measurement(line, line_number).map(Into::into))
        .collect()
}

fn parse_ground_truth_csv_content(content: &str) -> Result<Vec<GroundTruthState>, String> {
    csv_data_lines(content)
        .map(|(line_number, line)| parse_raw_ground_truth_state(line, line_number).map(Into::into))
        .collect()
}

fn parse_camera_frames_csv_content(
    content: &str,
    image_dir: &Path,
) -> Result<Vec<CameraFrame>, String> {
    csv_data_lines(content)
        .map(|(line_number, line)| {
            parse_raw_camera_frame(line, line_number)
                .map(|raw_frame| raw_frame.into_camera_frame(image_dir))
        })
        .collect()
}

fn pair_stereo_frames(
    cam0_frames: Vec<CameraFrame>,
    cam1_frames: Vec<CameraFrame>,
) -> Result<Vec<StereoPair>, String> {
    if cam0_frames.len() != cam1_frames.len() {
        return Err(format!(
            "cam0 and cam1 must contain the same number of frames, got {} and {}",
            cam0_frames.len(),
            cam1_frames.len()
        ));
    }

    cam0_frames
        .into_iter()
        .zip(cam1_frames)
        .map(|(cam0_frame, cam1_frame)| {
            if cam0_frame.timestamp != cam1_frame.timestamp {
                return Err(format!(
                    "cam0 timestamp {} does not match cam1 timestamp {}",
                    cam0_frame.timestamp, cam1_frame.timestamp
                ));
            }

            Ok(StereoPair {
                timestamp: cam0_frame.timestamp,
                cam0_image_path: cam0_frame.image_path,
                cam1_image_path: cam1_frame.image_path,
            })
        })
        .collect()
}

fn csv_data_lines(content: &str) -> impl Iterator<Item = (usize, &str)> {
    content.lines().enumerate().filter_map(|(index, line)| {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            None
        } else {
            Some((index + 1, trimmed))
        }
    })
}

fn parse_raw_imu_measurement(line: &str, line_number: usize) -> Result<RawImuMeasurement, String> {
    let fields = csv_fields(line);
    expect_field_count(&fields, 7, "IMU measurement", line_number)?;

    Ok(RawImuMeasurement {
        timestamp: parse_u64_field(fields[0], "timestamp", line_number)?,
        angular_velocity: [
            parse_f64_field(fields[1], "angular_velocity.x", line_number)?,
            parse_f64_field(fields[2], "angular_velocity.y", line_number)?,
            parse_f64_field(fields[3], "angular_velocity.z", line_number)?,
        ],
        acceleration: [
            parse_f64_field(fields[4], "acceleration.x", line_number)?,
            parse_f64_field(fields[5], "acceleration.y", line_number)?,
            parse_f64_field(fields[6], "acceleration.z", line_number)?,
        ],
    })
}

fn parse_raw_ground_truth_state(
    line: &str,
    line_number: usize,
) -> Result<RawGroundTruthState, String> {
    let fields = csv_fields(line);
    expect_field_count(&fields, 17, "ground truth state", line_number)?;

    Ok(RawGroundTruthState {
        timestamp: parse_u64_field(fields[0], "timestamp", line_number)?,
        position: [
            parse_f64_field(fields[1], "position.x", line_number)?,
            parse_f64_field(fields[2], "position.y", line_number)?,
            parse_f64_field(fields[3], "position.z", line_number)?,
        ],
        orientation: [
            parse_f64_field(fields[4], "orientation.w", line_number)?,
            parse_f64_field(fields[5], "orientation.x", line_number)?,
            parse_f64_field(fields[6], "orientation.y", line_number)?,
            parse_f64_field(fields[7], "orientation.z", line_number)?,
        ],
        velocity: [
            parse_f64_field(fields[8], "velocity.x", line_number)?,
            parse_f64_field(fields[9], "velocity.y", line_number)?,
            parse_f64_field(fields[10], "velocity.z", line_number)?,
        ],
        gyroscope_bias: [
            parse_f64_field(fields[11], "gyroscope_bias.x", line_number)?,
            parse_f64_field(fields[12], "gyroscope_bias.y", line_number)?,
            parse_f64_field(fields[13], "gyroscope_bias.z", line_number)?,
        ],
        accelerometer_bias: [
            parse_f64_field(fields[14], "accelerometer_bias.x", line_number)?,
            parse_f64_field(fields[15], "accelerometer_bias.y", line_number)?,
            parse_f64_field(fields[16], "accelerometer_bias.z", line_number)?,
        ],
    })
}

fn parse_raw_camera_frame(line: &str, line_number: usize) -> Result<RawCameraFrame, String> {
    let fields = csv_fields(line);
    expect_field_count(&fields, 2, "camera frame", line_number)?;

    if fields[1].is_empty() {
        return Err(format!(
            "camera frame line {} has an empty filename",
            line_number
        ));
    }

    Ok(RawCameraFrame {
        timestamp: parse_u64_field(fields[0], "timestamp", line_number)?,
        filename: fields[1].to_string(),
    })
}

fn csv_fields(line: &str) -> Vec<&str> {
    line.split(',').map(str::trim).collect()
}

fn expect_field_count(
    fields: &[&str],
    expected: usize,
    record_name: &str,
    line_number: usize,
) -> Result<(), String> {
    if fields.len() != expected {
        return Err(format!(
            "{} line {} must contain {} fields, got {}",
            record_name,
            line_number,
            expected,
            fields.len()
        ));
    }

    Ok(())
}

fn parse_u64_field(value: &str, field_name: &str, line_number: usize) -> Result<u64, String> {
    value.parse::<u64>().map_err(|e| {
        format!(
            "Failed to parse {} on line {} as u64: {}",
            field_name, line_number, e
        )
    })
}

fn parse_f64_field(value: &str, field_name: &str, line_number: usize) -> Result<f64, String> {
    value.parse::<f64>().map_err(|e| {
        format!(
            "Failed to parse {} on line {} as f64: {}",
            field_name, line_number, e
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deserializes_camera_yaml_into_raw_type() {
        let yaml = "
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
";

        let raw: RawCameraCalibration = from_str(yaml).unwrap();
        let calibration = CameraCalibration::try_from(raw).unwrap();

        assert_eq!(calibration.resolution, (752, 480));
        assert_eq!(calibration.rate_hz, 20.0);
        assert_eq!(calibration.t_bs[(0, 3)], 0.1);
        assert_eq!(calibration.t_bs[(1, 3)], 0.2);
        assert_eq!(calibration.t_bs[(2, 3)], 0.3);
        assert_eq!(calibration.intrinsics[0], 458.654);
        assert_eq!(calibration.distortion_coefficients[2], 0.00019359);
    }

    #[test]
    fn rejects_camera_yaml_with_bad_transform_shape() {
        let yaml = "
T_BS:
  cols: 3
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0]
rate_hz: 20.0
resolution: [752, 480]
intrinsics: [458.654, 457.296, 367.215, 248.375]
distortion_coefficients: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
";

        let raw: RawCameraCalibration = from_str(yaml).unwrap();
        let error = CameraCalibration::try_from(raw).unwrap_err();

        assert_eq!(error, "T_BS must be 4x4, got 4x3");
    }

    #[test]
    fn deserializes_imu_yaml_into_raw_type() {
        let yaml = "
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0, 1.0, 0.3, 0.0, 0.0, 0.0, 1.0]
rate_hz: 200.0
gyroscope_noise_density: 0.00016968
gyroscope_random_walk: 1.9393e-05
accelerometer_noise_density: 0.002
accelerometer_random_walk: 0.003
";

        let raw: RawImuCalibration = from_str(yaml).unwrap();
        let calibration = ImuCalibration::try_from(raw).unwrap();

        assert_eq!(calibration.rate_hz, 200.0);
        assert_eq!(calibration.t_bs[(0, 3)], 0.1);
        assert_eq!(calibration.gyroscope_noise_density, 0.00016968);
        assert_eq!(calibration.accelerometer_random_walk, 0.003);
    }

    #[test]
    fn parses_imu_measurements_csv_content() {
        let csv = "
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1403636579758555392,0.1,0.2,0.3,9.7,9.8,9.9
1403636579763555584,-0.1,-0.2,-0.3,-9.7,-9.8,-9.9
";

        let measurements = parse_imu_measurements_csv_content(csv).unwrap();

        assert_eq!(measurements.len(), 2);
        assert_eq!(measurements[0].timestamp, 1403636579758555392);
        assert_eq!(
            measurements[0].angular_velocity,
            Vector3::new(0.1, 0.2, 0.3)
        );
        assert_eq!(measurements[0].acceleration, Vector3::new(9.7, 9.8, 9.9));
        assert_eq!(
            measurements[1].angular_velocity,
            Vector3::new(-0.1, -0.2, -0.3)
        );
    }

    #[test]
    fn rejects_imu_measurement_with_wrong_field_count() {
        let csv = "
# header
1403636579758555392,0.1,0.2
";

        let error = parse_imu_measurements_csv_content(csv).unwrap_err();

        assert_eq!(error, "IMU measurement line 3 must contain 7 fields, got 3");
    }

    #[test]
    fn parses_ground_truth_csv_content() {
        let csv = "
#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z [],v_RS_R_x [m s^-1],v_RS_R_y [m s^-1],v_RS_R_z [m s^-1],b_w_RS_S_x [rad s^-1],b_w_RS_S_y [rad s^-1],b_w_RS_S_z [rad s^-1],b_a_RS_S_x [m s^-2],b_a_RS_S_y [m s^-2],b_a_RS_S_z [m s^-2]
1403636579758555392,1.0,2.0,3.0,1.0,0.0,0.0,0.0,4.0,5.0,6.0,0.01,0.02,0.03,0.11,0.12,0.13
";

        let states = parse_ground_truth_csv_content(csv).unwrap();

        assert_eq!(states.len(), 1);
        assert_eq!(states[0].timestamp, 1403636579758555392);
        assert_eq!(states[0].position, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(states[0].orientation.w, 1.0);
        assert_eq!(states[0].orientation.i, 0.0);
        assert_eq!(states[0].velocity, Vector3::new(4.0, 5.0, 6.0));
        assert_eq!(states[0].gyroscope_bias, Vector3::new(0.01, 0.02, 0.03));
        assert_eq!(states[0].accelerometer_bias, Vector3::new(0.11, 0.12, 0.13));
    }

    #[test]
    fn rejects_ground_truth_with_wrong_field_count() {
        let csv = "
# header
1403636579758555392,1.0,2.0
";

        let error = parse_ground_truth_csv_content(csv).unwrap_err();

        assert_eq!(
            error,
            "ground truth state line 3 must contain 17 fields, got 3"
        );
    }

    #[test]
    fn parses_camera_frames_csv_content() {
        let csv = "
#timestamp [ns],filename
1403636579763555584,1403636579763555584.png
1403636579813555456,1403636579813555456.png
";

        let frames = parse_camera_frames_csv_content(csv, Path::new("mav0/cam0/data")).unwrap();

        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].timestamp, 1403636579763555584);
        assert_eq!(
            frames[0].image_path,
            PathBuf::from("mav0/cam0/data/1403636579763555584.png")
        );
        assert_eq!(
            frames[1].image_path,
            PathBuf::from("mav0/cam0/data/1403636579813555456.png")
        );
    }

    #[test]
    fn rejects_camera_frame_with_wrong_field_count() {
        let csv = "
# header
1403636579763555584
";

        let error = parse_camera_frames_csv_content(csv, Path::new("mav0/cam0/data")).unwrap_err();

        assert_eq!(error, "camera frame line 3 must contain 2 fields, got 1");
    }

    #[test]
    fn rejects_camera_frame_with_empty_filename() {
        let csv = "
# header
1403636579763555584,
";

        let error = parse_camera_frames_csv_content(csv, Path::new("mav0/cam0/data")).unwrap_err();

        assert_eq!(error, "camera frame line 3 has an empty filename");
    }

    #[test]
    fn pairs_matching_camera_frames() {
        let cam0_frames = vec![
            CameraFrame {
                timestamp: 1403636579763555584,
                image_path: PathBuf::from("mav0/cam0/data/1403636579763555584.png"),
            },
            CameraFrame {
                timestamp: 1403636579813555456,
                image_path: PathBuf::from("mav0/cam0/data/1403636579813555456.png"),
            },
        ];
        let cam1_frames = vec![
            CameraFrame {
                timestamp: 1403636579763555584,
                image_path: PathBuf::from("mav0/cam1/data/1403636579763555584.png"),
            },
            CameraFrame {
                timestamp: 1403636579813555456,
                image_path: PathBuf::from("mav0/cam1/data/1403636579813555456.png"),
            },
        ];

        let pairs = pair_stereo_frames(cam0_frames, cam1_frames).unwrap();

        assert_eq!(pairs.len(), 2);
        assert_eq!(pairs[0].timestamp, 1403636579763555584);
        assert_eq!(
            pairs[0].cam0_image_path,
            PathBuf::from("mav0/cam0/data/1403636579763555584.png")
        );
        assert_eq!(
            pairs[0].cam1_image_path,
            PathBuf::from("mav0/cam1/data/1403636579763555584.png")
        );
    }

    #[test]
    fn rejects_stereo_pairing_with_different_lengths() {
        let cam0_frames = vec![CameraFrame {
            timestamp: 1403636579763555584,
            image_path: PathBuf::from("mav0/cam0/data/1403636579763555584.png"),
        }];
        let cam1_frames = Vec::new();

        let error = pair_stereo_frames(cam0_frames, cam1_frames).unwrap_err();

        assert_eq!(
            error,
            "cam0 and cam1 must contain the same number of frames, got 1 and 0"
        );
    }

    #[test]
    fn rejects_stereo_pairing_with_mismatched_timestamps() {
        let cam0_frames = vec![CameraFrame {
            timestamp: 1403636579763555584,
            image_path: PathBuf::from("mav0/cam0/data/1403636579763555584.png"),
        }];
        let cam1_frames = vec![CameraFrame {
            timestamp: 1403636579813555456,
            image_path: PathBuf::from("mav0/cam1/data/1403636579813555456.png"),
        }];

        let error = pair_stereo_frames(cam0_frames, cam1_frames).unwrap_err();

        assert_eq!(
            error,
            "cam0 timestamp 1403636579763555584 does not match cam1 timestamp 1403636579813555456"
        );
    }
}
