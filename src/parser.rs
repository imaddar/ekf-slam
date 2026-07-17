use std::fs::read_to_string;
use std::path::PathBuf;

use nalgebra::{Matrix4, UnitQuaternion, Vector3, Vector4};
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
pub struct StereoPair {
    pub timestamp: u64,
    pub cam0_image_path: String,
    pub cam1_image_path: String,
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
}
