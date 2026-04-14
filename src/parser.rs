use std::path::PathBuf;
use std::fs::read_to_string;
use nalgebra::{Matrix4, UnitQuaternion, Vector2, Vector3, Vector4};

pub struct Dataset {
    pub cam0_calibration: CameraCalibration,
    pub cam1_calibration: CameraCalibration,
    pub imu_calibration: ImuCalibration,
    pub stereo_pairs: Vec<StereoPair>,
    pub imu_measurements: Vec<ImuMeasurement>,
    pub ground_truth_states: Vec<GroundTruthState>,    
}

pub struct ImuMeasurement {
    pub timestamp: u64,
    pub acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
}

pub struct StereoPair {
    pub timestamp: u64,
    pub cam0_image_path: String,
    pub cam1_image_path: String,
}

pub struct GroundTruthState {
    pub timestamp: u64,
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub velocity: Vector3<f64>,
    pub gyroscope_bias: Vector3<f64>,
    pub accelerometer_bias: Vector3<f64>,
}

#[derive(Deserialize)]
pub struct CameraCalibration {
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub resolution: (u32, u32),
    pub intrinsics: Vector4<f64>,
    pub distortion_coefficients: Vector4<f64>,
}

pub struct ImuCalibration {
    pub t_bs: Matrix4<f64>,
    pub rate_hz: f64,
    pub gyroscope_noise_density: f64,
    pub gyroscope_random_walk: f64,
    pub accelerometer_noise_density: f64,
    pub accelerometer_random_walk: f64,
}


fn parse_yaml(file_path: PathBuf) -> Result<CameraCalibration, String> {
    let content_yaml = read_to_string(file_path).map_err(|e| format!("Failed to read YAML file: {}", e))?;
    let content = from_str(&content_yaml).map_err(|e| format!("Failed to deserialize YAML string: {}", e))?;
    
    todo!();
}
