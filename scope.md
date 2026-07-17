# EKF-SLAM in Rust — Project Scope

## Goal

Build a real-time visual-inertial EKF-SLAM system from scratch in Rust, evaluated on the EuRoC MAV dataset. Intended as an MS-level portfolio piece targeting autonomy/embedded/algorithms roles (Anduril, Boston Dynamics, Skydio, Figure).

---

## Architecture

| Component | Choice |
|---|---|
| Filter | Error-State EKF (ESEKF) on SE(3) |
| State vector | Pose (SE(3)), velocity, gyro bias, accel bias |
| Landmark parameterization | Inverse depth |
| IMU rate | 200 Hz |
| Camera rate | 20 Hz stereo, hardware-synchronized |
| Linear algebra | `nalgebra` |
| ROS 2 | `rclrs` |
| Target hardware | Jetson Orin Nano |
| Dataset | EuRoC MAV |

---

## Phases

### Phase 1 — Offline ESEKF (Weeks 1–10)

Single-threaded, runs against recorded EuRoC sequences. Deliverable: ATE/RPE benchmark results.

#### Week 1–2: EuRoC Dataset Parser
- [ ] Top-level `Dataset` struct
- [ ] `CameraCalibration` — T_BS, intrinsics, distortion, resolution, rate (cam0 + cam1)
- [ ] `ImuCalibration` — T_BS, noise density, random walk parameters
- [ ] `ImuMeasurement` — timestamp, angular velocity, linear acceleration (CSV)
- [ ] `StereoPair` — timestamp, cam0/cam1 image paths (CSV)
- [ ] `GroundTruthState` — timestamp, position, orientation, velocity, biases (CSV)
- [ ] YAML parsing via `serde_yaml` with `TbsHelper` intermediate for T_BS
- [ ] Error handling: fail fast on missing files, malformed data, IMU gaps; survive camera frame gaps

#### Week 3–5: IMU Propagation
- [ ] ESEKF state and covariance definition
- [ ] Continuous-time IMU model discretization
- [ ] Propagation step at 200 Hz
- [ ] Bias integration

#### Week 6–8: Camera Measurement Update
- [ ] Feature detection and tracking (decide: KLT vs descriptor-based)
- [ ] Inverse-depth landmark initialization
- [ ] EKF update step
- [ ] Landmark marginalization

#### Week 9–10: Benchmarking
- [ ] ATE (Absolute Trajectory Error) computation
- [ ] RPE (Relative Pose Error) computation
- [ ] Evaluation against EuRoC sequences (MH_01 through V2_03)
- [ ] Results writeup

---

### Phase 2 — Real-Time Pipeline on Jetson (Weeks 11–16)

#### Week 11–13: Multithreading
- [ ] IMU thread (200 Hz, hard deadline)
- [ ] Camera thread (20 Hz)
- [ ] State publication thread
- [ ] Lock-free or channel-based data handoff between threads

#### Week 14–15: ROS 2 Integration
- [ ] `rclrs` node setup
- [ ] IMU subscriber (`sensor_msgs/Imu`)
- [ ] Image subscriber (`sensor_msgs/Image`)
- [ ] Odometry publisher (`nav_msgs/Odometry`)

#### Week 16: Hardware Deployment and Profiling
- [ ] Cross-compile for Jetson Orin Nano (aarch64)
- [ ] End-to-end latency profiling
- [ ] Deadline miss analysis

---

## Error Handling Policy

| Condition | Behavior |
|---|---|
| Missing calibration file | Hard fail |
| Malformed YAML | Hard fail |
| IMU data gap | Hard fail |
| Stereo timestamp mismatch | Hard fail |
| Camera frame gap | Warn, continue |
| Bad IMU record | Hard fail |

---

## Out of Scope

- Loop closure
- Map reuse / relocalization
- GPU acceleration
- Custom IMU or camera drivers