# Architecture

This document describes the architecture of what is currently implemented in
`ekf-slam`. For the full project plan (weekly breakdown, deliverables,
out-of-scope items, and the target filter/system design that hasn't been built
yet) see [scope.md](scope.md).

## What exists

```
src/main.rs     binary entry point, wires up the `parser` module
src/parser.rs   EuRoC dataset ingestion (YAML calibration + CSV records)
```

There is no filter, state, or estimation code yet, and no `Dataset::load(...)`
that assembles a full sequence — only the individual parsing functions below.

## `parser.rs`

### Domain types

- `Dataset` — top-level struct grouping `cam0_calibration`, `cam1_calibration`,
  `imu_calibration`, `stereo_pairs`, `imu_measurements`, `ground_truth_states`.
  Nothing currently constructs one; the fields exist as the target shape for a
  future loader.
- `CameraCalibration` — `t_bs: Matrix4<f64>`, `rate_hz`, `resolution: (u32, u32)`,
  `intrinsics: Vector4<f64>`, `distortion_coefficients: Vector4<f64>`.
- `ImuCalibration` — `t_bs: Matrix4<f64>`, `rate_hz`, gyro/accel noise density and
  random walk (`f64` each).
- `ImuMeasurement` — `timestamp: u64`, `acceleration: Vector3<f64>`,
  `angular_velocity: Vector3<f64>`.
- `GroundTruthState` — `timestamp: u64`, `position: Vector3<f64>`,
  `orientation: UnitQuaternion<f64>`, `velocity: Vector3<f64>`,
  `gyroscope_bias: Vector3<f64>`, `accelerometer_bias: Vector3<f64>`.
- `StereoPair` — `timestamp: u64`, `cam0_image_path: String`,
  `cam1_image_path: String`. Nothing currently produces these (no CSV parser for
  the stereo image index yet).

### Raw → domain conversion pattern

Every parsed input has a private `Raw*` struct matching the on-disk format
exactly, converted into the public domain type:

- `RawCameraCalibration`, `RawImuCalibration` — `#[derive(Deserialize)]`,
  parsed from YAML via `serde_yaml`, converted with `TryFrom` (fallible: e.g.
  `T_BS` must be a 4x4, 16-value matrix; `intrinsics`/`distortion_coefficients`
  must have exactly 4 values; `resolution` must have exactly 2).
- `RawImuMeasurement`, `RawGroundTruthState` — parsed by hand from CSV rows
  (fixed-size arrays), converted with `From` (infallible — field counts are
  already validated before construction).

`matrix4_from_raw_transform`, `vector4_from_vec`, and `tuple2_from_vec` are the
shared shape-validation helpers used by the `TryFrom` impls.

### Entry points

- `parse_imu_measurements_csv(path)` — public. Reads a file, skips blank/`#`
  comment lines, parses each remaining line as 7 comma-separated fields
  (timestamp, angular velocity xyz, acceleration xyz).
- `parse_ground_truth_csv(path)` — public. Same pattern, 17 fields (timestamp,
  position xyz, orientation wxyz, velocity xyz, gyro bias xyz, accel bias xyz).
- `parse_camera_yaml(path)` / `parse_imu_yaml(path)` — private, parse a single
  EuRoC `sensor.yaml`-style calibration file into `CameraCalibration` /
  `ImuCalibration`.

### Error handling

No panics. Every fallible function returns `Result<_, String>`. Error messages
name the field, the line number (for CSV), and what was expected vs. found
(e.g. `"T_BS must be 4x4, got 4x3"`, `"IMU measurement line 3 must contain 7
fields, got 3"`). This matches the hard-fail-by-default policy in
[scope.md](scope.md): missing files, malformed YAML, and malformed CSV records
all hard fail. Nothing implements the "camera frame gap → warn, continue"
behavior yet since there's no sequence-level loader to detect gaps in.

### Tests

Colocated in `#[cfg(test)] mod tests` at the bottom of `parser.rs`, using
inline YAML/CSV string fixtures (no fixture files, no dependency on
`datasets/`). Coverage: successful parse + one shape/field-count rejection for
each of camera calibration, IMU calibration, IMU measurements CSV, and ground
truth CSV.

## Keeping this document current

This file must reflect only what is actually implemented in `src/`. Update it
whenever a change adds, removes, or restructures modules, public types, or
entry points — see the instruction in [CLAUDE.md](CLAUDE.md).
