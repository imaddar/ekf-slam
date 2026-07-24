# Architecture

This document describes the architecture of what is currently implemented in
`ekf-slam`. For the full project plan (weekly breakdown, deliverables,
out-of-scope items, and the target filter/system design that hasn't been built
yet) see [scope.md](scope.md).

## What exists

```
CMakeLists.txt      C++ build and GoogleTest test configuration
parser.hpp          C++ parser function declarations
parser.cpp          C++ YAML/CSV parser implementation
types.hpp           C++ parser output type declarations
tests/parser_test.cpp  C++ parser tests
```

There is no filter, state, or estimation code yet.

## C++ root skeleton

`parser.hpp` declares the C++ parser functions. `parser.cpp` currently
implements EuRoC camera and IMU calibration YAML parsing, IMU measurement CSV
parsing, ground-truth CSV parsing, stereo-pair CSV parsing, and top-level
EuRoC sequence loading into `Dataset`.

`tests/parser_test.cpp` contains the GoogleTest coverage for the C++ parser.
Current coverage includes successful camera/IMU YAML parsing, camera calibration
transform-shape rejection, successful IMU/ground-truth/stereo CSV parsing, IMU
and ground-truth field-count rejection, stereo timestamp mismatch rejection, and
top-level dataset loading from both a temporary EuRoC-like directory and the
checked-in `datasets/machine_hall/MH_01_easy` sequence.

`types.hpp` defines the intended C++ parser output data structures:

- `TimestampNs` — alias for `std::uint64_t` nanosecond timestamps.
- `CameraCalibration` and `ImuCalibration` — YAML calibration output structs.
- `StereoPair`, `ImuMeasurement`, and `GroundTruthState` — parsed sensor and
  label records.
- `Dataset` — top-level parsed dataset containing `sequence_root`, the
  calibration structs, stereo pairs, IMU measurements, and ground-truth states.

### Entry points

- `parse_imu_measurements_csv(path)` — public. Reads a file, skips blank/`#`
  comment lines, parses each remaining line as 7 comma-separated fields
  (timestamp, angular velocity xyz, acceleration xyz).
- `parse_ground_truth_csv(path)` — public. Same pattern, 17 fields (timestamp,
  position xyz, orientation wxyz, velocity xyz, gyro bias xyz, accel bias xyz).
- `parse_camera_yaml(path)` / `parse_imu_yaml(path)` — public. Parse a single
  EuRoC `sensor.yaml`-style calibration file into `CameraCalibration` /
  `ImuCalibration`.
- `parse_stereo_pairs_csv(cam0_csv_path, cam0_image_dir, cam1_csv_path,
  cam1_image_dir)` — public. Parses camera frame CSVs and hard-fails on frame
  count or timestamp mismatch.
- `parse_dataset(sequence_root)` — public. Loads the standard EuRoC `mav0`
  directory layout into `Dataset`.

### Error handling

No panics. Every fallible function returns `ParseResult<T>`. Error messages
name the field, the line number (for CSV), and what was expected vs. found
(e.g. `"T_BS must be 4x4, got 4x3"`, `"IMU measurement line 3 must contain 7
fields, got 3"`). This matches the hard-fail-by-default policy in
[scope.md](scope.md): missing files, malformed YAML, and malformed CSV records
all hard fail. Nothing implements the "camera frame gap → warn, continue"
behavior yet.

### Tests

Tests live in `tests/parser_test.cpp`, using inline YAML/CSV fixtures plus a
smoke test against `datasets/machine_hall/MH_01_easy`.

## Keeping this document current

This file must reflect only what is actually implemented. Update it
whenever a change adds, removes, or restructures modules, public types, or
entry points — see the instruction in [CLAUDE.md](CLAUDE.md).
