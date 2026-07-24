# Roadmap

This file tracks near-term parser API direction. It is intentionally separate
from `ARCHITECTURE.md`, which only describes what is implemented today.

## Parser API

The current parser API should stay narrow:

```cpp
ParseResult<Dataset> parse_dataset(const std::filesystem::path& sequence_root);
```

That gives downstream math and filter code one stable way to load a complete
EuRoC sequence while we keep the file-format parsing details private.

## Future Streaming Readers

When the filter needs incremental input instead of a fully materialized
`Dataset`, add reader types that expose sensor-specific streams:

- `ImuReader`: reads IMU CSV records incrementally and yields
  `ImuMeasurement` values in timestamp order.
- `StereoReader`: reads the two camera CSV streams, matches cam0/cam1 frames by
  timestamp, and yields `StereoPair` values without storing the whole sequence.
- `CalibrationReader`: reads calibration YAML for a configurable sensor set.
  This should be less EuRoC-static than the current private calibration helpers,
  so it can handle different camera/IMU layouts and future sensor types without
  forcing a new public function per YAML shape.

The intended direction is for `parse_dataset()` to eventually become a thin
collector built on top of these readers. The readers should be promoted into the
public header only when the filter has a concrete streaming requirement.

## Dataset Validation

Add a validation pass after parsing and before returning `Dataset`. Keep this
separate from low-level file parsing so the parser can distinguish "bad file
syntax" from "valid records that do not form a usable sensor dataset."

Initial checks should include:

- Monotonic timestamps for IMU measurements, stereo pairs, and ground-truth
  states.
- IMU gap detection against the expected IMU rate.
- Stereo frame count/timestamp consistency, preserving the current hard fail for
  mismatched cam0/cam1 timestamps.
- Optional image existence checks for stereo image paths, with a clear policy for
  whether missing camera frames hard fail or warn and continue.
- Calibration sanity checks: positive rates, positive image resolution, expected
  vector sizes, and finite numeric values.
- Dataset coverage checks so the stereo, IMU, and ground-truth streams have
  overlapping time ranges before the filter consumes them.

This should likely become a small internal `validate_dataset(...)` step first.
If downstream tools later need explicit diagnostics, promote the validation
result shape instead of returning only a string.
