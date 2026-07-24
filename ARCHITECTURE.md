# Architecture

This document describes the architecture of what is currently implemented in
`ekf-slam`. For the full project plan (weekly breakdown, deliverables,
out-of-scope items, and the target filter/system design that hasn't been built
yet) see [scope.md](scope.md).

## What exists

```
CMakeLists.txt      C++ build and GoogleTest test configuration
parser.hpp          Public C++ parser API declaration
parser.cpp          Top-level dataset loading orchestration
parser_csv.hpp/cpp  Internal CSV parsing and stereo frame pairing
parser_yaml.hpp/cpp Internal EuRoC calibration YAML parsing
types.hpp           C++ parser output type declarations
roadmap.md          Planned parser API direction
tests/parser_test.cpp  C++ parser tests
```

There is no filter, state, or estimation code yet.

## C++ root skeleton

`parser.hpp` exposes only the top-level EuRoC dataset loader. `parser.cpp`
orchestrates top-level EuRoC sequence loading into `Dataset`. `parser_yaml.cpp`
implements private EuRoC camera and IMU calibration YAML parsing.
`parser_csv.cpp` implements private IMU measurement CSV parsing, ground-truth
CSV parsing, camera CSV parsing, and stereo-pair matching.

`tests/parser_test.cpp` contains the GoogleTest coverage for the C++ parser.
Current coverage validates those behaviors through the public `parse_dataset`
entry point: successful camera/IMU YAML parsing, camera calibration
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

### Entry Points

- `parse_dataset(sequence_root)` — public. Loads the standard EuRoC `mav0`
  directory layout into `Dataset`.

Lower-level YAML, CSV, and stereo-pair parsing functions are private
implementation details in `parser_yaml.cpp` and `parser_csv.cpp`. Planned future
reader APIs are tracked in [roadmap.md](roadmap.md), but are not implemented.

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
