# ekf-slam

Visual-inertial Error-State EKF (ESEKF) SLAM, built from scratch in C++,
evaluated offline against the EuRoC MAV dataset, eventually deployed as a
real-time ROS 2 pipeline on a Jetson Orin Nano. MS-level portfolio project
targeting autonomy/algorithms roles.

Full project plan and phase breakdown: [scope.md](scope.md).
System design and module layout: [ARCHITECTURE.md](ARCHITECTURE.md).

## Current state

The C++ EuRoC dataset parser lives in `parser.cpp`, with public declarations in
`parser.hpp` and output data structures in `types.hpp`. No filter, state, or
estimation code exists yet. Treat any request to "run the
filter" or "propagate the state" as premature; check `ARCHITECTURE.md` before
assuming a module or type already exists — it only documents what's actually
built, not the target design in `scope.md`.

## Keep ARCHITECTURE.md current

`ARCHITECTURE.md` documents only what's implemented — no planned/future work.
Whenever a change adds, removes, or restructures a module, public type, or
entry point in the C++ source/header files, update `ARCHITECTURE.md` in the same change so it stays
accurate. If a change is purely internal (e.g. renaming a private helper, a
refactor with no change to public shape or behavior), no update is needed.

## Commands

```
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build
ctest --test-dir build --output-on-failure
```

No CI config or benchmark harness yet. `parse_dataset(...)` loads a EuRoC
sequence into the current C++ `Dataset` shape.

## Conventions to follow

- **Error handling.** No panics in library code. Every fallible operation
  returns `ParseResult<T>` with a message naming the field, line number, and
  what was expected vs. what was found. This matches the hard-fail-by-default
  policy in `scope.md` (missing files, malformed YAML/CSV, IMU gaps all hard
  fail; camera frame gaps are meant to warn-and-continue once a sequence loader
  exists).
- **Eigen types at the domain boundary.** Parser output structs use Eigen types
  (`Matrix4d`, `Vector2i`, `Vector3d`, `Vector4d`, `Quaterniond`) for math-facing
  data.
- **Tests live in `tests/`** and run through CMake/CTest with GoogleTest.
- **Comment style.** Comments should be brief, clear, and quick to digest for a
  junior engineer who is somewhat familiar with the math or system being
  implemented and wants small notes here and there for clarity. Use comments
  sparingly in both source files and agent instruction files.
  - Prefer comments that mark a short conceptual step, convention, invariant,
    trade-off, or TODO.
  - Avoid restating names, types, or operations that are already obvious from
    the code.
  - Keep comments close to the code they explain, usually one line above the
    relevant block.
  - Use precise project vocabulary: frame convention, covariance ordering,
    discretization choice, noise model, parser invariant.
  - Keep comments short enough to scan during a code review; if a comment needs
    a paragraph, prefer moving the detail to `ARCHITECTURE.md` or a design note.

## Working in this repo

- `datasets/` contains real EuRoC MAV sequences (images, CSVs, calibration YAML)
  — large, checked-in binary/data files. Don't read or grep through
  `datasets/**/data/*.png` wholesale; use it only as fixture data if a test
  needs a real sample file, and prefer the small inline YAML/CSV fixtures already
  used in `tests/parser_test.cpp`.
- When implementing a new phase item from `scope.md`, check off the
  corresponding checkbox in that file as part of the change.
- This project has no `AGENTS.md` distinct from this file — `AGENTS.md` is a
  symlink to `CLAUDE.md` so both names resolve to the same instructions.
