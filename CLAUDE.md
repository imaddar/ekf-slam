# ekf-slam

Visual-inertial Error-State EKF (ESEKF) SLAM, built from scratch in C++,
evaluated offline against the EuRoC MAV dataset, eventually deployed as a
real-time ROS 2 pipeline on a Jetson Orin Nano. MS-level portfolio project
targeting autonomy/algorithms roles.

Full project plan and phase breakdown: [scope.md](scope.md).
System design and module layout: [ARCHITECTURE.md](ARCHITECTURE.md).

## Current state

The EuRoC dataset parser lives in `parser.cpp` (public declarations in
`parser.hpp`, output structs in `types.hpp`, private YAML/CSV internals in
`parser_yaml.cpp` and `parser_csv.cpp`). The nominal ESEKF state and 15x15
covariance type are in `state.hpp`; IMU nominal-state and covariance propagation
is in `propagation.cpp`. `synthetic.cpp` generates analytic trajectories, IMU
streams, and stereo observations for controlled validation.

There is no measurement update, no landmark state, no error-state struct, and no
ATE/RPE/NEES evaluation code. Treat any request to "run the filter" or "update
from a camera frame" as premature; check `ARCHITECTURE.md` before assuming a
module or type already exists — it only documents what's actually built, not the
target design in `scope.md`. Metrics and tolerances live in `BENCHMARKS.md`.

## Keep ARCHITECTURE.md current

`ARCHITECTURE.md` documents only what's implemented — no planned/future work.
Whenever a change adds, removes, or restructures a module, public type, or
entry point in the C++ source/header files, update `ARCHITECTURE.md` in the same change so it stays
accurate. If a change is purely internal (e.g. renaming a private helper, a
refactor with no change to public shape or behavior), no update is needed.

Also as a part of `ARCHITECTURE.md`, include a DESIGN DECISIONS section, that is dedicated to that.
These should cover tradeoffs, specific intentional choices vs alternatives, and a high level picture of how these decisions work together.

## Commands

```
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build
ctest --test-dir build --output-on-failure
```

42 tests across `parser_tests`, `state_tests`, `propagation_tests`, and
`synthetic_tests`. No CI config and no standalone benchmark binary yet — metrics
are asserted inside the test suite and recorded in `BENCHMARKS.md`.
`parse_dataset(...)` loads a EuRoC sequence into the current `Dataset` shape.

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


## Project Presentation
- As a side note, I will be needing to present on this project as a technical deep dive. any information that would be useful for building a presentation and presenting it should go in the present.md file. This can be design decisions, technical tradeoffs, results, and anything that would be useful to an engineer.
- A good outline for a presentation is something like this:

* Describe the problem you solved. Why is the problem hard? Why is it important to solve it?

* Describe the approach you chose to solve the problem. How does your approach compare to others employed for this problem?

* Describe how your solution was implemented (e.g., software or hardware).

* What metrics did you use to quantify performance of your solution?

* Show results (ideally assessed using data) demonstrating the effectiveness of your approach.

* Describe lessons learned from the project and potential future directions for additional research.

* If the this was a group project, what was your specific contribution to the work?

 