# ekf-slam

Visual-inertial Error-State EKF (ESEKF) SLAM, built from scratch in Rust,
evaluated offline against the EuRoC MAV dataset, eventually deployed as a
real-time ROS 2 pipeline on a Jetson Orin Nano. MS-level portfolio project
targeting autonomy/algorithms roles.

Full project plan and phase breakdown: [scope.md](scope.md).
System design and module layout: [ARCHITECTURE.md](ARCHITECTURE.md).

## Current state

Only the EuRoC dataset parser (`src/parser.rs`) exists. No filter, state, or
estimation code yet — `src/main.rs` is a stub. Treat any request to "run the
filter" or "propagate the state" as premature; check `ARCHITECTURE.md` before
assuming a module or type already exists — it only documents what's actually
built, not the target design in `scope.md`.

## Keep ARCHITECTURE.md current

`ARCHITECTURE.md` documents only what's implemented — no planned/future work.
Whenever a change adds, removes, or restructures a module, public type, or
entry point in `src/`, update `ARCHITECTURE.md` in the same change so it stays
accurate. If a change is purely internal (e.g. renaming a private helper, a
refactor with no change to public shape or behavior), no update is needed.

## Commands

```
cargo build
cargo test              # parser.rs has unit tests colocated in #[cfg(test)] mod tests
cargo test <name>       # run a single test
cargo fmt
cargo clippy
```

No CI config, no benchmark harness, no `Dataset::load` entry point yet — there is
no way to run the tool end-to-end against a real dataset directory yet.

## Conventions to follow

- **Raw → domain conversion pattern.** Every parsed input type has a `Raw*`
  struct matching the on-disk format exactly, converted into the public domain
  type via `TryFrom` (if validation can fail, e.g. matrix shape) or `From` (if
  conversion is infallible, e.g. fixed-size array → `Vector3`). Follow this
  pattern for any new parsed input rather than deserializing straight into the
  domain type.
- **Error handling.** No panics in library code. Every fallible operation
  returns `Result<_, String>` with a message naming the field, line number, and
  what was expected vs. what was found. This matches the hard-fail-by-default
  policy in `scope.md` (missing files, malformed YAML/CSV, IMU gaps all hard
  fail; camera frame gaps are meant to warn-and-continue once a sequence loader
  exists).
- **nalgebra types at the domain boundary.** Raw structs use plain
  `Vec`/arrays/primitives (serde-friendly); domain structs use `nalgebra`
  types (`Matrix4<f64>`, `Vector3<f64>`, `Vector4<f64>`, `UnitQuaternion<f64>`).
  Convert at the `Raw* -> domain` boundary, not before.
- **Tests live next to the code** in a `#[cfg(test)] mod tests` block at the
  bottom of the file, not in a separate `tests/` directory. Keep following that
  layout for `parser.rs`; revisit only if integration-style tests (e.g. against
  a real dataset fixture) are added later.
- **Comments are rare.** The existing code has almost none; only add one for a
  genuinely non-obvious constraint (e.g. why `T_BS` must be exactly 4x4), not to
  restate what a line does.

## Working in this repo

- `datasets/` contains real EuRoC MAV sequences (images, CSVs, calibration YAML)
  — large, checked-in binary/data files. Don't read or grep through
  `datasets/**/data/*.png` wholesale; use it only as fixture data if a test
  needs a real sample file, and prefer the small inline YAML/CSV fixtures already
  used in `parser.rs`'s tests.
- When implementing a new phase item from `scope.md`, check off the
  corresponding checkbox in that file as part of the change.
- This project has no `AGENTS.md` distinct from this file — `AGENTS.md` is a
  symlink to `CLAUDE.md` so both names resolve to the same instructions.
