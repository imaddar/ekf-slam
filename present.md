# Technical Deep Dive — Visual-Inertial ESEKF SLAM

Presentation material for this project. Follows the standard deep-dive outline:
problem, approach, implementation, metrics, results, lessons, contribution.

**Status as of 2026-08-14.** Implemented: EuRoC dataset parser, nominal ESEKF
state, IMU propagation of state and covariance, storage for a bounded joint SLAM
state, a synthetic ground-truth harness, and 54 tests. Not implemented: SLAM
covariance propagation, state augmentation, camera measurement update, ATE/RPE/NEES
evaluation, ROS 2, Jetson deployment. Every number below is measured; nothing is
projected. Keep that boundary explicit when presenting — the strongest thing to
show right now is *verification methodology on a half-built filter*, not a
finished SLAM system.

---

## 1. The problem

**Estimate the 6-DoF pose of a moving platform, in real time, from a stereo
camera and an IMU, with no external reference.**

No GPS, no motion capture, no prior map. The vehicle has to answer "where am I
and how fast am I moving" from its own sensors alone, at a rate fast enough to
close a control loop around.

### Why it is hard

**Neither sensor can do the job alone.**

An IMU measures specific force and angular rate at 200 Hz. Integrating it to get
position means integrating twice, so a constant accelerometer bias — the slowly
drifting offset every MEMS IMU has — grows as `t^2` in position, and a constant
*gyroscope* bias is worse: it ramps the tilt error linearly, which tips gravity
into the horizontal axes and grows as `t^3`. Measured on this project, dead
reckoning EuRoC MH_01 from perfect initial conditions:

| Horizon | Position error |
|---|---|
| 1 s | 0.017 m |
| 5 s | 0.215 m |
| 10 s | 0.364 m |
| 20 s | 3.75 m |
| 30 s | 19.3 m |

That is with *ground-truth* initial state and *ground-truth* biases. Half a
minute of unaided inertial navigation puts a drone 19 m from where it thinks it
is. This is the single most useful slide in the deck: it is the entire
justification for the camera.

A camera has the opposite failure mode. It is drift-free in the sense that a
landmark seen twice constrains the motion between the two views, but it is slow
(20 Hz), it fails under motion blur and low texture, and a monocular camera
cannot observe scale at all. Stereo fixes scale but not the rest.

**The fusion itself is the hard part.**

- *Rotations are not a vector space.* You cannot maintain a Gaussian over a
  rotation matrix or a unit quaternion the way you can over a position — the
  representation is constrained, and a covariance in those coordinates is
  singular.
- *The system is nonlinear and the linearization point moves.* Gravity has to be
  removed in the world frame, which means every accelerometer sample is rotated
  by an orientation that is itself being estimated. An orientation error leaks
  directly into acceleration as `g * sin(dtheta)` — a 1° tilt error looks like
  0.17 m/s² of horizontal acceleration, which integrates to 8.6 m of position
  error in 10 s.
- *Some states are only weakly observable.* Accelerometer bias and gravity
  direction are nearly indistinguishable when the platform is not accelerating.
  Yaw is unobservable in a gravity-aligned inertial system. A filter that does
  not respect this becomes over-confident and then diverges.
- *The covariance is where the bugs hide.* A wrong nominal integrator shows up
  immediately as visible drift. A transposed block in the covariance transition
  matrix does not — the trajectory still looks fine, and the filter just weights
  the next camera update wrongly. This project spent most of its verification
  effort here, and section 4 is about that.
- *It has to run in a hard-deadline loop.* 200 Hz means a 5 ms budget per IMU
  sample, on an embedded target, with no allocation, no exceptions, and no
  garbage collection pause.

### Why it matters

Every autonomous system that operates without GPS needs this: drones indoors or
under bridges, warehouse robots, legged robots, AR headsets, anything in a
contested or denied environment. Visual-inertial odometry is the standard answer
because cameras and IMUs are cheap, small, and passive. It is a load-bearing
component in essentially every autonomy stack, which is why it is a reasonable
thing to build from scratch as a portfolio piece: the failure modes are exactly
the ones that separate "it works in a demo" from "it works on a vehicle".

---

## 2. Approach

### The choice: Error-State EKF

The state is split into two pieces.

- A **nominal state** carrying the full, large-signal estimate: position,
  velocity, orientation as an `SO(3)` element, accelerometer bias, gyroscope
  bias. This is integrated directly from the IMU with no probabilistic
  machinery.
- An **error state**: a 15-dimensional vector living in the tangent space,
  representing the small deviation between the nominal state and the truth. All
  the covariance and Kalman algebra happens here, where everything is a plain
  vector and a plain matrix.

Why this works: the error is small, so its dynamics are well approximated by a
linear model, and the awkward rotation lives entirely in the nominal state where
`SO(3)` handles it exactly. The rotation error is a 3-vector in the tangent
space — minimal, unconstrained, non-singular covariance.

### Compared to the alternatives

| Approach | Trade-off |
|---|---|
| **Direct EKF** (covariance over a quaternion) | Simpler to describe, but the covariance is singular over a constrained 4-vector and normalization has to be patched in. The ESEKF removes the problem rather than managing it. |
| **MSCKF** | Keeps a sliding window of past *poses* instead of landmarks, marginalizing landmarks at update time. Cost stays bounded — this is what most production VIO on constrained hardware does. Chosen against for a first implementation because the bookkeeping obscures the core filter math, which is the thing worth learning and demonstrating. Noted as an open decision. |
| **Classic EKF-SLAM** (landmarks in the state) | What this project is heading toward. Conceptually clean, but the covariance is `O(n^2)` in landmark count and updates are `O(n^2)`–`O(n^3)`. Fine for a bounded landmark set; the reason nobody builds large maps this way. |
| **Optimization / fixed-lag smoothing** (VINS-Mono, OKVIS, ORB-SLAM3) | Better accuracy — relinearizes the whole window instead of once — at higher and less predictable compute. Filters win on a fixed CPU budget with a hard deadline, which is the stated target. |
| **Learned VIO** | Not competitive on interpretability, and there is no story for what the covariance means. |

The honest framing: the ESEKF is the *right pedagogical and embedded* choice,
and a filter is a genuinely defensible engineering answer when the requirement
is a 5 ms deadline on a Jetson Orin Nano. It is not the highest-accuracy option
in the literature.

### Design commitments that shaped everything else

1. **Errors are values, never panics.** Every fallible operation returns
   `std::expected<T, std::string>`. No exceptions, no aborts, in library code —
   because the same code has to run inside a hard-deadline thread later.
2. **Correctness is pinned by generated truth, not by inspection.** A synthetic
   harness produces trajectories with closed-form analytic state, so propagation
   error is *measured against exact truth*, not eyeballed against a plot.
3. **Cheap-and-analyzable before exact.** Every numerical choice is the simplest
   form whose error is understood and quantified, with a recorded upgrade path
   and a benchmark that would show the upgrade paying off.

Full rationale, alternatives, and open questions:
[ARCHITECTURE.md](ARCHITECTURE.md#design-decisions).

---

## 3. Implementation

C++23, Eigen for linear algebra, Sophus for `SO(3)`, GoogleTest via CTest.
~1,450 lines of source and headers against ~1,400 lines of tests. No framework,
no ROS dependency yet, no third-party parser.

```
parser.cpp / parser_yaml.cpp / parser_csv.cpp   EuRoC loading (hand-written YAML + CSV)
state.hpp                                       NominalState, IMU-only ImuStateCovariance
propagation.cpp                                 IMU-only nominal + covariance propagation
slam_state.hpp/cpp                              bounded joint SLAM state and registry
synthetic.cpp                                   analytic trajectory, IMU, stereo generator
```

The bounded SLAM container receives both the initial nominal robot state and
the initial robot covariance explicitly. The existing IMU-only propagation path
also receives `P` from its caller and does not define a universal default
uncertainty, so the SLAM container does not invent either starting value.

The container now owns the bookkeeping needed before covariance math: metric XYZ
landmark positions, an external-ID registry, capacity-bounded covariance blocks,
and batch compaction for removal. State augmentation and joint covariance
propagation remain separate stages.

### The propagation step

One IMU sample in, updated state and covariance out:

1. **Remove bias estimates** from the raw accelerometer and gyroscope readings.
2. **Rotate into the world frame and add gravity.** The IMU measures specific
   force, so `a_W = R_WB * (a_m - b_a) + [0, 0, -9.81]`.
3. **Integrate the nominal state.** Constant input over the step, keeping the
   second-order `0.5 a dt^2` position term; orientation via
   `R <- R * Exp(omega dt)`, right-multiplied to match the local error
   convention.
4. **Build the continuous-time error dynamics `F`.** Five non-zero 3x3 blocks:
   `dp/dv = I`, `dv/dtheta = -R [a]x`, `dv/db_a = -R`, `dtheta/dtheta = -[w]x`,
   `dtheta/db_g = -I`.
5. **Discretize and propagate**: `Phi = I + F dt`, then
   `P <- Phi P Phi^T + G Q G^T dt`, with `Q` built from the EuRoC noise
   densities and random walks.

Useful explanation for questions: `F` is the continuous-time error dynamics at
the current nominal state; `Phi` is the one-step discrete transition applied to
the covariance. In the current first-order implementation, `Phi = I + F dt`.
When landmarks are added, the same robot `Phi` propagates the robot block and
the robot-landmark cross blocks, while landmark-landmark covariance stays fixed
during IMU-only prediction.

Measured cost: **1.84–1.91 us per call** on the dev machine, against a 5,000 us
budget at 200 Hz. Fixed-size Eigen types mean no heap allocation in the step.

### The synthetic harness — the part worth presenting

This is the piece that makes the rest verifiable, and it is where the
interesting engineering is.

A `SyntheticTrajectory` is a closed-form motion model: constant world
acceleration, optional sinusoidal world acceleration, constant body-frame
angular velocity, with configurable initial pose, velocity, and true biases.
Because it is closed form, `state_at(t)` returns *exact* truth at any continuous
time — no numerical integration, no interpolation error.

From it, the harness generates:

- **IMU measurements** by inverting the physics: take analytic acceleration,
  subtract gravity, rotate into the body frame, add the true biases. If
  propagation then fails to recover the trajectory, the bug is in propagation —
  there is nowhere else for it to be.
- **Ground-truth states** sampled from the same continuous function, so no
  interpolation error is introduced when camera and IMU timestamps disagree.
- **Stereo observations** by projecting known world landmarks into two pinhole
  cameras.

Three details that turned out to matter more than expected:

- **The trajectory owns the time origin and the true biases**, not the
  per-stream configs. An earlier version duplicated them, which let the ground
  truth and the IMU stream disagree silently — precisely the failure a
  bias-estimation test exists to catch.
- **Noise is specified as continuous-time density**, the same way EuRoC reports
  it, and the same config fills both the injected noise and the filter's `Q`.
  If those two describe different processes, a NEES number is meaningless.
- **Gaussian draws use hand-written Box-Muller over raw `mt19937_64` bits**,
  because `std::normal_distribution`'s engine-to-variate mapping is
  implementation-defined — seeded output differs between libc++ and libstdc++.
  Reproducible test failures have to survive the Jetson cross-compile.

---

## 4. Metrics

The interesting claim of this project is not "the trajectory looks right." It is
**"the uncertainty is right,"** and that needs a different kind of test.

### The problem with testing a covariance

The natural test is to hand-compute the expected blocks of `P` after one step
and assert on them. That test is nearly worthless, and the reason is worth a
slide:

Hand-computable cases are simple cases — identity orientation, zero angular
velocity. But at identity orientation, `R` and `R^T` are the same matrix. At
zero angular velocity, the `[w]x` block is zero. So a transposed rotation block
or a sign-flipped skew block **passes every hand-checked assertion**, and then
mis-weights every camera update forever after.

### The gates actually used

| Metric | What it catches |
|---|---|
| **Monte Carlo covariance consistency** | Any transposed, sign-flipped, or mis-scaled block in `F` |
| **Analytic propagation error** vs. closed-form truth | Integrator bugs, gravity/frame convention errors |
| **Timestep convergence ratio** | Whether the integrator has the order of accuracy claimed |
| **Noise discretization check** | Whether injected noise and the filter's `Q` are the same process |
| **Covariance validity** (symmetry, min eigenvalue) | Numerical breakdown |
| **EuRoC smoke test** | Real-data plumbing: parsing, calibration, timestamps, units |

### The Monte Carlo test

Propagate 6,000 randomly perturbed trajectories 200 steps with noisy IMU input,
take the sample covariance of the resulting errors, and compare it against what
the filter *predicted* — normalized by the filter's own standard deviations so
every block is judged on one scale, and kept signed so a flipped block cannot
pass.

Two things make it sharp:

- It starts from a **non-trivial `P0`**. With zero initial bias uncertainty, the
  bias-coupling blocks of `F` carry no weight and a sign flip in them is
  invisible.
- It uses an **excited state** — non-identity rotation, non-zero angular
  velocity — which is exactly what the hand-checked tests cannot do.

**The tolerance is set by the truncation error of `Phi = I + F dt`, not by
sampling noise.** More samples will not lower it. That is what makes the number
meaningful: correct code sits at ~0.04, and corrupting any block of `F` scores
≥ 0.23. The 0.15 threshold sits in the gap.

---

## 5. Results

All 48 tests pass. Full table and provenance in [BENCHMARKS.md](BENCHMARKS.md).

### Covariance consistency

| Seed | Max normalized deviation | Gate |
|---|---|---|
| 1 | 0.0393 | < 0.15 |
| 2 | 0.0410 | < 0.15 |
| 3 | 0.0301 | < 0.15 |

Deliberately corrupting any single block of `F` pushes this to ≥ 0.23. The test
has a demonstrated discrimination margin of roughly 6x between correct and
subtly-wrong code — that is the claim to make, not "the test passes."

### Propagation against analytic truth

| Scenario | Error after 1–2 s at 200 Hz |
|---|---|
| Stationary | < 1e-9 (position, velocity, orientation) |
| Constant acceleration | < 1e-9 (position, velocity) |
| Constant yaw rate | < 1e-9 rad |
| Fixed biases | < 4e-3 position/velocity, < 1e-9 rad |
| Fully excited (translation + rotation, 2 s) | < 4e-3 position/velocity, < 1e-9 rad |

### Integrator order — measured, not assumed

| Rate | Combined position + velocity error | Ratio |
|---|---|---|
| 200 Hz | 0.00137943 | — |
| 400 Hz | 0.000689604 | 0.4999 |
| 800 Hz | 0.000344773 | 0.4999 |

Halving the timestep halves the error, to four digits. The integrator is first
order overall and behaves exactly as first order — which also says the
second-order position term is buying a constant factor, not an order.

### Noise model self-consistency

At 200 Hz over 100 s (20,001 samples): accelerometer residual stddev `0.142245`
against the predicted `0.01 * sqrt(200) = 0.141421`; gyroscope `0.0281825`
against `0.002 * sqrt(200) = 0.0282843`. Within 0.6%. The injected noise and the
filter's `Q` are the same process.

### Real data — EuRoC MH_01

36,820 IMU samples, 3,682 stereo pairs, 36,382 ground-truth states parsed. One
second of IMU-only propagation from the first ground-truth state:

| Quantity | Measured | Test gate |
|---|---|---|
| Position error | 0.0173 m | < 2.0 m |
| Velocity error | 0.0216 m/s | < 3.0 m/s |
| Orientation error | 0.00120 rad (0.069°) | < 0.5 rad |
| Min covariance eigenvalue | 6.6e-8 | > -1e-8 |

The gates are deliberately ~100x looser than the measured values: this is a
plumbing check on real data, not an accuracy claim, and it should not fail
because of normal variation in ground truth or calibration.

### Dead reckoning drift, and the honest observation

| Horizon | Position error | Reported `sqrt(trace(P_pp))` |
|---|---|---|
| 1 s | 0.017 m | 0.007 m |
| 2 s | 0.053 m | 0.034 m |
| 5 s | 0.215 m | 0.342 m |
| 10 s | 0.364 m | 2.44 m |
| 20 s | 3.75 m | 18.7 m |
| 30 s | 19.3 m | 62.4 m |

Two things to say about this table:

1. Error growth is super-linear, as expected for double-integrated bias error.
   This is the slide that motivates the camera update.
2. The filter's *reported* uncertainty grows faster than the actual error at
   long horizons, and slower at 1 s. That is a single run against one ground
   truth, so it is an observation, not a NEES result — a real consistency claim
   needs many runs and a chi-square interval. Saying so is better than
   over-claiming; it is also precisely the measurement the next phase adds.

### Runtime

6,001 sequential propagation steps in 11.1–11.4 ms → **1.84–1.91 us per step**,
against a 5,000 us budget at 200 Hz (0.04% of it). Dev machine, not Jetson;
`RelWithDebInfo`.

---

## 6. Lessons learned

**A test that cannot fail is not a test.** The hand-computed covariance
assertions felt like real verification and could not detect a transposed
rotation block. Asking "what bug would this test catch?" — and deliberately
introducing that bug to check — changed how the whole suite was written. The
Monte Carlo test earned its keep only after being shown to score 0.23+ on
corrupted code.

**Build the truth generator before the thing being tested.** The synthetic
harness felt like a detour from "real" filter work. It is the reason a
propagation failure now has exactly one possible cause. Debugging on EuRoC
first would have meant every failure was ambiguous between the parser, the
calibration, the time alignment, the integrator, and ground truth itself.

**Chase the small discrepancy.** Propagation error came out 17x worse than
expected (0.009 vs 0.0005) on one test. It looked like an integrator bug. It was
`0.29 * 200` evaluating to `57.999999999999993`, so a `floor` silently dropped
the last sample. The integrator was fine; the harness was lying. Floating-point
sample-grid arithmetic is a correctness concern, not a detail.

**Validate the inputs a physicist would never think to validate.** A zero sample
rate produced NaN timestamps and aborted inside Sophus. A negative duration
produced an enormous `reserve`. A negative `dt` from one out-of-order IMU sample
ran the covariance update *backwards* — subtracting information instead of
adding it, and driving the minimum eigenvalue to `-2.4e-4`, leaving `P`
indefinite and every subsequent Kalman gain meaningless. Silent numerical
corruption is worse than a crash, and both are worse than a returned error.

**Reproducibility is a design constraint, not a nicety.** `std::normal_distribution`
is not specified to produce the same values from the same seed across standard
library implementations. A seeded test that passes on macOS and fails on the
Jetson for that reason would have cost days.

**Optimization flags are a correctness input for template-heavy code.** The
default unoptimized build made the suite ~50x slower, which put a
useful-sample-count Monte Carlo test out of reach — the *methodology* was gated
on the build type.

---

## 7. Future directions

**Immediate (finishes Phase 1):**

- Camera measurement update: feature tracking, metric XYZ landmark
  initialization from stereo triangulation, EKF update, marginalization.
- ATE / RPE / NEES evaluation across MH_01–V2_03. NEES is the one that closes
  the loop on the covariance work above — the Monte Carlo test proves the
  propagated covariance is self-consistent; NEES proves it is consistent with
  *reality*.
- Chi-square innovation gating. A VIO filter without outlier rejection fails on
  the first bad match.

**Landmark-state direction:** use classic EKF-SLAM with a bounded active map.
Allocate the joint covariance once at `(15 + N_max * 3)^2`, track the active
dimension, initialize stereo landmarks as 3D metric XYZ points, and remove
landmarks by batch compaction. Inverse depth stays a future option for larger
scales, long-range points, or monocular initialization, where its better depth
conditioning may justify the extra parameterization and anchored-frame
bookkeeping.

**Numerical upgrades, each with a metric that would show the payoff:**

- Exact `Q_d` via Van Loan instead of the first-order form (the current version
  leaves the position block of `Q_d` exactly zero — visible in NEES).
- Midpoint or RK4 nominal integration (measurable on the timestep-convergence
  gate, which currently reads first order).
- Joseph-form update and explicit symmetrization once updates start subtracting
  information from `P`.

**Phase 2 — real time:** ROS 2 node, threading with a 200 Hz hard deadline,
Jetson Orin Nano cross-compile, latency and deadline-miss profiling.

**Research-adjacent, if time allows:** observability-constrained EKF to remove
the spurious yaw observability that standard VIO EKFs gain from inconsistent
linearization; online IMU-camera extrinsic and time-offset calibration.

---

## 8. Contribution

Solo project. Every line of source, test, and documentation is mine: the parser,
the state and propagation math, the synthetic harness, the verification
methodology, and the build.

External dependencies are Eigen (linear algebra), Sophus (`SO(3)` exponential
and logarithm), and GoogleTest. The EuRoC MAV dataset provides the imagery, IMU
records, calibration, and Vicon ground truth. The ESEKF formulation follows
standard VIO literature — the contribution is the implementation and,
specifically, the verification strategy built around it.

---

## Appendix — presenting this

**If you have 5 minutes:** the drift table (§1), the "why hand-checked
covariance tests cannot fail" argument (§4), the Monte Carlo discrimination
margin (§5).

**If you have 20 minutes:** add the ESEKF split and why rotations force it
(§2), the synthetic harness inversion trick (§3), and two lessons (§6).

**Figures worth building:**

1. Drift vs. horizon, log-y, with the reported 1-sigma band overlaid — one plot
   carries both §1's motivation and §5's honest observation about consistency.
2. The 15x15 `F` block structure as a labeled grid, with the five non-zero
   blocks highlighted and the two that hand-checked tests cannot verify marked
   in a different color.
3. Monte Carlo deviation: correct code (~0.04) vs. each corrupted-`F` variant
   (≥ 0.23) as a bar chart against the 0.15 threshold line.
4. Timestep convergence on log-log axes with a reference slope-1 line.

**Questions to expect:**

- *Why a filter and not a smoother?* Fixed compute budget and a hard deadline;
  see the comparison table in §2. Be ready to concede accuracy.
- *Why not MSCKF?* Deliberate — the classic form makes the filter math legible,
  and it is recorded as an open decision, with the `O(n^2)` cost understood.
- *Is 1.9 us/step meaningful for Jetson?* No. It is a dev-machine number and
  should be presented as one; the Jetson figure is Phase 2 work.
- *Have you shown the filter is consistent?* Not yet, and say so plainly. The
  Monte Carlo test shows the propagated covariance matches the propagated error
  distribution. NEES against real data comes with the measurement update.
