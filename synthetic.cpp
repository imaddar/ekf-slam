#include "synthetic.hpp"

#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <random>
#include <string_view>

namespace {

const Eigen::Vector3d kGravity{0.0, 0.0, -9.81};
constexpr double kNanosecondsPerSecond = 1e9;
constexpr double kRigidTransformTolerance = 1e-9;
// Products like 0.29 * 200 evaluate to 57.999999999999993, so round to the nearest
// whole step before truncating or the final sample is silently dropped.
constexpr double kStepRoundingTolerance = 1e-9;

TimestampNs timestamp_at(TimestampNs start_timestamp, double time_seconds) {
    return start_timestamp + static_cast<TimestampNs>(std::llround(time_seconds * kNanosecondsPerSecond));
}

std::int64_t sample_count(double rate_hz, double duration_seconds) {
    const double steps = duration_seconds * rate_hz;
    const double rounded = std::round(steps);
    const double whole_steps = std::abs(steps - rounded) <= kStepRoundingTolerance * std::max(1.0, rounded)
        ? rounded
        : std::floor(steps);
    return static_cast<std::int64_t>(whole_steps) + 1;
}

// Portable standard normal. std::normal_distribution's engine-to-variate mapping is
// implementation-defined, so seeded output differs between libc++ and libstdc++ —
// which would break reproducibility once this project cross-compiles to Jetson.
class GaussianSampler {
public:
    explicit GaussianSampler(std::uint64_t seed) : engine_(seed) {}

    double operator()() {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }

        // Box-Muller over raw engine bits, which are bit-specified for mt19937_64.
        const double magnitude = std::sqrt(-2.0 * std::log(uniform_open_unit()));
        const double angle = 2.0 * std::numbers::pi * uniform_half_open_unit();
        spare_ = magnitude * std::sin(angle);
        has_spare_ = true;
        return magnitude * std::cos(angle);
    }

    Eigen::Vector3d sample_vector() {
        // Braced init sequences left to right, so the draw order is deterministic.
        return Eigen::Vector3d{operator()(), operator()(), operator()()};
    }

private:
    // (0, 1], keeping the logarithm finite.
    double uniform_open_unit() { return static_cast<double>((engine_() >> 11) + 1) * 0x1.0p-53; }
    // [0, 1)
    double uniform_half_open_unit() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }

    std::mt19937_64 engine_;
    double spare_ = 0.0;
    bool has_spare_ = false;
};

// Camera pose pre-inverted once per run rather than once per landmark per frame.
struct PinholeCamera {
    Eigen::Matrix3d rotation_camera_from_body;
    Eigen::Vector3d translation_camera_from_body;
    Eigen::Vector4d intrinsics;
    Eigen::Vector2i resolution;
};

PinholeCamera make_pinhole_camera(const CameraCalibration& calibration) {
    const Eigen::Matrix3d rotation_body_from_camera = calibration.t_bs.block<3, 3>(0, 0);
    const Eigen::Vector3d translation_body_from_camera = calibration.t_bs.block<3, 1>(0, 3);
    // Rigid inverse; validate_rigid_transform guarantees it matches t_bs.inverse().
    return {
        .rotation_camera_from_body = rotation_body_from_camera.transpose(),
        .translation_camera_from_body =
            -rotation_body_from_camera.transpose() * translation_body_from_camera,
        .intrinsics = calibration.intrinsics,
        .resolution = calibration.resolution,
    };
}

ParseResult<void> validate_sampling(double rate_hz, double duration_seconds) {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
        return std::unexpected(std::format("rate_hz: expected a finite positive rate, found {}", rate_hz));
    }

    if (!std::isfinite(duration_seconds) || duration_seconds < 0.0) {
        return std::unexpected(std::format(
            "duration_seconds: expected a finite non-negative duration, found {}", duration_seconds));
    }

    return {};
}

ParseResult<void> validate_stddev(double value, std::string_view field_name) {
    if (!std::isfinite(value) || value < 0.0) {
        return std::unexpected(
            std::format("{}: expected a finite non-negative value, found {}", field_name, value));
    }

    return {};
}

ParseResult<void> validate_rigid_transform(const Eigen::Matrix4d& t_bs, std::string_view camera_name) {
    const Eigen::Matrix3d rotation = t_bs.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), kRigidTransformTolerance)
        || std::abs(rotation.determinant() - 1.0) > kRigidTransformTolerance) {
        return std::unexpected(std::format(
            "{}.t_bs: expected a rotation block in SO(3), found a non-orthonormal block", camera_name));
    }

    const Eigen::Vector4d bottom_row = t_bs.row(3);
    if ((bottom_row - Eigen::Vector4d{0.0, 0.0, 0.0, 1.0}).norm() > kRigidTransformTolerance) {
        return std::unexpected(std::format(
            "{}.t_bs: expected bottom row [0, 0, 0, 1], found [{}, {}, {}, {}]",
            camera_name,
            bottom_row.x(),
            bottom_row.y(),
            bottom_row.z(),
            bottom_row.w()));
    }

    return {};
}

// u0 - u1 is only a disparity for a rectified rig, so reject anything else instead
// of reporting a meaningless number or silently dropping every observation.
ParseResult<void> validate_rectified_stereo(
    const CameraCalibration& cam0_calibration,
    const CameraCalibration& cam1_calibration) {
    const Eigen::Matrix3d cam0_rotation = cam0_calibration.t_bs.block<3, 3>(0, 0);
    const Eigen::Matrix3d cam1_rotation = cam1_calibration.t_bs.block<3, 3>(0, 0);
    if ((cam0_rotation - cam1_rotation).norm() > kRigidTransformTolerance) {
        return std::unexpected(
            std::string{"stereo rig: expected cam0 and cam1 to share an orientation, found a relative rotation"});
    }

    const Eigen::Vector3d baseline_camera = cam0_rotation.transpose()
        * (cam1_calibration.t_bs.block<3, 1>(0, 3) - cam0_calibration.t_bs.block<3, 1>(0, 3));
    if (baseline_camera.x() <= 0.0) {
        return std::unexpected(std::format(
            "stereo rig: expected a positive x baseline from cam0 to cam1, found {}", baseline_camera.x()));
    }

    if (baseline_camera.tail<2>().norm() > kRigidTransformTolerance * baseline_camera.norm()) {
        return std::unexpected(std::format(
            "stereo rig: expected a baseline along cam0 x, found y={} z={}",
            baseline_camera.y(),
            baseline_camera.z()));
    }

    return {};
}

ParseResult<void> validate_camera(const CameraCalibration& calibration, std::string_view camera_name) {
    if (auto rigid = validate_rigid_transform(calibration.t_bs, camera_name); !rigid) {
        return rigid;
    }

    if (calibration.resolution.x() <= 0 || calibration.resolution.y() <= 0) {
        return std::unexpected(std::format(
            "{}.resolution: expected positive dimensions, found {}x{}",
            camera_name,
            calibration.resolution.x(),
            calibration.resolution.y()));
    }

    if (calibration.intrinsics.x() <= 0.0 || calibration.intrinsics.y() <= 0.0) {
        return std::unexpected(std::format(
            "{}.intrinsics: expected positive fx and fy, found fx={} fy={}",
            camera_name,
            calibration.intrinsics.x(),
            calibration.intrinsics.y()));
    }

    return {};
}

std::optional<Eigen::Vector2d> project_landmark(
    const GroundTruthState& truth,
    const SyntheticLandmark& landmark,
    const PinholeCamera& camera) {
    const Sophus::SO3d orientation{truth.orientation};
    const Eigen::Vector3d landmark_body = orientation.inverse() * (landmark.position_world - truth.position);
    const Eigen::Vector3d landmark_camera =
        camera.rotation_camera_from_body * landmark_body + camera.translation_camera_from_body;

    if (landmark_camera.z() <= 0.0) {
        return std::nullopt;
    }

    const double fx = camera.intrinsics.x();
    const double fy = camera.intrinsics.y();
    const double cx = camera.intrinsics.z();
    const double cy = camera.intrinsics.w();
    const Eigen::Vector2d pixel{
        fx * landmark_camera.x() / landmark_camera.z() + cx,
        fy * landmark_camera.y() / landmark_camera.z() + cy,
    };

    if (pixel.x() < 0.0 || pixel.x() >= static_cast<double>(camera.resolution.x())
        || pixel.y() < 0.0 || pixel.y() >= static_cast<double>(camera.resolution.y())) {
        return std::nullopt;
    }

    return pixel;
}

}  // namespace

GroundTruthState SyntheticTrajectory::state_at(double time_seconds) const {
    const Sophus::SO3d orientation =
        initial_orientation * Sophus::SO3d::exp(body_angular_velocity * time_seconds);
    Eigen::Vector3d sinusoidal_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d sinusoidal_position = Eigen::Vector3d::Zero();
    if (sinusoidal_frequency_rad_per_sec != 0.0) {
        const double frequency = sinusoidal_frequency_rad_per_sec;
        sinusoidal_velocity =
            sinusoidal_world_acceleration * ((1.0 - std::cos(frequency * time_seconds)) / frequency);
        sinusoidal_position =
            sinusoidal_world_acceleration
            * ((time_seconds / frequency) - (std::sin(frequency * time_seconds) / (frequency * frequency)));
    }

    return {
        .timestamp = timestamp_at(start_timestamp, time_seconds),
        .position = initial_position + initial_velocity * time_seconds
            + 0.5 * world_acceleration * time_seconds * time_seconds + sinusoidal_position,
        .orientation = orientation.unit_quaternion(),
        .velocity = initial_velocity + world_acceleration * time_seconds + sinusoidal_velocity,
        .gyroscope_bias = gyroscope_bias,
        .accelerometer_bias = accelerometer_bias,
    };
}

Eigen::Vector3d SyntheticTrajectory::acceleration_at(double time_seconds) const {
    return world_acceleration
        + sinusoidal_world_acceleration * std::sin(sinusoidal_frequency_rad_per_sec * time_seconds);
}

Eigen::Vector3d SyntheticTrajectory::angular_velocity_at(double /*time_seconds*/) const {
    return body_angular_velocity;
}

double SyntheticTrajectory::time_at(TimestampNs timestamp) const {
    // Signed difference so a timestamp before the trajectory start does not wrap.
    const auto elapsed_ns = static_cast<std::int64_t>(timestamp) - static_cast<std::int64_t>(start_timestamp);
    return static_cast<double>(elapsed_ns) / kNanosecondsPerSecond;
}

ImuCalibration make_synthetic_imu_calibration(
    double rate_hz,
    const std::optional<SyntheticImuNoiseConfig>& noise) {
    return {
        .t_bs = Eigen::Matrix4d::Identity(),
        .rate_hz = rate_hz,
        .gyroscope_noise_density = noise ? noise->gyroscope_noise_density : 0.0,
        .gyroscope_random_walk = 0.0,
        .accelerometer_noise_density = noise ? noise->accelerometer_noise_density : 0.0,
        // Biases are constant in this trajectory model, so there is no random walk.
        .accelerometer_random_walk = 0.0,
    };
}

CameraCalibration make_synthetic_pinhole_camera_calibration(
    const Eigen::Matrix4d& t_bs,
    double rate_hz,
    const Eigen::Vector2i& resolution,
    const Eigen::Vector4d& intrinsics) {
    return {
        .t_bs = t_bs,
        .rate_hz = rate_hz,
        .resolution = resolution,
        .intrinsics = intrinsics,
        .distortion_coefficients = Eigen::Vector4d::Zero(),
    };
}

std::vector<SyntheticLandmark> make_synthetic_landmarks() {
    return {
        {.id = 0, .position_world = Eigen::Vector3d{-1.0, -0.5, 4.0}},
        {.id = 1, .position_world = Eigen::Vector3d{0.0, 0.0, 5.0}},
        {.id = 2, .position_world = Eigen::Vector3d{1.2, 0.4, 7.5}},
        {.id = 3, .position_world = Eigen::Vector3d{-2.0, 1.0, 12.0}},
        {.id = 4, .position_world = Eigen::Vector3d{2.5, -1.2, 20.0}},
    };
}

ParseResult<std::vector<GroundTruthState>> sample_ground_truth(
    const SyntheticTrajectory& trajectory,
    double rate_hz,
    double duration_seconds) {
    if (auto sampling = validate_sampling(rate_hz, duration_seconds); !sampling) {
        return std::unexpected(sampling.error());
    }

    const std::int64_t samples = sample_count(rate_hz, duration_seconds);
    std::vector<GroundTruthState> states;
    states.reserve(static_cast<std::size_t>(samples));

    const double timestep_seconds = 1.0 / rate_hz;
    for (std::int64_t sample = 0; sample < samples; ++sample) {
        states.push_back(trajectory.state_at(static_cast<double>(sample) * timestep_seconds));
    }

    return states;
}

ParseResult<std::vector<ImuMeasurement>> synthesize_imu(
    const SyntheticTrajectory& trajectory,
    const SyntheticImuConfig& config) {
    if (auto sampling = validate_sampling(config.rate_hz, config.duration_seconds); !sampling) {
        return std::unexpected(sampling.error());
    }

    if (config.noise) {
        if (auto valid =
                validate_stddev(config.noise->accelerometer_noise_density, "accelerometer_noise_density");
            !valid) {
            return std::unexpected(valid.error());
        }

        if (auto valid = validate_stddev(config.noise->gyroscope_noise_density, "gyroscope_noise_density");
            !valid) {
            return std::unexpected(valid.error());
        }
    }

    const std::int64_t samples = sample_count(config.rate_hz, config.duration_seconds);
    std::vector<ImuMeasurement> measurements;
    measurements.reserve(static_cast<std::size_t>(samples));

    GaussianSampler sampler{config.noise ? config.noise->seed : 0};
    // Discretize the densities onto the sample grid: sigma_d = sigma_c * sqrt(rate).
    const double accelerometer_stddev =
        config.noise ? config.noise->accelerometer_noise_density * std::sqrt(config.rate_hz) : 0.0;
    const double gyroscope_stddev =
        config.noise ? config.noise->gyroscope_noise_density * std::sqrt(config.rate_hz) : 0.0;

    const double timestep_seconds = 1.0 / config.rate_hz;
    for (std::int64_t sample = 0; sample < samples; ++sample) {
        const double time_seconds = static_cast<double>(sample) * timestep_seconds;
        const auto truth = trajectory.state_at(time_seconds);
        const Sophus::SO3d orientation{truth.orientation};
        const Eigen::Vector3d specific_force =
            orientation.inverse() * (trajectory.acceleration_at(time_seconds) - kGravity);

        Eigen::Vector3d acceleration = specific_force + trajectory.accelerometer_bias;
        Eigen::Vector3d angular_velocity =
            trajectory.angular_velocity_at(time_seconds) + trajectory.gyroscope_bias;
        if (config.noise) {
            acceleration += accelerometer_stddev * sampler.sample_vector();
            angular_velocity += gyroscope_stddev * sampler.sample_vector();
        }

        measurements.push_back({
            .timestamp = timestamp_at(trajectory.start_timestamp, time_seconds),
            .acceleration = acceleration,
            .angular_velocity = angular_velocity,
        });
    }

    return measurements;
}

ParseResult<std::vector<SyntheticStereoObservation>> synthesize_stereo_observations(
    const SyntheticTrajectory& trajectory,
    const std::vector<SyntheticLandmark>& landmarks,
    const CameraCalibration& cam0_calibration,
    const CameraCalibration& cam1_calibration,
    const SyntheticCameraConfig& config) {
    if (auto sampling = validate_sampling(config.rate_hz, config.duration_seconds); !sampling) {
        return std::unexpected(sampling.error());
    }

    if (auto valid = validate_stddev(config.minimum_disparity_pixels, "minimum_disparity_pixels"); !valid) {
        return std::unexpected(valid.error());
    }

    if (config.pixel_noise) {
        if (auto valid = validate_stddev(config.pixel_noise->pixel_stddev, "pixel_stddev"); !valid) {
            return std::unexpected(valid.error());
        }
    }

    if (auto valid = validate_camera(cam0_calibration, "cam0"); !valid) {
        return std::unexpected(valid.error());
    }

    if (auto valid = validate_camera(cam1_calibration, "cam1"); !valid) {
        return std::unexpected(valid.error());
    }

    if (auto valid = validate_rectified_stereo(cam0_calibration, cam1_calibration); !valid) {
        return std::unexpected(valid.error());
    }

    const PinholeCamera cam0 = make_pinhole_camera(cam0_calibration);
    const PinholeCamera cam1 = make_pinhole_camera(cam1_calibration);

    const std::int64_t frames = sample_count(config.rate_hz, config.duration_seconds);
    std::vector<SyntheticStereoObservation> observations;
    observations.reserve(static_cast<std::size_t>(frames) * landmarks.size());

    GaussianSampler sampler{config.pixel_noise ? config.pixel_noise->seed : 0};
    const double pixel_stddev = config.pixel_noise ? config.pixel_noise->pixel_stddev : 0.0;

    const double timestep_seconds = 1.0 / config.rate_hz;
    for (std::int64_t frame = 0; frame < frames; ++frame) {
        const double time_seconds = static_cast<double>(frame) * timestep_seconds;
        const auto truth = trajectory.state_at(time_seconds);
        const TimestampNs timestamp = timestamp_at(trajectory.start_timestamp, time_seconds);

        for (const auto& landmark : landmarks) {
            const auto cam0_pixel = project_landmark(truth, landmark, cam0);
            const auto cam1_pixel = project_landmark(truth, landmark, cam1);
            if (!cam0_pixel || !cam1_pixel) {
                continue;
            }

            Eigen::Vector2d noisy_cam0_pixel = *cam0_pixel;
            Eigen::Vector2d noisy_cam1_pixel = *cam1_pixel;
            if (config.pixel_noise) {
                noisy_cam0_pixel += pixel_stddev * Eigen::Vector2d{sampler(), sampler()};
                noisy_cam1_pixel += pixel_stddev * Eigen::Vector2d{sampler(), sampler()};
            }

            const double disparity_pixels = noisy_cam0_pixel.x() - noisy_cam1_pixel.x();
            if (disparity_pixels <= config.minimum_disparity_pixels) {
                continue;
            }

            observations.push_back({
                .timestamp = timestamp,
                .landmark_id = landmark.id,
                .cam0_pixel = noisy_cam0_pixel,
                .cam1_pixel = noisy_cam1_pixel,
                .disparity_pixels = disparity_pixels,
            });
        }
    }

    return observations;
}
