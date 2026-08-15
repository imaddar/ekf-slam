#include "landmark_augmentation.hpp"

#include "augmentation_jacobians.hpp"
#include "stereo_geometry.hpp"
#include "triangulation.hpp"

#include <format>

ParseResult<void> augment_landmark(
    SlamState& state,
    LandmarkId id,
    const Eigen::Vector2d& pixel_cam0,
    const Eigen::Vector2d& pixel_cam1,
    const CameraCalibration& cam0,
    const CameraCalibration& cam1,
    const Eigen::Matrix4d& pixel_covariance,
    double minimum_disparity_pixels) {
    const auto triangulation = triangulate_stereo(
        pixel_cam0,
        pixel_cam1,
        cam0,
        cam1,
        pixel_covariance,
        minimum_disparity_pixels);
    if (!triangulation) {
        return std::unexpected(triangulation.error());
    }

    const auto world_point = camera_point_to_world(
        state.robot, cam0, triangulation->point_camera);
    if (!world_point) {
        return std::unexpected(world_point.error());
    }

    const auto jacobians = make_augmentation_jacobians(
        state.robot, cam0, triangulation->point_camera);
    if (!jacobians) {
        return std::unexpected(jacobians.error());
    }

    const auto robot_covariance = state.robot_covariance();
    const Eigen::Matrix<double, kRobotDim, kLandmarkDim> robot_cross_covariance =
        robot_covariance.block<kRobotDim, 3>(0, 0)
            * jacobians->robot.block<3, 3>(0, 0).transpose()
        + robot_covariance.block<kRobotDim, 3>(0, 6)
            * jacobians->robot.block<3, 3>(0, 6).transpose();
    Eigen::Matrix<double, 6, 6> pose_covariance;
    pose_covariance << robot_covariance.block<3, 3>(0, 0), robot_covariance.block<3, 3>(0, 6),
        robot_covariance.block<3, 3>(6, 0), robot_covariance.block<3, 3>(6, 6);
    Eigen::Matrix<double, 3, 6> pose_jacobian;
    pose_jacobian << jacobians->robot.block<3, 3>(0, 0), jacobians->robot.block<3, 3>(0, 6);
    const Eigen::Matrix3d landmark_covariance =
        pose_jacobian * pose_covariance * pose_jacobian.transpose()
        + jacobians->camera * triangulation->covariance * jacobians->camera.transpose();

    const int old_active_dim = state.active_dim();
    Eigen::MatrixXd covariance_column(old_active_dim + kLandmarkDim, kLandmarkDim);
    covariance_column.topRows(kRobotDim) = robot_cross_covariance;
    if (old_active_dim > kRobotDim) {
        const auto robot_landmark_covariance = state.robot_landmark_covariance();
        covariance_column.middleRows(kRobotDim, old_active_dim - kRobotDim) =
            robot_landmark_covariance.block<3, Eigen::Dynamic>(0, 0, 3, robot_landmark_covariance.cols()).transpose()
                * jacobians->robot.block<3, 3>(0, 0).transpose()
            + robot_landmark_covariance.block<3, Eigen::Dynamic>(6, 0, 3, robot_landmark_covariance.cols()).transpose()
                * jacobians->robot.block<3, 3>(0, 6).transpose();
    }
    covariance_column.bottomRows(kLandmarkDim) = landmark_covariance;

    return state.add_landmark(id, *world_point, covariance_column);
}
