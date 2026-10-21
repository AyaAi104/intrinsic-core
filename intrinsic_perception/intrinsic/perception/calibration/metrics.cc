// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/perception/core/math_utils.h"
#include "intrinsic/perception/core/pose_utils.h"
#include "intrinsic/perception/proto/v1/camera_setup.pb.h"
#include "intrinsic/perception/proto/v1/camera_to_robot_calibration.pb.h"
#include "intrinsic/perception/proto_conversion/eigen_conversions.h"

namespace intrinsic::perception {

namespace {

using ::intrinsic_proto::perception::v1::CAMERA_SETUP_MOVING;
using ::intrinsic_proto::perception::v1::CAMERA_SETUP_UNSPECIFIED;
using ::intrinsic_proto::perception::v1::CameraSetup;
using ::intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest;
using ::intrinsic_proto::perception::v1::CameraToRobotCalibrationResult;

Eigen::Isometry3d ComputedEstimatePose(
    const CameraToRobotCalibrationRequest::InputPosePair& pose_pair,
    const CameraSetup& type, const Eigen::Isometry3d& pose) {
  const Eigen::Isometry3d camera_t_object =
      ToEigen(pose_pair.camera_t_object());
  const Eigen::Isometry3d base_t_flange = ToEigen(pose_pair.base_t_flange());

  if (type == CAMERA_SETUP_MOVING) {
    return base_t_flange * pose * camera_t_object;
  }
  return base_t_flange.inverse() * pose * camera_t_object;
}

CameraToRobotCalibrationResult ComputeErrorMetricsForRelativePoses(
    const CameraToRobotCalibrationRequest& request,
    const CameraToRobotCalibrationResult& result) {
  CameraToRobotCalibrationResult result_with_err = result;
  double translation_root_mean_square_err = 0;
  double translation_max_err = std::numeric_limits<double>::lowest();
  double rotation_root_mean_square_err = 0;
  double rotation_max_err = std::numeric_limits<double>::lowest();

  Eigen::Isometry3d result_pose;
  if (request.type() == CAMERA_SETUP_MOVING) {
    result_pose =
        ToEigen(result.moving_camera_result_poses().flange_t_camera());
  } else {
    result_pose =
        ToEigen(result.stationary_camera_result_poses().base_t_camera());
  }

  for (int i = 0; i < request.input_pose_pairs_size(); ++i) {
    int index1 = i;
    for (int j = 0; j < request.input_pose_pairs_size(); ++j) {
      int index2 = j;
      if (i == j) {
        continue;
      }
      const Eigen::Isometry3d est_pose1 = ComputedEstimatePose(
          request.input_pose_pairs(index1), request.type(), result_pose);
      const Eigen::Isometry3d est_pose2 = ComputedEstimatePose(
          request.input_pose_pairs(index2), request.type(), result_pose);
      const double rotation_err =
          Rad2Deg(ComputePoseRotationError(est_pose1, est_pose2));
      rotation_root_mean_square_err += Sqr(rotation_err);
      rotation_max_err = std::max(rotation_max_err, rotation_err);

      const double translation_err =
          ComputePoseTranslationError(est_pose1, est_pose2);
      translation_root_mean_square_err += Sqr(translation_err);
      translation_max_err = std::max(translation_max_err, translation_err);
    }
  }
  result_with_err.set_translation_maximum_error(translation_max_err);
  const double num_samples =
      (request.input_pose_pairs_size() * request.input_pose_pairs_size() -
       request.input_pose_pairs_size());
  result_with_err.set_translation_root_mean_square_error(
      std::sqrt(translation_root_mean_square_err / num_samples));
  result_with_err.set_rotation_maximum_error_in_degrees(rotation_max_err);
  result_with_err.set_rotation_root_mean_square_error_in_degrees(
      std::sqrt(rotation_root_mean_square_err / num_samples));
  return result_with_err;
}

CameraToRobotCalibrationResult ComputeErrorMetrics(
    const CameraToRobotCalibrationRequest& request,
    const CameraToRobotCalibrationResult& result) {
  CameraToRobotCalibrationResult result_with_err = result;
  std::vector<double> translation_errors;
  std::vector<double> rotation_errors;
  translation_errors.reserve(request.input_pose_pairs_size());
  rotation_errors.reserve(request.input_pose_pairs_size());

  Eigen::Isometry3d base_t_stationary;
  Eigen::Isometry3d flange_t_moving;
  if (request.type() == CAMERA_SETUP_MOVING) {
    base_t_stationary =
        ToEigen(result.moving_camera_result_poses().base_t_object());
    flange_t_moving =
        ToEigen(result.moving_camera_result_poses().flange_t_camera());
  } else {
    base_t_stationary =
        ToEigen(result.stationary_camera_result_poses().base_t_camera());
    flange_t_moving =
        ToEigen(result.stationary_camera_result_poses().flange_t_object());
  }

  for (const auto& pose_pair : request.input_pose_pairs()) {
    const Eigen::Isometry3d camera_t_object =
        ToEigen(pose_pair.camera_t_object());
    const Eigen::Isometry3d base_t_flange = ToEigen(pose_pair.base_t_flange());

    Eigen::Isometry3d camera_t_object_est;
    if (request.type() == CAMERA_SETUP_MOVING) {
      camera_t_object_est = flange_t_moving.inverse() *
                            base_t_flange.inverse() * base_t_stationary;
    } else {
      camera_t_object_est =
          base_t_stationary.inverse() * base_t_flange * flange_t_moving;
    }
    const double rotation_err =
        Rad2Deg(ComputePoseRotationError(camera_t_object, camera_t_object_est));
    rotation_errors.push_back(rotation_err);

    const double translation_err =
        ComputePoseTranslationError(camera_t_object, camera_t_object_est);
    translation_errors.push_back(translation_err);
  }

  if (translation_errors.empty()) {
    LOG(ERROR) << "No pose pairs were provided for the metric computation. "
                  "Returning without metrics.";
    return result_with_err;
  }

  const auto avg = [](const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  };
  const auto rms = [](const std::vector<double>& values) {
    return std::sqrt(
        std::accumulate(values.begin(), values.end(), 0.0,
                        [](double a, double b) { return a + Sqr(b); }) /
        values.size());
  };

  // Log a table with more detailed calibration results.
  for (int i = 0; i < translation_errors.size(); ++i) {
    LOG(INFO) << "Pose pair " << i << ":\t" << translation_errors[i] * 1000.0
              << "mm\t" << rotation_errors[i] << "deg";
  }

  std::sort(translation_errors.begin(), translation_errors.end());
  std::sort(rotation_errors.begin(), rotation_errors.end());
  LOG(INFO) << "P90: "
            << translation_errors[translation_errors.size() * 0.9] * 1000.0
            << "mm\t" << rotation_errors[rotation_errors.size() * 0.9] << "deg";
  LOG(INFO) << "Max: " << translation_errors.back() * 1000.0 << "mm\t"
            << rotation_errors.back() << "deg";
  LOG(INFO) << "Avg: " << avg(translation_errors) * 1000.0 << "mm\t"
            << avg(rotation_errors) << "deg";
  LOG(INFO) << "RMS: " << rms(translation_errors) * 1000.0 << "mm\t"
            << rms(rotation_errors) << "deg";

  result_with_err.set_translation_maximum_error(translation_errors.back());
  result_with_err.set_translation_root_mean_square_error(
      rms(translation_errors));
  result_with_err.set_rotation_maximum_error_in_degrees(rotation_errors.back());
  result_with_err.set_rotation_root_mean_square_error_in_degrees(
      rms(rotation_errors));
  return result_with_err;
}

}  // namespace

absl::StatusOr<CameraToRobotCalibrationResult>
ComputeCameraToRobotCalibrationErrorMetrics(
    const CameraToRobotCalibrationRequest& request,
    const CameraToRobotCalibrationResult& result) {
  if (request.type() == CAMERA_SETUP_UNSPECIFIED) {
    return absl::InvalidArgumentError("Invalid CameraSetup.");
  }

  if (result.has_moving_camera_result_poses() &&
      result.moving_camera_result_poses().has_base_t_object()) {
    return ComputeErrorMetrics(request, result);
  }
  if (result.has_stationary_camera_result_poses() &&
      result.stationary_camera_result_poses().has_flange_t_object()) {
    return ComputeErrorMetrics(request, result);
  }
  if (request.input_pose_pairs_size() < 2) {
    return absl::FailedPreconditionError(
        "At least 2 valid captures are required to evaluate relative pose "
        "metrics.");
  }
  return ComputeErrorMetricsForRelativePoses(request, result);
}

absl::StatusOr<intrinsic_proto::perception::v1::CameraToRobotValidationResult>
ComputeCameraToRobotValidationMetrics(
    const intrinsic_proto::perception::v1::CameraToRobotCalibrationRequest&
        request,
    const intrinsic_proto::Pose& pose) {
  if (request.input_pose_pairs_size() < 2) {
    return absl::FailedPreconditionError(
        absl::StrCat("At least 2 valid captures are required to validate the "
                     "camera-to-robot "
                     "calibration, but only ",
                     request.input_pose_pairs_size(), " were provided."));
  }

  intrinsic_proto::perception::v1::CameraToRobotCalibrationResult dummy_result;
  if (request.type() == CAMERA_SETUP_MOVING) {
    *dummy_result.mutable_moving_camera_result_poses()
         ->mutable_flange_t_camera() = pose;
  } else {
    *dummy_result.mutable_stationary_camera_result_poses()
         ->mutable_base_t_camera() = pose;
  }

  auto result_with_err =
      ComputeErrorMetricsForRelativePoses(request, dummy_result);

  intrinsic_proto::perception::v1::CameraToRobotValidationResult
      validation_result;
  validation_result.set_translation_root_mean_square_error(
      result_with_err.translation_root_mean_square_error());
  validation_result.set_translation_maximum_error(
      result_with_err.translation_maximum_error());
  validation_result.set_rotation_root_mean_square_error_in_degrees(
      result_with_err.rotation_root_mean_square_error_in_degrees());
  validation_result.set_rotation_maximum_error_in_degrees(
      result_with_err.rotation_maximum_error_in_degrees());

  return validation_result;
}

}  // namespace intrinsic::perception
