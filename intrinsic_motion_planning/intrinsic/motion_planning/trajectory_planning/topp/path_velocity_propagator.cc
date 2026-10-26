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

#include "intrinsic/motion_planning/trajectory_planning/topp/path_velocity_propagator.h"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/math/numopt/simplex.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic {
namespace topp {

namespace {

// This factor is used to tighten the constraints in the backward propagation of
// the phase-state to create a safety numerical buffer for the forward pass.
constexpr double kConstraintTighteningFactor = 0.99;

// Threshold below which a coefficient is considered to be numerically zero.
constexpr double kZeroThreshold = 1.0e-12;

// Validates that all input spans have consistent dimensions relative to the
// total number of path waypoints.
//
// Specifically, ensures that `path_increments` contains exactly one fewer
// element than `squared_path_velocity_bounds` (N - 1), and that optional joint
// or Cartesian constraint vectors match the waypoint count (N) if provided.
// Returns an InvalidArgumentError upon any dimension mismatch.
absl::Status ValidateInputContainersSize(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints,
    absl::Span<const PhaseSpaceCartesianConstraint> cartesian_constraints) {
  const size_t num_samples = squared_path_velocity_bounds.size();
  if (path_increments.size() + 1 != num_samples) {
    return absl::InvalidArgumentError(
        "The size of path_increments + 1 must match the size of "
        "squared_path_velocity_bounds.");
  }
  if (!joint_constraints.empty() && (joint_constraints.size() != num_samples)) {
    return absl::InvalidArgumentError(
        "The size of joint_constraints must match the size of "
        "squared_path_velocity_bounds.");
  }
  if (!cartesian_constraints.empty() &&
      (cartesian_constraints.size() != num_samples)) {
    return absl::InvalidArgumentError(
        "The size of cartesian_constraints must match the size of "
        "squared_path_velocity_bounds.");
  }
  return absl::OkStatus();
}

// Validates that the data of the input containers is consistent.
//
// Ensures that path increments are strictly positive (guaranteeing monotonic
// path progression), squared path velocity bounds are non-negative (allowing a
// minimal tolerance for floating-point noise), and all inner Eigen vector
// components within the joint constraints have identical dimensions matching
// the robot's degrees of freedom. Returns an InvalidArgumentError if any
// condition is violated.
absl::Status ValidateDataOfInputContainersIsConsistent(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints) {
  for (size_t i = 0; i < path_increments.size(); ++i) {
    if (path_increments[i] <= 0.0) {
      return absl::InvalidArgumentError(
          absl::StrCat("Path increments must be strictly positive. Got ",
                       path_increments[i], " at index ", i, "."));
    }
  }
  for (size_t i = 0; i < squared_path_velocity_bounds.size(); ++i) {
    if (squared_path_velocity_bounds[i] < 0.0) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Squared path velocity bounds must be non-negative. Got ",
          squared_path_velocity_bounds[i], " at index ", i, "."));
    }
  }
  if (!joint_constraints.empty()) {
    const int num_dofs = joint_constraints.front().coeff_b.size();
    for (const auto& constraint : joint_constraints) {
      if ((constraint.coeff_b.size() != num_dofs) ||
          (constraint.twice_coeff_bp.size() != num_dofs) ||
          (constraint.offset.size() != num_dofs) ||
          (constraint.limits.size() != num_dofs)) {
        return absl::InvalidArgumentError(absl::StrCat(
            "All joint constraint components must have the same size."));
      }
    }
  }
  return absl::OkStatus();
}

// Analytical method to solve a 1D problem. We strictly lock the current squared
// path velocity to `initial_b` to guarantee continuity.
absl::StatusOr<double> SolveForwardPassStep(
    const double initial_b, const double ds,
    const Eigen::Ref<eigenmath::MatrixXd>& A,
    const Eigen::Ref<eigenmath::VectorXd> b,
    const double feasibility_tolerance = 1.0e-6) {
  double bp_min = -std::numeric_limits<double>::infinity();
  double bp_max = std::numeric_limits<double>::infinity();

  // Loop through all active constraints and isolate `bp[k]`.
  for (size_t i = 0; i < A.rows(); ++i) {
    const double a0 = A(i, 0);
    const double a1 = A(i, 1);
    const double rhs = b(i);

    // Evaluate the slack left after accounting for the known `initial_b`.
    const double slack = rhs - a0 * initial_b;

    if (std::abs(a1) > kZeroThreshold) {
      const double value = slack / a1;
      if (a1 > 0.0) {
        // Choose the most constraining upper bound.
        bp_max = std::min(bp_max, value);
      } else {
        // Choose the most constraining lower bound.
        bp_min = std::max(bp_min, value);
      }
    } else {
      // If `a1` is ~0.0, the constraint is independent of the control `bp`. We
      // need to ensure `slack >= 0`. Otherwise, the problem is infeasible.
      if (slack < -feasibility_tolerance) {
        return absl::InternalError(
            "Forward pass infeasible at current velocity.");
      }
    }
  }

  // Feasibility check and selection of the control `bp`. If the maximum
  // allowed step is smaller than the minimum required step, check whether
  // `bp_max` still satisfies all constraints within `feasibility_tolerance`
  // (avoiding false infeasibility when a tiny `a1` coefficient magnifies
  // floating-point roundoff upon division).
  if (bp_max < bp_min - feasibility_tolerance &&
      (A.col(0) * initial_b + A.col(1) * bp_max - b).maxCoeff() >
          feasibility_tolerance) {
    return absl::InternalError(
        "Forward pass constraints are mutually exclusive.");
  }

  // Since ds > 0, to maximize b_{k+1} = initial_b + ds * bp, we must pick the
  // maximum valid bp.
  return std::max(initial_b + ds * bp_max, 0.0);
}

// Solves a single-step linear program (LP) to propagate valid squared path
// velocity bounds to the adjacent waypoint in the (b, b') phase space. `b`
// denotes the squared path velocity (ds/dt^2) and `bp` its first derivative
// with respect to the path (`bp = db(s)/ds`).
//
// This function determines the maximum reachable squared path velocity at the
// target waypoint while strictly satisfying dynamic, kinematic, and velocity
// constraints over the discrete interval `ds`. The LP decision variables are
// structured natively as y = [b[k], bp[k]]^T.
//
// Depending on the operational mode (`is_forward_pass`), the LP optimizes:
//   - Forward Pass:  Maximizes downstream velocity b[k+1] = b[k] + ds * bp[k]
//                    by setting the objective vector c = [1.0, ds]^T.
//   - Backward Pass: Maximizes current velocity b[k] to find the upper boundary
//                    of the controllable set by setting c = [1.0, 0.0]^T.
//
// Constraints enforced within the pre-allocated buffers `A` and `b`:
//   1. Path Velocity Bounds: 0 <= b[k] <= upper_b and
//                            0 <= b[k] + ds * bp[k] <= next_b_bound.
//   2. Joint Constraints: Kinematic acceleration or dynamic torque bounds
//      mapped to phase space. Enforced at both the current and next waypoints
//      (interpolated per Appendix D of TOPP-RA).
//   3. Cartesian Constraints: Labeled translation and rotation 2-norm tracking
//      limits mapped as inner octagon half-space approximations.
//
// Parameters:
//   `is_forward_pass`: True optimizes the downstream state (forward
//                      integration); false optimizes the current state
//                      (backward reachability).
//   `ds`: Spatial grid step size (s[k+1] - s[k]) for the current interval.
//   `upper_b`: Maximum allowable squared path speed at the current waypoint k.
//   `next_b_bound`: Maximum allowable squared path speed at the target
//                   waypoint.
//   `joint_pt`: Joint limit basis coefficients at the current waypoint.
//   `joint_next_pt`: Joint limit basis coefficients at the next waypoint.
//   `cart_pt`: Cartesian tracking limits at the current waypoint.
//   `cart_next_pt`: Cartesian tracking limits at the next waypoint.
//   `A`: Pre-allocated LP constraint coefficient matrix block.
//   `b`: Pre-allocated LP constraint limit vector block.
//   `use_analytical_forward_pass`: If true, the constraint tightening factor
//   `kConstraintTighteningFactor` is used to provide a safety buffer for the
//   backward pass.
//
// Returns the optimized target squared path velocity (b[k+1] for forward pass,
// b[k] for backward pass), or an InternalError if the LP could not be solved to
// optimality.
absl::StatusOr<double> PropagateReachablePathSpeeds(
    bool is_forward_pass, const double ds, const double upper_b,
    const double next_b_bound,
    const JointConstraintStep* absl_nullable joint_pt,
    const JointConstraintStep* absl_nullable joint_next_pt,
    const PhaseSpaceCartesianConstraint* absl_nullable cart_pt,
    const PhaseSpaceCartesianConstraint* absl_nullable cart_next_pt,
    Eigen::Ref<eigenmath::MatrixX2d> A, Eigen::Ref<eigenmath::VectorXd> b,
    const bool use_analytical_forward_pass = false) {
  // Only for the backward pass, we use a constraint tightening factor to
  // provide a safety buffer when using an analytical approach to propagate the
  // reachable path speeds.
  const double constraint_tightening_factor =
      (!is_forward_pass && use_analytical_forward_pass)
          ? kConstraintTighteningFactor
          : 1.0;

  const eigenmath::Vector2d c(1.0, is_forward_pass ? ds : 0.0);

  // Constraints:  0 <= b <= upper_b
  //               0 <= b + ds * bp <= next_b_bound
  A.col(0).head<4>() << -1.0, 1.0, -1.0, 1.0;
  A.col(1).head<4>() << 0.0, 0.0, -ds, ds;
  b.head<4>() << 0.0, constraint_tightening_factor * upper_b, 0.0,
      constraint_tightening_factor * next_b_bound;

  size_t start = 4;
  if (joint_pt && joint_next_pt) {
    const auto num_dimensions = joint_pt->coeff_b.size();

    A.block(start, 0, num_dimensions, 1) = joint_pt->coeff_b;
    A.block(start, 1, num_dimensions, 1) = 0.5 * joint_pt->twice_coeff_bp;
    b.segment(start, num_dimensions) =
        constraint_tightening_factor * joint_pt->limits - joint_pt->offset;
    start += num_dimensions;

    A.block(start, 0, num_dimensions, 1) = -joint_pt->coeff_b;
    A.block(start, 1, num_dimensions, 1) = -0.5 * joint_pt->twice_coeff_bp;
    b.segment(start, num_dimensions) =
        constraint_tightening_factor * joint_pt->limits + joint_pt->offset;
    start += num_dimensions;

    A.block(start, 0, num_dimensions, 1) = joint_next_pt->coeff_b;
    A.block(start, 1, num_dimensions, 1) =
        0.5 * joint_next_pt->twice_coeff_bp + ds * joint_next_pt->coeff_b;
    b.segment(start, num_dimensions) =
        constraint_tightening_factor * joint_next_pt->limits -
        joint_next_pt->offset;
    start += num_dimensions;

    A.block(start, 0, num_dimensions, 1) = -joint_next_pt->coeff_b;
    A.block(start, 1, num_dimensions, 1) =
        -(0.5 * joint_next_pt->twice_coeff_bp + ds * joint_next_pt->coeff_b);
    b.segment(start, num_dimensions) =
        constraint_tightening_factor * joint_next_pt->limits +
        joint_next_pt->offset;
    start += num_dimensions;
  }

  if (cart_pt && cart_next_pt) {
    auto add_2norm_constraints =
        [&](const PhaseSpaceCartAccTwoNormConstraint& c_space, bool is_next) {
          eigenmath::Vector4d h1_mapped = c_space.h1;
          if (is_next) {
            h1_mapped += ds * c_space.h0;
          }
          A.block<4, 1>(start, 0) = c_space.h0;
          A.block<4, 1>(start, 1) = h1_mapped;
          A.block<4, 1>(start + 4, 0) = -c_space.h0;
          A.block<4, 1>(start + 4, 1) = -h1_mapped;
          b.segment<8>(start).setConstant(constraint_tightening_factor *
                                          c_space.effective_limit);
          start += 8;
        };

    add_2norm_constraints(cart_pt->trans_constraint, /*is_next=*/false);
    add_2norm_constraints(cart_pt->rot_constraint, /*is_next=*/false);
    add_2norm_constraints(cart_next_pt->trans_constraint, /*is_next=*/true);
    add_2norm_constraints(cart_next_pt->rot_constraint, /*is_next=*/true);
  }

  // Use an analytical method to solve the 1D problem in the forward pass.
  if (is_forward_pass && use_analytical_forward_pass) {
    return SolveForwardPassStep(upper_b, ds, A.topRows(start), b.head(start));
  }

  // Use the simplex method to solve the 2D problem in the backward pass or
  // forward pass if `use_analytical_forward_pass` is false.
  static const SimplexOptions kSimplexOptions{.free_primal_variables = {1}};
  INTR_ASSIGN_OR_RETURN(const SimplexResult result,
                        SimplexSolve(c, A, b, kSimplexOptions));
  if (result.status != SimplexStatus::kOptimal) {
    return absl::InternalError(
        is_forward_pass
            ? "Forward pass problem could not be solved to optimality."
            : "Backward pass problem could not be solved to optimality.");
  }

  return result.solution(0) + c(1) * result.solution(1);
}

// Computes the upper boundary of the controllable velocity set by propagating
// physical constraints backward from the end of the path.
//
// Anchors the terminal squared path velocity to zero (guaranteeing the robot
// can safely come to a full stop at the target frame) and iteratively solves
// single-step linear programs backward from the path end to the path start. At
// each step, it identifies the maximum entry speed that can admissibly
// transition to the valid downstream state while respecting feasibility limits
// (e.g. joint or Cartesian velocity limits given by
// `squared_path_velocity_bounds`, joint acceleration limits given by
// `joint_constraints`, Cartesian acceleration limits given by
// `cartesian_constraints`, and next reachable path velocity given by
// `backward_pass_squared_path_velocity_bounds`). The `path_increments` are the
// spatial grid step sizes `ds[k] = s[k+1] - s[k]` between consecutive path
// samples. The matrix `A` and vector `b` are a pre-allocated storage to
// formulate the optimization problem efficiently.
//
// Returns a vector of the backward reachable squared path velocities,
// or an InternalError if the profile is physically infeasible to traverse.
absl::StatusOr<std::vector<double>> ComputeBackwardReachabilityPass(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints,
    absl::Span<const PhaseSpaceCartesianConstraint> cartesian_constraints,
    Eigen::Ref<eigenmath::MatrixX2d> A, Eigen::Ref<eigenmath::VectorXd> b,
    const bool use_analytical_forward_pass) {
  const int num_samples = squared_path_velocity_bounds.size();
  std::vector<double> backward_velocities(num_samples, 0.0);

  // Boundary configuration: squared path velocity ends at zero.
  double upper_b = 0.0;
  backward_velocities.back() = upper_b;

  // Extract raw base pointers once
  const JointConstraintStep* const joint_ptr =
      joint_constraints.empty() ? nullptr : joint_constraints.data();
  const PhaseSpaceCartesianConstraint* const cart_ptr =
      cartesian_constraints.empty() ? nullptr : cartesian_constraints.data();

  for (int i = num_samples - 2; i >= 0; --i) {
    INTR_ASSIGN_OR_RETURN(
        upper_b,
        PropagateReachablePathSpeeds(
            /*is_forward_pass=*/false, path_increments[i],
            squared_path_velocity_bounds[i],
            std::min(squared_path_velocity_bounds[i + 1], upper_b),
            joint_ptr ? joint_ptr + i : nullptr,
            joint_ptr ? joint_ptr + i + 1 : nullptr,
            cart_ptr ? cart_ptr + i : nullptr,
            cart_ptr ? cart_ptr + i + 1 : nullptr, A, b,
            use_analytical_forward_pass),
        _ << "Backward pass generation is physically infeasible.");

    backward_velocities[i] = upper_b;
  }

  return backward_velocities;
}

// Integrates the maximal feasible velocity profile forward along the path.
//
// Performs a greedy forward integration of the squared path velocity profile,
// starting from `start_squared_path_velocity` and bounded by the intersection
// of the input limit envelopes and the pre-computed backward reachability pass.
// At each interval, it solves a local linear program to maximize the next-step
// squared velocity. Returns the integrated velocity vector, or an error status
// if the integration violates feasibility limits (e.g. joint or Cartesian
// velocity limits given by `squared_path_velocity_bounds`, joint acceleration
// limits given by `joint_constraints`, Cartesian acceleration limits given by
// `cartesian_constraints`, and next reachable path velocity given by
// `backward_pass_squared_path_velocity_bounds`). The `path_increments` are the
// spatial grid step sizes `ds[k] = s[k+1] - s[k]` between consecutive path
// samples. `start_squared_path_velocity` represents the start value for the
// squared path velocity `b[0]`. The matrix `A` and vector `b` are a
// pre-allocated storage to formulate the optimization problem efficiently.
absl::StatusOr<std::vector<double>> ComputeForwardIntegrationPass(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const double> backward_pass_squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints,
    absl::Span<const PhaseSpaceCartesianConstraint> cartesian_constraints,
    const double start_squared_path_velocity,
    Eigen::Ref<eigenmath::MatrixX2d> A, Eigen::Ref<eigenmath::VectorXd> b,
    const bool use_analytical_forward_pass = false) {
  if (start_squared_path_velocity >
      backward_pass_squared_path_velocity_bounds.front()) {
    return absl::InvalidArgumentError(
        "A path that starts with the given velocity cannot come to a stop in "
        "time.");
  }

  const int num_samples = squared_path_velocity_bounds.size();
  std::vector<double> forward_velocities(num_samples, 0.0);

  // Boundary configuration: squared path velocity begins at zero.
  double upper_b = start_squared_path_velocity;
  forward_velocities.front() = upper_b;

  // Extract raw base pointers once
  const JointConstraintStep* const joint_ptr =
      joint_constraints.empty() ? nullptr : joint_constraints.data();
  const PhaseSpaceCartesianConstraint* const cart_ptr =
      cartesian_constraints.empty() ? nullptr : cartesian_constraints.data();

  for (int i = 1; i < num_samples - 1; ++i) {
    INTR_ASSIGN_OR_RETURN(
        upper_b,
        PropagateReachablePathSpeeds(
            /*is_forward_pass=*/true, path_increments[i - 1], upper_b,
            std::min(backward_pass_squared_path_velocity_bounds[i],
                     squared_path_velocity_bounds[i]),
            joint_ptr ? joint_ptr + i - 1 : nullptr,
            joint_ptr ? joint_ptr + i : nullptr,
            cart_ptr ? cart_ptr + i - 1 : nullptr,
            cart_ptr ? cart_ptr + i : nullptr, A, b,
            use_analytical_forward_pass),
        _ << "Forward pass trajectory integration broke feasibility limits.");

    forward_velocities[i] = upper_b;
  }

  forward_velocities.back() = 0.0;
  return forward_velocities;
}

}  // namespace

absl::StatusOr<std::vector<double>> PropagateSquaredPathVelocities(
    absl::Span<const double> path_increments,
    absl::Span<const double> squared_path_velocity_bounds,
    absl::Span<const JointConstraintStep> joint_constraints,
    absl::Span<const PhaseSpaceCartesianConstraint> cartesian_constraints,
    const double start_squared_path_velocity, bool skip_data_validation,
    const bool use_analytical_forward_pass) {
  INTR_RETURN_IF_ERROR(
      ValidateInputContainersSize(path_increments, squared_path_velocity_bounds,
                                  joint_constraints, cartesian_constraints));
  if (!skip_data_validation) {
    INTR_RETURN_IF_ERROR(ValidateDataOfInputContainersIsConsistent(
        path_increments, squared_path_velocity_bounds, joint_constraints));
  }

  // Prepare the storage to define the optimization problem.
  const size_t num_dofs =
      joint_constraints.empty() ? 0 : joint_constraints.front().coeff_b.size();
  const size_t num_rows = 4 + (!joint_constraints.empty() ? 4 * num_dofs : 0) +
                          (!cartesian_constraints.empty() ? 4 * 8 : 0);
  eigenmath::MatrixX2d A(num_rows, 2);
  eigenmath::VectorXd b(num_rows);

  // Run backward and forward passes.
  INTR_ASSIGN_OR_RETURN(
      const std::vector<double> backward_pass_squared_path_velocity_bounds,
      ComputeBackwardReachabilityPass(
          path_increments, squared_path_velocity_bounds, joint_constraints,
          cartesian_constraints, A, b, use_analytical_forward_pass));

  return ComputeForwardIntegrationPass(
      path_increments, squared_path_velocity_bounds,
      backward_pass_squared_path_velocity_bounds, joint_constraints,
      cartesian_constraints, start_squared_path_velocity, A, b,
      use_analytical_forward_pass);
}

}  // namespace topp
}  // namespace intrinsic
