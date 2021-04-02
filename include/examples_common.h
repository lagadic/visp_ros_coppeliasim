// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

/**
 * @file examples_common.h
 * Contains common types and functions for the examples.
 */

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] q_goal Target joint positions.
   */
  MotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return Joint positions for use inside a control loop.
   */
  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};

vpMatrix InteractionMatrixPoint(const double &Z, const double &x, const double &y);

vpMatrix InteractionMatrix4Points(const vpColVector &Z, const vpColVector &s);

vpMatrix compute_Lz(const vpColVector &s,const vpColVector &Z);

vpMatrix dt_Lz(const vpColVector &s, const vpColVector &ds,
		       const vpColVector &Z,const vpColVector &dZ);

vpMatrix dt_InteractionMatrixPoint(const double &Z, const double &dZ, const double &x,
		                           const double &y, const double &dx, const double &dy);

vpMatrix dt_InteractionMatrix4Points(const vpColVector &Z, const vpColVector &dZ,
		                             const vpColVector &s, const vpColVector &ds);

vpMatrix compute_interaction_matrix_3D(const vpHomogeneousMatrix &cdMc);

vpColVector Modul_2pi(const vpColVector& x);
