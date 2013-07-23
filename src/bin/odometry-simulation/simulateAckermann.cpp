/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file simulateDifferential.cpp
    \brief This file simulates and tests the Ackermann odometry.
  */

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "utils/utils.h"
#include "generic/AckermannOdometry.h"
#include "generic/DifferentialOdometry.h"
#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"
#include "generic/AckermannKalmanFilter.h"
#include "generic/AckermannOdometryKalmanFilter.h"

using namespace janeth;

int main(int argc, char** argv) {
  const size_t steps = 5000;
  const double T = 0.1;
  std::vector<Eigen::Vector3d> u_true;
  const double sineWaveAmplitude = 1.0;
  const double sineWaveFrequency = 0.01;
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);
  DifferentialOdometry::Parameters diffParams = {0.285, 0.285, 0.285, 0.285,
    1.285, 1.285, 0.000045};
  AckermannOdometry::Parameters ackParams = {2.6996375777616732};
  AckermannOdometry odometry(diffParams, ackParams);
  double timestamp = 0;
  std::ofstream ackOdoFile("ackOdo.txt");
  for (auto it = u_true.cbegin(); it != u_true.cend(); ++it) {
    odometry.updateRearWheelTranslationalVelocitiesSteering(
      (*it)(0) - (*it)(2) * diffParams.rearWheelTrack * 0.5,
      (*it)(0) + (*it)(2) * diffParams.rearWheelTrack * 0.5,
      atan(ackParams.wheelBase * (*it)(2) / (*it)(0)), timestamp);
    timestamp += T;
    ackOdoFile << std::fixed << std::setprecision(16)
      << odometry.getPose().transpose() << std::endl;
  }
  KalmanFilter::Parameters kfParams = {Eigen::Matrix2d::Identity() * 1e6};
  DifferentialKalmanFilter::Parameters diffKfParams = {0.1, 0.1};
  AckermannKalmanFilter::Parameters ackKfParams = {0.1, 0.1, 0.1};
  AckermannOdometryKalmanFilter kfOdometry(diffParams, ackParams, kfParams,
    diffKfParams, ackKfParams);
  timestamp = 0;
  std::ofstream ackOdoKfFile("ackOdoKf.txt");
  for (auto it = u_true.cbegin(); it != u_true.cend(); ++it) {
    const double steering = atan(ackParams.wheelBase * (*it)(2) / (*it)(0));
    const double vRearLeft = (*it)(0) - (*it)(2) * diffParams.rearWheelTrack *
      0.5;
    const double vRearRight = (*it)(0) + (*it)(2) * diffParams.rearWheelTrack *
      0.5;
    const double phi_l = std::atan(std::tan(steering) * ackParams.wheelBase /
      (ackParams.wheelBase - diffParams.frontWheelTrack * 0.5 *
      std::tan(steering)));
    const double phi_r = std::atan(std::tan(steering) * ackParams.wheelBase /
      (ackParams.wheelBase + diffParams.frontWheelTrack * 0.5 *
      std::tan(steering)));
    const double vFrontLeft = ((*it)(0) - (*it)(2) * diffParams.frontWheelTrack
      * 0.5) / cos(phi_l);
    const double vFrontRight = ((*it)(0) + (*it)(2) * diffParams.frontWheelTrack
      * 0.5) / cos(phi_r);
    kfOdometry.updateRearLeftWheelTranslationalVelocity(vRearLeft, timestamp);
    kfOdometry.updateRearRightWheelTranslationalVelocity(vRearRight, timestamp);
    kfOdometry.updateFrontLeftWheelTranslationalVelocitySteering(vFrontLeft,
      steering, timestamp);
    kfOdometry.updateFrontRightWheelTranslationalVelocitySteering(vFrontRight,
      steering, timestamp);
    kfOdometry.updateSteering(steering, timestamp);
    timestamp += T;
    ackOdoKfFile << std::fixed << std::setprecision(16)
      << kfOdometry.getPose().transpose() << std::endl;
  }
  return 0;
}
