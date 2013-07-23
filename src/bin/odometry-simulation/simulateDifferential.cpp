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
    \brief This file simulates and tests the differential odometry.
  */

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "utils/utils.h"
#include "generic/DifferentialOdometry.h"
#include "generic/DifferentialOdometryKalmanFilter.h"
#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"

using namespace janeth;

int main(int argc, char** argv) {
  const size_t steps = 5000;
  const double T = 0.1;
  std::vector<Eigen::Vector3d> u_true;
  const double sineWaveAmplitude = 1.0;
  const double sineWaveFrequency = 0.01;
  genSineWavePath(u_true, steps, sineWaveAmplitude, sineWaveFrequency, T);
  DifferentialOdometry::Parameters params = {0.285, 0.285, 0.285, 0.285,
    1.285, 1.285, 0.000045};
  DifferentialOdometry odometry(params);
  double timestamp = 0;
  std::ofstream diffOdoFile("diffOdo.txt");
  for (auto it = u_true.cbegin(); it != u_true.cend(); ++it) {
    odometry.updateRearWheelTranslationalVelocities(
      (*it)(0) - (*it)(2) * params.rearWheelTrack * 0.5,
      (*it)(0) + (*it)(2) * params.rearWheelTrack * 0.5, timestamp);
    timestamp += T;
    diffOdoFile << std::fixed << std::setprecision(16)
      << odometry.getPose().transpose() << std::endl;
  }
  KalmanFilter::Parameters kfParams = {Eigen::Matrix2d::Identity() * 1e6};
  DifferentialKalmanFilter::Parameters diffKfParams = {0.1, 0.1};
  DifferentialOdometryKalmanFilter kfOdometry(params, kfParams, diffKfParams);
  timestamp = 0;
  std::ofstream diffOdoKfFile("diffOdoKf.txt");
  for (auto it = u_true.cbegin(); it != u_true.cend(); ++it) {
    kfOdometry.updateRearWheelTranslationalVelocities(
      (*it)(0) - (*it)(2) * params.rearWheelTrack * 0.5,
      (*it)(0) + (*it)(2) * params.rearWheelTrack * 0.5, timestamp);
    timestamp += T;
    diffOdoKfFile << std::fixed << std::setprecision(16)
      << kfOdometry.getPose().transpose() << std::endl;
  }
  return 0;
}
