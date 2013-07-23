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

/** \file DifferentialOdometryKalmanFilter.cpp
    \brief This file is a testing binary for the
           DifferentialOdometryKalmanFilter class.
  */

#include <iostream>

#include <gtest/gtest.h>

#include "generic/DifferentialOdometry.h"
#include "generic/DifferentialOdometryKalmanFilter.h"
#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"

template <typename Derived>
::std::ostream& operator<<(::std::ostream& os, const Eigen::EigenBase<Derived>&
    mat) {
  return os << mat;
}

template <typename T>
::std::ostream& operator<<(::std::ostream& os, const std::vector<T>& vec) {
  for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    os << *it;
  return os;
}

using namespace janeth;

TEST(JanethOdometryCalibrationTestSuite, testDifferentialOdometryKalmanFilter) {
  DifferentialOdometry::Parameters diffParams = {0.285, 0.285, 0.285, 0.285,
    1.285, 1.285, 0.000045};
  KalmanFilter::Parameters kfParams = {Eigen::Matrix2d::Identity()};
  DifferentialKalmanFilter::Parameters diffKfParams = {0.1, 0.1};
  DifferentialOdometryKalmanFilter odometry(diffParams, kfParams, diffKfParams);
  ASSERT_EQ(odometry.getKalmanFilterParameters(), kfParams);
  ASSERT_EQ(odometry.getDifferentialKalmanFilterParameters(), diffKfParams);
  ASSERT_EQ(odometry.getDifferentialParameters(), diffParams);
  ASSERT_EQ(odometry.getStateEstimate(), Eigen::Vector2d::Zero());
  ASSERT_EQ(odometry.getStateEstimateCovariance(), Eigen::Matrix2d::Zero());
}
