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

/** \file DifferentialOdometry.cpp
    \brief This file is a testing binary for the DifferentialOdometry class
  */

#include <iostream>

#include <gtest/gtest.h>

#include "generic/DifferentialOdometry.h"

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

TEST(JanethOdometryCalibrationTestSuite, testDifferentialOdometry) {
  DifferentialOdometry::Parameters params = {0.285, 0.285, 0.285, 0.285,
    1.285, 1.285, 0.000045};
  DifferentialOdometry odometry(params);
  ASSERT_EQ(odometry.getDifferentialParameters(), params);
  odometry.updateRearWheelTranslationalVelocities(0.0, 0.0, 0);
  odometry.updateRearWheelTranslationalVelocities(1.0, 1.0, 1);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 0, 0));
  odometry.reset();
  odometry.updateRearWheelRotationalVelocities(0, 0, 0);
  odometry.updateRearWheelRotationalVelocities(M_PI / 4, M_PI / 4, 1);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(M_PI / 4 * 0.285, 0, 0));
  odometry.reset();
  odometry.updateRearWheelTicks(100, 100, 1);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(params.metersPerTick * 100, 0,
    0));
  odometry.reset();
  odometry.updateRearWheelDisplacements(1.0, 1.0, 1.0);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1.0, 0, 0));

}
