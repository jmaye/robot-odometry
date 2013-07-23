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

/** \file Odometry.cpp
    \brief This file is a testing binary for the Odometry class
  */

#include <iostream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "generic/Odometry.h"

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

TEST(JanethOdometryCalibrationTestSuite, testOdometry) {
  Odometry odometry;
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d::Zero());
  ASSERT_EQ(odometry.getTimestamp(), 0);
  ASSERT_EQ(odometry.getPoses(), std::vector<Eigen::Vector3d>(
    {Eigen::Vector3d::Zero()}));
  ASSERT_EQ(odometry.getTimestamps(), std::vector<double>({0}));
  odometry.updateCOGDisplacement(0.5, 0, 5);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(0.5, 0, 0));
  ASSERT_EQ(odometry.getTimestamp(), 5);
  ASSERT_THROW(odometry.updateCOGDisplacement(0, M_PI / 4, 0),
    std::runtime_error);
  odometry.updateCOGDisplacement(0, M_PI / 4, 6);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(0.5, 0, M_PI / 4));
  ASSERT_EQ(odometry.getTimestamp(), 6);
  odometry.reset();
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d::Zero());
  ASSERT_EQ(odometry.getTimestamp(), 0);
  odometry.updateCOGVelocity(0.0, 0.0, 0.0);
  odometry.updateCOGVelocity(0.1, 0, 10);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 0, 0));
  ASSERT_EQ(odometry.getTimestamp(), 10);
  odometry.updateCOGVelocity(0.0, M_PI / 16, 18);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 0, M_PI / 2));
  ASSERT_EQ(odometry.getTimestamp(), 18);
  odometry.insertPose(Eigen::Vector3d(1, 1, M_PI / 2), 19);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 1, M_PI / 2));
  ASSERT_EQ(odometry.getTimestamp(), 19);
  ASSERT_THROW(odometry.insertPose(Eigen::Vector3d(1, 1, M_PI / 2), 10),
    std::runtime_error);
  odometry.reset(Eigen::Vector3d(1, 2, M_PI / 6), 10);
  ASSERT_EQ(odometry.getPose(), Eigen::Vector3d(1, 2, M_PI / 6));
  ASSERT_EQ(odometry.getTimestamp(), 10);
}
