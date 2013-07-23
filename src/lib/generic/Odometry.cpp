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

#include "generic/Odometry.h"

#include <cmath>
#include <stdexcept>

#include "utils/utils.h"

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  Odometry::Odometry(const Eigen::Vector3d& initialPose, double
      initialTimestamp) {
    reset(initialPose, initialTimestamp);
  }

  Odometry::~Odometry() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const Eigen::Vector3d& Odometry::getPose() const {
    return poses_.back();
  }

  double Odometry::getTimestamp() const {
    return timestamps_.back();
  }

  const std::vector<Eigen::Vector3d>& Odometry::getPoses() const {
    return poses_;
  }

  const std::vector<double>& Odometry::getTimestamps() const {
    return timestamps_;
  }

  void Odometry::reset(const Eigen::Vector3d& initialPose, double
      initialTimestamp) {
    poses_.assign(1, initialPose);
    timestamps_.assign(1, initialTimestamp);
    lastVelocityTimestamp_ = -1;
  }

  void Odometry::insertPose(const Eigen::Vector3d& pose, double timestamp) {
    if (timestamp < getTimestamp())
      throw std::runtime_error("Odometry::insertPose: "
        "timestamps must be increasing");
    poses_.push_back(pose);
    timestamps_.push_back(timestamp);
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void Odometry::updateCOGDisplacement(double dTrans, double dRot, double
      timestamp) {
    if (timestamp == getTimestamp())
      return;
    if (timestamp < getTimestamp())
      throw std::runtime_error("Odometry::updateCOGDisplacement: "
        "timestamps must be increasing");
    const Eigen::Vector3d& pose_km1 = poses_.back();
    poses_.push_back(Eigen::Vector3d(
      pose_km1(0) + dTrans * cos(pose_km1(2) + dRot * 0.5),
      pose_km1(1) + dTrans * sin(pose_km1(2) + dRot * 0.5),
      angleMod(pose_km1(2) + dRot)));
    timestamps_.push_back(timestamp);
  }

  void Odometry::updateCOGVelocity(double vTrans, double vRot, double
      timestamp) {
    if (lastVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastVelocityTimestamp_;
      updateCOGDisplacement(vTrans * dT, vRot * dT, timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
  }

}
