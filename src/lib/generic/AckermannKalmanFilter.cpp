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

#include "generic/AckermannKalmanFilter.h"

#include <iostream>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  AckermannKalmanFilter::AckermannKalmanFilter(const GrandParent::Parameters&
      kfParameters, const Parent::Parameters& diffKfParameters, const
      Parameters& ackKfParameters) :
      Parent(kfParameters, diffKfParameters),
      ackermannKalmanFilterParameters_(ackKfParameters) {
    resetFilter();
  }

  AckermannKalmanFilter::~AckermannKalmanFilter() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  AckermannKalmanFilter::Parameters&
      AckermannKalmanFilter::getAckermannKalmanFilterParameters() {
    return ackermannKalmanFilterParameters_;
  }

  const AckermannKalmanFilter::Parameters&
      AckermannKalmanFilter::getAckermannKalmanFilterParameters() const {
    return ackermannKalmanFilterParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void AckermannKalmanFilter::updateFrontLeftWheelTranslationalVelocitySteering(
      double vLeftWheel, double steering, double timestamp) {
    if (lastFrontLeftVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastFrontLeftVelocityTimestamp_;
      updateFrontLeftWheelDisplacementSteering(vLeftWheel * dT, steering,
        timestamp);
    }
    lastFrontLeftVelocityTimestamp_ = timestamp;
  }

  void AckermannKalmanFilter::
      updateFrontRightWheelTranslationalVelocitySteering(double vRightWheel,
      double steering, double timestamp) {
    if (lastFrontRightVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastFrontRightVelocityTimestamp_;
      updateFrontRightWheelDisplacementSteering(vRightWheel * dT, steering,
        timestamp);
    }
    lastFrontRightVelocityTimestamp_ = timestamp;
  }

  void AckermannKalmanFilter::updateFrontLeftWheelDisplacementSteering(double
      dLeftWheel, double steering, double timestamp) {
    const double phi_l = std::atan(std::tan(steering) * getWheelBase() /
      (getWheelBase() - getFrontWheelTrack() * 0.5 * std::tan(steering)));
    updateMeasurement(dLeftWheel * std::cos(phi_l),
      ackermannKalmanFilterParameters_.frontLeftWheelVariance,
      (Eigen::Matrix<double, 1, 2>()
      << 1, -getFrontWheelTrack() * 0.5).finished(), timestamp);
  }

  void AckermannKalmanFilter::updateFrontRightWheelDisplacementSteering(double
      dRightWheel, double steering, double timestamp) {
    const double phi_r = std::atan(std::tan(steering) * getWheelBase() /
      (getWheelBase() + getFrontWheelTrack() * 0.5 * std::tan(steering)));
    updateMeasurement(dRightWheel * std::cos(phi_r),
      ackermannKalmanFilterParameters_.frontRightWheelVariance,
      (Eigen::Matrix<double, 1, 2>()
      << 1, getFrontWheelTrack() * 0.5).finished(), timestamp);
  }

  void AckermannKalmanFilter::updateSteering(double steering, double
      timestamp) {
    if (std::fabs(x_k_(0)) < 1e-1)
      return;
    updateMeasurement(std::tan(steering),
      ackermannKalmanFilterParameters_.frontLeftWheelVariance,
      (Eigen::Matrix<double, 1, 2>()
      << -getWheelBase() * x_k_(1) / x_k_(0) / x_k_(0),
      getWheelBase() / x_k_(0)).finished(), timestamp);
  }

  void AckermannKalmanFilter::updateRearWheelDisplacementsSteering(double
      dLeftWheel, double dRightWheel, double steering, double timestamp) {
    updateFrontLeftWheelDisplacementSteering(dLeftWheel, steering, timestamp);
    updateFrontRightWheelDisplacementSteering(dRightWheel, steering, timestamp);
    updateSteering(steering, timestamp);
  }

  void AckermannKalmanFilter::resetFilter() {
    DifferentialKalmanFilter::resetFilter();
    lastFrontLeftVelocityTimestamp_ = -1;
    lastFrontRightVelocityTimestamp_ = -1;
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      AckermannKalmanFilter::Parameters& params) {
    return os << params.frontLeftWheelVariance << " "
      << params.frontRightWheelVariance << " " << params.steeringVariance;
  }

}
