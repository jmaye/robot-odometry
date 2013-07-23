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

#include "generic/DifferentialKalmanFilter.h"

#include <iostream>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  DifferentialKalmanFilter::DifferentialKalmanFilter(const Parent::Parameters&
      kfParameters, const Parameters& diffKfParameters) :
      Parent(kfParameters),
      differentialKalmanFilterParameters_(diffKfParameters) {
    resetFilter();
  }

  DifferentialKalmanFilter::~DifferentialKalmanFilter() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  DifferentialKalmanFilter::Parameters&
      DifferentialKalmanFilter::getDifferentialKalmanFilterParameters() {
    return differentialKalmanFilterParameters_;
  }

  const DifferentialKalmanFilter::Parameters&
      DifferentialKalmanFilter::getDifferentialKalmanFilterParameters() const {
    return differentialKalmanFilterParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void DifferentialKalmanFilter::
      updateRearLeftWheelTranslationalVelocity(double vLeftWheel, double
      timestamp) {
    if (lastRearLeftVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastRearLeftVelocityTimestamp_;
      updateRearLeftWheelDisplacement(vLeftWheel * dT, timestamp);
    }
    lastRearLeftVelocityTimestamp_ = timestamp;
  }

  void DifferentialKalmanFilter::
      updateRearRightWheelTranslationalVelocity(double vRightWheel, double
      timestamp) {
    if (lastRearRightVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastRearRightVelocityTimestamp_;
      updateRearRightWheelDisplacement(vRightWheel * dT, timestamp);
    }
    lastRearRightVelocityTimestamp_ = timestamp;
  }

  void DifferentialKalmanFilter::updateRearLeftWheelDisplacement(double
      dLeftWheel, double timestamp) {
    updateMeasurement(dLeftWheel,
      differentialKalmanFilterParameters_.rearLeftWheelVariance,
      (Eigen::Matrix<double, 1, 2>()
      << 1, -getRearWheelTrack() * 0.5).finished(), timestamp);
  }

  void DifferentialKalmanFilter::updateRearRightWheelDisplacement(double
      dRightWheel, double timestamp) {
    updateMeasurement(dRightWheel,
      differentialKalmanFilterParameters_.rearRightWheelVariance,
      (Eigen::Matrix<double, 1, 2>()
      << 1, getRearWheelTrack() * 0.5).finished(), timestamp);
  }

  void DifferentialKalmanFilter::updateRearWheelDisplacements(double dLeftWheel,
      double dRightWheel, double timestamp) {
    updateRearLeftWheelDisplacement(dLeftWheel, timestamp);
    updateRearRightWheelDisplacement(dRightWheel, timestamp);
  }

  void DifferentialKalmanFilter::resetFilter() {
    KalmanFilter::resetFilter();
    lastRearLeftVelocityTimestamp_ = -1;
    lastRearRightVelocityTimestamp_ = -1;
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      DifferentialKalmanFilter::Parameters& params) {
    return os << params.rearLeftWheelVariance << " "
      << params.rearRightWheelVariance;
  }

}
