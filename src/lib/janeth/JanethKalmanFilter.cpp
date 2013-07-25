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

#include "janeth/JanethKalmanFilter.h"

#include <iostream>
#include <limits>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  JanethKalmanFilter::JanethKalmanFilter(const GrandGrandParent::Parameters&
      kfParameters, const GrandParent::Parameters& diffKfParameters, const
      Parent::Parameters& ackKfParameters, const Parameters&
      janethKfParameters) :
      Parent(kfParameters, diffKfParameters, ackKfParameters),
      janethKalmanFilterParameters_(janethKfParameters) {
    resetFilter();
  }

  JanethKalmanFilter::~JanethKalmanFilter() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  JanethKalmanFilter::Parameters&
      JanethKalmanFilter::getJanethKalmanFilterParameters() {
    return janethKalmanFilterParameters_;
  }

  const JanethKalmanFilter::Parameters&
      JanethKalmanFilter::getJanethKalmanFilterParameters() const {
    return janethKalmanFilterParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void JanethKalmanFilter::updateDMIMeasurement(double dmi, double timestamp) {
    if (lastDMIMeasurement_ != std::numeric_limits<double>::infinity()) {
      const double dmiDisplacement = dmi - lastDMIMeasurement_;
      updateMeasurement(dmiDisplacement,
        janethKalmanFilterParameters_.dmiVariance,
        (Eigen::Matrix<double, 1, 2>()
        << 1, -getRearWheelTrack() * 0.5).finished(),
        timestamp);
    }
    lastDMIMeasurement_ = dmi;
  }

  void JanethKalmanFilter::updateRearWheelMeasurement(double leftWheel, double
      rightWheel, double timestamp) {
    updateRearLeftWheelTranslationalVelocity(
      getRearLeftWheelTranslationalVelocity(leftWheel), timestamp);
    updateRearRightWheelTranslationalVelocity(
      getRearRightWheelTranslationalVelocity(rightWheel), timestamp);
  }

  void JanethKalmanFilter::updateFrontWheelSteeringMeasurement(double leftWheel,
      double rightWheel, double steering, double timestamp) {
    updateFrontLeftWheelTranslationalVelocitySteering(
      getFrontLeftWheelTranslationalVelocity(leftWheel), getSteering(steering),
      timestamp);
    updateFrontRightWheelTranslationalVelocitySteering(
      getFrontRightWheelTranslationalVelocity(rightWheel), getSteering(steering),
      timestamp);
  }

  void JanethKalmanFilter::updateSteeringMeasurement(double steering, double
      timestamp) {
    updateSteering(getSteering(steering), timestamp);
  }

  void JanethKalmanFilter::resetFilter() {
    AckermannKalmanFilter::resetFilter();
    lastDMIMeasurement_ = std::numeric_limits<double>::infinity();
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      JanethKalmanFilter::Parameters& params) {
    return os << params.dmiVariance;
  }

}
