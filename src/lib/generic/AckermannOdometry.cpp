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

#include "generic/AckermannOdometry.h"

#include <iostream>
#include <cmath>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  AckermannOdometry::AckermannOdometry(const Parent::Parameters&
      differentialParameters, const Parameters& ackermannParameters, const
      Eigen::Vector3d& initialPose, double initialTimestamp) :
      Parent(differentialParameters, initialPose, initialTimestamp),
      ackermannParameters_(ackermannParameters) {
  }

  AckermannOdometry::~AckermannOdometry() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const AckermannOdometry::Parameters&
      AckermannOdometry::getAckermannParameters() const {
    return ackermannParameters_;
  }

  AckermannOdometry::Parameters& AckermannOdometry::getAckermannParameters() {
    return ackermannParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void AckermannOdometry::updateRearWheelTranslationalVelocitiesSteering(double
      vLeftWheel, double vRightWheel, double steering, double timestamp) {
    if (lastVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastVelocityTimestamp_;
      updateRearWheelDisplacementsSteering(vLeftWheel * dT, vRightWheel * dT,
        steering, timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
  }

  void AckermannOdometry::updateRearWheelRotationalVelocitiesSteering(double
      wLeftWheel, double wRightWheel, double steering, double timestamp) {
    if (lastVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastVelocityTimestamp_;
      updateRearWheelDisplacementsSteering(
        differentialParameters_.rearLeftWheelRadius * wLeftWheel * dT,
        differentialParameters_.rearRightWheelRadius * wRightWheel * dT,
        steering, timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
  }

  void AckermannOdometry::updateRearWheelTicksSteering(double dLeftWheel, double
      dRightWheel, double steering, double timestamp) {
    updateRearWheelDisplacementsSteering(
      dLeftWheel * differentialParameters_.metersPerTick,
      dRightWheel * differentialParameters_.metersPerTick, steering, timestamp);
  }

  void AckermannOdometry::updateRearWheelDisplacementsSteering(double
    dLeftWheel, double dRightWheel, double steering, double timestamp) {
    updateCOGDisplacement(0.5 * (dRightWheel + dLeftWheel),
      0.5 * (dRightWheel + dLeftWheel) / ackermannParameters_.wheelBase *
      std::tan(steering), timestamp);
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      AckermannOdometry::Parameters& params) {
    return os << params.wheelBase;
  }

}
