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

#include "generic/DifferentialOdometry.h"

#include <iostream>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  DifferentialOdometry::DifferentialOdometry(const Parameters& parameters,
      const Eigen::Vector3d& initialPose, double initialTimestamp) :
      Parent(initialPose, initialTimestamp),
      differentialParameters_(parameters) {
  }

  DifferentialOdometry::~DifferentialOdometry() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const DifferentialOdometry::Parameters&
      DifferentialOdometry::getDifferentialParameters() const {
    return differentialParameters_;
  }

  DifferentialOdometry::Parameters&
      DifferentialOdometry::getDifferentialParameters() {
    return differentialParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void DifferentialOdometry::updateRearWheelTranslationalVelocities(double
      vLeftWheel, double vRightWheel, double timestamp) {
    if (lastVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastVelocityTimestamp_;
      updateRearWheelDisplacements(vLeftWheel * dT, vRightWheel * dT,
        timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
  }

  void DifferentialOdometry::updateRearWheelRotationalVelocities(double
      wLeftWheel, double wRightWheel, double timestamp) {
    if (lastVelocityTimestamp_ != -1) {
      const double dT = timestamp - lastVelocityTimestamp_;
      updateRearWheelDisplacements(
        differentialParameters_.rearLeftWheelRadius * wLeftWheel * dT,
        differentialParameters_.rearRightWheelRadius * wRightWheel * dT,
        timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
  }

  void DifferentialOdometry::updateRearWheelTicks(double dLeftWheel, double
      dRightWheel, double timestamp) {
    updateRearWheelDisplacements(
      dLeftWheel * differentialParameters_.metersPerTick,
      dRightWheel * differentialParameters_.metersPerTick, timestamp);
  }

  void DifferentialOdometry::updateRearWheelDisplacements(double dLeftWheel,
      double dRightWheel, double timestamp) {
    updateCOGDisplacement(0.5 * (dRightWheel + dLeftWheel),
      (dRightWheel - dLeftWheel) / differentialParameters_.rearWheelTrack,
      timestamp);
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      DifferentialOdometry::Parameters& params) {
    return os << params.rearLeftWheelRadius << " "
      << params.rearRightWheelRadius << " " << params.frontLeftWheelRadius
      << " " << params.frontRightWheelRadius << " "
      << params.rearWheelTrack << " " << params.frontWheelTrack << " "
      << params.metersPerTick;
  }

}
