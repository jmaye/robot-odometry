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

#include "janeth/JanethOdometry.h"

#include <iostream>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  JanethOdometry::JanethOdometry(const GrandParent::Parameters&
      differentialParameters, const Parent::Parameters& ackermannParameters,
      const Parameters& janethParameters, const Eigen::Vector3d& initialPose,
      double initialTimestamp) :
      Parent(differentialParameters, ackermannParameters, initialPose,
        initialTimestamp),
      janethParameters_(janethParameters) {
    reset(initialPose, initialTimestamp);
  }

  JanethOdometry::~JanethOdometry() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const JanethOdometry::Parameters&
      JanethOdometry::getJanethParameters() const {
    return janethParameters_;
  }

  JanethOdometry::Parameters& JanethOdometry::getJanethParameters() {
    return janethParameters_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void JanethOdometry::reset(const Eigen::Vector3d& initialPose, double
      initialTimestamp) {
    AckermannOdometry::reset(initialPose, initialTimestamp);
    lastDMIMeasurement_ = std::numeric_limits<double>::infinity();
  }

  void JanethOdometry::updateRawMeasurements(double vRearLeftWheel, double
      vRearRightWheel, double vFrontLeftWheel, double vFrontRightWheel, double
      steering, double dmi, double timestamp) {
    if (lastVelocityTimestamp_ != -1 &&
        lastDMIMeasurement_ != std::numeric_limits<double>::infinity()) {
      const double dT = timestamp - lastVelocityTimestamp_;
      const double trueSteering = getSteering(steering);
      const double trueVRearLeftWheel =
        getRearLeftWheelTranslationalVelocity(vRearLeftWheel);
      const double trueVRearRightWheel =
        getRearLeftWheelTranslationalVelocity(vRearRightWheel);
      const double trueVFrontLeftWheel =
        getFrontLeftWheelTranslationalVelocity(vFrontLeftWheel);
      const double trueVFrontRightWheel =
        getFrontRightWheelTranslationalVelocity(vFrontRightWheel);
      const double dmiDisplacement = dmi - lastDMIMeasurement_;
      const double phi_l = std::atan(std::tan(trueSteering) *
        ackermannParameters_.wheelBase / (ackermannParameters_.wheelBase -
        differentialParameters_.frontWheelTrack * 0.5 *
        std::tan(trueSteering)));
      const double phi_r = std::atan(std::tan(trueSteering) *
        ackermannParameters_.wheelBase / (ackermannParameters_.wheelBase +
        differentialParameters_.frontWheelTrack * 0.5 *
        std::tan(trueSteering)));
      const Eigen::MatrixXd A = (Eigen::MatrixXd(5, 2)
        << 1, -differentialParameters_.rearWheelTrack * 0.5,
        1, -differentialParameters_.rearWheelTrack * 0.5,
        1, differentialParameters_.rearWheelTrack * 0.5,
        1, -differentialParameters_.frontWheelTrack * 0.5,
        1, differentialParameters_.frontWheelTrack * 0.5).finished();
      const Eigen::VectorXd b = (Eigen::MatrixXd(5, 1)
        << dmiDisplacement, trueVRearLeftWheel * dT, trueVRearRightWheel * dT,
        trueVFrontLeftWheel * cos(phi_l) * dT,
        trueVFrontRightWheel * cos(phi_r) * dT).finished();
      Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU |
        Eigen::ComputeThinV).solve(b);
//      updateCOGDisplacement(x(0), x(0) / ackermannParameters_.wheelBase *
//        std::tan(trueSteering), timestamp);
      updateCOGDisplacement(x(0), x(1), timestamp);
    }
    lastVelocityTimestamp_ = timestamp;
    lastDMIMeasurement_ = dmi;
  }

  double JanethOdometry::getRearLeftWheelTranslationalVelocity(double x) const {
    return janethParameters_.k_rl * x;
  }

  double JanethOdometry::getRearRightWheelTranslationalVelocity(double x)
      const {
    return janethParameters_.k_rr * x;
  }

  double JanethOdometry::getFrontLeftWheelTranslationalVelocity(double x)
      const {
    return janethParameters_.k_fl * x;
  }

  double JanethOdometry::getFrontRightWheelTranslationalVelocity(double x)
      const {
    return janethParameters_.k_fr * x;
  }

  double JanethOdometry::getSteering(double x) const {
    return janethParameters_.a0 + janethParameters_.a1 * x +
      janethParameters_.a2 * x * x + janethParameters_.a3 * x * x * x;
  }

  ::std::ostream& operator<<(::std::ostream& os, const
      JanethOdometry::Parameters& params) {
    return os << " " << params.k_rl << " " << params.k_rr << " " << params.k_fl
      << " " << params.k_fr << " " << params.a0 << " " << params.a1 << " "
      << params.a2 << " " << params.a3;
  }

}
