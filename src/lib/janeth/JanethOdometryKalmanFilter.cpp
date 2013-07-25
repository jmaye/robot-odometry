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

#include "janeth/JanethOdometryKalmanFilter.h"

#include <cmath>

#include <Eigen/Dense>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  JanethOdometryKalmanFilter::JanethOdometryKalmanFilter(const
      DifferentialOdometry::Parameters& diffParameters, const
      AckermannOdometry::Parameters& ackParameters, const
      JanethOdometry::Parameters& janethParameters, const
      KalmanFilter::Parameters& kfParameters, const
      DifferentialKalmanFilter::Parameters& diffKfParameters, const
      AckermannKalmanFilter::Parameters& ackKfParameters, const
      JanethKalmanFilter::Parameters janethKfParameters, const Eigen::Vector3d&
      initialPose, double initialTimestamp) :
      JanethOdometry(diffParameters, ackParameters, janethParameters,
        initialPose, initialTimestamp),
      JanethKalmanFilter(kfParameters, diffKfParameters, ackKfParameters,
        janethKfParameters) {
  }

  JanethOdometryKalmanFilter::~JanethOdometryKalmanFilter() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void JanethOdometryKalmanFilter::updateCOGDisplacement(double dTrans,
      double dRot, double timestamp) {
    JanethOdometry::updateCOGDisplacement(dTrans, dRot, timestamp);
  }

  double JanethOdometryKalmanFilter::getRearWheelTrack() const {
    return differentialParameters_.rearWheelTrack;
  }

  double JanethOdometryKalmanFilter::getFrontWheelTrack() const {
    return differentialParameters_.frontWheelTrack;
  }

  double JanethOdometryKalmanFilter::getWheelBase() const {
    return ackermannParameters_.wheelBase;
  }

  double JanethOdometryKalmanFilter::getRearLeftWheelTranslationalVelocity(
      double x) const {
    return JanethOdometry::getRearLeftWheelTranslationalVelocity(x);
  }

  double JanethOdometryKalmanFilter::getRearRightWheelTranslationalVelocity(
      double x) const {
    return JanethOdometry::getRearRightWheelTranslationalVelocity(x);
  }

  double JanethOdometryKalmanFilter::getFrontLeftWheelTranslationalVelocity(
      double x) const {
    return JanethOdometry::getFrontLeftWheelTranslationalVelocity(x);
  }

  double JanethOdometryKalmanFilter::getFrontRightWheelTranslationalVelocity(
      double x) const {
    return JanethOdometry::getFrontRightWheelTranslationalVelocity(x);
  }

  double JanethOdometryKalmanFilter::getSteering(double x) const {
    return JanethOdometry::getSteering(x);
  }

}
