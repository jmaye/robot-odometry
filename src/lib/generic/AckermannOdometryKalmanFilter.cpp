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

#include "generic/AckermannOdometryKalmanFilter.h"

#include <cmath>

#include <Eigen/Dense>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  AckermannOdometryKalmanFilter::AckermannOdometryKalmanFilter(const
      DifferentialOdometry::Parameters& diffParameters, const
      AckermannOdometry::Parameters& ackParameters, const
      KalmanFilter::Parameters& kfParameters, const
      DifferentialKalmanFilter::Parameters& diffKfParameters, const
      AckermannKalmanFilter::Parameters& ackKfParameters, const Eigen::Vector3d&
      initialPose, double initialTimestamp) :
      AckermannOdometry(diffParameters, ackParameters, initialPose,
        initialTimestamp),
      AckermannKalmanFilter(kfParameters, diffKfParameters, ackKfParameters) {
  }

  AckermannOdometryKalmanFilter::~AckermannOdometryKalmanFilter() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void AckermannOdometryKalmanFilter::updateCOGDisplacement(double dTrans,
      double dRot, double timestamp) {
    AckermannOdometry::updateCOGDisplacement(dTrans, dRot, timestamp);
  }

  double AckermannOdometryKalmanFilter::getRearWheelTrack() const {
    return differentialParameters_.rearWheelTrack;
  }

  double AckermannOdometryKalmanFilter::getFrontWheelTrack() const {
    return differentialParameters_.frontWheelTrack;
  }

  double AckermannOdometryKalmanFilter::getWheelBase() const {
    return ackermannParameters_.wheelBase;
  }

}
