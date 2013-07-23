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

/** \file DifferentialOdometryKalmanFilter.h
    \brief This file defines the DifferentialOdometryKalmanFilter class, which
           implements an odometry model based on differential geometry with
           a Kalman filter.
  */

#ifndef JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_KALMAN_FILTER_H
#define JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_KALMAN_FILTER_H

#include <Eigen/Core>

#include "generic/DifferentialOdometry.h"
#include "generic/DifferentialKalmanFilter.h"
#include "generic/KalmanFilter.h"

namespace janeth {

  /** The class DifferentialOdometryKalmanFilter implements an odometry model
      based on differential geometry with a Kalman filter.
      \brief Differential odometry Kalman filter
    */
  class DifferentialOdometryKalmanFilter :
    public DifferentialOdometry,
    public DifferentialKalmanFilter {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef DifferentialOdometryKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    DifferentialOdometryKalmanFilter(const DifferentialOdometry::Parameters&
      diffParameters, const KalmanFilter::Parameters& kfParameters, const
      DifferentialKalmanFilter::Parameters& diffKfParameters, const
      Eigen::Vector3d& initialPose = Eigen::Vector3d::Zero(), double
      initialTimestamp = 0);
    /// Copy constructor
    DifferentialOdometryKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    DifferentialOdometryKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    DifferentialOdometryKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    DifferentialOdometryKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~DifferentialOdometryKalmanFilter();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with COG displacement
    virtual void updateCOGDisplacement(double dTrans, double dRot, double
      timestamp);
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Returns the rear wheel track
    virtual double getRearWheelTrack() const;
    /** @}
      */

  };

}

#endif // JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_KALMAN_FILTER_H
