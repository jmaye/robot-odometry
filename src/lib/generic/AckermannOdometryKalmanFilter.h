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

/** \file AckermannOdometryKalmanFilter.h
    \brief This file defines the AckermannOdometryKalmanFilter class, which
           implements an odometry model based on Ackermann geometry with
           a Kalman filter.
  */

#ifndef JANETH_GENERIC_ACKERMANN_ODOMETRY_KALMAN_FILTER_H
#define JANETH_GENERIC_ACKERMANN_ODOMETRY_KALMAN_FILTER_H

#include <Eigen/Core>

#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"
#include "generic/AckermannKalmanFilter.h"
#include "generic/DifferentialOdometry.h"
#include "generic/AckermannOdometry.h"

namespace janeth {

  /** The class AckermannOdometryKalmanFilter implements an odometry model
      based on Ackermann geometry with a Kalman filter.
      \brief Ackermann odometry Kalman filter
    */
  class AckermannOdometryKalmanFilter :
    public AckermannKalmanFilter,
    public AckermannOdometry {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef AckermannOdometryKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    AckermannOdometryKalmanFilter(const DifferentialOdometry::Parameters&
      diffParameters, const AckermannOdometry::Parameters& ackParameters, const
      KalmanFilter::Parameters& kfParameters, const
      DifferentialKalmanFilter::Parameters& diffKfParameters, const
      AckermannKalmanFilter::Parameters& ackKfParameters, const Eigen::Vector3d&
      initialPose = Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Copy constructor
    AckermannOdometryKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    AckermannOdometryKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    AckermannOdometryKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    AckermannOdometryKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~AckermannOdometryKalmanFilter();
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
    /// Returns the front wheel track
    virtual double getFrontWheelTrack() const;
    /// Returns the wheel base
    virtual double getWheelBase() const;
    /** @}
      */

  };

}

#endif // JANETH_GENERIC_ACKERMANN_ODOMETRY_KALMAN_FILTER_H
