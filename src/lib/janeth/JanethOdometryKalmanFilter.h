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

/** \file JanethOdometryKalmanFilter.h
    \brief This file defines the JanethOdometryKalmanFilter class, which
           implements a Kalman filter odometry model for the JanETH car.
  */

#ifndef JANETH_JANETH_JANETH_ODOMETRY_KALMAN_FILTER_H
#define JANETH_JANETH_JANETH_ODOMETRY_KALMAN_FILTER_H

#include <Eigen/Core>

#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"
#include "generic/AckermannKalmanFilter.h"
#include "janeth/JanethKalmanFilter.h"
#include "generic/DifferentialOdometry.h"
#include "generic/AckermannOdometry.h"
#include "janeth/JanethOdometry.h"

namespace janeth {

  /** The class JanethOdometryKalmanFilter implements a Kalman filter odometry
      model for the JanETH car.
      \brief Kalman filter odometery model for JanETH car
    */
  class JanethOdometryKalmanFilter :
    public JanethKalmanFilter,
    public JanethOdometry {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef JanethOdometryKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    JanethOdometryKalmanFilter(const DifferentialOdometry::Parameters&
      diffParameters, const AckermannOdometry::Parameters& ackParameters, const
      JanethOdometry::Parameters& janethParameters, const
      KalmanFilter::Parameters& kfParameters, const
      DifferentialKalmanFilter::Parameters& diffKfParameters, const
      AckermannKalmanFilter::Parameters& ackKfParameters, const
      JanethKalmanFilter::Parameters janethKfParameters, const Eigen::Vector3d&
      initialPose = Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Copy constructor
    JanethOdometryKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    JanethOdometryKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    JanethOdometryKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    JanethOdometryKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~JanethOdometryKalmanFilter();
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
    /// Returns the DMI distance to odometry center
    virtual double getDMIDistance() const;
    /// Returns the corrected rear left wheel translational velocity
    virtual double getRearLeftWheelTranslationalVelocity(double x) const;
    /// Returns the corrected rear right wheel translational velocity
    virtual double getRearRightWheelTranslationalVelocity(double x) const;
    /// Returns the corrected front left wheel translational velocity
    virtual double getFrontLeftWheelTranslationalVelocity(double x) const;
    /// Returns the corrected front left wheel translational velocity
    virtual double getFrontRightWheelTranslationalVelocity(double x) const;
    /// Returns the corrected steering
    virtual double getSteering(double x) const;
    /** @}
      */

  };

}

#endif // JANETH_JANETH_JANETH_ODOMETRY_KALMAN_FILTER_H
