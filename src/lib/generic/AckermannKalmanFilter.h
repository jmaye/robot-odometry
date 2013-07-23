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

/** \file AckermannKalmanFilter.h
    \brief This file defines the AckermannKalmanFilter class, which
           implements an Ackermann Kalman filter.
  */

#ifndef JANETH_GENERIC_ACKERMANN_KALMAN_FILTER_H
#define JANETH_GENERIC_ACKERMANN_KALMAN_FILTER_H

#include <iosfwd>

#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"

namespace janeth {

  /** The class AckermannKalmanFilter implements an Ackermann Kalman filter.
      \brief Ackermann Kalman filter
    */
  class AckermannKalmanFilter :
    public DifferentialKalmanFilter {
  public:
    /** \name Types definitions
      @{
      */
    /// Kalman filter parameters
    struct Parameters {
      /// Variance for front left wheel measurement
      double frontLeftWheelVariance;
      /// Variance for front right wheel measurement
      double frontRightWheelVariance;
      /// Variance for steering measurement
      double steeringVariance;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (frontLeftWheelVariance == other.frontLeftWheelVariance) &&
          (frontRightWheelVariance == other.frontRightWheelVariance) &&
          (steeringVariance == other.steeringVariance);
      }
    };
    /// Grand parent type
    typedef KalmanFilter GrandParent;
    /// Parent type
    typedef DifferentialKalmanFilter Parent;
    /// Self type
    typedef AckermannKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with parameters
    AckermannKalmanFilter(const GrandParent::Parameters& kfParameters, const
      Parent::Parameters& diffKfParameters, const Parameters& ackKfParameters);
    /// Copy constructor
    AckermannKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    AckermannKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    AckermannKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    AckermannKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~AckermannKalmanFilter();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Update odometry with front left wheel translational velocity and steer.
    virtual void updateFrontLeftWheelTranslationalVelocitySteering(double
      vLeftWheel, double steering, double timestamp);
    /// Update odometry with front right wheel translational velocity and steer.
    virtual void updateFrontRightWheelTranslationalVelocitySteering(double
      vRightWheel, double steering, double timestamp);
    /// Update odometry with front left wheel displacement and steering
    virtual void updateFrontLeftWheelDisplacementSteering(double dLeftWheel,
      double steering, double timestamp);
    /// Update odometry with front right wheel displacement and steering
    virtual void updateFrontRightWheelDisplacementSteering(double dRightWheel,
      double steering, double timestamp);
    /// Update odometry with steering
    virtual void updateSteering(double steering, double timestamp);
    /// Update odometry with rear wheel displacements and steering
    virtual void updateRearWheelDisplacementsSteering(double dLeftWheel, double
      dRightWheel, double steering, double timestamp);
    /// Reset the filter
    virtual void resetFilter();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the Kalman filter parameters
    Parameters& getAckermannKalmanFilterParameters();
    /// Returns the Kalman filter parameters
    const Parameters& getAckermannKalmanFilterParameters() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Returns the front wheel track
    virtual double getFrontWheelTrack() const = 0;
    /// Returns the wheel base
    virtual double getWheelBase() const = 0;
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// Kalman filter parameters
    Parameters ackermannKalmanFilterParameters_;
    /// Last front wheel velocity measurement timestamp
    double lastFrontLeftVelocityTimestamp_;
    /// Last front wheel velocity measurement timestamp
    double lastFrontRightVelocityTimestamp_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    AckermannKalmanFilter::Parameters& params);

}

#endif // JANETH_GENERIC_ACKERMANN_KALMAN_FILTER_H
