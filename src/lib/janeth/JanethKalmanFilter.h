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

/** \file JanethKalmanFilter.h
    \brief This file defines the JanethKalmanFilter class, which implements an
           Kalman filter for JanETH.
  */

#ifndef JANETH_JANETH_JANETH_KALMAN_FILTER_H
#define JANETH_JANETH_JANETH_KALMAN_FILTER_H

#include <iosfwd>

#include "generic/KalmanFilter.h"
#include "generic/DifferentialKalmanFilter.h"
#include "generic/AckermannKalmanFilter.h"

namespace janeth {

  /** The class JanethKalmanFilter implements a Kalman filter for JanETH.
      \brief Kalman filter for JanETH
    */
  class JanethKalmanFilter :
    public AckermannKalmanFilter {
  public:
    /** \name Types definitions
      @{
      */
    /// Kalman filter parameters
    struct Parameters {
      /// Variance for DMI measurement
      double dmiVariance;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (dmiVariance == other.dmiVariance);
      }
    };
    /// Grand Grand parent type
    typedef KalmanFilter GrandGrandParent;
    /// Grand parent type
    typedef DifferentialKalmanFilter GrandParent;
    /// Parent type
    typedef AckermannKalmanFilter Parent;
    /// Self type
    typedef JanethKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with parameters
    JanethKalmanFilter(const GrandGrandParent::Parameters& kfParameters, const
      GrandParent::Parameters& diffKfParameters, const Parent::Parameters&
      ackKfParameters, const Parameters& janethKfParameters);
    /// Copy constructor
    JanethKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    JanethKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    JanethKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    JanethKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~JanethKalmanFilter();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Update odometry with DMI measurement
    virtual void updateDMIMeasurement(double dmi, double timestamp);
    /// Update odometry with rear wheels measurement
    virtual void updateRearWheelMeasurement(double leftWheel, double rightWheel,
      double timestamp);
    /// Update odometry with front wheels and steering measurement
    virtual void updateFrontWheelSteeringMeasurement(double leftWheel, double
      rightWheel, double steering, double timestamp);
    /// Update odometry with steering measurement
    virtual void updateSteeringMeasurement(double steering, double timestamp);
    /// Reset the filter
    virtual void resetFilter();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the Kalman filter parameters
    Parameters& getJanethKalmanFilterParameters();
    /// Returns the Kalman filter parameters
    const Parameters& getJanethKalmanFilterParameters() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Returns the DMI distance to odometry center
    virtual double getDMIDistance() const = 0;
    /// Returns the corrected rear left wheel translational velocity
    virtual double getRearLeftWheelTranslationalVelocity(double x) const = 0;
    /// Returns the corrected rear right wheel translational velocity
    virtual double getRearRightWheelTranslationalVelocity(double x) const = 0;
    /// Returns the corrected front left wheel translational velocity
    virtual double getFrontLeftWheelTranslationalVelocity(double x) const = 0;
    /// Returns the corrected front left wheel translational velocity
    virtual double getFrontRightWheelTranslationalVelocity(double x) const = 0;
    /// Returns the corrected steering
    virtual double getSteering(double x) const = 0;
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// Kalman filter parameters
    Parameters janethKalmanFilterParameters_;
    /// Last DMI measurement
    double lastDMIMeasurement_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    JanethKalmanFilter::Parameters& params);

}

#endif // JANETH_JANETH_JANETH_KALMAN_FILTER_H
