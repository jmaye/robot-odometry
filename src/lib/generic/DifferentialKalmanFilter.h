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

/** \file DifferentialKalmanFilter.h
    \brief This file defines the DifferentialKalmanFilter class, which
           implements a differential Kalman filter.
  */

#ifndef JANETH_GENERIC_DIFFERENTIAL_KALMAN_FILTER_H
#define JANETH_GENERIC_DIFFERENTIAL_KALMAN_FILTER_H

#include <iosfwd>

#include "generic/KalmanFilter.h"

namespace janeth {

  /** The class DifferentialKalmanFilter implements a differential Kalman
      filter.
      \brief Differential Kalman filter
    */
  class DifferentialKalmanFilter :
    public KalmanFilter {
  public:
    /** \name Types definitions
      @{
      */
    /// Kalman filter parameters
    struct Parameters {
      /// Variance for rear left wheel measurement
      double rearLeftWheelVariance;
      /// Variance for rear right wheel measurement
      double rearRightWheelVariance;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (rearLeftWheelVariance == other.rearLeftWheelVariance) &&
          (rearRightWheelVariance == other.rearRightWheelVariance);
      }
    };
    /// Parent type
    typedef KalmanFilter Parent;
    /// Self type
    typedef DifferentialKalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with parameters
    DifferentialKalmanFilter(const Parent::Parameters& kfParameters, const
      Parameters& diffKfParameters);
    /// Copy constructor
    DifferentialKalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    DifferentialKalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    DifferentialKalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    DifferentialKalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~DifferentialKalmanFilter();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Update odometry with rear left wheel translational velocity
    virtual void updateRearLeftWheelTranslationalVelocity(double vLeftWheel,
      double timestamp);
    /// Update odometry with rear right wheel translational velocity
    virtual void updateRearRightWheelTranslationalVelocity(double vRightWheel,
      double timestamp);
    /// Update odometry with rear left wheel displacement
    virtual void updateRearLeftWheelDisplacement(double dLeftWheel, double
      timestamp);
    /// Update odometry with rear right wheel displacement
    virtual void updateRearRightWheelDisplacement(double dRightWheel, double
      timestamp);
    /// Update odometry with rear wheel displacements
    virtual void updateRearWheelDisplacements(double dLeftWheel, double
      dRightWheel, double timestamp);
    /// Reset the filter
    virtual void resetFilter();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the Kalman filter parameters
    Parameters& getDifferentialKalmanFilterParameters();
    /// Returns the Kalman filter parameters
    const Parameters& getDifferentialKalmanFilterParameters() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Returns the rear wheel track
    virtual double getRearWheelTrack() const = 0;
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// Kalman filter parameters
    Parameters differentialKalmanFilterParameters_;
    /// Last rear left wheel velocity measurement timestamp
    double lastRearLeftVelocityTimestamp_;
    /// Last rear right wheel velocity measurement timestamp
    double lastRearRightVelocityTimestamp_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    DifferentialKalmanFilter::Parameters& params);

}

#endif // JANETH_GENERIC_DIFFERENTIAL_KALMAN_FILTER_H
