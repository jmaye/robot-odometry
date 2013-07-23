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

/** \file KalmanFilter.h
    \brief This file defines the KalmanFilter class, which implements a generic
           Kalman filter.
  */

#ifndef JANETH_GENERIC_KALMAN_FILTER_H
#define JANETH_GENERIC_KALMAN_FILTER_H

#include <iosfwd>

#include <Eigen/Core>

namespace janeth {

  /** The class KalmanFilter implements a generic Kalman filter.
      \brief Generic Kalman filter
    */
  class KalmanFilter {
  public:
    /** \name Types definitions
      @{
      */
    /// Kalman filter parameters
    struct Parameters {
      /// Covariance of the Wiener process
      Eigen::Matrix2d Q;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (Q == other.Q);
      }
    };
    /// Self type
    typedef KalmanFilter Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with parameters
    KalmanFilter(const Parameters& parameters);
    /// Copy constructor
    KalmanFilter(const Self& other) = delete;
    /// Copy assignment operator
    KalmanFilter& operator = (const Self& other) = delete;
    /// Move constructor
    KalmanFilter(Self&& other) = delete;
    /// Move assignment operator
    KalmanFilter& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~KalmanFilter();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with COG displacement
    virtual void updateCOGDisplacement(double dTrans, double dRot, double
      timestamp) = 0;
    /// Reset the filter
    virtual void resetFilter();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the Kalman filter parameters
    Parameters& getKalmanFilterParameters();
    /// Returns the Kalman filter parameters
    const Parameters& getKalmanFilterParameters() const;
    /// Returns the current state estimate
    const Eigen::Vector2d& getStateEstimate() const;
    /// Returns the current covariance of the state estimate
    const Eigen::Matrix2d& getStateEstimateCovariance() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Applies the prediction stage of the Kalman filter
    Eigen::Matrix2d predictState(double timestamp) const;
    /// Adds a new measurement to the filter
    void updateMeasurement(double measurement, double variance, const
      Eigen::Matrix<double, 1, 2>& C, double timestamp);
    /// Applies the update stage of the Kalman filter
    void updateState(double measurement, double variance,
      const Eigen::Matrix<double, 1, 2>& C, const Eigen::Matrix2d& P_k_km1);
    /// Update the pose
    void updatePose(double timestamp);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// Kalman filter parameters
    Parameters kalmanFilterParameters_;
    /// Current state estimate
    Eigen::Vector2d x_k_;
    /// Current state estimate covariance
    Eigen::Matrix2d P_k_k_;
    /// Last measurement timestamp
    double lastMeasurementTimestamp_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const KalmanFilter::Parameters&
    params);

}

#endif // JANETH_GENERIC_KALMAN_FILTER_H
