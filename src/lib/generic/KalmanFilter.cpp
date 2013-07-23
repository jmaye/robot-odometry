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

#include "generic/KalmanFilter.h"

#include <iostream>

#include <Eigen/Dense>

namespace janeth {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  KalmanFilter::KalmanFilter(const Parameters& parameters) :
      kalmanFilterParameters_(parameters) {
    resetFilter();
  }

  KalmanFilter::~KalmanFilter() {
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  KalmanFilter::Parameters& KalmanFilter::getKalmanFilterParameters() {
    return kalmanFilterParameters_;
  }

  const KalmanFilter::Parameters& KalmanFilter::getKalmanFilterParameters()
      const {
    return kalmanFilterParameters_;
  }

  const Eigen::Vector2d& KalmanFilter::getStateEstimate() const {
    return x_k_;
  }

  const Eigen::Matrix2d& KalmanFilter::getStateEstimateCovariance() const {
    return P_k_k_;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void KalmanFilter::resetFilter() {
    x_k_ = Eigen::Vector2d::Zero();
    P_k_k_ = Eigen::Matrix2d::Zero();
    lastMeasurementTimestamp_ = -1;
  }

  Eigen::Matrix2d KalmanFilter::predictState(double timestamp) const {
    Eigen::Matrix2d P_k_km1;
    if (timestamp > lastMeasurementTimestamp_)
      P_k_km1 = P_k_k_ + kalmanFilterParameters_.Q;
    else
      P_k_km1 = P_k_k_;
    return P_k_km1;
  }

  void KalmanFilter::updateMeasurement(double measurement, double variance,
      const Eigen::Matrix<double, 1, 2>& C, double timestamp) {
    updatePose(timestamp);
    updateState(measurement, variance, C, predictState(timestamp));
    lastMeasurementTimestamp_ = timestamp;
  }

  void KalmanFilter::updateState(double measurement, double variance, const
      Eigen::Matrix<double, 1, 2>& C, const Eigen::Matrix2d& P_k_km1) {
    Eigen::Matrix<double, 2, 1> K_k = P_k_km1 * C.transpose() *
      (C * P_k_km1 * C.transpose() +
      (Eigen::Matrix<double, 1, 1>() << variance).finished()).inverse();
    P_k_k_ = (Eigen::Matrix2d::Identity() - K_k * C) * P_k_km1;
    x_k_ = x_k_ + K_k *
      ((Eigen::Matrix<double, 1, 1>() << measurement).finished() - C * x_k_);
  }

  void KalmanFilter::updatePose(double timestamp) {
    if (timestamp > lastMeasurementTimestamp_ &&
        lastMeasurementTimestamp_ != -1)
      updateCOGDisplacement(x_k_(0), x_k_(1), lastMeasurementTimestamp_);
  }

  ::std::ostream& operator<<(::std::ostream& os, const KalmanFilter::Parameters&
      params) {
    return os << params.Q;
  }

}
