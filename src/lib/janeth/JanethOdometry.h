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

/** \file JanethOdometry.h
    \brief This file defines the JanethOdometry class, which implements
           an odometry model for the JanETH car.
  */

#ifndef JANETH_JANETH_JANETH_ODOMETRY_H
#define JANETH_JANETH_JANETH_ODOMETRY_H

#include <iosfwd>

#include <Eigen/Core>

#include "generic/DifferentialOdometry.h"
#include "generic/AckermannOdometry.h"

namespace janeth {

  /** The class JanethOdometry implements an odometry model for JanETH.
      \brief JanETH odometry
    */
  class JanethOdometry :
    public AckermannOdometry {
  public:
    /** \name Types definitions
      @{
      */
    /// JanETH odometry parameters
    struct Parameters {
      /// Correction factor for rear left wheel
      double k_rl;
      /// Correction factor for rear right wheel
      double k_rr;
      /// Correction factor for front left wheel
      double k_fl;
      /// Correction factor for front right wheel
      double k_fr;
      /// Correction factor for steering (degree 0)
      double a0;
      /// Correction factor for steering (degree 1)
      double a1;
      /// Correction factor for steering (degree 2)
      double a2;
      /// Correction factor for steering (degree 3)
      double a3;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (k_rl == other.k_rl) && (k_rr == other.k_rr) &&
          (k_fl == other.k_fl) && (k_fr == other.k_fr) && (a0 == other.a0) &&
          (a1 == other.a1) && (a2 == other.a2) && (a3 == other.a3);
      }
    };
    /// Grand Parent type
    typedef DifferentialOdometry GrandParent;
    /// Parent type
    typedef AckermannOdometry Parent;
    /// Self type
    typedef JanethOdometry Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    JanethOdometry(const GrandParent::Parameters& differentialParameters, const
      Parent::Parameters& ackermannParameters, const Parameters&
      janethParameters, const Eigen::Vector3d& initialPose =
      Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Copy constructor
    JanethOdometry(const Self& other) = delete;
    /// Copy assignment operator
    JanethOdometry& operator = (const Self& other) = delete;
    /// Move constructor
    JanethOdometry(Self&& other) = delete;
    /// Move assignment operator
    JanethOdometry& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~JanethOdometry();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with raw JanETH measurements
    virtual void updateRawMeasurements(double vRearLeftWheel, double
      vRearRightWheel, double vFrontLeftWheel, double vFrontRightWheel, double
      steering, double dmi, double timestamp);
    virtual void reset(const Eigen::Vector3d& initialPose =
      Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the parameters
    const Parameters& getJanethParameters() const;
    /// Returns the parameters
    Parameters& getJanethParameters();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Returns the corrected rear left wheel translational velocity
    double getRearLeftWheelTranslationalVelocity(double x) const;
    /// Returns the corrected rear right wheel translational velocity
    double getRearRightWheelTranslationalVelocity(double x) const;
    /// Returns the corrected front left wheel translational velocity
    double getFrontLeftWheelTranslationalVelocity(double x) const;
    /// Returns the corrected front left wheel translational velocity
    double getFrontRightWheelTranslationalVelocity(double x) const;
    /// Returns the corrected steering
    double getSteering(double x) const;
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// Parameters
    Parameters janethParameters_;
    /// Last DMI measurement
    double lastDMIMeasurement_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    JanethOdometry::Parameters& params);

}

#endif // JANETH_JANETH_JANETH_ODOMETRY_H
