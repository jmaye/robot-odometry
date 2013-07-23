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

/** \file DifferentialOdometry.h
    \brief This file defines the DifferentialOdometry class, which implements
           an odometry model based on differential geometry.
  */

#ifndef JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_H
#define JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_H

#include <iosfwd>

#include <Eigen/Core>

#include "generic/Odometry.h"

namespace janeth {

  /** The class DifferentialOdometry implements an odometry model based on
      differential geometry.
      \brief Differential odometry
    */
  class DifferentialOdometry :
    public Odometry {
  public:
    /** \name Types definitions
      @{
      */
    /// Differential odometry parameters
    struct Parameters {
      /// Radius of the rear left wheel [m]
      double rearLeftWheelRadius;
      /// Radius of the rear right wheel [m]
      double rearRightWheelRadius;
      /// Radius of the front left wheel [m]
      double frontLeftWheelRadius;
      /// Radius of the front right wheel [m]
      double frontRightWheelRadius;
      /// Rear wheel track [m]
      double rearWheelTrack;
      /// Front wheel track [m]
      double frontWheelTrack;
      /// Meters per tick
      double metersPerTick;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (rearLeftWheelRadius == other.rearLeftWheelRadius) &&
          (rearRightWheelRadius == other.rearRightWheelRadius) &&
          (frontLeftWheelRadius == other.frontLeftWheelRadius) &&
          (frontRightWheelRadius == other.frontRightWheelRadius) &&
          (rearWheelTrack == other.rearWheelTrack) &&
          (frontWheelTrack == other.frontWheelTrack) &&
          (metersPerTick == other.metersPerTick);
      }
    };
    /// Parent type
    typedef Odometry Parent;
    /// Self type
    typedef DifferentialOdometry Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    DifferentialOdometry(const Parameters& parameters, const Eigen::Vector3d&
      initialPose = Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Copy constructor
    DifferentialOdometry(const Self& other) = delete;
    /// Copy assignment operator
    DifferentialOdometry& operator = (const Self& other) = delete;
    /// Move constructor
    DifferentialOdometry(Self&& other) = delete;
    /// Move assignment operator
    DifferentialOdometry& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~DifferentialOdometry();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with rear wheel translational velocities
    virtual void updateRearWheelTranslationalVelocities(double vLeftWheel,
      double vRightWheel, double timestamp);
    /// Update odometry with rear wheel rotational velocities
    virtual void updateRearWheelRotationalVelocities(double wLeftWheel, double
      wRightWheel, double timestamp);
    /// Update odometry with rear wheel ticks difference
    virtual void updateRearWheelTicks(double dLeftWheel, double dRightWheel,
      double timestamp);
    /// Update odometry with rear wheel displacements
    virtual void updateRearWheelDisplacements(double dLeftWheel, double
      dRightWheel, double timestamp);
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the parameters
    const Parameters& getDifferentialParameters() const;
    /// Returns the parameters
    Parameters& getDifferentialParameters();
    /** @}
      */

  protected:
    /** \name Protected members
      @{
      */
    /// Parameters
    Parameters differentialParameters_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    DifferentialOdometry::Parameters& params);

}

#endif // JANETH_GENERIC_DIFFERENTIAL_ODOMETRY_H
