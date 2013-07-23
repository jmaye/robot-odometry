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

/** \file AckermannOdometry.h
    \brief This file defines the AckermannOdometry class, which implements
           an odometry model based on Ackermann geometry.
  */

#ifndef JANETH_GENERIC_ACKERMANN_ODOMETRY_H
#define JANETH_GENERIC_ACKERMANN_ODOMETRY_H

#include <iosfwd>

#include <Eigen/Core>

#include "generic/DifferentialOdometry.h"

namespace janeth {

  /** The class AckermannOdometry implements an odometry model based on
      Ackermann geometry.
      \brief Ackermann odometry
    */
  class AckermannOdometry :
    public DifferentialOdometry {
  public:
    /** \name Types definitions
      @{
      */
    /// Ackermann odometry parameters
    struct Parameters {
      /// Wheel base [m]
      double wheelBase;
      /// Equality operator for comparing parameters
      bool operator == (const Parameters &other) const {
        return (wheelBase == other.wheelBase);
      }
    };
    /// Parent type
    typedef DifferentialOdometry Parent;
    /// Self type
    typedef AckermannOdometry Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor with initial pose and parameters
    AckermannOdometry(const Parent::Parameters& differentialParameters, const
      Parameters& ackermannParameters, const Eigen::Vector3d& initialPose =
      Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Copy constructor
    AckermannOdometry(const Self& other) = delete;
    /// Copy assignment operator
    AckermannOdometry& operator = (const Self& other) = delete;
    /// Move constructor
    AckermannOdometry(Self&& other) = delete;
    /// Move assignment operator
    AckermannOdometry& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~AckermannOdometry();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with rear wheel translational velocities and steering
    virtual void updateRearWheelTranslationalVelocitiesSteering(double
      vLeftWheel, double vRightWheel, double steering, double timestamp);
    /// Update odometry with rear wheel rotational velocities and steering
    virtual void updateRearWheelRotationalVelocitiesSteering(double wLeftWheel,
      double wRightWheel, double steering, double timestamp);
    /// Update odometry with rear wheel ticks difference and steering
    virtual void updateRearWheelTicksSteering(double dLeftWheel, double
      dRightWheel, double steering, double timestamp);
    /// Update odometry with rear wheel displacements and steering
    virtual void updateRearWheelDisplacementsSteering(double dLeftWheel, double
      dRightWheel, double steering, double timestamp);
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the parameters
    const Parameters& getAckermannParameters() const;
    /// Returns the parameters
    Parameters& getAckermannParameters();
    /** @}
      */

  protected:
    /** \name Protected members
      @{
      */
    /// Parameters
    Parameters ackermannParameters_;
    /** @}
      */

  };

  /// Streaming operator for parameters
  ::std::ostream& operator<<(::std::ostream& os, const
    AckermannOdometry::Parameters& params);

}

#endif // JANETH_GENERIC_ACKERMANN_ODOMETRY_H
