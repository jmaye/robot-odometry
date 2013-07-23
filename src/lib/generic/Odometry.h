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

/** \file Odometry.h
    \brief This file defines the Odometry class, which is the base class for
           odometry models.
  */

#ifndef JANETH_GENERIC_ODOMETRY_H
#define JANETH_GENERIC_ODOMETRY_H

#include <vector>

#include <Eigen/Core>

namespace janeth {

  /** The class Odometry is the base class for odometry models.
      \brief Base odometry model
    */
  class Odometry {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef Odometry Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    Odometry(const Eigen::Vector3d& initialPose = Eigen::Vector3d::Zero(),
      double initialTimestamp = 0);
    /// Copy constructor
    Odometry(const Self& other) = delete;
    /// Copy assignment operator
    Odometry& operator = (const Self& other) = delete;
    /// Move constructor
    Odometry(Self&& other) = delete;
    /// Move assignment operator
    Odometry& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~Odometry();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the current pose
    const Eigen::Vector3d& getPose() const;
    /// Returns the current timestamp
    double getTimestamp() const;
    /// Returns the poses
    const std::vector<Eigen::Vector3d>& getPoses() const;
    /// Returns the timestamps
    const std::vector<double>& getTimestamps() const;
    /// Resets pose history and sets initial pose
    virtual void reset(const Eigen::Vector3d& initialPose =
      Eigen::Vector3d::Zero(), double initialTimestamp = 0);
    /// Inserts a new pose
    void insertPose(const Eigen::Vector3d& pose, double timestamp);;
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Updates odometry with COG displacement
    void updateCOGDisplacement(double dTrans, double dRot, double timestamp);
    /// Updates odometry with COG velocity
    void updateCOGVelocity(double vTrans, double vRot, double timestamp);
    /** @}
      */

  protected:
    /** \name Protected members
      @{
      */
    /// Pose history
    std::vector<Eigen::Vector3d> poses_;
    /// Timestamp history
    std::vector<double> timestamps_;
    /// Last velocity update timestamp
    double lastVelocityTimestamp_;
    /** @}
      */

  };

}

#endif // JANETH_GENERIC_ODOMETRY_H
