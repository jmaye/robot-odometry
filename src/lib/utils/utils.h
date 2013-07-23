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

/** \file utils.h
    \brief This file contains a bunch of utilities.
  */

#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

#include <vector>

#include <Eigen/Core>

namespace janeth {

  /** \name Methods
    @{
    */
  /// Generate a sine wave path
  void genSineWavePath(std::vector<Eigen::Matrix<double, 3, 1> >& u,
    size_t steps, double amplitude, double frequency, double T);
  /// Converts an angle in radians into [-pi, pi] range
  double angleMod(double radians);
  /** @}
    */

}

#endif // UTILS_UTILS_H
