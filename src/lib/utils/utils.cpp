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

#include "utils/utils.h"

#include <cmath>

namespace janeth {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void genSineWavePath(std::vector<Eigen::Matrix<double, 3, 1> >& u, size_t
      steps, double amplitude, double frequency, double T) {
    u.clear();
    u.reserve(steps);
    u.push_back(Eigen::Matrix<double, 3, 1>::Zero());
    std::vector<double> cost;
    for (double t = 0; t < steps * T; t += T)
      cost.push_back(amplitude * cos(2 * M_PI * frequency * t));
    double theta = 0;
    for (size_t i = 1; i < cost.size(); ++i) {
      const double theta_new = atan(cost[i]);
      const double v = 2 * M_PI * frequency *
        (sqrt(1 + cost[i] * cost[i]) +
        sqrt(1 + cost[i - 1] * cost[i - 1])) / 2;
      const double om = (theta_new - theta) / T;
      theta = theta_new;
      u.push_back(Eigen::Matrix<double, 3, 1>(v, 0, om));
    }
  }

  double angleMod(double radians) {
    return atan2(sin(radians), cos(radians));
  }

}
