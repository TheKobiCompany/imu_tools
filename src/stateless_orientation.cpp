/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu_filter_madgwick/stateless_orientation.h"

namespace imu_filter_madgwick {

bool StatelessOrientation::computeOrientation(WorldFrame::WorldFrame frame,
                                              const Eigen::Vector3f &Av,
                                              const Eigen::Vector3f &Ev,
                                              Eigen::Quaternionf &orientation) {
  Eigen::Vector3f Hv = Ev.cross(Av);
  float normH = Hv.norm();
  if (normH < 1E-7) {
    // device is close to free fall (or in space?), or close to
    // magnetic north pole.
    // mag in T => Threshold 1E-7, typical values are  > 1E-5.
    return false;
  }
  Hv.stableNormalize();
  Eigen::Vector3f Avn = Av.stableNormalized();
  Eigen::Vector3f Mv = Avn.cross(Hv);

  Eigen::Matrix3f R;
  switch (frame) {
  case WorldFrame::NED:
    R.col(0) = Mv;
    R.col(1) = Hv;
    R.col(2) = -Avn;
    break;

  case WorldFrame::NWU:
    R.col(0) = Mv;
    R.col(1) = -Hv;
    R.col(2) = Avn;
    break;

  default:
  case WorldFrame::ENU:
    R.col(0) = Hv;
    R.col(1) = Mv;
    R.col(2) = Avn;
    break;
  }
  orientation = Eigen::Quaternionf(R).inverse();
  return true;
}

bool StatelessOrientation::computeOrientation(
    WorldFrame::WorldFrame frame, geometry_msgs::Vector3 A,
    geometry_msgs::Vector3 E, geometry_msgs::Quaternion &orientation) {

  // A: pointing up
  Eigen::Vector3f Av(A.x, A.y, A.z);
  Eigen::Vector3f Ev(E.x, E.y, E.z);
  Eigen::Quaternionf q;
  bool result = computeOrientation(frame, Av, Ev, q);
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
  return result;
}

bool StatelessOrientation::computeOrientation(WorldFrame::WorldFrame frame,
                                              const Eigen::Vector3f &A,
                                              Eigen::Quaternionf &orientation) {
  Eigen::Vector3f E;
  if (fabs(A.x()) > 0.1 || fabs(A.y()) > 0.1) {
    E.x() = A.y();
    E.y() = A.x();
    E.z() = 0.0;
  } else if (fabs(A.z()) > 0.1) {
    E.x() = 0.0;
    E.y() = A.z();
    E.z() = A.y();
  } else {
    // free fall
    return false;
  }

  return computeOrientation(frame, A, E, orientation);
}

bool StatelessOrientation::computeOrientation(
    WorldFrame::WorldFrame frame, geometry_msgs::Vector3 A,
    geometry_msgs::Quaternion &orientation) {
  Eigen::Vector3f Av(A.x, A.y, A.z);
  Eigen::Quaternionf q;
  bool result = computeOrientation(frame, Av, q);
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();
  return result;
}

} // namespace imu_filter_madgwick
