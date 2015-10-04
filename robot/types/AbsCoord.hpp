/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#pragma once

#include "RRCoord.hpp"
#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "types/boostSerializationEigenTypes.hpp"
#include <Eigen/Eigen>

struct AbsCoord {
   AbsCoord(float x, float y, float theta) : vec(x, y, theta) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   AbsCoord() : vec(0, 0, 0) {
      var.setZero();
      var(0, 0) = SQUARE(FULL_FIELD_LENGTH);
      var(1, 1) = SQUARE(FULL_FIELD_WIDTH);
      var(2, 2) = SQUARE(M_PI);
      weight = 1.0;
   }

   Eigen::Vector3f vec;
   Eigen::Matrix<float, 3, 3> var;
   float weight;

   const float x() const {
      return vec[0];
   }

   float &x() {
      return vec[0];
   }

   const float y() const {
      return vec[1];
   }

   float &y() {
      return vec[1];
   }

   const float theta() const {
      return vec[2];
   }

   float &theta() {
      return vec[2];
   }

   float getVar(int m, int n) const {
      return var(m,n);
   }
   
   bool operator== (const AbsCoord &other) const {
      return vec == other.vec;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & vec;
      ar & var;
   }
   
   /**
    * This assumes that the AbsCoord is already in robot relative coordinates, this will simply
    * convert it to polar coords.
    */
   RRCoord convertToRobotRelative(void) const {
      return RRCoord(sqrtf(vec.x()*vec.x() + vec.y()*vec.y()), atan2f(vec.y(), vec.x()), 0.0f);
   }
   
   RRCoord convertToRobotRelative(const AbsCoord &robotPose) const {
      float xdiff = x() - robotPose.x();
      float ydiff = y() - robotPose.y();
      float distance = sqrtf(xdiff*xdiff + ydiff*ydiff);
      
      float angle = atan2(ydiff, xdiff);
      float rrHeading = normaliseTheta(angle - robotPose.theta());
      
      return RRCoord(distance, rrHeading, 0.0f);
   }
   
   AbsCoord convertToRobotRelativeCartesian(const AbsCoord &robotPose) const {
      RRCoord rrCoord = convertToRobotRelative(robotPose);
      return AbsCoord(
            rrCoord.distance() * cos(rrCoord.heading()), 
            rrCoord.distance() * sin(rrCoord.heading()),
            0.0f);
   }
};
