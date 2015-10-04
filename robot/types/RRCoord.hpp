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

#include <math.h>
#include <string.h>
#include <Eigen/Eigen>
#include <boost/serialization/version.hpp>
#include "types/Point.hpp"

struct RRCoord {
   /**
    * RRCoord
    * If any param is set to NAN this implies that the value is either
    * not known or irrelevant to the particular context.
    *
    * @param heading       heading from robot to object
    * @param distance      distance from robot to object
    * @param orientation   angle between robot front and object front
    */
   RRCoord(float distance, float heading = NAN, float orientation = NAN)
      : vec(distance, heading, orientation)
   {
      var.setZero();
   }

   Point toCartesian() const {
      return Point(distance()*cosf(heading()), distance()*sinf(heading()));
   }

   RRCoord() {
      vec.setZero();
      var.setZero();
   }
   
   RRCoord(const RRCoord &other) {
      this->vec = other.vec;
      this->var = other.var;
   }

   RRCoord& operator=(const RRCoord &other) {
      for (unsigned i = 0; i < 3; i++) {
         this->vec(i, 0) = other.vec(i, 0);
      }
      for (unsigned i = 0; i < 9; i++) {
         this->var[i] = other.var[i];
      }
      
      return *this;
   }
   
   Eigen::Vector3f vec;
   Eigen::Matrix<float, 3, 3> var;

   /* Distance to object */
   const float distance() const {
      return vec[0];
   }

   float &distance() {
      return vec[0];
   }

   /* Heading to object */
   const float heading() const {
      return vec[1];
   }

   float &heading() {
      return vec[1];
   }

   /* Angle between robot's front and object's front */
   const float orientation() const {
      return vec[2];
   }

   float &orientation() {
      return vec[2];
   }

   bool operator== (const RRCoord &other) const {
      return vec == other.vec;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version);
};

inline std::ostream& operator<<(std::ostream& os, const RRCoord& coord) {
   for (unsigned i = 0; i < 3; i++) {
      float a = coord.vec[i];
      os.write((char*) &a, sizeof(float));
   }
   
   for (unsigned i = 0; i < 9; i++) {
      float a = coord.var[i];
      os.write((char*) &a, sizeof(float));
   }
   return os;
}

inline std::istream& operator>>(std::istream& is, RRCoord& coord) {
   for (unsigned i = 0; i < 3; i++) {
      float a;
      is.read((char*) &a, sizeof(float));
      coord.vec[i] = a;
   }
   
   for (unsigned i = 0; i < 9; i++) {
      float a;
      is.read((char*) &a, sizeof(float));
      coord.var[i] = a;
   }

   return is;
}

typedef enum {
   DIST,
   HEADING,
   ORIENTATION,
} RRCoordEnum;

#include "types/RRCoord.tcc"

