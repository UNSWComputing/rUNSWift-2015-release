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

#include <iostream>

struct Odometry {
   float forward;
   float left;
   float turn;

   Odometry(float f = 0.0f, float l = 0.0f, float t = 0.0f)
      : forward(f), left(l), turn(t) {}

   inline void clear() {
      forward = left = turn = 0.0f;
   }

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version)
   {
      ar & forward & left & turn;
   }
};

inline std::ostream& operator<<(std::ostream& os, const Odometry& odometry) {
   os.write((char*) &(odometry.forward), sizeof(float));
   os.write((char*) &(odometry.left), sizeof(float));
   os.write((char*) &(odometry.turn), sizeof(float));

   return os;
}

inline std::istream& operator>>(std::istream& is, Odometry& odometry) {
   is.read((char*) &(odometry.forward), sizeof(float));
   is.read((char*) &(odometry.left), sizeof(float));
   is.read((char*) &(odometry.turn), sizeof(float));

   return is;
}

#ifndef SWIG
   static inline Odometry operator+(const Odometry& a, const Odometry& b) {
      Odometry c;
      c.forward = a.forward + b.forward;
      c.left = a.left + b.left;
      c.turn = a.turn + b.turn;
      return c;
   }

   static inline Odometry operator-(const Odometry& a, const Odometry& b) {
      Odometry c;
      c.forward = a.forward - b.forward;
      c.left = a.left - b.left;
      c.turn = a.turn - b.turn;
      return c;
   }
#endif
