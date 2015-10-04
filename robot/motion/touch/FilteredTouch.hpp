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

#include <fstream>
#include "motion/touch/Touch.hpp"
#include "motion/touch/TorsoStateFilter.hpp"
#include "motion/touch/SensorOutput.hpp"

using namespace boost::numeric::ublas;

class FilteredTouch : Touch {
   public:
      explicit FilteredTouch(Touch* t);
      ~FilteredTouch();
      SensorValues getSensors(Kinematics &kinematics);
      void readOptions(const boost::program_options::variables_map& config);
      bool getStanding();
      ButtonPresses getButtons();
   private:
      Touch* touch;
      SensorOutput csv;
      bool init;
      SensorValues state;     //filtered
      SensorValues update;    //raw
      FeetState feetState;

      //offset calibration variables
      float imuOffset[3][3];
      float targetOffset[3][3];
      float avg[3][3];
      float count[3][3];
      float k[3][3];
      float err[3][3];
      float prevAng[2];

      //kinematics body state
      Kinematics kinematics;
      matrix<float> lastBodyPosition;
      matrix<float> bodyRotation;
      matrix<float> bodyOrientation;
//      float footPos[2][3];

      TorsoStateFilter kf[2];

      float getScaledGyr(int isY);
      void updateIMUOffset();
      void filterOffset(int IMUid, int dir, float obs);
      void filterSensors();
};
