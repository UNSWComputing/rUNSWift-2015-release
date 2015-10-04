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

#include <stdint.h>
#include <math.h>
#include "types/RRCoord.hpp"
#include "types/RansacTypes.hpp"
#include "perception/vision/Camera.hpp"
#include "types/SensorValues.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/CameraDefs.hpp"
#include "perception/kinematics/Pose.hpp"

#include "types/RRCoord.hpp"
#include "types/SensorValues.hpp"
#include "types/Point.hpp"

/**
 * All constant measurements are in mm or radians
 **/

class CameraToRR {
   public:
      CameraToRR();
      ~CameraToRR();

      void setCamera(Camera *camera);
      void updateAngles(SensorValues values);
      RRCoord convertToRR(int16_t i, int16_t j, bool isBall) const;
      RRCoord convertToRR(const Point &p, bool isBall) const;
      Point convertToRRXY(const Point &p) const;
      RANSACLine convertToRRLine(const RANSACLine &l) const;
      Point convertToImageXY(const Point &p) const;

      Pose pose;
      float pixelSeparationToDistance(int pixelSeparation, int realSeparation) const;
      float ballDistanceByRadius(int radius, bool top) const;
      bool isRobotMoving() const;

      /**
       * Finds the saliency scan coordinates where vertical scans
       * should be stopped to avoid considering the robots own body
       * The coordinates in the array returned are image co-ords
       **/
      void findEndScanValues();

      int topEndScanCoords[IMAGE_COLS];
      int botEndScanCoords[IMAGE_COLS];

   private:
      Camera *camera;
      SensorValues values;
};



