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

#include <vector>
#include <utility>

#include "VisionConstants.hpp"
#include "VisionDefs.hpp"
#include "CameraToRR.hpp"
#include "VisionFrame.hpp"
#include "Fovea.hpp"

#include "types/FieldEdgeInfo.hpp"
#include "types/Point.hpp"

class FieldEdgeDetection
{
   public:
      /**
       * Points to be ransaced
      **/
      std::vector<Point> edgePointsTop;
      std::vector<Point> edgePointsBot;

      /**
       * The coordinates of the top of the field in the image
       * The coordinates are given in image coordinates
       **/
      int topStartScanCoords[IMAGE_COLS];
      int botStartScanCoords[IMAGE_COLS];

      void findFieldEdges(VisionFrame &frame,
                          const Fovea &topFovea,
                          const Fovea &botFovea,
                          CameraToRR *convRR,
                          unsigned int *seed);


      /**
       * Find coordinates of points that may be at the edge
       * of the field by using the saliency scan
       * @param frame      Current vision frame
       * @param fovea      Current fovea to be searched
       **/
      void fieldEdgePoints(VisionFrame &frame,
                           const Fovea &fovea,
                           bool top);

      /**
       * Find up to two lines formed by field edge points
       * using the RANSAC algorithm
       **/
      void fieldEdgeLines(unsigned int *seed,
                          CameraToRR *convRR,
                          bool top);

      /**
       * Fills the startScanCoords array to find the coordinates
       * in the saliency scan where the field starts
       **/
      void findStartScanCoords(VisionFrame &frame,
                               const Fovea &fovea);

      explicit FieldEdgeDetection();

      std::vector<FieldEdgeInfo> fieldEdges;

   private:
      static const int consecutive_green;

      void lsRefineLine(
            RANSACLine               &line,
            const std::vector<Point> &points,
            const std::vector<bool>  &cons);


      Point adjustVertEdgePoint(
            VisionFrame &frame,
            const Fovea &fovea,
            Point p);

      /**
       * A cummulative count of green pixels occuring
       * at the top of the image
       */
      int greenTops[TOP_SALIENCY_COLS];
      int totalGreens;
};

