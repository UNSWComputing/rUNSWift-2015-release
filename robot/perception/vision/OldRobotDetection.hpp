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
#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/CameraToRR.hpp"
#include "perception/vision/ImageRegion.hpp"
#include "Fovea.hpp"
#include "utils/body.hpp"
#include "types/RobotInfo.hpp"
#include "types/AbsCoord.hpp"


class OldRobotDetection {
   friend class VisionTab;
   
   public:

      // sonar distance in mm
      std::vector< std::vector <int> > sonar; 
      
      OldRobotDetection();
      
      std::vector<RobotInfo> robots;

      void findRobots(VisionFrame &frame,
                      std::vector<Point> edgePoints,
                      const Fovea &saliency
                     );

   private:
      //Reference data
      CameraToRR convRR;
      bool saliency_top;
      int saliency_cols;
      int saliency_rows;
      int saliency_density;
      std::vector<int> startOfScan;
      std::vector<Point> edgePoints;

      
      //Internal + debug data
      typedef struct RDData {
         RobotInfo robot;
         int colourProfile[cNUM_COLOURS+1]; // last value is for edge count
         int colourTotal;
         Point midFoot;
         float width;
         bool skip;
      } RDData;
      
      std::vector<RDData>  allRobots;
      std::vector<int>     robotPoints;
      std::vector<bool>    pointsCheck;
      std::vector<BBox>    postsCheck;

      
      //Field edge deviation method
      void generateDeviationPoints(const Fovea &saliency);

      // Find the number of pixels or a certain colour in a lookDistance neighbourhood      
      inline int  getColorAmount(const Fovea &saliency, int r, int c, Colour colour, int lookDistance);

      // Adds a robot point, checking if its below the field edge and not in a post
      inline void addRobotPoint(int value, int index);

      // Creates robot regions out of robot points
      void robotPointsToRegion();
      inline void completePreviousRobotRegion();

      // Enlarges the BBox to capture the new point (x,y)
      inline void updateBBox(BBox& box, int x, int y);

      // Rejoining robots that are closed together based on RR distance
      void mergeRobotRegion(const Fovea &saliency);

      // Check robots for reasonable colour, edges, presence of waistband and sonar
      void sanityCheckRobots(const Fovea &saliency);

      // Try to reduce the size of the bounding box if it has green on left, bottom or right edges
      inline void greenShrink(const Fovea & saliency, BBox &box);

      // Update the distance to match sonar if sufficiently close, and return true if verified
      bool SonarVerifyDist(std::vector<int> &sonar, RRCoord &location, bool footSeen = true);
    
    
};
