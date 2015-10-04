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

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <utility>
#include <string>
#include "utils/Timer.hpp"

#include "types/FootInfo.hpp"
#include "types/BallInfo.hpp"
#include "types/PostInfo.hpp"
#include "types/RobotInfo.hpp"
#include "types/FieldEdgeInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/Ipoint.hpp"
#include "types/Odometry.hpp"

#include "NNMC.hpp"
#include "Camera.hpp"
#include "VisionDefs.hpp"
#include "CameraToRR.hpp"
#include "yuv.hpp"
#include "BallDetection.hpp"
#include "FieldLineDetection.hpp"
#include "GoalDetection.hpp"
#include "OldRobotDetection.hpp"
#include "robotdetection/RobotDetection.hpp"
#include "FieldEdgeDetection.hpp"
#include "Fovea.hpp"
#include "SurfDetection.hpp"
#include "GoalMatcher.hpp"
#include "FootDetection.hpp"
class VisionFrame;

class Vision {
   friend class VisionAdapter;
   friend class CalibrationTab;
   friend class VisionTab;
   friend class OverviewTab;
   friend class Tab;
   friend class CameraPoseTab;
   friend class ControlTab;
   friend class SurfTab;
   friend class ICPTab;
   friend class FootDetection;
   friend class GoalDetection;
   friend class BallDetection;
   friend class RobotDetection;
   friend class SurfDetection;

   public:
      /**
       * Constructor for vision module. Initialises the camera
       * Only displays an error if camera not initated
       **/
      Vision(bool dumpframes,
             int dumprate,
             std::string dumpfile,
             std::string calibrationFileTop,
             std::string calibrationFileBot,
             std::string goalMap,
				 std::string vocabFile,
             bool visionEnabled,
             bool seeBluePosts,
             bool seeLandmarks);

      /* Destructor */
      ~Vision();

      /* Pointer to current vision frame */
      boost::shared_ptr<VisionFrame> frame;

      /**
       * Camera object, talks to v4l or NaoQi
       * Initilised by main
       **/
      static Camera *camera;
      static Camera *top_camera;
      static Camera *bot_camera;

      std::vector<Ipoint>           landmarks;
      std::vector<Point>		    feetDebug;
      std::vector<FootInfo>		 	feetBoxes;
      std::vector<BallInfo>         balls;
      BallHint                      ballHint;
      std::vector<PostInfo>         posts;
      std::vector<RobotInfo>        robots;
      std::vector<FieldEdgeInfo>    fieldEdges;
      std::vector<FieldFeatureInfo> fieldFeatures;
      unsigned int                  missedFrames;
      std::pair<int, int>           dxdy;
      PostInfo::Type                goalArea;
      float                         awayGoalProb;
      int                           awayMapSize;
      int                           homeMapSize; 

      FoveaT<hGoals, eGrey> topSaliency;
      FoveaT<hGoals, eGrey> botSaliency;

      /**
       * C-Space lookup table
       **/
      NNMC nnmc_top, nnmc_bot;

   private:

      /**
       * Variables for turning parts of vision on and off
       **/
      bool seeBluePosts;
      bool seeLandmarks;

      /**
       * Pointer to the frame currently being processed
       **/
      uint8_t const* currentFrame;
      uint8_t const* topFrame;
      uint8_t const* botFrame;
      

      /**
       * Seed value for rand_r
       **/
      unsigned int seed;

      /**
       * which camera is in use
       **/
      WhichCamera whichCamera;

      /**
       * A nice wall clock
       **/
      Timer timer;

      /**
       * Settings related to recording frames to disk
       **/
      bool dumpframes;
      int dumprate;
      std::string dumpfile;
      std::string calibrationFileTop, calibrationFileBot;
      std::string robotMap;
      std::string goalMap;
      std::string vocabFile;
      bool visionEnabled;

      /**
       * The major sections of vision processing used by process frame
       **/
          
      CameraToRR convRR;
      SurfDetection surfDetection;
      RobotDetection robotDetection;
      OldRobotDetection oldRobotDetection;
      FieldEdgeDetection fieldEdgeDetection;
      FieldLineDetection fieldLineDetection;
      BallDetection ballDetection;
      GoalDetection goalDetection;
      GoalMatcher goalMatcher;
      FootDetection footDetection;

      /**
       * Get a frame from the camera and put a pointer to it
       * in currentFrame
       **/
      void getFrame();

      /**
       * Processes a single frame, reading it from the camera
       * finding all interesting objects, and running sanity checks
       **/
      void processFrame();

      /**
       * Subsample the image, forming a colour histogram
       * in the x-axis and y-axis, for each colour in
       * the C-PLANE.
       **/
      void saliencyScan();
      void edgeSaliencyScan();

      void pickPost(std::vector<PostInfo>& posts,
                    std::vector<FieldFeatureInfo>& features);
};

