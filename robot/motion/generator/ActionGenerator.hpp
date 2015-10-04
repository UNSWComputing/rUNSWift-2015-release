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

#include <string>
#include <vector>
#include "motion/generator/Generator.hpp"

/* Determine whether the class is just called */
#define NOT_RUNNING -1

/* Hack the stiffness */
#define MAX_STIFF 1.0

class ActionGenerator : Generator {
   public:
      explicit ActionGenerator(std::string filename);
      ~ActionGenerator();
      
      virtual JointValues makeJoints(ActionCommand::All* request,
                                     Odometry* odometry,
                                     const SensorValues &sensors,
                                     BodyModel &bodyModel,
                                     float ballX,
                                     float ballY);
      virtual bool isActive();
      void reset();
      void stop();
      void readOptions(const boost::program_options::variables_map &config);

   private:
      int current_time;
      std::string file_name;
      std::vector<JointValues> joints;
      ActionCommand::Body active;

      /**
       * Determine the duration before the robot
       * actually begins to execute the sequence
       * of poses
       */
      int max_iter;

      /**
       * Interpolate the time that are determined by the
       * duration between the new joints with the previous
       * joints that are read in the file
       * @param newJoint the value of the joint that need to be
       * interpolated with the previous value
       * @param duration if duration = 0, it will do the interpolation
       * between the newJoint value with the joint at MAX_ITER position.
       * Otherwise, it will interpolate between the new joint and the previous
       * joint.
       */
      void interpolate(JointValues newJoint, int duration = 0);

      /**
       * Reading the file from the provided path and construct
       * the pose
       * @param path the directory to read the pose file
       */
      void constructPose(std::string path);
};
