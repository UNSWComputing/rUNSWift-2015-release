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
#include <map>
#include "motion/effector/Effector.hpp"
#include "motion/generator/Generator.hpp"
#include "motion/touch/FilteredTouch.hpp"
#include "blackboard/Adapter.hpp"
#include "motion/generator/BodyModel.hpp"
#include "perception/kinematics/Kinematics.hpp"
#include "motion/SonarRecorder.hpp"

/**
 * MotionAdapter - interfaces between Motion & rest of system via Blackboard
 *
 * The MotionAdapter is the controlling class for everything Motion. It reads
 * ActionCommands from the Blackboard, and SensorValues from its Touch instance.
 *
 * It then passes AC::Head, AC::Body & SensorValues to a Generator instance,
 * which processes them and determines the correct JointValues for the next DCM
 * cycle. In practice, the Generator will usually be a DistributorGenerator,
 * which will select the most appropriate Generator based on the AC's.
 *
 * These JointValues, plus AC::LED are passed to an Effector instance. The
 * effector actuates the joints and LEDs according to these instructions.
 */
class MotionAdapter : Adapter {
   public:
      /* Constructor */
      MotionAdapter(Blackboard *bb);
      /* Destructor */
      ~MotionAdapter();
      /* One cycle of this thread */
      void tick();
      /* Read values from global options */
      void readOptions(const boost::program_options::variables_map& config);
   private:
      Odometry odometry;
      /* Buffers so synchronises with vision thread */
      std::vector<SensorValues> sensorBuffer;
      /* Sonar window recorder */
      SonarRecorder sonarRecorder;
      /* Duration since we last were told to stand up by libagent (seconds) */
      float uptime;
      /* Maps of available switchable instances */
      std::map<std::string, Touch*> touches;
      std::map<std::string, Effector*> effectors;
      /* Motion module instance */
      Touch* nakedTouch;  //original, unfiltered
      Touch* touch;
      Generator* generator;
      Effector* effector;
      BodyModel bodyModel;
      Kinematics kinematics;
};
