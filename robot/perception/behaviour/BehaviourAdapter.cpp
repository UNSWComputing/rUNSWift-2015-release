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

#include <limits>
#include <utility>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include "perception/behaviour/BehaviourAdapter.hpp"
#include "perception/behaviour/BehaviourHelpers.hpp"

#include "ReadySkillPositionAllocation.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "types/BehaviourRequest.hpp"
#include "utils/body.hpp"
#include "utils/speech.hpp"
#include "utils/basic_maths.hpp"
#include "types/SensorValues.hpp"
#include "perception/behaviour/python/PythonSkill.hpp"

#include <boost/python.hpp>

using namespace std;
using namespace boost::python;

BehaviourAdapter::BehaviourAdapter(Blackboard *bb) : Adapter(bb), calibrationSkill(bb) {
   llog(INFO) << "Constructing BehaviourAdapter" << endl;
   pythonSkill = new PythonSkill(bb);

   std::stringstream startupSpeech;
   startupSpeech << std::string("Player ") << BehaviourHelpers::playerNumber(blackboard) <<
      " team " << BehaviourHelpers::teamNumber(blackboard);
   SAY(startupSpeech.str());
}

BehaviourAdapter::~BehaviourAdapter() {
}

void BehaviourAdapter::tick() {
   BehaviourRequest behaviourRequest;
   // Run the python skills
   behaviourRequest = pythonSkill->execute();

   // run current skill, or kinematics calibrator
   if (readFrom(kinematics, isCalibrating)) {
      behaviourRequest = calibrationSkill.execute();
   } else {
      //behaviourRequest = skillInstance->tick(blackboard); //, Skill::NULL_FLAGS);
   }
   
   // override camera from offnao if necessary
   string whichCamera = blackboard->config["default.whichCamera"].as<string>();
   if (whichCamera != "BEHAVIOUR") {
      if (whichCamera == "TOP_CAMERA") {
         behaviourRequest.whichCamera = TOP_CAMERA;
      } else if (whichCamera == "BOTTOM_CAMERA") {
         behaviourRequest.whichCamera = BOTTOM_CAMERA;
      }
   }

   // Extract walkingTo and shootingTo
   AbsCoord walkingTo(behaviourRequest.walkingToX,
                      behaviourRequest.walkingToY,
                      0);
   AbsCoord shootingTo(behaviourRequest.shootingToX,
                       behaviourRequest.shootingToY,
                       0);
   writeTo(behaviour, walkingTo, walkingTo);
   writeTo(behaviour, shootingTo, shootingTo);
   
   // Write ActionCommands to blackboard
   int writeBuf = (readFrom(behaviour, readBuf) + 1) % 2;
   writeTo(behaviour, request[writeBuf], safetySkill.wrapRequest(behaviourRequest, readFrom(motion, sensors)));
   writeTo(behaviour, readBuf, writeBuf);

   if (behaviourRequest.readyPositionAllocation0 > 0) {
      std::vector<int> readyAllocation;
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation0);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation1);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation2);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation3);
      readyAllocation.push_back(behaviourRequest.readyPositionAllocation4);
      
      ReadySkillPositionAllocation newPositionAllocation(
            readFrom(gameController, player_number), readyAllocation);
      
      ReadySkillPositionAllocation currentPositionAllocation =
            readFrom(behaviour, behaviourSharedData).readyPositionAllocation;
      if (newPositionAllocation.canOverride(currentPositionAllocation)) {
         writeTo(behaviour, behaviourSharedData.readyPositionAllocation, newPositionAllocation);
      }
   }
   
   // Write GoalieAttacking request to behaviour blackboard, from where it'll be broadcast to the team
   writeTo(behaviour, behaviourSharedData.goalieAttacking, behaviourRequest.goalieAttacking);
   writeTo(behaviour, behaviourSharedData.doingBallLineUp, behaviourRequest.doingBallLineUp);
   writeTo(behaviour, behaviourSharedData.isInReadyMode, behaviourRequest.isInReadyMode);
   writeTo(behaviour, behaviourSharedData.timeToReachBall, behaviourRequest.timeToReachBall);
   writeTo(behaviour, behaviourSharedData.timeToReachDefender, behaviourRequest.timeToReachDefender);
   writeTo(behaviour, behaviourSharedData.timeToReachMidfielder, behaviourRequest.timeToReachMidfielder);
   writeTo(behaviour, behaviourSharedData.timeToReachUpfielder, behaviourRequest.timeToReachUpfielder);
   writeTo(behaviour, behaviourSharedData.currentRole, behaviourRequest.currentRole);
}
