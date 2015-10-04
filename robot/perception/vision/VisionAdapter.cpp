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

#include "VisionAdapter.hpp"
#include "VarianceCalculator.hpp"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <pthread.h>
#include <vector>
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "perception/kinematics/Pose.hpp"

#include "types/BallInfo.hpp"

#include "transmitter/Team.hpp"
#include "receiver/Team.hpp"
#include "receiver/RemoteControl.hpp"
#include "gamecontroller/GameController.hpp"

using namespace std;
using namespace boost::algorithm;

TeamTransmitter *transmitter = NULL;
TeamReceiver *receiver = NULL;
RemoteControlReceiver *remoteControl = NULL;
GameController *gameController = NULL;
unsigned networkCount = 0;

static void networkTick(Blackboard *bb) {
   return;
   if (transmitter == NULL || receiver == NULL || 
       remoteControl == NULL || gameController == NULL) {
      transmitter = new TeamTransmitter(bb);
      receiver = new TeamReceiver(bb);
      remoteControl = new RemoteControlReceiver(bb);
      gameController = new GameController(bb);
   }
   
   transmitter->tick();
   receiver->tick();
   remoteControl->tick();
   gameController->tick();
}


VisionAdapter::VisionAdapter(Blackboard *bb)
   : Adapter(bb),
     V((blackboard->config)["vision.dumpframes"].as<bool>(),
       (blackboard->config)["vision.dumprate"].as<int>(),
       (blackboard->config)["vision.dumpfile"].as<string>(),
       (blackboard->config)["vision.top_calibration"].as<string>(),
       (blackboard->config)["vision.bot_calibration"].as<string>(),
       (blackboard->config)["vision.goal_map"].as<string>(),
       (blackboard->config)["vision.vocab"].as<string>(),
       (blackboard->config)["debug.vision"].as<bool>(),
       (blackboard->config)["vision.seeBluePosts"].as<bool>(),
       (blackboard->config)["vision.seeLandmarks"].as<bool>())
{
   writeTo(vision, topSaliency, (Colour*)V.topSaliency._colour);
   writeTo(vision, botSaliency, (Colour*)V.botSaliency._colour);
}

VisionAdapter::~VisionAdapter() {
}

void VisionAdapter::tick() {
   timer.restart();

   llog(VERBOSE) << "Vision.. ticking away" << endl;
   Timer t;

   // Read current Pose from blakboard
   V.convRR.pose = readFrom(motion, pose);

   // Read whichCamera from blackboard
   //int behaviourReadBuf = readFrom(behaviour, readBuf);
   //V.whichCamera = readFrom(behaviour, request[behaviourReadBuf].whichCamera);
   V.getFrame();
   //writeTo(vision, currentFrame, V.currentFrame);
   writeTo(vision, topFrame, V.topFrame);
   writeTo(vision, botFrame, V.botFrame);
   V.convRR.findEndScanValues();
   llog(VERBOSE) << "getFrame() took " << t.elapsed_us() << " us" << endl;
   t.restart();
   acquireLock(serialization);
   llog(VERBOSE) << "acquireLock(serialization) took " << t.elapsed_us()
      << " us" << endl;
   t.restart();

   SensorValues values = readFrom(motion, sensors);
   SensorValues valuesLagged = readFrom(kinematics, sensorsLagged);
   V.convRR.updateAngles(valuesLagged);

   // robot detection needs sonar
   V.robotDetection._sonar = readFrom(kinematics, sonarFiltered);
   V.oldRobotDetection.sonar = readFrom(kinematics, sonarFiltered);

   // goal matcher needs to know which end is which and other details to store landmarks
   RoboCupGameControlData gameData = readFrom(gameController, data);
   V.goalMatcher.state = gameData.state;
   V.goalMatcher.secondaryState = gameData.secondaryState;
   V.goalMatcher.robotPos = readFrom(localisation, robotPos);
   V.goalMatcher.headYaw = valuesLagged.joints.angles[Joints::HeadYaw];

   // field line detection uses current robotPos to search for features
   V.fieldLineDetection.robotPos = readFrom(localisation, robotPos);

   // ball detection looks for current ball
   V.ballDetection.localisationBall = readFrom(localisation, ballPosRR);
   V.ballDetection.latestAngleX = values.sensors[Sensors::InertialSensor_AngleX];
   
   pthread_yield();
   usleep(1); // force sleep incase yield sucks
   V.processFrame();
   llog(VERBOSE) << "processFrame() took " << t.elapsed_us() << " us" << endl;

   t.restart();

   // Calculate covariance matrices for observations
   //VarianceCalculator::setVariance(V.feet);
   VarianceCalculator::setVariance(V.balls);
   VarianceCalculator::setVariance(V.posts);
   //VarianceCalculator::setVariance(V.robots);
   //VarianceCalculator::setVariance(V.fieldEdges);
   VarianceCalculator::setVariance(V.fieldFeatures);
   writeTo (vision, timestamp,      V.frame->timestamp);
   writeTo (vision, feetBoxes,           V.feetBoxes);
   writeTo (vision, balls,          V.balls        );
   writeTo (vision, ballHint,       V.ballHint     );
   writeTo (vision, posts,          V.posts        );
   writeTo (vision, robots,         V.robots       );
   writeTo (vision, landmarks,      V.landmarks    );
   writeTo (vision, fieldEdges,     V.fieldEdges   );
   writeTo (vision, fieldFeatures,  V.fieldFeatures);
   writeTo (vision, missedFrames,   V.missedFrames ); 
   writeTo (vision, dxdy,           V.dxdy         );
   writeTo (vision, goalArea,       V.goalArea     );
   writeTo (vision, awayGoalProb,   V.awayGoalProb );
   writeTo (vision, awayMapSize,    V.awayMapSize  );
   writeTo (vision, homeMapSize,    V.homeMapSize  );
   releaseLock(serialization);
   llog(VERBOSE) << "Blackboard write took " << t.elapsed_us() << " us" << endl;
   llog(VERBOSE) << "Vision took " << timer.elapsed_us() << "us" << endl;
   if (timer.elapsed_us() > 33666) {
      llog(VERBOSE) << "WARNING: Vision took too long!" << endl;
   }
   
   networkCount++;
   if (networkCount%8 == 0) {
      networkTick(blackboard);
   }
}

