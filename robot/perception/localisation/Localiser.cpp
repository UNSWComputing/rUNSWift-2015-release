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

#include <cassert>

#include "Localiser.hpp"
#include "LocalisationUtils.hpp"
#include "utils/Logger.hpp"
#include "utils/incapacitated.hpp"
#include "utils/speech.hpp"

static const int MAX_GAUSSIANS = 8;

Localiser::Localiser(int playerNumber) {
   this->myPlayerNumber = playerNumber;
   ballLostCount = 0;
   worldDistribution = new MultiGaussianDistribution(MAX_GAUSSIANS, playerNumber);
   sharedDistribution = new SharedDistribution();
}

Localiser::~Localiser() {
   delete worldDistribution;
   delete sharedDistribution;
}

void Localiser::setReset(void) {
   worldDistribution->resetDistributionToPenalisedPose();
   resetSharedUpdateData();
}

void Localiser::resetToPenaltyShootout(void) {
   worldDistribution->resetToPenaltyShootout();
}

void Localiser::setLineUpMode(bool enabled) {
   worldDistribution->setLineUpMode(enabled);
}

void Localiser::setReadyMode(bool enabled) {
   LocalisationConstantsProvider& constantsProvider(LocalisationConstantsProvider::instance());
   constantsProvider.setReadyMode(enabled);
   
   worldDistribution->setReadyMode(enabled);
   sharedDistribution->setReadyMode(enabled);
}

void Localiser::startOfPlayReset(void) {
   worldDistribution->startOfPlayReset();
   resetSharedUpdateData();
}

AbsCoord Localiser::getRobotPose(void) {
   assert(worldDistribution != NULL);
   return worldDistribution->getTopGaussian()->getRobotPose();
}

AbsCoord Localiser::getBallPosition(void) {
   assert(worldDistribution != NULL);
   return worldDistribution->getTopGaussian()->getBallPosition();
}

AbsCoord Localiser::getBallVelocity(void) {
   assert(worldDistribution != NULL);
   return worldDistribution->getTopGaussian()->getBallVelocity();
}

unsigned Localiser::getBallLostCount(void) {
   return ballLostCount;
}

double Localiser::getRobotPosUncertainty(void) const {
   return worldDistribution->getTopGaussian()->getRobotPosUncertainty();
}

double Localiser::getRobotHeadingUncertainty(void) const {
   return worldDistribution->getTopGaussian()->getRobotHeadingUncertainty();
}

double Localiser::getBallPosUncertainty(void) const {
   return worldDistribution->getTopGaussian()->getBallPosUncertainty();
}

double Localiser::getBallVelocityUncertainty(void) const {
   return worldDistribution->getTopGaussian()->getBallVelocityUncertainty();
}

SharedLocalisationUpdateBundle Localiser::getSharedUpdateData(void) const {
   return sharedDistribution->getBroadcastData();
}

void Localiser::resetSharedUpdateData(void) {
   const SimpleGaussian *topGaussian = worldDistribution->getTopGaussian();
   sharedDistribution->reset(topGaussian);
}

void Localiser::localise(const LocaliserBundle &lb, const bool canDoObservations) {
   if (lb.visionUpdateBundle.visibleBalls.size() > 1) {
      std::cout << "MULTIBALL!!" << std::endl;
   }
   
   bool canSeeBall = lb.visionUpdateBundle.visibleBalls.size() > 0;
   
   worldDistribution->processUpdate(lb.odometry, lb.dTimeSeconds, canSeeBall);
   if (canDoObservations) {
      worldDistribution->visionUpdate(lb.visionUpdateBundle);
   }
   
   sharedDistribution->processUpdate(lb.odometry, lb.dTimeSeconds, canSeeBall);
   if (canDoObservations && worldDistribution->getTopGaussian()->getHaveLastVisionUpdate()) {
      sharedDistribution->visionUpdate(
            worldDistribution->getTopGaussian()->getLastAppliedICPUpdate(),
            worldDistribution->getTopGaussian()->getLastAppliedVisionUpdate());
   }

   if (lb.visionUpdateBundle.visibleBalls.size() > 0 && canDoObservations) {
      ballLostCount = 0;
   } else {
      ballLostCount++;
   }
}

void Localiser::applyRemoteUpdate(const BroadcastData &broadcastData, int playerNumber) {
   if (playerNumber == myPlayerNumber || !broadcastData.sharedLocalisationBundle.isUpdateValid) {
      return;
   }

   MY_ASSERT_2(playerNumber >= 1 && playerNumber <= 5);
   
   // Convert the player number to a teammate index. There are two cases to handle, if the player number
   // greater than our own, or if it is less than our own.
   int teammateIndex = 0;
   if (playerNumber < myPlayerNumber) {
      teammateIndex = playerNumber - 1;
   } else {
      teammateIndex = playerNumber - 2;
   }
   
   MY_ASSERT(teammateIndex >= 0 && teammateIndex <= 3, "Invalid teammateIndex");
   worldDistribution->applyRemoteUpdate(broadcastData, teammateIndex, playerNumber == 1);
}

double Localiser::getLastObservationLikelyhood(void) const {
   return worldDistribution->getLastObservationLikelyhood();
}
