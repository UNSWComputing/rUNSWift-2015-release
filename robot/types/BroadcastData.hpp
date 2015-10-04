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

#include "types/AbsCoord.hpp"
#include "types/RRCoord.hpp"
#include "types/ActionCommand.hpp"
#include "types/BehaviourSharedData.hpp"
#include "perception/localisation/SharedLocalisationUpdateBundle.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

class BroadcastData {
   public:
      BroadcastData()
         : playerNum(0), team(0), robotPos(0.0, 0.0, 0.0), 
           ballPosAbs(), ballPosRR(), lostCount(0),
           sharedLocalisationBundle(), behaviourSharedData(),
           acB(ActionCommand::Body::DEAD), uptime(0.0), gameState(STATE_INITIAL) {}

      BroadcastData(const BroadcastData& bd)
         : playerNum(bd.playerNum),
           team(bd.team),
           robotPos(bd.robotPos),
           ballPosAbs(bd.ballPosAbs),
           ballPosRR(bd.ballPosRR),
           lostCount(bd.lostCount), // TODO: this really should be "ballLostCount"
           sharedLocalisationBundle(bd.sharedLocalisationBundle),
           behaviourSharedData(bd.behaviourSharedData),
           acB(bd.acB),
           uptime(bd.uptime),
           gameState(bd.gameState) {}

      BroadcastData(const int &playerNum, const int &team,
                    const AbsCoord &robotPos,
                    const AbsCoord &ballPosAbs,
                    const RRCoord &ballPosRR,
                    const uint32_t &lostCount,
                    const SharedLocalisationUpdateBundle &sharedLocalisationBundle,
                    const BehaviourSharedData &behaviourSharedData,
                    const ActionCommand::Body::ActionType &acB,
                    const float &uptime,
                    const uint8_t &gameState)
         : playerNum(playerNum),
           team(team),
           robotPos(robotPos),
           ballPosAbs(ballPosAbs),
           ballPosRR(ballPosRR),
           lostCount(lostCount),
           sharedLocalisationBundle(sharedLocalisationBundle),
           behaviourSharedData(behaviourSharedData),
           acB(acB),
           uptime(uptime),
           gameState(gameState) {}
      
      int playerNum;
      int team;
      
      AbsCoord robotPos;
      AbsCoord ballPosAbs;
      RRCoord ballPosRR;
      uint32_t lostCount;
      
      SharedLocalisationUpdateBundle sharedLocalisationBundle;
      
      // Data set by the Python behaviours that is shared with other robots.
      BehaviourSharedData behaviourSharedData;

      ActionCommand::Body::ActionType acB;
      float uptime;
      uint8_t gameState;

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & playerNum;
         ar & team;
         ar & robotPos;
         ar & ballPosAbs;
         ar & ballPosRR;
         ar & lostCount;
         ar & sharedLocalisationBundle;
         ar & behaviourSharedData;
         ar & acB;
         ar & uptime;
         ar & gameState;
      }
};
