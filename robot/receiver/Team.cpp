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

#include <ctime>
#include <iostream>
#include "Team.hpp"
#include "types/SPLStandardMessage.hpp"
#include "utils/incapacitated.hpp"
#include "thread/Thread.hpp"
#include "perception/behaviour/ReadySkillPositionAllocation.hpp"

using namespace std;

TeamReceiver::TeamReceiver(Blackboard *bb, void(TeamReceiver::*handler)
                           (const boost::system::error_code & error, std::size_t))
   : Adapter(bb), NaoReceiver(this,
                              handler,
                              (bb->config)["transmitter.base_port"].as<int>()
                              + (bb->config)["player.team"].as<int>()) {}

void TeamReceiver::naoHandler(const boost::system::error_code &error,
                              std::size_t size) {
   if (Thread::name == NULL) {
      Thread::name = "TeamReceiverBoostThread";
   }
   
   const SPLStandardMessage &m = (const SPLStandardMessage &)recvBuffer;
   const BroadcastData &bd = (const BroadcastData &)m.data;
   if (size == sizeof(SPLStandardMessage)) {
      if (m.playerNum >= 1 && m.playerNum <= ROBOTS_PER_TEAM &&
          m.teamNum == readFrom(gameController, our_team).teamNumber &&
          m.numOfDataBytes == sizeof(BroadcastData) &&
          bd.team == readFrom(receiver, team)) {
         
         std::vector<bool> pendingIncomingUpdates = readFrom(localisation, havePendingIncomingSharedBundle);
         pendingIncomingUpdates[bd.playerNum - 1] = true;
         writeTo(localisation, havePendingIncomingSharedBundle, pendingIncomingUpdates);
         
         writeTo(receiver, message[bd.playerNum - 1], m);
         writeTo(receiver, data[bd.playerNum - 1], bd);
         writeTo(receiver, lastReceived[bd.playerNum - 1], time(NULL));

         // calculate incapacitated
         bool incapacitated = false;
         if (readFrom(gameController, our_team).players[bd.playerNum - 1].penalty
             != PENALTY_NONE) {
            incapacitated = true;
         }

         const ActionCommand::Body::ActionType &acB =
            readFrom(receiver, data)[bd.playerNum - 1].acB;
         incapacitated |= isIncapacitated(acB);

         writeTo(receiver, incapacitated[bd.playerNum - 1], incapacitated);
         
         // If the received ready skill position allocation overrides my current one, then overwrite it. 
         ReadySkillPositionAllocation currentPositionAllocation = 
               readFrom(behaviour, behaviourSharedData).readyPositionAllocation;
         if (bd.behaviourSharedData.readyPositionAllocation.canOverride(currentPositionAllocation)) {
            writeTo(behaviour, behaviourSharedData.readyPositionAllocation, bd.behaviourSharedData.readyPositionAllocation);
         }
      }
   } else {
      llog(WARNING) << "Received packet of " << size << " bytes, but expected "
                                                        "packet of " << sizeof(BroadcastData) << " bytes."  << endl;
   }
   startReceive(this, &TeamReceiver::naoHandler);
}

void TeamReceiver::stdoutHandler(const boost::system::error_code &error,
                                 std::size_t size) {
   const SPLStandardMessage &m = (const SPLStandardMessage &)recvBuffer;
   const BroadcastData &bd = (const BroadcastData &)m.data;
   cout << "Received data from player " << bd.playerNum << endl;
   startReceive(this, &TeamReceiver::stdoutHandler);
}

void TeamReceiver::tick() {
   for (int robot = 0; robot < ROBOTS_PER_TEAM; ++robot) {
      if (time(NULL) - readFrom(receiver, lastReceived)[robot] > SECS_TILL_INCAPACITATED) {
         bool incapacitated = true;
         writeTo(receiver, incapacitated[robot], incapacitated);
      }
   }
}

