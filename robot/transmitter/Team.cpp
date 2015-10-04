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

#include "Team.hpp"
#include "blackboard/Blackboard.hpp"
#include "types/SPLStandardMessage.hpp"
#include "utils/incapacitated.hpp"
//#include "types/

#include <iostream>

using namespace boost::asio;
using namespace std;

TeamTransmitter::TeamTransmitter(Blackboard *bb) :
   Adapter(bb),
   NaoTransmitter((bb->config)["transmitter.base_port"].as<int>()
                  + (bb->config)["player.team"].as<int>(),
                  (bb->config)["transmitter.address"].as<string>()),
   service(),
   socket(service, ip::udp::v4()),
   broadcast_endpoint(ip::address::from_string("255.255.255.255"),
      GAMECONTROLLER_PORT),
      delay(0)  {
   
   socket_base::broadcast option(true);
   socket.set_option(option);
   boost::system::error_code ec;
   socket.connect(broadcast_endpoint, ec);   
      
}

void TeamTransmitter::tick() {
   BroadcastData bd((blackboard->config)["player.number"].as<int>(),
                    (blackboard->config)["player.team"].as<int>(),
                    readFrom(localisation, robotPos),
                    readFrom(localisation, ballPos),
                    readFrom(localisation, ballPosRR),
                    readFrom(localisation, ballLostCount),
                    readFrom(localisation, sharedLocalisationBundle),
                    readFrom(behaviour, behaviourSharedData),
                    readFrom(motion, active).body.actionType,
                    readFrom(motion, uptime),
                    readFrom(gameController, gameState));

   // calculate incapacitated
   int playerNum = (blackboard->config)["player.number"].as<int>();
   bool incapacitated = false;
   if (readFrom(gameController, our_team).players[playerNum - 1].penalty
       != PENALTY_NONE) {
      incapacitated = true;
   }

   const ActionCommand::Body::ActionType &acB =
            readFrom(motion, active).body.actionType;
   incapacitated |= isIncapacitated(acB);

   const AbsCoord &robotPos = readFrom(localisation, robotPos);
   const AbsCoord &walkingTo = readFrom(behaviour, walkingTo);
   const AbsCoord &shootingTo = readFrom(behaviour, shootingTo);

   // Get intention from behaviour
   int behaviourReadBuf = readFrom(behaviour, readBuf);
   int role = readFrom(behaviour, request[behaviourReadBuf]).currentRole;
   int intention = 0;
   if      (role == 1) intention = 1; // goalie
   else if (role == 2) intention = 3; // striker
   else if (role == 3) intention = 2; // defender

   SPLStandardMessage m (playerNum,
                         readFrom(gameController, our_team).teamNumber, // team
                         incapacitated, // fallen
                         robotPos,
                         walkingTo,
                         shootingTo,
                         readFrom(localisation, ballLostCount),
                         readFrom(localisation, ballPos),
                         readFrom(localisation, ballVelRRC),
                         intention,
                         bd);

   writeTo(localisation, havePendingOutgoingSharedBundle, false);
   NaoTransmitter::tick(boost::asio::buffer(&m, sizeof(SPLStandardMessage)));

   // hax to send the gc packet once every two team ticks
   ++delay;
   if (delay > 5) {
      sendToGameController();
      delay = 0;
   }
}

TeamTransmitter::~TeamTransmitter() {
   socket.close();
}

void TeamTransmitter::sendToGameController() {
   boost::system::error_code ec = boost::system::error_code();
   RoboCupGameControlReturnData d = RoboCupGameControlReturnData();
   d.team = (blackboard->config)["player.team"].as<int>();
   d.player = (blackboard->config)["player.number"].as<int>();
   d.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

   socket.send(boost::asio::buffer(&d, sizeof(RoboCupGameControlReturnData)), 0, ec);
}
