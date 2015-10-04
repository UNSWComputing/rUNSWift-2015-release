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

/**
 * GameController.hpp
 * Modified: 2009-11-13
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#include "gamecontroller/GameController.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netdb.h>
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

#define POLL_TIMEOUT 200

using namespace std;

GameController::GameController(Blackboard *bb)
   : Adapter(bb), team_red(false), connected(false) {
   lastState = STATE_INVALID;
   myLastPenalty = PENALTY_NONE;
   if (readFrom(gameController, connect)) {
      initialiseConnection();
   }
   actOnWhistle = (bb->config)["debug.act_on_whistle"].as<bool>();
}

GameController::~GameController() {
   close(sock);
}

void GameController::tick() {

   // Notes:
   // gameState represents what we think the game state should be
   // It is the same as data.state unless we hear a whistle
   // If we hear a whistle, set gameState to playing to tell our teammates
   // If the team agrees, then we override the official data.state with playing

   // -- Standard Game Controller Packet Update --
   uint8_t previousGameState = readFrom(gameController, gameState);
   data = readFrom(gameController, data);
   teamNumber = readFrom(gameController, our_team).teamNumber;
   playerNumber = readFrom(gameController, player_number);

   if (!connected && readFrom(gameController, connect)) initialiseConnection();
   if (connected) wirelessUpdate();
   buttons = readFrom(motion, buttons);
   buttonUpdate();
   writeTo(motion, buttons, buttons);
   // make our_team point to the my actual team, based on teamNumber
   TeamInfo *our_team = NULL;
   if (data.teams[TEAM_BLUE].teamNumber == teamNumber) {
      our_team = &(data.teams[TEAM_BLUE]);
      team_red = false;
   } else if (data.teams[TEAM_RED].teamNumber == teamNumber) {
      our_team = &(data.teams[TEAM_RED]);
      team_red = true;
   }

   // -- Extras for Whistle Detection --
   uint8_t gameState = data.state;

   // If we previously heard a whistle and changed our gameState to playing
   // Then don't let it get overriden by the game controller
   if (gameState == STATE_SET && previousGameState == STATE_PLAYING) {
      gameState = previousGameState;
   }

   // Heard whistles - if we are in SET and whistle heard in last 3 seconds
   if (gameState == STATE_SET && whistleHeard(3)) {
      if (actOnWhistle == true) {
         gameState = STATE_PLAYING;
         cout << "WHISTLE HEARD, TELLING TEAM MATES" << endl;
         SAY("Whistle heard");
      }
      else {
         SAY("Heard whistle not moving");
      }
   }

   // Check the team opinion on whether a whistle has been heard
   if (data.state == STATE_SET) {
      float numTeammatesPlaying = 0;
      float numActiveTeammates = 0;
      for (int i = 0; i < ROBOTS_PER_TEAM; ++i) {
         if (readFrom(receiver, data)[i].gameState == STATE_PLAYING) {
            ++numTeammatesPlaying;
         }
         if (!readFrom(receiver, incapacitated)[i]) {
            ++numActiveTeammates;
         }
      }

      // Don't forget to count ourselves
      if (gameState == STATE_PLAYING) ++numTeammatesPlaying;
      ++numActiveTeammates;

      // If enough of the team thinks we should play, lets play
      float ratio = numTeammatesPlaying / numActiveTeammates;
      if (ratio >= 0.49 && actOnWhistle) {
         data.state = STATE_PLAYING;
         cout << "WHISTLE HEARD BY TEAMMATES ENTERING STATE PLAYING YAY" << endl;
         SAY("Whistle heard by teammates");
      }
   }

   writeTo(gameController, data, data);
   writeTo(gameController, our_team, *our_team);
   writeTo(gameController, team_red, team_red);
   writeTo(gameController, gameState, data.state);


   // In the case where we've heard a whistle, but haven't decided to play yet
   // We want to keep the official data as set, but tell the team we think its play time
   // So override our gameState variable, thus making gameState != data.state
   if (gameState == STATE_PLAYING && data.state == STATE_SET) {
      writeTo(gameController, gameState, gameState);
   }
}

void GameController::initialiseConnection() {
   llog(INFO) << "GameController: Connecting on port "
              << GAMECONTROLLER_PORT << endl;
   stringstream s;
   s << GAMECONTROLLER_PORT;

   struct addrinfo myInfo, *results;
   memset(&myInfo, 0, sizeof myInfo);
   myInfo.ai_family = AF_UNSPEC;
   myInfo.ai_socktype = SOCK_DGRAM;
   myInfo.ai_flags = AI_PASSIVE;  // use my IP

   if (getaddrinfo(NULL, s.str().c_str(), &myInfo, &results) == -1) {
      llog(ERROR) << "GameController: Invalid Address Information" << endl;
      return;
   }

   // loop through all the results and bind to the first we can
   struct addrinfo *p;
   for (p = results; p != NULL; p = p->ai_next) {
      if ((sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
         llog(INFO) << "GameController: Cannot use Socket, trying next"
                    << endl;
         continue;
      }

      if (bind(sock, p->ai_addr, p->ai_addrlen) == -1) {
         close(sock);
         llog(INFO) << "GameController: Cannot Bind, trying next" << endl;
         continue;
      }

      break;
   }
   if (p == NULL) {
      llog(ERROR) << "GameController: Failed to bind socket" << endl;
      return;
   }

   // We don't want memory leaks...
   freeaddrinfo(results);

   llog(INFO) << "GameController: Connected on port - " << s.str() << endl;
   connected = true;
   writeTo(gameController, connected, connected);
}

void GameController::buttonUpdate() {
   if (buttons.pop(1)) {
      llog(INFO) << "button pushed once, switching state" << endl;
      switch (data.state) {
      case STATE_INITIAL:
      case STATE_PLAYING:
         data.state = STATE_PENALISED;
         data.teams[team_red].players[playerNumber - 1].penalty =
            PENALTY_MANUAL;
         SAY("Penalised");
         break;
      default:
         data.state = STATE_PLAYING;
         data.teams[team_red].players[playerNumber - 1].penalty =
            PENALTY_NONE;
         SAY("Playing");
      }
   }
}

void GameController::wirelessUpdate() {
   // Setup receiving client
   int bytesRecieved;
   struct sockaddr_storage clientAddress;
   socklen_t addr_len = sizeof(clientAddress);

   // Setup buffer to write to
   int dataSize = sizeof(RoboCupGameControlData);
   unsigned char buffer[dataSize + 1];

   // Setup for polling
   struct pollfd ufds[1];
   ufds[0].fd = sock;
   ufds[0].events = POLLIN;            // For incoming packets

   // Congested WiFi: Try to grab several packets in one tick
   // to clear buffers and lower effective latency
   for (int i = 0; i < 5; i++) {
      int rv = poll(ufds, 1, POLL_TIMEOUT);  // Wait up to POLL_TIMEOUT ms

      // Check to see if we've received a packet
      if (rv > 0) {
         bytesRecieved = recvfrom(sock, buffer, dataSize, 0,
                                  (struct sockaddr *)&clientAddress, &addr_len);
         if (bytesRecieved > 0) {
            parseData((RoboCupGameControlData*)buffer);
         }
      }
   }
}

bool GameController::whistleHeard(int numSeconds) {
   const char *WHISTLE_FILE_FORMAT = "whistle_%Y_%m_%d_%H%M%S.wav";
   const char *NAO_WHISTLE_LOCATION = "/home/nao/whistle";
   DIR *dir;
   struct dirent *ent;
   struct tm fileDateTime;
   time_t now = time(0);   // get time now
   bool found = false;

   if ((dir = opendir (NAO_WHISTLE_LOCATION)) != NULL) {
      /* go through all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
         strptime(ent->d_name, WHISTLE_FILE_FORMAT, &fileDateTime);
         double seconds = difftime(now, mktime(&fileDateTime));
         //cout << "seconds" << seconds << endl;

         // If file created in last numSeconds
         if (abs(seconds) < numSeconds &&
             seconds < numSeconds) {
            found = true;
            //cout << "found" << endl;
         }
      }
      closedir (dir);
   } else {
      /* could not open directory */
      //cout << ("No directory") << endl;
   }
   return found;
}

void GameController::parseData(RoboCupGameControlData *update) {
   if (isValidData(update)) {
      /* Normalise the team structure order so that BLUE is always first */
      if (update->teams[TEAM_BLUE].teamColour != TEAM_BLUE) {
         rawSwapTeams(update);
      }

      // Heard whistles - if GameController still saying we are in SET
      // but we are already PLAYING, keep PLAYING
      if (int(update->state) == STATE_SET && data.state == STATE_PLAYING) {
         update->state = STATE_PLAYING;
      }

      // Update the data
      if (!gameDataEqual(update, &data)) {
         memcpy(&data, update, sizeof(RoboCupGameControlData));
      }

      llog(VERBOSE) << "GameController: Valid data" << endl;
      if (data.state != lastState) {
         // Shamelessly copied from: http://stackoverflow.com/a/1995057/1101109
         char comboState[100];
         strcpy(comboState, gameControllerSecStateNames[update->secondaryState]);
         strcat(comboState, gameControllerStateNames[data.state]);
         SAY(comboState);

         /*
         if (data.state == STATE_READY
             && data.firstHalf == 1
             && lastState == STATE_INITIAL
             && playerNumber == 2) {
            system("/usr/bin/aplay -q /home/nao/data/startup1.wav &");
         }
         */
         lastState = data.state;
      }

      uint8 myPenalty = data.teams[team_red].players[playerNumber - 1].penalty;

      if (myPenalty != PENALTY_NONE) {
         data.state = STATE_PENALISED;
      }

      if (myPenalty != myLastPenalty) {
         if (myPenalty == PENALTY_NONE) {
            SAY("Unpenalised");
         } else {
            SAY((string("Penalised for ") +
                 gameControllerPenaltyNames[myPenalty]).c_str());
         }
         myLastPenalty = myPenalty;
      }
   } else {
      llog(ERROR) << "GameController: Invalid data" << endl;
   }
}

bool GameController::isValidData(RoboCupGameControlData *gameData) {
   // check the right structure header has come in
   if (!(checkHeader(gameData->header))) {
      llog(ERROR) << "GameController: DATA HEADER MISMATCH! "
                  << "Expected: " << GAMECONTROLLER_STRUCT_HEADER
                  << " received: " << gameData->header << endl;
      return false;
   }

   // check for partial packets
   if (sizeof(*gameData) != sizeof(RoboCupGameControlData)) {
      llog(ERROR) << "GameController: RECEIVED PARTIAL PACKET! "
                  << "Expected: " << sizeof(RoboCupGameControlData)
                  << " received: " << sizeof(*gameData) << endl;
      return false;
   }

   // check the right version of the structure is being used
   if (gameData->version != GAMECONTROLLER_STRUCT_VERSION) {
      llog(ERROR) << "GameController: DATA VERSION MISMATCH! "
                  << "Expected: " << GAMECONTROLLER_STRUCT_VERSION
                  << " received: " << gameData->version << endl;
      return false;
   }

   // check whether this packet belongs to this game at all
   if (!isThisGame(gameData)) {
      llog(ERROR) << "GameController: DATA NOT FOR THIS GAME!" << endl;
      return false;
   }

   // Data is valid ^_^
   return true;
}

bool GameController::checkHeader(char* header) {
   for (int i = 0; i < 4; i++) {
      if (header[i] != GAMECONTROLLER_STRUCT_HEADER[i]) return false;
   }
   return true;
}

bool GameController::isThisGame(RoboCupGameControlData* gameData) {
   if (gameData->teams[TEAM_BLUE].teamNumber != teamNumber &&
       gameData->teams[TEAM_RED].teamNumber  != teamNumber) {
      return false;
   }
   return true;
}

bool GameController::gameDataEqual(void* gameData, void* previous) {
   if (!memcmp(previous, gameData, sizeof(RoboCupGameControlData))) {
      return true;
   }
   return false;
}

void GameController::rawSwapTeams(RoboCupGameControlData* gameData) {
   size_t teamSize = sizeof(TeamInfo);
   TeamInfo* blueTeam = &(gameData->teams[TEAM_BLUE]);
   TeamInfo* redTeam  = &(gameData->teams[TEAM_RED]);

   TeamInfo tempTeam;
   memcpy(&tempTeam, blueTeam, teamSize);

   /* swap the teams */
   memcpy(blueTeam, redTeam, teamSize);
   memcpy(redTeam, &tempTeam, teamSize);
}
