#pragma once

#include "types/ActionCommand.hpp"
#include "types/AbsCoord.hpp"
#include "perception/vision/WhichCamera.hpp"

class BehaviourRequest {
   public:
      WhichCamera whichCamera;
      ActionCommand::All actions;
      bool goalieAttacking;
      
      // Whether the robot is in the process of lining up with the ball to kick it.
      bool doingBallLineUp;
      
      // If the behaviour is currently executing the ready mode behaviour.
      bool isInReadyMode;
      
      // The estimated time (in seconds) that this robot needs to get to the ball. This is 
      // the primary method that we use to decide which robot should be the Striker.
      float timeToReachBall;

      // Estimated time to different positions, used for role assignment
      float timeToReachDefender;
      float timeToReachMidfielder;
      float timeToReachUpfielder;
      
      // The encoded enum value for the robots current role.
      int currentRole;

      // Where we are currently walking and shooting to
      // Absolute field coordinates, not relative
      int walkingToX;
      int walkingToY;
      int shootingToX;
      int shootingToY;

      // Where we are currently aiming to kick to
      //int kickingTo;

      // TODO: this is horrible, I just cant figure out how to pass back a list/vector/array from
      // python back to C++.
      int readyPositionAllocation0;
      int readyPositionAllocation1;
      int readyPositionAllocation2;
      int readyPositionAllocation3;
      int readyPositionAllocation4;

      BehaviourRequest() {
         whichCamera = TOP_CAMERA;
         goalieAttacking = false;
         doingBallLineUp = false;
         isInReadyMode = false;
         timeToReachBall = 10000.0f;
         currentRole = 0;
         walkingToX = 0;
         walkingToY = 0;
         shootingToX = 0;
         shootingToY = 0;
         
         readyPositionAllocation0 = -1;
         readyPositionAllocation1 = -1;
         readyPositionAllocation2 = -1;
         readyPositionAllocation3 = -1;
         readyPositionAllocation4 = -1;
      };

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & whichCamera;
         ar & actions;
         ar & goalieAttacking;
         ar & doingBallLineUp;
         ar & isInReadyMode;
         ar & timeToReachBall;
         ar & timeToReachDefender;
         ar & timeToReachMidfielder;
         ar & timeToReachUpfielder;
         ar & currentRole;
         
         ar & readyPositionAllocation0;
         ar & readyPositionAllocation1;
         ar & readyPositionAllocation2;
         ar & readyPositionAllocation3;
         ar & readyPositionAllocation4;
      }
};
