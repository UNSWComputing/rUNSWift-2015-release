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

#include <pthread.h>
#include "types/ActionCommand.hpp"
#include "types/JointValues.hpp"
#include "types/SensorValues.hpp"
#include "types/ButtonPresses.hpp"

#define AGENT_MEMORY "/libagent-memory"
#define AGENT_SEMAPHORE "/libagent-semaphore"
#define AL_ON -2.0f
#define AL_command stiffnesses[Joints::LShoulderPitch]
#define AL_x angles[Joints::LShoulderPitch]
#define AL_y angles[Joints::LShoulderRoll]
#define AL_theta angles[Joints::LElbowYaw]
#define AL_frequency angles[Joints::LElbowRoll]
#define AL_stop angles[Joints::LHipYawPitch]
#define AL_reset angles[Joints::LHipRoll]
#define AL_height angles[Joints::LHipPitch]
#define AL_bend angles[Joints::LKneePitch]
#define AL_isActive joints.temperatures[Joints::LShoulderPitch]

struct AgentData {
   volatile uint8_t sensors_read;
   volatile uint8_t sensors_latest;
   volatile uint8_t actuators_latest;
   volatile uint8_t actuators_read;

   SensorValues sensors[3];
   ButtonPresses buttons[3];
   JointValues joints[3];
   ActionCommand::LED leds[3];
   float sonar[3];
   char sayTexts[3][35];   //the longest say string for overheating is length 30

   volatile bool standing;
   pthread_mutex_t lock;

   void init() {
      sensors_read = 0;
      sensors_latest = 0;
      actuators_read = 0;
      actuators_latest = 0;
      for (int i = 0; i < 3; i++) {
         sayTexts[i][0] = 0;
      }
   }
};
