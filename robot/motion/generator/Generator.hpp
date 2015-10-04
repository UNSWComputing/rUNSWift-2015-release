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

#include <boost/program_options.hpp>
#include "types/ActionCommand.hpp"
#include "types/JointValues.hpp"
#include "types/Odometry.hpp"
#include "types/SensorValues.hpp"
#include "motion/generator/BodyModel.hpp"

/**
 * Generator - responsible for tranforming ActionCommands into joint angles.
 * May consider feedback from sensors.
 */
class Generator {
   public:
      virtual ~Generator() {}
      /**
       * makeJoints - generate joint angles & stiffnesses for next cycle
       * @param request New command for body movement. Generator should replace
       *           with the command is is currently doing if it is not the same
       * @param odometry Running tally of walk distance in f, l, t
       * @param sensors Last-read values of sensors
       * @param model of the robot
       * @param ballX relative x position of the ball
       * @param ballY relative y position of the ball
       * @return Values of joint actuators in next cycle
       */
      virtual JointValues makeJoints(ActionCommand::All* request,
                                     Odometry* odometry,
                                     const SensorValues &sensors,
                                     BodyModel &bodyModel,
                                     float ballX,
                                     float ballY) = 0;
      /**
       * isActive - informs the parent of whether the robot is in a neutral stance
       *            (i.e., whether a different generator can be switched to)
       */
      virtual bool isActive() = 0;
      /**
       * reset - return to neutral stance and become inactive immediately
       */
      virtual void reset() = 0;

      /**
       * stop - like reset, but time is given to transition to the neutral stance
       */
      virtual void stop() { /* defaults to reset() - can be overridden */
      }

      virtual void readOptions(const boost::program_options::variables_map &config) {}
};
