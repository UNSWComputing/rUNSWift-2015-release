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

#include "motion/generator/StandGenerator.hpp"
#include "motion/generator/ActionGenerator.hpp"
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"

using boost::program_options::variables_map;

StandGenerator::StandGenerator()
   : phi(DEG2RAD(0.0f)) {    //was 15, using 20 knee bend to prevent jerky transition between walk and stand
   llog(INFO) << "StandGenerator constructed" << std::endl;
   posGen = (Generator*)(new ActionGenerator("stand"));
   if (!posGen)
      llog(FATAL) << "stand generator is NULL!" << std::endl;
}

StandGenerator::~StandGenerator() {
   llog(INFO) << "StandGenerator destroyed" << std::endl;
   delete posGen;
}

JointValues StandGenerator::makeJoints(ActionCommand::All* request,
                                       Odometry* odometry,
                                       const SensorValues &sensors,
                                       BodyModel &bodyModel,
                                       float ballX,
                                       float ballY) {
//   JointValues joints = sensors.joints;
//   uint8_t i = Joints::HeadYaw;
//   joints.stiffnesses[i++] = 0.0f;  // HeadYaw
//   joints.stiffnesses[i++] = 0.0f;  // HeadPitch
//   joints.stiffnesses[i++] = 0.0f;  // LShoulderPitch
//   joints.stiffnesses[i++] = 0.0f;  // LShoulderRoll
//   joints.stiffnesses[i++] = 0.0f;  // LElbowYaw
//   joints.stiffnesses[i++] = 0.0f;  // LElbowRoll
//   joints.angles[i] = 0.0f,  joints.stiffnesses[i++] = 0.66f;  // LHipYawPitch
//   joints.angles[i] = 0.0f,  joints.stiffnesses[i++] = 0.66f;  // LHipRoll
//   interpolate(joints.angles[i], -phi),  joints.stiffnesses[i++] = 0.66f;  // LHipPitch
//   interpolate(joints.angles[i], 2 * phi), joints.stiffnesses[i++] = 0.66f;  // LKneePitch
//   interpolate(joints.angles[i], -phi),  joints.stiffnesses[i++] = 0.66f;  // LAnklePitch
//   joints.angles[i] = 0.0f,  joints.stiffnesses[i++] = 0.66f;  // LAnkleRoll
//   joints.angles[i] = 0.0f,  joints.stiffnesses[i++] = 0.66f;  // RHipRoll
//   interpolate(joints.angles[i], -phi),  joints.stiffnesses[i++] = 0.66f;  // RHipPitch
//   interpolate(joints.angles[i], 2 * phi), joints.stiffnesses[i++] = 0.66f;  // RKneePitch
//   interpolate(joints.angles[i], -phi),  joints.stiffnesses[i++] = 0.66f;  // RAnklePitch
//   joints.angles[i] = 0.0f,  joints.stiffnesses[i++] = 0.66f;  // RAnkleRoll
//   joints.stiffnesses[i++] = 0.0f;  // RShoulderPitch
//   joints.stiffnesses[i++] = 0.0f;  // RShoulderRoll
//   joints.stiffnesses[i++] = 0.0f;  // RElbowYaw
//   joints.stiffnesses[i++] = 0.0f;  // RElbowRoll
   return posGen->makeJoints(request, odometry, sensors, bodyModel, ballX, ballY);
}

bool StandGenerator::isActive() {
   return posGen->isActive();
}

void StandGenerator::reset() {
   posGen->reset();
}

void StandGenerator::stop() {
   posGen->stop();
}

void StandGenerator::readOptions(const boost::program_options::variables_map &config) {
   posGen->readOptions(config);
//   phi = DEG2RAD(config["walk.b"].as<float>());
//   llog(INFO) << "Successfully changed stand generator options" << std::endl;
//   llog(INFO) << "Stand generator options ignored" << std::endl;
}
