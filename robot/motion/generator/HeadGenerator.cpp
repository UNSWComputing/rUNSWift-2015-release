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

#include "motion/generator/HeadGenerator.hpp"
#include "utils/basic_maths.hpp"
#include "utils/clip.hpp"
#include "utils/Logger.hpp"

HeadGenerator::HeadGenerator()
   : yaw(0.0f),
     pitch(0.0f) {
   llog(INFO) << "HeadGenerator constructed" << std::endl;
}

HeadGenerator::~HeadGenerator() {
   llog(INFO) << "HeadGenerator destroyed" << std::endl;
}

/* Borrowed from Aaron's 2009 code. Uses a PID controller. */
JointValues HeadGenerator::makeJoints(ActionCommand::All* request,
                                      Odometry* odometry,
                                      const SensorValues &sensors,
                                      BodyModel &bodyModel,
                                      float ballX,
                                      float ballY) {
   // Simple state of the head angles
   float desY = 0.0, desP = 0.0;    // Current requested angles

   // Boundary checking (NAN values)
   if (isnan(request->head.yaw) || isnan(request->head.pitch)) {
      llog(INFO) << "HeadGenerator: NAN angles - Doing Nothing" << std::endl;
      return sensors.joints;
   }
   // TODO(stuartr): init these variables above.
   /* This seemed to not work as well as just remember where we should be */
   // pitch = sensors.joints.angles[Joints::HeadPitch];
   // yaw = sensors.joints.angles[Joints::HeadYaw];

   if (request->head.isRelative) {
      desY = yaw + request->head.yaw;
      //desP = pitch + request->head.pitch;
   } else {
      desY = request->head.yaw;
   }
   desP = request->head.pitch;

   float diffY = desY - yaw;
   float diffP = desP - pitch;
   diffY = CLIP<float>(diffY, Joints::Radians::HeadYawSpeed *
                       request->head.yawSpeed);
   diffP = CLIP<float>(diffP, Joints::Radians::HeadPitchSpeed *
                       request->head.pitchSpeed);
   yaw += diffY;
   pitch += diffP;

   // Return JointValues with head angles changed, all others same
   JointValues j = sensors.joints;
   j.angles[Joints::HeadYaw] = yaw;
   j.angles[Joints::HeadPitch] = pitch;
   j.stiffnesses[Joints::HeadYaw] = 0.8f;
   j.stiffnesses[Joints::HeadPitch] = 0.8f;
   return j;
}

bool HeadGenerator::isActive() {
   return true;
}

void HeadGenerator::reset() {
   yaw = 0.0f;
   pitch = 0.0f;
}
