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

#include "perception/behaviour/SafetySkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/Logger.hpp"

SafetySkill::SafetySkill() {
   filtered_fsr_sum = 5.0;  // assume we start standing
   blink = 0;
   llog(INFO) << "SafetySkill constructed" << std::endl;
}

SafetySkill::~SafetySkill() {
   llog(INFO) << "SafetySkill destroyed" << std::endl;
}

BehaviourRequest SafetySkill::wrapRequest(const BehaviourRequest &request, const SensorValues &s) {
   BehaviourRequest r = request;
   if(r.actions.body.actionType == ActionCommand::Body::MOTION_CALIBRATE){
       return r;
   }

   float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft]
                   + s.sensors[Sensors::LFoot_FSR_FrontRight]
                   + s.sensors[Sensors::LFoot_FSR_RearLeft]
                   + s.sensors[Sensors::LFoot_FSR_RearRight]
                   + s.sensors[Sensors::RFoot_FSR_FrontLeft]
                   + s.sensors[Sensors::RFoot_FSR_FrontRight]
                   + s.sensors[Sensors::RFoot_FSR_RearLeft]
                   + s.sensors[Sensors::RFoot_FSR_RearRight];
   filtered_fsr_sum = filtered_fsr_sum + 0.2 * (fsr_sum - filtered_fsr_sum);
   blink = !blink;
   // if lying face down
   // std::cout << s.sensors[Sensors::InertialSensor_AccX] << " "
   //           << s.sensors[Sensors::InertialSensor_AccY] << std::endl;
//   if (s.sensors[Sensors::InertialSensor_AccX] < -FALLEN) {
//      r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
//      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
//      r.actions.body = ActionCommand::Body::GETUP_BACK;
//   } else if (s.sensors[Sensors::InertialSensor_AccX] > FALLEN) {
//      r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
//      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
//      r.actions.body = ActionCommand::Body::GETUP_FRONT;
//      // about to fall
//   } else if (ABS(s.sensors[Sensors::InertialSensor_AccX]) > FALLING ||
//              ABS(s.sensors[Sensors::InertialSensor_AccY]) > FALLING) {
//      r.actions.leds.leftEye = ActionCommand::rgb(blink, blink);
//      r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
//      r.actions.body = ActionCommand::Body::DEAD;
//      // if ref picks us up
//   }
   float ang[2] = {RAD2DEG(s.sensors[Sensors::InertialSensor_AngleX]),
                   RAD2DEG(s.sensors[Sensors::InertialSensor_AngleY])};
   if(ang[1] < -FALLEN_ANG || ang[0] < -FALLEN_ANG){
      r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
      r.actions.body = ActionCommand::Body::GETUP_BACK;
   } else if(ang[1] > FALLEN_ANG || ang[0] > FALLEN_ANG){
      r.actions.leds.leftEye = ActionCommand::rgb(blink, 0);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
      r.actions.body = ActionCommand::Body::GETUP_FRONT;
   } else if (ABS(ang[0]) > FALLING_ANG || ABS(ang[1]) > FALLING_ANG) {
      r.actions.leds.leftEye = ActionCommand::rgb(blink, blink);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
      r.actions.body = ActionCommand::Body::DEAD;
   }else if (filtered_fsr_sum < MIN_STANDING_WEIGHT) {
      r.actions.leds.leftEye = ActionCommand::rgb(0, blink);
      r.actions.leds.rightEye = ActionCommand::rgb(0, blink);
      r.actions.body = ActionCommand::Body::REF_PICKUP;
   }
//   std::cout << r.actions.body.actionType << " " << s.sensors[Sensors::InertialSensor_AccX] << std::endl;
   return r;
}
