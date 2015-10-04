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

#include <cmath>
#include <vector>
#include "motion/touch/FilteredTouch.hpp"
#include <sys/time.h>
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/Timer.hpp"

//2.4 /3.2
#define GYR_SCALE 2.35//3.2
#define ACC_SCALE 9.81/65
#define ANG 0
#define GYR 1
#define ACC 2
#define X 0
#define Y 1
#define Z 2

using namespace std;

FilteredTouch::FilteredTouch(Touch* t)
   : touch(t),
     init(true){
   llog(INFO) << "FilteredTouch constructed" << std::endl;

   kf[1].init(1.0/100.0, 0.01, 0.35, true);  //0.01, 0.25
   kf[0].init(1.0/100.0, 0.01, 0.35, false);

   // init body position
   lastBodyPosition = vec4<float>(0, 0, 390, 1);

   // init the inertial sensor calibration offsets
   for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
         imuOffset[i][j] = 0;
         targetOffset[i][j] = 0;
         count[i][j] = 0;
         avg[i][j] = 0;
         err[i][j] = 0;
         k[i][j] = 0;
      }
   }
   prevAng[0] = prevAng[1] = 0;

//   csv.enable();
}

FilteredTouch::~FilteredTouch() {
   llog(INFO) << "FilteredTouch destYroyed" << std::endl;
}

void FilteredTouch::readOptions(const boost::program_options::variables_map& config){
   imuOffset[GYR][0] =
         config["touch.gyrXOffset"].as<float>();
   imuOffset[GYR][1] =
         config["touch.gyrYOffset"].as<float>();
   targetOffset[GYR][0] = imuOffset[GYR][0];
   targetOffset[GYR][1] = imuOffset[GYR][1];
   targetOffset[ANG][0] = DEG2RAD(config["touch.angleXOffset"].as<float>());
   targetOffset[ANG][1] = DEG2RAD(config["touch.angleYOffset"].as<float>());
}

float FilteredTouch::getScaledGyr(int isY){
   return DEG2RAD(update.sensors[Sensors::InertialSensor_GyrX + isY])/GYR_SCALE;
}

void FilteredTouch::filterOffset(int IMUid, int dir, float obs){
   const float procErr[3] = {0.001, 1, 1};
   const float measErr[3] = {0.004, 12, 12};
   const float toCount[3] = {400, 200, 200};
   count[IMUid][dir]++;
   avg[IMUid][dir] += obs;
   if(count[IMUid][dir] > toCount[IMUid]) {
      avg[IMUid][dir] = avg[IMUid][dir]/count[IMUid][dir];
      err[IMUid][dir] += procErr[IMUid];
      k[IMUid][dir] = err[IMUid][dir]/(err[IMUid][dir] + measErr[IMUid]);
      targetOffset[IMUid][dir] += (avg[IMUid][dir] - targetOffset[IMUid][dir]) * k[IMUid][dir];
      err[IMUid][dir] = (1.0 - k[IMUid][dir]) * err[IMUid][dir];
      count[IMUid][dir] = 0;
      avg[IMUid][dir] = 0;
   }
}

void FilteredTouch::updateIMUOffset(){
   float realVel[2];
   realVel[0] = RAD2DEG(state.sensors[Sensors::InertialSensor_AngleX] - prevAng[0]) * 100.0;
   realVel[1] = RAD2DEG(state.sensors[Sensors::InertialSensor_AngleY] - prevAng[1]) * 100.0;
   prevAng[0] = state.sensors[Sensors::InertialSensor_AngleX];
   prevAng[1] = state.sensors[Sensors::InertialSensor_AngleY];

   if(feetState.groundContact[0] || feetState.groundContact[1]){
      //0 velocity
      for(int i = 0; i < 2; i++){
         if(fabs(realVel[i]) < 0.3){
            filterOffset(GYR, i, -update.sensors[Sensors::InertialSensor_GyrX + i]);
         }
      }
   }

   //slowly adjust to target bias
   for(int j = 0; j < 3; j++){
      for(int i = 0; i < 3; i++){
         imuOffset[j][i] += (targetOffset[j][i] - imuOffset[j][i]) * .03;
      }
   }

   //update offset
   for(int i = 0; i < 2; i++){
      update.sensors[Sensors::InertialSensor_AngleX + i] += imuOffset[ANG][i];
      update.sensors[Sensors::InertialSensor_GyrX + i] += imuOffset[GYR][i];
   }
   for(int i = 0; i < 3; i++){
      update.sensors[Sensors::InertialSensor_AccX + i] += imuOffset[ACC][i];
   }

//   cout << realVel[0] << " " << realVel[1] << " " << areFeetOnGround() << endl;
}

float updateAvg(float number, int i) {
   static float sum[2] = {0, 0};
   static int count[2] = {0, 0};
   float avg;
   if(count[i] > 0){
      avg = ((sum[i] + number) * count[i] - (count[i] + 1) * sum[i])/((count[i]+1) * count[i])
            + sum[i]/count[i];
   } else {
      avg = number;
   }
   sum[i] += number;
   count[i]++;
   return avg;
}

void FilteredTouch::filterSensors(){
   state.sensors[Sensors::InertialSensor_AccX] = update.sensors[Sensors::InertialSensor_AccelerometerX];
   state.sensors[Sensors::InertialSensor_AccY] = update.sensors[Sensors::InertialSensor_AccelerometerY];
   state.sensors[Sensors::InertialSensor_AccZ] = update.sensors[Sensors::InertialSensor_AccelerometerZ];

   updateIMUOffset();

   state.sensors[Sensors::LFoot_FSR_CenterOfPressure_X] = feetState.CoP[0][0];
   state.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y] = feetState.CoP[0][1];
   state.sensors[Sensors::RFoot_FSR_CenterOfPressure_X] = feetState.CoP[1][0];
   state.sensors[Sensors::RFoot_FSR_CenterOfPressure_Y] = feetState.CoP[1][1];

   matrix<float> obs[2] = {matrix<float>(2, 1), matrix<float>(2, 1)};
   for(int i = 0; i < 2; i++){
      obs[i](0, 0) = update.sensors[Sensors::InertialSensor_AngleX + i];
      obs[i](1, 0) = getScaledGyr(i);
      matrix<float> est = kf[i].update(obs[i], feetState);
      state.sensors[Sensors::InertialSensor_AngleX + i] = est(0, 0);
      state.sensors[Sensors::InertialSensor_GyrX + i] = est(1, 0);
   }

   state.sensors[Sensors::InertialSensor_GyroscopeZ] = update.sensors[Sensors::InertialSensor_GyroscopeZ];

   //just using some unused values to make plots on offnao
//   state.sensors[Sensors::LFoot_FSR_CenterOfPressure_X] = update.sensors[Sensors::InertialSensor_AngleX];
//   state.sensors[Sensors::RFoot_FSR_CenterOfPressure_X] = update.sensors[Sensors::InertialSensor_AngleY];
//   state.sensors[Sensors::Battery_Charge] = getScaledGyr(0);
//   state.sensors[Sensors::Battery_Current] = getScaledGyr(1);
}

SensorValues FilteredTouch::getSensors(Kinematics &kinematics) {
   update = touch->getSensors(kinematics);
   FilteredTouch::kinematics = kinematics;

   struct timeval tv;
   static bool said = false;
   gettimeofday(&tv, NULL);
   if (tv.tv_sec % 5 == 0 && !said) {
      if (update.sonar[0] < Sonar::MIN) {
//         SAY("sonar error");  for it to shut up about sonar error
      }
   }
   said = (tv.tv_sec % 5 == 0);
   if (init) {
      init = false;
      state = update;
      state.sensors[Sensors::InertialSensor_GyrX] = 0;
      state.sensors[Sensors::InertialSensor_GyrY] = 0;
      for (uint8_t i = 0; i < Sonar::NUMBER_OF_READINGS; ++i) {
         if (state.sonar[i] >= Sonar::INVALID )
            state.sonar[i] = Sonar::DISCARD;
         if (state.sonar[i] <= Sonar::MIN)
            state.sonar[i] = Sonar::MIN;
      }
   } else {
      state.joints = update.joints;
      uint8_t i;
      for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i){
         //don't erase previous sensor values
         if((i < Sensors::InertialSensor_AccX || i > Sensors::InertialSensor_AngleY) && i != Sensors::InertialSensor_GyroscopeZ)
            state.sensors[i] = update.sensors[i];
      }
      for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i) {
         if (update.sonar[i] >= Sonar::INVALID)
            update.sonar[i] = Sonar::DISCARD;
         if (update.sonar[i] >= Sonar::MIN)
            state.sonar[i] += (update.sonar[i] - state.sonar[i]) * 1.0;
      }
      feetState.update(update, kinematics);
      filterSensors();
   }

   csv.log(update, state);
   return state;
}

bool FilteredTouch::getStanding() {
   return touch->getStanding();
}

ButtonPresses FilteredTouch::getButtons() {
   return touch->getButtons();
}
