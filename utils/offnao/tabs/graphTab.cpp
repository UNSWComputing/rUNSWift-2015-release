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

#include <QMenu>
#include <QMenuBar>
#include <QDebug>

#include <iostream>
#include <sstream>
#include "graphTab.hpp"
#include "utils/basic_maths.hpp"
#include "utils/body.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/rle.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/Vision.hpp"


using namespace std;

GraphTab::GraphTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision)  {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->parent = parent;
}

void GraphTab::initMenu(QMenuBar *) {
}

void GraphTab::init() {
  
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);
   layout->setHorizontalSpacing(5);
   layout->setVerticalSpacing(5);

   last_frame = -1;

   for (int i = 0; i < 3; i++) {
      accPlots[i] = new MultiPlot(this, Sensors::sensorNames[Sensors::InertialSensor_AccX+i], 2, -15, 15);
      layout->addWidget(accPlots[i], 0, i);
   }

   for (int i = 0; i < 2; i++) {
      gyrPlots[i] = new MultiPlot(this, Sensors::sensorNames[Sensors::InertialSensor_GyrX+i], 2, -2, 2);
      layout->addWidget(gyrPlots[i], 1, i);
   }
   gyrPlots[2] = new MultiPlot(this, Sensors::sensorNames[Sensors::InertialSensor_GyroscopeZ], 1, -2, 2);
   layout->addWidget(gyrPlots[2], 1, 2);

   for (int i = 0; i < 2; i++) {
      anglePlots[i] = new MultiPlot(this, Sensors::sensorNames[Sensors::InertialSensor_AngleX+i], 1, -1, 1);
      layout->addWidget(anglePlots[i], 2, i);
   }
   odomPlot = new MultiPlot(this, "Turn rate", 2, -180, 180);
   layout->addWidget(odomPlot, 2, 2);
}

void GraphTab::updatePlots(SensorValues s, Odometry o, bool left) {
    for (int i = 0; i < 3; i++) {
        std::vector<float> acc;
        acc.push_back(s.sensors[Sensors::InertialSensor_AccX+i]);
        acc.push_back(s.sensors[Sensors::InertialSensor_AccelerometerX+i]);
        accPlots[i]->push(acc, left);
    }
    for (int i = 0; i < 2; i++) {
        std::vector<float> gyro;
        gyro.push_back(s.sensors[Sensors::InertialSensor_GyrX+i]);
        gyro.push_back(s.sensors[Sensors::InertialSensor_GyroscopeX+i]);
        gyrPlots[i]->push(gyro, left);
    }
    std::vector<float> gyro;
    gyro.push_back(s.sensors[Sensors::InertialSensor_GyroscopeZ]);
    gyrPlots[2]->push(gyro, left);
    for (int i = 0; i < 2; i++) {
        std::vector<float> angle;
        angle.push_back(s.sensors[Sensors::InertialSensor_AngleX+i]);
        anglePlots[i]->push(angle, left);
    }
    std::vector<float> odom;
    odom.push_back(-RAD2DEG(s.sensors[Sensors::InertialSensor_GyroscopeZ]));
    odom.push_back(RAD2DEG(o.turn));
    odomPlot->push(odom, left);
}

void GraphTab::updatePlots(std::vector<SensorValues> s, std::vector<Odometry> o, bool left) {
    if (left)
        for (int i = s.size() - 1; i > 0; i--) {
            updatePlots(s[i], o[i], left);
        }
    else
        for (u_int i = 0; i < s.size(); i++) {
            updatePlots(s[i], o[i], left);
        }
}

void GraphTab::newNaoData(NaoData *naoData) {

   // if (parent->currentIndex() == parent->indexOf(this)) return;

   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      // clean up display, as read is finished
   } else if (naoData->getFramesTotal() != 0) {
      int new_frame = naoData->getCurrentFrameIndex();
      if (new_frame == last_frame + 1) {
         // special case for one frame at a time
         Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
         SensorValues s = readFrom(motion, sensors);
         Odometry odometry = readFrom(motion, odometry);
         updatePlots(s, odometry);
      } else if (new_frame == last_frame - 1) {
         // special case for one frame at a time
         /*
         Blackboard *blackboard = (naoData->getFrame(last_frame - PLOT_SIZE).blackboard);
         SensorValues s = readFrom(motion, sensors);
         acc_plot->push(s, true);
         fsr_plot->push(s, true);
         tilt_plot->push(s, true);
         sonar_plot->push(s, true);
         charge_plot->push(s, true);
         current_plot->push(s, true);
         */
      } else if (ABS(new_frame - last_frame) > PLOT_SIZE) {
         // scrap all data and pass in new array
         std::vector<SensorValues> s;
         std::vector<Odometry> o;
         SensorValues emptyS;
         Odometry emptyO;
         if (new_frame < PLOT_SIZE - 1)
            for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
               emptyS.sensors[i] = 0.0f;
         for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i) {
            if (i >= 0) {
               s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
               o.push_back(naoData->getFrame(i).blackboard->motion.odometry);
            } else {
               s.push_back(emptyS);
               o.push_back(emptyO);
            }
         }
         updatePlots(s, o);
      } else if (new_frame < last_frame) {
         // push some new data to the front of graph
          std::vector<SensorValues> s;
          std::vector<Odometry> o;
          SensorValues emptyS;
          Odometry emptyO;
          if (new_frame < PLOT_SIZE - 1)
             for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
                emptyS.sensors[i] = 0.0f;
          for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i) {
             if (i >= 0) {
                s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
                o.push_back(naoData->getFrame(i).blackboard->motion.odometry);
             } else {
                s.push_back(emptyS);
                o.push_back(emptyO);
             }
          }
   
         updatePlots(s, o, true);
      } else if (new_frame > last_frame) {
         // push some new data to the end of graph
         std::vector<SensorValues> s;
         std::vector<Odometry> o;
         for (int i = last_frame + 1; i <= new_frame; ++i) {
            s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
            o.push_back(naoData->getFrame(i).blackboard->motion.odometry);
         }
         updatePlots(s, o);
      }
      last_frame = new_frame;
   }
}

void GraphTab::readerClosed() {
}
