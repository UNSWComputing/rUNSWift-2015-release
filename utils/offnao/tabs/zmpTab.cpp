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
#include "zmpTab.hpp"
#include "utils/basic_maths.hpp"
#include "utils/body.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/rle.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/Vision.hpp"

using namespace std;

ZMPTab::ZMPTab(QTabWidget *parent, QMenuBar *menuBar,
      Vision *vision)  {
   initMenu(menuBar);
   init();
   this->vision = vision;
   this->parent = parent;
}

void ZMPTab::initMenu(QMenuBar *) {
}

void ZMPTab::init() {
   layout = new QGridLayout(this);
   setLayout(layout);
   layout->setAlignment(layout, Qt::AlignTop);
   layout->setHorizontalSpacing(5);
   layout->setVerticalSpacing(5);

   last_frame = -1;

   coronal_plot = (DataPlot*) new CoronalZMPPlot(this);
   sagittal_plot = (DataPlot*) new SagittalZMPPlot(this);
   com_x_plot = (XYZPlot*) new COMxPlot(this);
   com_y_plot = (XYZPlot*) new COMyPlot(this);

   layout->addWidget(coronal_plot, 0, 0);
   layout->addWidget(sagittal_plot, 1, 0);
   layout->addWidget(com_y_plot, 0, 1);
   layout->addWidget(com_x_plot, 1, 1);
   //layout->addWidget(diagram, 1, 0, 2, 1);
}

void ZMPTab::newNaoData(NaoData *naoData) {
   // return;
   // if (parent->currentIndex() == parent->indexOf(this)) return;

   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      // clean up display, as read is finished
   } else if (naoData->getFramesTotal() != 0) {
      int new_frame = naoData->getCurrentFrameIndex();
      if (new_frame == last_frame + 1) {
         // special case for one frame at a time
         Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
         SensorValues s = readFrom(motion, sensors);
         XYZ_Coord com = readFrom(motion, com);
         coronal_plot->push(s);
         sagittal_plot->push(s);
         com_x_plot->push(com);
         com_y_plot->push(com);
      } else if (new_frame == last_frame - 1) {
         // special case for one frame at a time
         /*
         Blackboard *blackboard = (naoData->getFrame(last_frame - PLOT_SIZE).blackboard);
         SensorValues s = readFrom(motion, sensors);
         XYZ_Coord com = readFrom(motion, com);
         coronal_plot->push(s, true);
         sagittal_plot->push(s, true);
         com_x_plot->push(com);
         com_y_plot->push(com);
         */
      } else if (ABS(new_frame - last_frame) > PLOT_SIZE) {
         // scrap all data and pass in new array
         std::vector<SensorValues> s;
         SensorValues null;
         std::vector<XYZ_Coord> coms;
         XYZ_Coord temp;
         if (new_frame < PLOT_SIZE - 1) {
            for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
               null.sensors[i] = 0.0f;
            temp.x = 0;
            temp.y = 0;
            temp.z = 0;
         }
         for (int i = new_frame - PLOT_SIZE + 1; i <= new_frame; ++i) {
            if (i >= 0) {
               s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
               coms.push_back(naoData->getFrame(i).blackboard->motion.com);
            } else {
               s.push_back(null);
               coms.push_back(temp);
            }
         }
         coronal_plot->push(s);
         sagittal_plot->push(s);
         com_x_plot->push(coms);
         com_y_plot->push(coms);
      } else if (new_frame < last_frame) {
         // push some new data to the front of graph
         std::vector<SensorValues> s;
         SensorValues null;
         std::vector<XYZ_Coord> coms;
         XYZ_Coord temp;
         if (new_frame < PLOT_SIZE - 1) {
            for (uint8_t i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
               null.sensors[i] = 0.0f;
            temp.x = 0;
            temp.y = 0;
            temp.z = 0;
         }
         for (int i = new_frame - PLOT_SIZE + 1; i <= last_frame - PLOT_SIZE; ++i) {
            if (i < 0) {
               s.push_back(null);
               coms.push_back(temp);
            } else {
               s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
               coms.push_back(naoData->getFrame(i).blackboard->motion.com);
            }
         }
         coronal_plot->push(s, true);
         sagittal_plot->push(s, true);
         com_x_plot->push(coms, true);
         com_y_plot->push(coms, true);
      } else if (new_frame > last_frame) {
         // push some new data to the end of graph
         std::vector<SensorValues> s;
         std::vector<XYZ_Coord> coms;
         for (int i = last_frame + 1; i <= new_frame; ++i) {
            s.push_back(naoData->getFrame(i).blackboard->motion.sensors);
            coms.push_back(naoData->getFrame(i).blackboard->motion.com);
         }
         coronal_plot->push(s);
         sagittal_plot->push(s);
         com_x_plot->push(coms);
         com_y_plot->push(coms);
      }
      last_frame = new_frame;
   }
}

void ZMPTab::readerClosed() {
}
