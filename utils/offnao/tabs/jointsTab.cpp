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
#include <QFileDialog>
#include <QDebug>
#include <QMouseEvent>

#include <iostream>
#include "perception/vision/yuv.hpp"
#include "jointsTab.hpp"
#include "blackboard/Blackboard.hpp"

using namespace std;

JointsTab::JointsTab(QTabWidget *parent,
      QMenuBar *menuBar, Vision *vision)  {
   initMenu(menuBar);
   init();
   this->vision = vision;
   currentFrame = NULL;
   this->parent = parent;
}


void JointsTab::initMenu(QMenuBar *menuBar) {
   jointsMenu = new QMenu("Joints");
   menuBar->addMenu(jointsMenu);
}

void JointsTab::init() {
   int i;

   // Lay them all out
   layout = new QGridLayout();
   this->setLayout(layout);

   // Set up pixmaps for camera image and RGB, YUV, HSV histograms
   for (i = 0; i < NUM_GRAPH_LOCATIONS; i++) {
      // Create pixmap and label
      graphPixmaps[i] = QPixmap(320, 240);
      graphPixmaps[i].fill(Qt::darkGray);
      graphLabels[i] = new QLabel();
      graphLabels[i]->setPixmap(graphPixmaps[i]);

      // Set alignment and size of pixmaps
      graphLabels[i]->setAlignment(Qt::AlignTop);
      graphLabels[i]->setMinimumSize(IMAGE_COLS/2, IMAGE_ROWS/2);
      graphLabels[i]->setMaximumSize(IMAGE_COLS/2, IMAGE_ROWS/2);
   }

   // Position graph pixmaps
   layout->addWidget(graphLabels[TOP_LEFT], 0,0,1,1); // Graph at top left
   layout->addWidget(graphLabels[TOP_RIGHT], 1,0,1,1); // Graph at top right
   layout->addWidget(graphLabels[BOTTOM_RIGHT], 0,1,1,1); // Graph at bottom right
   layout->addWidget(graphLabels[BOTTOM_LEFT], 1,1,1,1); // Graph at bottom left
}


void JointsTab::redraw() {
cout << "Calling redraw\n";
   QImage image = QImage(IMAGE_COLS, IMAGE_ROWS, QImage::Format_RGB32);
   if(currentFrame) {
      //display normal image
      for (unsigned int row = 0; row < IMAGE_ROWS; ++row) {
         for (unsigned int col = 0; col < IMAGE_COLS; ++col) {
            image.setPixel(col, row,
                  getRGB(col, row, currentFrame, TOP_IMAGE_ROWS));
         }
      }
   } else {
      image.fill(Qt::darkGray);
   }
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void JointsTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      currentFrame = NULL;
   } else {
/* FIX THIS
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      if ((currentFrame = readFrom(vision, currentFrame)) != NULL)
         if (parent->currentIndex() == parent->indexOf(this))
            redraw();
*/
   }
}

