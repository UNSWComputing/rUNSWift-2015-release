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

#include <QTabWidget>
#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QGridLayout>
#include <QPixmap>
#include <QLabel>
#include <QPainter>
#include <QColor>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QPushButton>
#include <QString>
#include <QInputDialog>
#include <QMessageBox>
#include <QKeyEvent>
#include <QDial>
#include <QSlider>

#include <cstdio>
#include <iostream>
#include <fstream>
#include <deque>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


#include "tabs/tab.hpp"
#include "types/MapEntry.hpp"
#include "types/RobotInfo.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "perception/vision/VisionConstants.hpp"
#include "perception/vision/Vocab.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"



class Vision;

class Blackboard;

/*
 * This is tab for seeing more detail of surf localisation and for 
 * calibrating new map and vocab files
 */


class SurfTab : public Tab {
   Q_OBJECT
   public:
      SurfTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);

      void keyPressEvent(QKeyEvent *event);

   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();  
      void redrawImage();
      void drawImage(QImage *image); 

      typedef enum {
         mVIEW = 0,
         mVOCAB = 1,
         mMAP = 2,
         mROBOT = 3
      } Mode;    

      Mode mode; // viewing stream, learning vocab or building map

      QGridLayout *layout;

      QPushButton *learnVocabButton;  
      QPushButton *learnRobotButton;   
      QPushButton *buildMapButton;      
		QPushButton *saveButton;
      QPushButton *setLandmarkButton;
      QPushButton *clearLandmarkButton;
      QPushButton *cancelButton;

      QLabel *landmarkValueLabel;
      FieldView fieldView;

      QDial *dial;
      QSlider *slider;

      // Variables for camera image
      QPixmap imagePixmap;
      QLabel *camLabel;

      // Data
      Blackboard *blackboard;
      Colour saliency[TOP_SALIENCY_COLS][TOP_SALIENCY_ROWS];
      std::vector<Ipoint> landmarks;

      unsigned int resetMissedFrames;

      // for storing a map
      std::vector<MapEntry> map;
      int map_position;
      int frames_recorded; // if we want to record more than 1 set of interest points at each location

      // for learning robots
      std::vector<MapEntry> robots;
      std::vector<RobotInfo> robotInfo;
      std::vector<Ipoint> robotPoints;
      int range;
      int orientation;

      // for learning vocab
      Vocab vocab;
      std::vector<Ipoint> ipts;
      int images;


   public slots:
      void newNaoData(NaoData *naoData);
    
      void learnVocabSlot();
      void buildMapSlot();
      void learnRobotsSlot();
      void saveSlot();
      void setLandmarkSlot();
      void clearLandmarkSlot();
      void cancelSlot();

      void rangeSlot(int range);
      void orientationSlot(int orientation);
};

