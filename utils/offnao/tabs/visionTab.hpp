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

#include <QMenuBar>
#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QGridLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QRadioButton>
#include <QPushButton>
#include <QTabWidget>
#include <cstdio>
#include "tabs/tab.hpp"
#include "mediaPanel.hpp"

class Fovea;


/*
 * This contains the vision debuging tab.
 */
class VisionTab : public Tab {
   Q_OBJECT
   public:
      VisionTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);

      void loadNnmcFile(WhichCamera cam, const char *f);

   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      // Zoom stuff
      static const int ZOOM_LIMIT = 6;
      int zoomLevel, zoomLog; 
      int mouseX, mouseY;
      int prevMouseX, prevMouseY, prevZoomLevel;


      QMenu *visionMenu;
      QAction *reloadBothNnmcAct;
      QAction *loadBotNnmcAct;
      QAction *loadTopNnmcAct;
      QAction *loadGoalMapAct;
      QAction *loadVisualWordsAct;
      QAction *loadAllAct;

      /* objects that the debug info is drawn to */
      QPixmap imagePixmap;
      QLabel *camLabel;

      QGridLayout *layout;

      /* Image selection radio buttons */
      QRadioButton *radioImage;
      QRadioButton *radioSaliency;
      QRadioButton *radioSaliencyEdge;
      QRadioButton *radioSaliencyGrey;
      
      /* checkboxes */
      QCheckBox *checkBottomImage;
      QCheckBox *checkEdgeDirections;
      QCheckBox *checkEdgeOrientations;
      QCheckBox *checkPatches;
      QCheckBox *checkLowLines;

      QCheckBox *checkHorizon;
      QCheckBox *checkBall;
      QCheckBox *checkFeet;
      QCheckBox *checkFeetDebug;
      QCheckBox *checkGoals;
      QCheckBox *checkRobots;
      QCheckBox *checkRobotsDebug;
      QCheckBox *checkFieldEdges;
      QCheckBox *checkFieldFeatures;
      
      QImage lastRendering;

      /* Re-draw the image box from current frame. */
      void redraw();

      /*  Draw the image on top of a pixmap */
      void drawImage          (QImage *image, bool top = true);
      void drawSaliency       (QImage *image, bool top = true);
      void drawSaliencyFovea  (QImage *image, const Fovea &fovea);

      void drawSaliencyEdge   (QImage *image, bool top = true);
      void drawSaliencyEdgeFovea(QImage *image, const Fovea &fovea);

      void drawSaliencyGrey   (QImage *image, bool top = true);
      void drawEdgeDirections (QImage *image, bool top = true);

      /* Draw the overlays on top of that pixmap  */
      void drawOverlays(QImage *image, bool top);


      /* event stuff */
      void mouseMoveEvent(QMouseEvent * event);

      QPoint mousePosition;

      bool eventFilter(QObject *object, QEvent *event);

      bool classifyMouseEvent(QMouseEvent *e);
      bool classifyWheelEvent(QWheelEvent *e);

      bool nnmc_loaded;


   public slots:
      void newNaoData(NaoData *naoData);

      void reloadBothNnmc();
      void loadTopNnmc();
      void loadBotNnmc();
      void loadGoalMap();
      void loadVisualWords();
      void loadAll();

      void redrawSlot();
};

