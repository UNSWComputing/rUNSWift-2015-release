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
#include <QButtonGroup>
#include <QCheckBox>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMatrix>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QObject>
#include <QPainter>
#include <QProgressDialog>
#include <QPushButton>
#include <QRadioButton>
#include <QTabWidget>
#include <QWheelEvent>
#include <QWidget>
#include <QtNetwork/QFtp>


#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

#include "mediaPanel.hpp"
#include "tabs/tab.hpp"
#include "tabs/PointCloud.hpp"
#include "classifier.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"

class Vision;

/*
 *  Tab for image classification.
 */
class CalibrationTab : public Tab {
   Q_OBJECT
   public:
      CalibrationTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
      virtual ~CalibrationTab();
      QAction *loadDumpAct;

   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      QCheckBox *checkAutoWeight;
      QCheckBox *checkVisionOverlay;

      static const int ZOOM_LIMIT = 6;
      int zoomLevel, zoomLog;

      typedef enum {
         oNONE = 0,
         oSELECTED = 1,
         oSELECTED_UNCLASSIFIED = 2,
         oALL = 3,
         oALL_UNCLASSIFIED = 4,
         oNUM_OVERLAYS = 5
      } Overlay;

      // Access to widgets in higher-levels of the app
      // (is this dodgy, or the 'Qt Way?')
      // Ui::Visualiser *ui;

      // Image
      QLabel *camLabel;
      QPixmap imagePixmap;
      QImage lastRendering;

      // PointCloud
      PointCloud pointCloud;
      QRadioButton* qrbControls[2];
      QHBoxLayout qhblControls;
      QGridLayout qglPointCloud;
      QWidget qwPointCloud;
      QDockWidget qdwPointCloud;

      // Buttons
      QPushButton *undo;


      // Option Boxes
      QVBoxLayout *checkboxLayout;

      QButtonGroup *colourGroup;
      QVBoxLayout *colourGroupLayout;

      QButtonGroup *overlayGroup;
      QVBoxLayout *overlayGroupLayout;

      QGroupBox *kernelBox;
      QGridLayout *kernelLayout;
      QPushButton *newKernelButton;

      QGroupBox *colourBox;
      QRadioButton *colours[cNUM_COLOURS];
      QGroupBox *overlayBox;
      QRadioButton *overlays[oNUM_OVERLAYS];

      // Layout
      QGridLayout *layout;


      std::string topNNMCPath;
      std::string botNNMCPath;
      std::string topKernelPath;
      std::string botKernelPath;

      bool isTop();
      Classifier *getCurrentClassifier();
      // Data
      Classifier *topClassifier;
      Classifier *botClassifier;

      QRadioButton *topCamera;
      QRadioButton *botCamera;

      Colour colour;
      Overlay overlay;
      FILE *dumpFile;
      // uint8_t *currentFrame;

      // libsoccer utilities, for debugging vision
      // mouse overlay vars
      QPoint mousePosition;
      // Methods:
      // Re-draw the image box from current frame.
      void redraw();
      // Draw the image on top of a pixmap
      void drawImage(QImage *image);
      // Draw the overlays on top of that pixmap
      void drawOverlays(QImage *image);
      QPixmap drawCrosshair();
      QPoint translateToZoomedImageCoords(QPoint);
      void drawPointCloud();

      // Event filter, catches mouse clicks and the like
      bool eventFilter(QObject *object, QEvent *event);
      // Classify a pixel
      bool classifyMouseEvent(QMouseEvent *e);
      bool classifyWheelEvent(QWheelEvent *e);
      void mouseMoveEvent(QMouseEvent * event);
      int mouseX, mouseY;
      int prevMouseX, prevMouseY, prevZoomLevel;

      void createKernelGroup();


      // Menu stuff
      QMenu *calibrationMenu;

      QAction *saveBothKernelsAct;

      QAction *saveAsNNMCAct;
      QFtp *ftp;
      QFile *nnmcFile;
      QProgressDialog *progressDialog;
      QAction *undoAct;
      NaoData *naoData;
      public slots:
         // attached to the buttons/checkboxes of the same name
         void newKernel();
         void loadKernel();
         void loadKernelFile(std::string f);
         void saveKernel();
         void saveAsKernel();
         void saveBothKernels();
         void saveNnmc();
         void pushNnmc();
         void giveUp();
         void setColour(int);
         void setOverlay(int);
         void undoAction();
         void redrawSlot();
         void ftpCommandFinished(int commandId, bool error);
         void updateDataTransferProgress(qint64 readBytes, qint64 totalBytes);
         void cancelUpload();
         void newNaoData(NaoData *naoData);
};
