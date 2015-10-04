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

#include <QObject>
#include <QWidget>
#include <QMenuBar>
#include <QString>
#include <QRgb>
#include <QPaintDevice>
#include <QTabWidget>

#include <vector>
#include <utility>

#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/Logger.hpp"

#include "types/FootInfo.hpp"
#include "types/BallInfo.hpp"
#include "types/PostInfo.hpp"
#include "types/Ipoint.hpp"

#include "naoData.hpp"

class Vision;

class Tab : public QWidget {
   Q_OBJECT

   public:
      Tab() : vision(0), currentFrame(0) {}
      Tab(QTabWidget *parent) : QWidget(parent), parent(parent), vision(0),
                                topFrame(0), botFrame(0) {}

   protected:
      /* Returns the RGB value of the pixel at row, col
       * @arg: num_cols is used to vary the image resolution
       */
      virtual QRgb getRGB(unsigned int col, unsigned int row,
                          const uint8_t *yuv, int num_cols);

      QTabWidget *parent;

      /*  vision module from libsoccer */
      Vision *vision;

      /* Current working image
       * If you working with vision you need a frame.
       */
      const uint8_t *currentFrame;
      const uint8_t *topFrame;
      const uint8_t *botFrame;

      /*
       * Generic draw overlays function
       * Supply Null to any of the arguments if you do not wish to draw
       * a particular overlay. topImage is always required.
       */
      void drawOverlaysGeneric (QPaintDevice *topImage,
            QPaintDevice                        *botImage,
            const std::pair<int, int>           *horizon,
            const std::vector<FootInfo>         *feet,
            const std::vector<BallInfo>         *balls,
            const std::vector<PostInfo>         *posts,
            const std::vector<RobotInfo>        *robots,
            const std::vector<FieldEdgeInfo>    *fieldEdges,
            const std::vector<FieldFeatureInfo> *fieldFeatures,
            const std::vector<Ipoint>           *landmarks,
            float scale
      );

      virtual void tabSelected();
      virtual void tabDeselected();

   signals:
      virtual void showMessage(const QString &, int timeout = 0);

   public slots:
      virtual void newNaoData(NaoData *naoData) = 0;
      virtual void readerClosed() {}

   friend class Visualiser;
};
