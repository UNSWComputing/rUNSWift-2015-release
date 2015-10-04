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

#include "tabs/tab.hpp"
#include "mediaPanel.hpp"
#include "utils/FieldPainter.hpp"
#include "utils/FieldObject.hpp"
#include "types/AbsCoord.hpp"

#include <vector>

#include <QLineEdit>
#include <QPaintDevice>
#include <QPushButton>
#include <QButtonGroup>

#include <boost/random.hpp>
class QLabel;
class Localiser;

/*
 * This contains the localisation debuging tab.
 */
class LocalisationTab : public Tab {
   Q_OBJECT
   public:
      LocalisationTab();
      bool eventFilter (QObject *object, QEvent *event);

   private:
      void initInterface();
      void initFieldObjects();

      AbsCoord trueRobotPos;
      void createObservation(FieldObject *obj);

      RRCoord absToRR(AbsCoord obsAbs);

      QLabel *fieldLabel;
      QImage image;
      QPixmap imagePixmap;
      QLineEdit *trueX;
      QLineEdit *trueY;
      QLineEdit *trueTheta;
      QLineEdit *varianceDistance;
      QLineEdit *varianceHeading;
      QLineEdit *varianceOrientation;
      QLineEdit *biasDistance;
      QLineEdit *biasHeading;
      QLineEdit *biasOrientation;
      QPushButton *initButton;
      QButtonGroup *teamButtonGroup;

      void redraw();
      bool team_red;
      Localiser *localiser;
      QTransform transform;
      void setTransform(int device_width, int device_height);

      std::vector<FieldObject> fieldObjects;

      bool onMouseEnter(QMouseEvent *mevent);
      bool onMouseLeave(QMouseEvent *mevent);
      bool onMouseMove(QMouseEvent *mevent);
      bool onMouseButtonPress(QMouseEvent *mevent);
      bool onMouseButtonRelease(QMouseEvent *mevent);

      bool mouseDown;
      FieldObject *mouseOverObject;

      virtual void paintEvent(QPaintEvent*);

      float zeroMeanGaussianSample(float standardDeviation);

      boost::mt19937 rng;
   public slots:
      void newNaoData(NaoData *naoData);
      void initLocalisation();
      void setTeam(int);
};

