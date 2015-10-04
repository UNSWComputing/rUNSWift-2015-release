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
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>

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
#include <QLabel>
#include <list>

#include <cstdio>
#include <deque>
#include <QPainter>

#include "tabs/tab.hpp"
#include "tabs/variableView.hpp"
#include "tabs/plots.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/VisionDefs.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"


class WalkTab : public Tab {
   Q_OBJECT
   public:
      WalkTab(QTabWidget *parent, QMenuBar *menuBar, Vision *vision);
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      QGridLayout *layout;
      QwtPlot *bodyPlot[2];
      QwtPlotCurve *curve[2];
      QwtPlotCurve *axis[2][2];
      std::list<float> theta[2], dTheta[2];
      int lastCurrent;
      static const unsigned int numEntries;
      double atheta[2][5], aDTheta[2][5];
      double zero[100], line[100];

   public slots:
      void newNaoData(NaoData *naoData);
      void readerClosed();
};

