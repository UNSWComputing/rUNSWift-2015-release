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


#include <QColor>
#include <QEvent>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QObject>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QTabWidget>
#include <QThread>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QWidget>
#include <QVBoxLayout>

#include <cstdio>
#include <deque>

#include "blackboard/Blackboard.hpp"
#include "tabs/tab.hpp"
#include "utils/Logger.hpp"
#include "fieldView.hpp"
#include "mediaPanel.hpp"

#include <boost/system/error_code.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/thread.hpp>

#define PORT 10018
#define NUM_ROBOTS 5 

class Blackboard;

class TeamTab : public Tab {
   Q_OBJECT
   public:
      TeamTab(QTabWidget *parent, QMenuBar *menuBar);
      ~TeamTab();

      void startListening();
      void handleReceive(const boost::system::error_code &error,
                         std::size_t size);

      QPixmap *renderPixmap;
      QLabel *renderWidget;

      // Receiver buffer
      // 1500 is the generally accepted maximum transmission unit on ethernet
      char recvBuffer[1500];

      std::vector<AbsCoord> robots;
      std::vector<AbsCoord> walkingTo;
      std::vector<AbsCoord> shootingTo;
      std::vector<AbsCoord> balls;
      std::vector<double> timeOut;

   private:
      void init();
      void initMenu(QMenuBar *menuBar);

      QGridLayout *layout;
      QLineEdit *teamNum;
      QPushButton *teamUpdate;

      /* These variables are used to present the debug variables from the nao*/
      FieldView fieldView;

      // Boost socket stuff
      boost::asio::io_service service;
      boost::asio::ip::udp::socket socket;
      boost::asio::ip::udp::endpoint remoteEndpoint;
      boost::thread *t;

      // Team
      int team;

   public slots:
      void changeTeam();
      void newNaoData(NaoData *naoData);
      void redraw();

   signals:
      void update();
};
