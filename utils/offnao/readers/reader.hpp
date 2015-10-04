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

#include <QThread>
#include <QObject>
#include <QDebug>
#include <QAction>
#include "naoData.hpp"
#include "frame.hpp"
#include "transmitter/TransmitterDefs.hpp"
/*
 * This will be a virtual class that creates and feeds a naoData class.
 * The two intended subclasses to implement this interface will be
 * The diskReader and networkReader classes. edit: now includes dump reader for vision only debugging.
 */
class Reader :  public QThread {
   Q_OBJECT
   public slots:
   virtual void forwardMediaTrigger();
   virtual void backwardMediaTrigger();
   virtual void pauseMediaTrigger();
   virtual void playMediaTrigger();
   virtual void sliderMoved(int amount);
   virtual void refreshNaoData();
   virtual void stopMediaTrigger();
   virtual void recordMediaTrigger();
   /**
    * only for NetworkReader, but not sure how to put it there
    */
   virtual void sendCommandLineString(QString item) {};

   public:
   Reader() {}
   Reader(const NaoData &naoData) : naoData(naoData) {}
   virtual ~Reader() {}

   NaoData naoData;

   /* main loop that runs when the thread starts */
   virtual void run() = 0;
   virtual void finishUp() { isAlive = false;}
   void setSendingMask(OffNaoMask_t mask) {
      this->mask = mask;
   }
   signals:
   void newNaoData(NaoData *naoData);
   virtual void showMessage(const QString &, int timeout = 0);
   /**
    * lets the user select a file to open, then loads the file
    */
   void openFile();
   void disconnectFromNao();
   protected:
   bool isAlive;
   OffNaoMask_t mask;
};
