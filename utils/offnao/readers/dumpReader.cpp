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

#include <QDebug>
#include <QString>
#include <QMessageBox>
#include <string>
#include <sstream>
#include <cmath>
#include "utils/basic_maths.hpp"
#include "readers/dumpReader.hpp"
#include "blackboard/Blackboard.hpp"
#include "progopts.hpp"
#include <iostream>
 using namespace std;
DumpReader::DumpReader(QString fileName) {
   dumpFile = fopen(fileName.toAscii().data(), "r");
   if (dumpFile == NULL) {
      // QMessageBox::warning(this, "Error", "Could not open dump file");
   }
}

DumpReader::DumpReader(QString fileName, const NaoData &naoData) :
Reader(naoData) {
   dumpFile = fopen(fileName.toAscii().data(), "r");
   if (dumpFile == NULL) {
      // QMessageBox::warning(this, "Error", "Could not open dump file");
   }
}

void DumpReader::run() {

   int currentIndex = 0;
   //yuy2 encoding uses 16 bits per pixel, thus 2x8bits per pixel
   // load whole file into memory  dodgey for now.
   int frameLoaded = 1;
   do {
      uint8_t* top =
            (uint8_t *) malloc(sizeof(uint8_t)*TOP_SIZE);
      uint8_t* bot =
    		(uint8_t *) malloc(sizeof(uint8_t)*BOT_SIZE);
      if(fread(top, TOP_SIZE, 1, dumpFile) != 1){
         free(top);
         break;
      } else {
         if(fread(bot, BOT_SIZE ,1, dumpFile)!= 1){
        	 free(bot);
        	 break;
         } else {
			 Frame frame;
			 Blackboard *blackboard = new Blackboard(config);
			 OffNaoMask_t mask = RAW_IMAGE_MASK;
			 // writeTo(, mask, mask);
			 blackboard->write(&(blackboard->mask), mask);
			 writeTo(vision, topFrame, (const uint8_t*) top);
			 writeTo(vision, botFrame, (const uint8_t*) bot);
			 frame.blackboard = blackboard;
			 naoData.appendFrame(frame);
			 std::stringstream s;
			 s << "Loading frame " << frameLoaded << " from YUV dump.";
			 emit showMessage(s.str().c_str(), 2000);
			 frameLoaded++;
         }
      }
   } while (1);
   std::stringstream s;
   s << "Finshed loading YUV dump which consisted of " <<
        frameLoaded << " frames.";
   emit showMessage(s.str().c_str(), 5000);
   emit newNaoData(&naoData);

   isAlive = true;
   while (isAlive) {
      if (!naoData.getIsPaused() && naoData.getCurrentFrameIndex() <
           naoData.getFramesTotal() - 1) {
         naoData.nextFrame();
         emit newNaoData(&naoData);
      } else if (currentIndex != naoData.getCurrentFrameIndex()) {
         emit newNaoData(&naoData);
      }
      currentIndex = naoData.getCurrentFrameIndex();
      msleep(500);
   }
   emit newNaoData(NULL);
}


void DumpReader::stopMediaTrigger() {
}
