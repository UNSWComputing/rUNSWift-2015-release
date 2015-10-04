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

#include <boost/serialization/access.hpp>
#include <ctime>
#include <vector>
#include "frame.hpp"
/*
 * Nao data holds all the data that the ui will need when displaying information about the nao.
 * The idea is that a thread will be communicating with the nao and adding data to the NaoData
 * class while the ui reads this data.
 *
 * We will keep track of the current frame that the user in the UI is viewing and in this way
 * we can view data in the ui that is not in sync with what is comming from the nao.
 *
 * The data actually received from the Nao will be stored in frames and naoData will store an
 * array of these frames.
 */

class NaoData {
   public:
      inline Frame &getCurrentFrame() {
         if (currentFrame >= frames.size() ||
             currentFrame < 0) {
             static Frame frame;
             return frame;
         }
         return frames[currentFrame];
      }
      inline Frame &getFrame(int n) {return frames[n];}

      inline int nextFrame() {
         if (currentFrame != frames.size() - 1) currentFrame++;
         return currentFrame;
      }

      inline int prevFrame() {
         if (currentFrame != 0) currentFrame--;
         return currentFrame;
      }

      inline int getFramesTotal() {return frames.size(); }
      inline int getCurrentFrameIndex() { return currentFrame; }
      inline bool getIsPaused() { return isPaused; }
      inline void setPaused(bool b) { isPaused = b; }

      inline void setCurrentFrame(unsigned int index) {
         if (index >= frames.size()) return;
         currentFrame = index;
      }

      inline time_t timeTillNextFrame() {
         if (getCurrentFrameIndex() != getFramesTotal()) {
            return frames[currentFrame].timestamp -
                   frames[currentFrame].timestamp;
         }
         return 0;
      }

      NaoData() : currentFrame(0), isPaused(true) {}
      // this is public for testing purposes...
      // eventually need a smart way to push frames
      // to disk as this array could get quite large.
      inline void appendFrame(Frame frame) {frames.push_back(frame);}
   private:

      std::vector<Frame> frames;
      // index into frames that indicates what the UI is
      // viewing from the frames array.
      unsigned int currentFrame;

      // contains the unix time of when this data was actually recorded.
      time_t timeRecorded;


      bool isPaused;

      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version);
};
#include "naoData.tcc"
