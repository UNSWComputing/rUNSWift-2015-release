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

#include <malloc.h>
#include <limits.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include "perception/vision/NaoCameraV4.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"
#include "utils/Timer.hpp"

/* Used by setCamera only.
 * Can be removed when v3 backwards compadability is nolonger needed
 */
#include "perception/vision/Vision.hpp"


using namespace std;

/**
 * reads the system error and writes it to the log, then throws an exception
 * @param s an additional string, decribing the failed action
 */
static inline void errno_throw(const char *s) {
   llog(ERROR) << s << " error "<< errno << ", " << strerror(errno) << endl;
   throw runtime_error(strerror(errno));
}

/**
 * sets all bytes of x to 0
 * @param x the variable to clear
 */
#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define CAMERA_FRAMERATE 30

/**
 * Set the initial settings for the camera(format, etc.)
 */
NaoCameraV4::NaoCameraV4(Blackboard *blackboard, const char *filename, const IOMethod method,
                     const int format, const WhichCamera initialCamera)
:
   /* 0 is a dummy value that causes the protected constructor to be called */
   NaoCamera(blackboard, filename, method, format, WHICH_CAMERA_ERROR, 0)
{
   // open your camera
   open_device();
   // decide if its a file or a device
   struct stat buf;
   if (fstat(fd, &buf) < 0) {
      llog(ERROR) << "failed to stat " << filename << endl;
      throw runtime_error("inside NaoCamera.cpp, failed to stat camera");
   }
   v4lDeviceP = S_ISCHR(buf.st_mode);

   // fix green camera bug
   setControl(V4L2_CID_SET_DEFAULT_PARAMETERS, 0);

   // reopen your camera (SET_DEFAULT_PARAMETERS causes hang on DQBUF)
   close_device();
   open_device();

   init_buffers();

   if (!init_camera()) {
      llog(FATAL) << "Error initializing camera!\n";
      throw runtime_error("Error initializing camera");
   }

   start_capturing();

   // close everything
   //stop_capturing();
   //uninit_buffers();
//   close_device();

   // re open it again
//   open_device();
   //init_buffers();
/*
   if (!init_camera()) {
      llog(FATAL) << "Error initializing camera!\n";
      throw runtime_error("Error initializing camera");
   }
*/
   //start_capturing();
   
   dumpFile = NULL;
}

NaoCameraV4::~NaoCameraV4()
{
}

bool NaoCameraV4::setCamera(WhichCamera whichCamera) {
   if (whichCamera == TOP_CAMERA) {
      Vision::camera = Vision::top_camera;
      return true;
   } else if (whichCamera == BOTTOM_CAMERA) {
      Vision::camera = Vision::bot_camera;
      //Vision::camera = Vision::top_camera;
      return true;
   } else {
      return false;
   }
}

WhichCamera NaoCameraV4::getCamera() {
   if (Vision::camera == Vision::top_camera) {
      return TOP_CAMERA;
   } else {
      return BOTTOM_CAMERA;
   }
}
