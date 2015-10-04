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

#include "perception/vision/Camera.hpp"
#include "utils/Timer.hpp"
#include "utils/Logger.hpp"

using namespace std;

/**
 * Controls:


***** NOTE THAT THESE CONTROLS ARE NO LONGER USED        *****
***** WE NOW SET THEM ONE AT A TIME IN setCameraSettings ****
***** DUE TO CAMERA DRIVERS BEING RETARDEDLY BUGGY       *****
 
__u32 controlIds[NUM_CONTROLS] =
{V4L2_CID_VFLIP, V4L2_CID_HFLIP,
 V4L2_CID_DO_WHITE_BALANCE, V4L2_CID_GAIN, V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_EXPOSURE, V4L2_CID_BACKLIGHT_COMPENSATION, V4L2_CID_AUTO_WHITE_BALANCE,
 V4L2_CID_BLUE_BALANCE, V4L2_CID_RED_BALANCE, V4L2_CID_BRIGHTNESS,
 V4L2_CID_CONTRAST, V4L2_CID_SATURATION, V4L2_CID_HUE, V4L2_CID_HCENTER,
 V4L2_CID_VCENTER, V4L2_CID_SHARPNESS};
*/


__u32 controlIds[NUM_CONTROLS] =
/*
{V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST,
 V4L2_CID_SATURATION, V4L2_CID_SHARPNESS,
 V4L2_CID_HFLIP, V4L2_CID_VFLIP,
 V4L2_CID_BACKLIGHT_COMPENSATION, V4L2_CID_AUTO_WHITE_BALANCE,
 V4L2_CID_DO_WHITE_BALANCE, V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_HUE, V4L2_CID_AUTO_EXPOSURE_CORRECTION,
 V4L2_CID_EXPOSURE_CORRECTION, V4L2_CID_HUE_AUTO,
 V4L2_CID_AUTOGAIN, V4L2_CID_AUTOGAIN,
 V4L2_CID_EXPOSURE};
 */

{V4L2_CID_HFLIP, V4L2_CID_VFLIP,
 V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST,
 V4L2_CID_SATURATION, V4L2_CID_HUE,
 V4L2_CID_SHARPNESS,
 V4L2_CID_AUTO_WHITE_BALANCE, V4L2_CID_BACKLIGHT_COMPENSATION, 
 V4L2_CID_EXPOSURE_AUTO,
 V4L2_CID_EXPOSURE, V4L2_CID_GAIN,
 V4L2_CID_DO_WHITE_BALANCE};

__s32 controlValues_lights[NUM_CAMERAS][NUM_CONTROLS] =
{ {1, 1,
   1,
   220, 64,
   180, 0,
   2,
   0, 0x00,
   0,
   60, 100
   -100},
  {0, 0,
   1,
   220, 64,
   180, 0,
   2,
   0, 0x00,
   0,
   60, 120
   -100}
};


/*
{  { 220, 64,
     180, 2,
     1, 1,
     0, 0,
     -100, 0,
     0, 0,
     0, 0,
     0, 0,
     80},
   { 220, 64,
     220, 2,
     0, 0,
     0, 0,
     -120, 0,
     0, 0,
     0, 0,
     0, 0,
     80}
};
*/
/*
__s32 controlValues_lights[NUM_CAMERAS][NUM_CONTROLS] =
{  { true, true,
     -40, 131, false,
     0, false, false,
     128, 75, 128,
     64, 225, -43, 0,
     0, 2},
   { false, false,
     -40, 131, false,
     100, false, false,
     128, 75, 128,
     64, 225, -43, 0,
     0, 2}
};
*/
__s32 (*controlValues)[NUM_CONTROLS] = controlValues_lights;

Camera::Camera() : dumpFile(0) {
  // imageSize = IMAGE_WIDTH * IMAGE_HEIGHT * 2;
}

bool Camera::startRecording(const char *filename, uint32_t frequency_ms) {
   this->frequency_ms = frequency_ms;
   if (dumpFile != NULL) {
      fclose(dumpFile);
   }
   dumpFile = fopen(filename, "w");
   llog(INFO) << "Starting camera dump to file: " << filename << endl;
   return dumpFile != NULL;
}

void Camera::stopRecording() {
   if (dumpFile != NULL) {
      fclose(dumpFile);
      dumpFile = NULL;
   }
   llog(INFO) << "Finishing camera dump to file" << endl;
}

void Camera::writeFrame(const uint8_t*& imageTop,const uint8_t*& imageBot) {
   static Timer t;
   int topSize = 1280*960;
   int botSize = 640*480;
   if (dumpFile != NULL && imageTop != NULL && imageBot != NULL) {
      if (t.elapsed_ms() >= frequency_ms) {
         t.restart();
         llog(DEBUG3) << "Writing frame to dumpFile" << endl;
         int written = fwrite(imageTop, topSize*2, 1, dumpFile);
         llog(DEBUG3) << "wrote " << written << " top frames" << endl;
         fflush(dumpFile);
         written = fwrite(imageBot, botSize*2, 1, dumpFile);
         llog(DEBUG3) << "wrote " << written << " bot frames" << endl;
         fflush(dumpFile);
      }
   }
}
