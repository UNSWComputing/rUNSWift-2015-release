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

#include <string>

#include "perception/vision/NaoCamera.hpp"

/**
 * NaoCameraV4 provides methods for the vision module to interact with
 * the nao camera
 *
 * Changes from V3
 *    Camera manages only a single device, not two devices over the i2c bus.
 *    setCamera modifies the Vision::camera reference.
 */
class NaoCameraV4 : public NaoCamera {
  public:
   /**
    * Constructor
    * opens the device, calibrates it, and sets it up for streaming
    *
    * @param filename the device or file to get images from
    * @param method the io method to use
    * @see IOMethod
    * @param format the format of the image
    * @see AL::kVGA
    * @param initialCamera which camera to start with (top or bottom)
    * @see WhichCamera
    */
   NaoCameraV4(Blackboard *blackboard, const char *filename = "/dev/video0",
             const IOMethod method = IO_METHOD_MMAP,
             const int format = AL::k960p,
             const WhichCamera initialCamera = TOP_CAMERA);

   /**
    * Destructor
    * closes the device
    */
   virtual ~NaoCameraV4();

   /**
    * Set Vision::camera = Vision::top_camera.
    */
   bool setCamera(WhichCamera whichCamera);

   /**
    * Return which camera is currently in use.
    */
   WhichCamera getCamera();
};
