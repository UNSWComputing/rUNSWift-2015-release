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

#include "motion/effector/AgentEffector.hpp"
#include <sys/mman.h>        /* For shared memory */
#include <fcntl.h>           /* For O_* constants */
#include <stdexcept>
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

/*-----------------------------------------------------------------------------
 * Agent effector constructor
 *---------------------------------------------------------------------------*/
AgentEffector::AgentEffector() {
   // open shared memory as RW
   shared_fd = shm_open(AGENT_MEMORY, O_RDWR, 0600);
   if (shared_fd < 0) {
      throw std::runtime_error("AgentEffector: shm_open() failed");
   }
   // map shared memory to process memory
   shared_data = (AgentData*) mmap(NULL, sizeof(AgentData),
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED, shared_fd, 0);
   if (shared_data == MAP_FAILED) {
      throw std::runtime_error("AgentEffector: mmap() failed");
   }

   llog(INFO) << "AgentEffector constructed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * Agent effector destructor
 *---------------------------------------------------------------------------*/
AgentEffector::~AgentEffector() {
   if (shared_data != MAP_FAILED) {
      munmap(shared_data, sizeof(AgentData));
   }
   if (shared_fd >= 0) {
      close(shared_fd);
   }
   llog(INFO) << "AgentEffector destroyed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * Agent effector - actuate the joints to the desired position
 *---------------------------------------------------------------------------*/
void AgentEffector::actuate(JointValues joints, ActionCommand::LED leds,
                            float sonar) {
   static bool kill_standing = false;
   uint8_t i;
   // Find the right index to write to
   // Roger: this should have been actually swapping between slots right?
   for (i = 0; i != shared_data->actuators_latest &&
        i != shared_data->actuators_read; ++i) ;
   shared_data->leds[i] = leds;
   shared_data->joints[i] = joints;
   shared_data->sonar[i] = sonar;
   std::string sayText = GET_SAYTEXT();
   int size = sizeof(shared_data->sayTexts[i]);
   strncpy(shared_data->sayTexts[i], sayText.c_str(), size);
   shared_data->sayTexts[i][size - 1] = 0;
   shared_data->actuators_latest = i;

   // effector needs to set standing to false if we got standing
   // we need to wait one cycle in case standing was set after AgentTouch is run
   shared_data->standing = kill_standing;
   if (kill_standing) {
      kill_standing = false;
      shared_data->standing = false;
   } else {
      kill_standing = true;
   }
}
