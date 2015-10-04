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

#include <cstdlib>
#include <iostream>
#include <pthread.h>
#include <queue>
#include <sys/types.h>
#include <sys/wait.h>
#include <string>
#include <unistd.h>
#include "utils/Timer.hpp"


//#include <boost/thread.hpp>
//#include <boost/thread/condition_variable.hpp>
//#include <boost/thread/mutex.hpp>

class Speech
{
   public:

      static Speech& instance() {
         static Speech instance;
         return instance;
      }

      void add(std::string text) {
         pthread_mutex_lock(&mutex);
         sayText = text;
         pthread_mutex_unlock(&mutex);
      }

      std::string pop() {
         pthread_mutex_lock(&mutex);
         std::string returnText = sayText;
         sayText = "";
         pthread_mutex_unlock(&mutex);
         return returnText;
      }

   private:

      Speech() {
         int ret = pthread_mutex_init(&mutex, NULL);
         if (ret) std::cout << "failed to create say mutex" << std::endl;
      };
      
      // Declare copy constructors privately and don't implement them
      // This is to ensure singleton class
      Speech(Speech const& copy);
      Speech& operator=(Speech const& copy);

      std::string sayText;
      pthread_mutex_t mutex;
};

const inline void SAY(std::string text, bool blocking = false) {
   (Speech::instance()).add(text);
}

const inline std::string GET_SAYTEXT() {
    return (Speech::instance()).pop();
}
