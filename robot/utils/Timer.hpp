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

/* A timer class
 **/

#pragma once

#include <sys/time.h>
#include <stdint.h>
#include <time.h>


//  timer  -------------------------------------------------------------------//

//  A timer object measures elapsed time.

//  It is recommended that implementations measure wall clock rather than CPU
//  time since the intended use is performance measurement on systems where
//  total elapsed time is more important than just process or CPU time.

//  Warnings: The maximum measurable elapsed time may well be only 596.5+ hours
//  due to implementation limitations.  The accuracy of timings depends on the
//  accuracy of timing information provided by the underlying platform, and
//  this varies a great deal from platform to platform.

class Timer {
   public:
      Timer() {
         // _start_time = std::clock();
         gettimeofday(&timeStamp, NULL);
      }  // postcondition: elapsed()==0
      void  restart() {
         // _start_time = std::clock();
         gettimeofday(&timeStamp, NULL);
      }  // post: elapsed()==0

      float sec(const timeval& t) {
         return t.tv_sec + t.tv_usec / 1000000.0;
      }
      float msec(const timeval& t) {
         return t.tv_sec * 1000 + t.tv_usec / 1000.0;
      }

      float usec(const timeval& t) {
         return t.tv_sec * 1000000 + t.tv_usec;
      }

      /* return elapsed time in seconds */
      float elapsed() {
         timeval tmp;
         gettimeofday(&tmp, NULL);
         return sec(tmp) -
                sec(timeStamp);
      }

      uint32_t elapsed_ms() {
         timeval tmp;
         gettimeofday(&tmp, NULL);
         return msec(tmp) - msec(timeStamp);
      }

      uint32_t elapsed_us() {
         timeval tmp;
         gettimeofday(&tmp, NULL);
         return usec(tmp) - usec(timeStamp);
      }

      /* return estimated maximum value for elapsed() */
      float elapsed_max() {
         /* Portability warning: elapsed_max() may return too high a value on systems
          * where std::clock_t overflows or resets at surprising values.
          */
         return elapsed();
      }

      /* return minimum value for elapsed() */
      float elapsed_min() {
         return elapsed();
      }

   private:
      timeval timeStamp;
};

