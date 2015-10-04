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

#include "perception/vision/rle.hpp"

namespace rle {
   uint16_t encode(uint8_t *output, Colour *saliency) {
      Colour lastcolour = saliency[0];
      output[0] = (uint8_t)lastcolour;
      uint16_t outpos = 0;
      uint8_t run = 1;
      for (uint16_t i = 0;
            // File isn't used, so changed SALIENCY_DENSITY
            // to BOT_SALIENCY_DENSITY to make it compile :)
            i < (IMAGE_COLS/BOT_SALIENCY_DENSITY)*
                (IMAGE_ROWS/BOT_SALIENCY_DENSITY);
            ++i) {
         if (lastcolour != saliency[i] || run == 255) {
            // different colour, or number too big, start a new run
            output[outpos++] = (uint8_t)lastcolour;
            output[outpos++] = run;
            lastcolour = saliency[i];
            run = 1;
         } else {
            // the run continues
            ++run;
         }
      }
      output[outpos++] = (uint8_t)lastcolour;
      output[outpos++] = run;
      return outpos;
   }

   void decode(Colour *saliency, uint8_t *input, uint16_t size) {
      uint16_t inpos = 0;
      uint16_t saliencypos = 0;
      while (inpos < size) {
         Colour c = (Colour)input[inpos++];
         uint8_t run = input[inpos++];
         for (uint8_t i = 0; i < run; ++i) {
            saliency[saliencypos++] = c;
         }
      }
   }
}

