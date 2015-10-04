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

/**
 * the type of what this is actually receiving.
 * DO NOT CHANGE THIS WITHOUT FINISHING ALL THE TODOS IN OFFNAO TRANSMITTER
 * historically used only by the off-nao transmitter.  TODO(jayen) refactor to
 * have separate types for command masks and data masks.
 */
typedef uint64_t OffNaoMask_t;

enum {
   BLACKBOARD_MASK      = 0x0000000000000001ull,
   SALIENCY_MASK        = 0x0000000000000002ull,
   RAW_IMAGE_MASK       = 0x0000000000000004ull,
   PARTICLE_FILTER_MASK = 0x0000000000000008ull,
   ROBOT_FILTER_MASK    = 0x0000000000000010ull,
   INITIAL_MASK         = 0x0000000000000003ull,
   ALL_MASKS            = 0x000000000000003Full,
   LANDMARKS_MASK       = 0x0000000000000020ull,

   COMMAND_MASK         = 0x8000000000000000ull,
   TO_NAO_MASKS         = 0x8000000000000000ull
};
