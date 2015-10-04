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

#include "utils/SPLDefs.hpp"
#include "utils/angles.hpp"

/* Useful, but not in math.h */
#define PI_8 (M_PI_4 / 2.0)

/* Don't allow robot to exceed these coordinates */
#define FIELD_X_CLIP ((FIELD_LENGTH / 2.0f) + FIELD_LENGTH_OFFSET / 2.0f)
#define FIELD_Y_CLIP ((FIELD_WIDTH / 2.0f) + FIELD_WIDTH_OFFSET / 2.0f)

/* Max number of other robots on the field */
#define MAX_ROBOT_OBSTACLES 7

#define OFFNAO_TEAM TEAM_BLUE

/* Dimensions that are filtered by the KF */
typedef enum {
   X = 0,
   Y,
   THETA,
   STATE_VEC_DIM
} StateVector;

// FIELD_LENGTH away -- more likely to be on the field, more dynamic!!!
#define FIELD_DIAGONAL \
   sqrt(SQUARE(FULL_FIELD_WIDTH) + SQUARE(FULL_FIELD_LENGTH))

#define IS_VALID_DIST(dist) \
   (dist <= FIELD_DIAGONAL)

// Number of domensions in the main distribution. Includes robot pose, ball pos, ball vec, and
// robot poses of our 4 teammates.
#define MAIN_DIM 19

// Number of dimensions in the shared distribution. Includes the robot pose, ball pos, ball vel.
#define SHARED_DIM 7

/* Returns the minimum heading between 2 angles given in radians */
/* TODO(yanjinz) refactorme! */
static inline float minHeadingDiff(float thetaA, float thetaB) {
   float minTheta = MIN_ANGLE_360(RAD2DEG(thetaA),
                                  RAD2DEG(thetaB));
   if (minTheta < -180) minTheta += 360;
   if (minTheta > 180) minTheta -= 360;
   return minTheta;
}

#define LOCALISATION_DEBUG false