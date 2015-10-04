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

/** Field dimensions in mm
 *  NOTE: unless explicitly specified all dimensions includes line width
 */

#define FIELD_LINE_WIDTH 50


/* distance from field line to edge of field excluding line width
 * the values are for our current field, those commented out are what's
 * specified in the spec */

/* mexico field lines
#define FIELD_LENGTH_OFFSET 740
#define FIELD_WIDTH_OFFSET 680
#define OFFNAO_FIELD_LENGTH_OFFSET 675
#define OFFNAO_FIELD_WIDTH_OFFSET 675
*/

//#define FIELD_LENGTH_OFFSET 725
//#define FIELD_WIDTH_OFFSET 710

/* assumes that we are attacking the yellow goals */
// #define LEFT_FIELD_LENGTH_OFFSET 490
// #define RIGHT_FIELD_WIDTH_OFFSET 80  // 675

// Will need to re-measure field when it is built
#ifndef USING_SMALL_FIELD
   #define ROBOTS_PER_TEAM 5
   
   /** Field line dimensions */
   #define FIELD_WIDTH 6000
   #define FIELD_LENGTH 9000
   
   #define FIELD_LENGTH_OFFSET 755
   #define FIELD_WIDTH_OFFSET 675
   
   #define OFFNAO_FIELD_WIDTH_OFFSET 680
   #define OFFNAO_FIELD_LENGTH_OFFSET 680

   /** Goal box */
   #define GOAL_BOX_WIDTH 2200
   #define GOAL_BOX_LENGTH 600
   
   /** MARKERS -- white cross on the field*/
   #define MARKER_DIMENSIONS 100 /* i.e. dimensions of square fitted around it */
   #define DIST_GOAL_LINE_TO_MARKER 1300 /* to middle of closest marker */
   #define MARKER_CENTER_X (FIELD_LENGTH / 2 - DIST_GOAL_LINE_TO_MARKER)
   
   /** Center Circle */
   #define CENTER_CIRCLE_DIAMETER 1500
   
   /** Goal Posts */
   #define GOAL_POST_DIAMETER 100
   #define GOAL_BAR_DIAMETER 100  // Double check this once field is built
   #define GOAL_POST_HEIGHT 800 // Measured from the bottom of the crossbar to the ground
   
   #define GOAL_SUPPORT_DIAMETER 46
   #define GOAL_WIDTH 1600 /* top view end-to-end from middle of goal posts */
   #define GOAL_DEPTH 500 /* Measured from the front edge of the crossbar to the centre of the rear bar */
   
   //////////////////////////////////////////////////////////////
   
   // May need to define white support bar dimensions for field lines

#else
   #define ROBOTS_PER_TEAM 4

   /** Field line dimensions */
   #define FIELD_WIDTH 4010
   // #define FIELD_WIDTH 4000 pre-mexico
   #define FIELD_LENGTH 6000
   
   #define FIELD_LENGTH_OFFSET 370
   #define FIELD_WIDTH_OFFSET 170
   #define OFFNAO_FIELD_WIDTH_OFFSET 675
   #define OFFNAO_FIELD_LENGTH_OFFSET 675

   /** Goal box */
   #define GOAL_BOX_WIDTH 2210
   #define GOAL_BOX_LENGTH 600
   
   /** MARKERS -- white cross on the field*/
   #define MARKER_DIMENSIONS 100 /* i.e. dimensions of square fitted around it */
   #define DIST_GOAL_LINE_TO_MARKER 1825 /* to middle of closest marker */
   
   /** Center Circle */
   #define CENTER_CIRCLE_DIAMETER 1200
   #define MARKER_CENTER_X 1200
   
   /** Goal Posts */
   #define GOAL_POST_DIAMETER 100
   #define GOAL_BAR_DIAMETER 50
   #define GOAL_POST_HEIGHT 800
   #define GOAL_WIDTH 1410 /* top view end-to-end from middle of goal posts */
   #define GOAL_SUPPORT_DIMENSIONS 400 /* equilateral triangle */
#endif

/** Field dimensions including edge offsets */
#define FULL_FIELD_WIDTH (FIELD_WIDTH + (FIELD_WIDTH_OFFSET * 2))
#define OFFNAO_FULL_FIELD_WIDTH (FIELD_WIDTH + (OFFNAO_FIELD_WIDTH_OFFSET * 2))
#define FULL_FIELD_LENGTH (FIELD_LENGTH + (FIELD_LENGTH_OFFSET * 2))
#define OFFNAO_FULL_FIELD_LENGTH (FIELD_LENGTH + (OFFNAO_FIELD_LENGTH_OFFSET * 2))

/** The field coordinate system in mm and radiance (rad)
 *  X -- is along the length of the field, +ve towards opponent's goal
 *  Y -- is along the width of the field, +ve towards the left hand side
 *  0 rad -- facing straight towards opponent's goal at origin
 *  radiance is calculated counter clock-wise
 *  NOTE: we use -PI, not PI for 360 degrees
 */

/** (0,0) is the center of the center circle */
// This may need changing
#define UNKNOWN_COORD 10000
#define UNKNOWN_HEADING 4.0

/** Ball Dimensions */
#define BALL_RADIUS 28

/* Post positions in AbsCoord*/
#define GOAL_POST_ABS_X (FIELD_LENGTH / 2)
#define GOAL_POST_ABS_Y (GOAL_WIDTH / 2)

