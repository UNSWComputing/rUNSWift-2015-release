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
#include <boost/serialization/version.hpp>
#include <boost/serialization/serialization.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <utility>

#include <stdint.h>

#include "perception/vision/CameraDefs.hpp"
#include "types/Point.hpp"
#include "types/RRCoord.hpp"
#include "types/JointValues.hpp"
#include "types/XYZ_Coord.hpp"

/**
 * The Pose class contains precomputed kinematic data that is useful
 * to other modules. (Currently only vision).
 *
 * The idea is that the kinematics module computes the full kinematic chain
 * and then stores the resulting matrix in this Pose class.
 *
 * Vision then queries the Pose class and the cached results are used each
 * time.
 */
class Pose {
   public:
      explicit Pose(
         boost::numeric::ublas::matrix<float> topCameraToWorldTransform,
         boost::numeric::ublas::matrix<float> botCameraToWorldTransform,
         boost::numeric::ublas::matrix<float> neckToWorldTransform,
         std::pair<int, int> horizon);

      Pose();

      /**
       *  Returns a pair for the horizon.
       *  pair.first is the y position of the horizon at x = 0
       *  pair.second is the y position of the horizon at x = 640
       */
      std::pair<int, int> getHorizon() const;

      XYZ_Coord robotRelativeToNeckCoord(RRCoord coord, int h) const;
      
      /**
       * Returns the robot relative coord for a given pixel at a particular
       * height.
       *
       * @param x pixel
       * @param y pixel
       * @param h of intersection plane.
       */
      RRCoord imageToRobotRelative(int x, int y, int h) const;

      /**
       * Returns the robot relative coord for a given pixel at a particular
       * height.
       *
       * @param p point in image space
       * @param h of intersection plane.
       */
      RRCoord imageToRobotRelative(Point p, int h = 0) const;

      Point imageToRobotXY(const Point &image, int h = 0) const;
      Point robotToImageXY(Point robot, int h = 0) const;

      /* Returns a pointer to the exclusion arrays used by vision */
      const int16_t *getTopExclusionArray() const;
      int16_t *getTopExclusionArray();
      const int16_t *getBotExclusionArray() const;
      int16_t *getBotExclusionArray();

      const boost::numeric::ublas::matrix<float>
            getC2wTransform(bool top = true) const;

      boost::numeric::ublas::matrix<float> topCameraToWorldTransform;
      boost::numeric::ublas::matrix<float> botCameraToWorldTransform;
      boost::numeric::ublas::matrix<float> topWorldToCameraTransform;
      boost::numeric::ublas::matrix<float> botWorldToCameraTransform;
      
      boost::numeric::ublas::matrix<float> neckToWorldTransform;
      boost::numeric::ublas::matrix<float> worldToNeckTransform;

      static const uint16_t EXCLUSION_RESOLUTION = 100;
   private:
      boost::numeric::ublas::matrix<float> origin, zunit;
      boost::numeric::ublas::matrix<float> topCOrigin, botCOrigin;
      boost::numeric::ublas::matrix<float> topToFocus, botToFocus;
      boost::numeric::ublas::matrix<float> topWorldToCameraTransformT;
      boost::numeric::ublas::matrix<float> botWorldToCameraTransformT;

      void makeConstants();

      std::pair<int, int> horizon;
      int16_t topExclusionArray[EXCLUSION_RESOLUTION];
      int16_t botExclusionArray[EXCLUSION_RESOLUTION];

#ifndef SWIG
      BOOST_SERIALIZATION_SPLIT_MEMBER();
#endif

      friend class boost::serialization::access;
      template<class Archive>
      void serializeMembers(Archive &ar, const unsigned int version);

      template<class Archive>
      void save(Archive &ar, const unsigned int version) const;

      template<class Archive>
      void load(Archive &ar, const unsigned int version);
};

BOOST_CLASS_VERSION(Pose, 2);

#include "Pose.tcc"

