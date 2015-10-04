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

#include <utility>
#include <vector>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <perception/kinematics/Parameters.hpp>
#include <perception/kinematics/Pose.hpp>
#include <perception/vision/Camera.hpp>

#include <utils/matrix_helpers.hpp>
#include <types/RRCoord.hpp>
#include <types/JointValues.hpp>
#include <types/SensorValues.hpp>

#include "CKF.hpp"

#define CAMERA_DH_CHAIN_LEN 12
#define HEAD_DH_CHAIN_LEN 3
#define ARM_DH_CHAIN_LEN 14
#define LEG_DH_CHAIN_LEN 19

class Kinematics {
   public:
      friend class KinematicsAdapter;
      friend class KinematicsCalibrationSkill;
      friend class CameraPoseTab;
      Kinematics();

      enum Link {
         FOOT = 0,
         BODY = 8,
         CAMERA = 12,
         NECK = 9
      };

      enum Chain {
         LEFT_CHAIN = 0,
         RIGHT_CHAIN = 1
      };

      /* Creates a Pose with the current evaluated DH Chain */
      Pose getPose();

      void updateDHChain();

      boost::numeric::ublas::matrix<float>
      evaluateDHChain(Link from, Link to, Chain foot, bool top = true);

      boost::numeric::ublas::matrix<float> evaluateMassChain();

      boost::numeric::ublas::matrix<float>
      createBodyToFootOnGroundTransform(Chain foot, boost::numeric::ublas::matrix<float> b2f);

      boost::numeric::ublas::matrix<float>
      createCameraToFootTransform(Chain foot, bool top);
      
      boost::numeric::ublas::matrix<float>
      createNeckToFootTransform(Chain foot);

      boost::numeric::ublas::matrix<float>
      createFootToWorldTransform(Chain foot, bool top = true);

      boost::numeric::ublas::matrix<float>
      createCameraToWorldTransform(Chain foot, bool top);
      
      boost::numeric::ublas::matrix<float>
      createNeckToWorldTransform(Chain foot);

      boost::numeric::ublas::matrix<float>
      createWorldToFOVTransform(
         const boost::numeric::ublas::matrix<float> &m);

      boost::numeric::ublas::matrix<float>
      fovToImageSpaceTransform(
         const boost::numeric::ublas::matrix<float> &transform,
         const boost::numeric::ublas::matrix<float> &point, bool top);

      void determineBodyExclusionArray(
         const boost::numeric::ublas::matrix<float> &m,
         int16_t *points, bool top);

      Chain determineSupportChain();

      void setSensorValues(SensorValues sensorValues);

      std::pair<int, int> calculateHorizon(
         const boost::numeric::ublas::matrix<float> &m);
   private:
      // CKF is not really tuned and not being used at the moment
      //CKF ckf;
      SensorValues sensorValues;
      Chain supportChain;

      boost::numeric::ublas::matrix<float> transformLTop[CAMERA_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformLBot[CAMERA_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformRTop[CAMERA_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformRBot[CAMERA_DH_CHAIN_LEN];
      
      boost::numeric::ublas::matrix<float> cameraPanInverseHack;

      // DH matrices for mass
      boost::numeric::ublas::matrix<float> transformHB[HEAD_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformRAB[ARM_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformLAB[ARM_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformRFB[LEG_DH_CHAIN_LEN];
      boost::numeric::ublas::matrix<float> transformLFB[LEG_DH_CHAIN_LEN];

      // Contains the masses and centre position of each joint
      std::vector<float> masses;
      std::vector<boost::numeric::ublas::matrix<float> > massesCom;

      Parameters<float> parameters;

      std::vector<std::vector<boost::numeric::ublas::matrix<float> > >
      bodyParts;
};

