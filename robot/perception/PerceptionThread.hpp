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
#include "perception/vision/VisionAdapter.hpp"
#include "perception/localisation/LocalisationAdapter.hpp"
#include "perception/behaviour/BehaviourAdapter.hpp"
#include "perception/kinematics/KinematicsAdapter.hpp"
#include "perception/dumper/PerceptionDumper.hpp"
#include "blackboard/Adapter.hpp"

/* Wrapper class for vision, localisation and behaviour threads */
class PerceptionThread : Adapter {
   public:
      /* Constructor */
      PerceptionThread(Blackboard *bb);
      /* Destructor */
      ~PerceptionThread();

      /* Read from global options after update */
      void readOptions(const boost::program_options::variables_map &config);

      /* One cycle of this thread */
      void tick();

   private:
      KinematicsAdapter kinematicsAdapter;
      VisionAdapter visionAdapter;
      //SonarAdapter sonarAdapter;
      LocalisationAdapter localisationAdapter;
      BehaviourAdapter behaviourAdapter;

      PerceptionDumper *dumper;
      Timer dump_timer;
      unsigned int dump_rate;
};

