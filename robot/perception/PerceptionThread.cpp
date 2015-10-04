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

#include <pthread.h>
#include <ctime>
#include <utility>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "perception/PerceptionThread.hpp"
#include "perception/behaviour/SafetySkill.hpp"
#include "soccer.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "thread/Thread.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/bind.hpp>

using namespace std;
using namespace boost;

PerceptionThread::PerceptionThread(Blackboard *bb)
   : Adapter(bb), 
     kinematicsAdapter(bb),
     visionAdapter(bb),
     localisationAdapter(bb),
     behaviourAdapter(bb)
{
   dumper = NULL;

   releaseLock(serialization);
   //uint8_t const* currentFrame = readFrom(vision, currentFrame);
   uint8_t const* topFrame = readFrom(vision, topFrame);
   uint8_t const* botFrame = readFrom(vision, botFrame);
   size_t ret;

   if (topFrame != NULL && botFrame != NULL) {
      string file = "/home/nao/crashframe-" +
                    boost::lexical_cast<string>(time(NULL)) + ".yuv";
      FILE *errorFrameFile = fopen(file.c_str(), "w");
      ret = fwrite(topFrame, 640 * 480 * 2, 1, errorFrameFile);
      ret = fwrite(botFrame, 640 * 480 * 2, 1, errorFrameFile);
      fclose(errorFrameFile);
      file = "/usr/bin/tail -n 200 /var/volatile/runswift/*/Perception > "
             + file + ".log";
      ret = std::system(file.c_str());
   }


   readOptions(bb->config);
   writeTo(thread, configCallbacks[Thread::name],
           boost::function<void(const boost::program_options::variables_map &)>
           (boost::bind(&PerceptionThread::readOptions, this, _1)));
}

PerceptionThread::~PerceptionThread() {
   llog(INFO) << __PRETTY_FUNCTION__ << endl;
   writeTo(thread, configCallbacks[Thread::name], boost::function<void(const boost::program_options::variables_map &)>());
}

void PerceptionThread::tick() {
   llog(DEBUG1) << "Perception.. ticking away" << endl;
   Timer t1;
   Timer t;
   kinematicsAdapter.tick();
   uint32_t kinematics = t.elapsed_us();
   llog(VERBOSE) << "kinematics tick took " << kinematics << endl;
   if (t.elapsed_us() > 30000) {
      llog(ERROR) << "Kinematics took " << t.elapsed_us() << " us" << std::endl;
   }

   t.restart();
   if (blackboard->config["debug.vision"].as<bool>()) {
      visionAdapter.tick();
   }
   uint32_t vision = t.elapsed_us();
   llog(VERBOSE) << "vision tick took " << vision << endl;
   if (t.elapsed_us() > 30000) {
      llog(ERROR) << "Vision took " << t.elapsed_us() << " us" << std::endl;
   }

   t.restart();
   localisationAdapter.tick();
   uint32_t localisation = t.elapsed_us();
   llog(VERBOSE) << "localisation tick took " << localisation << endl;
   if (t.elapsed_us() > 30000) {
      llog(ERROR) << "Localisation took " << t.elapsed_us() << " us" << std::endl;
   }

   t.restart();
   pthread_yield();

   if (time(NULL) - readFrom(remoteControl, time_received) < 60) {
      /* we have fresh remote control data, use it */
	  int writeBuf = (readFrom(behaviour, readBuf) + 1) % 2;
	  //writeTo(behaviour, request[writeBuf], safetySkill.wrapRequest(readFrom(remoteControl, request), readFrom(motion, sensors)));
	  writeTo(behaviour, request[writeBuf], readFrom(remoteControl, request));
	  writeTo(behaviour, readBuf, writeBuf);
   } else if (blackboard->config["debug.behaviour"].as<bool>()) {
      /* run behaviour */
      behaviourAdapter.tick();
   }

   uint32_t behaviour = t.elapsed_us();
   llog(VERBOSE) << "behaviour tick took " << behaviour << endl;
   if (t.elapsed_us() > 30000) {
      llog(ERROR) << "Behaviour and perception yield took " << t.elapsed_us() << " us" << std::endl;
   }   
   uint32_t total = t1.elapsed_us();
   llog(VERBOSE) << "perception took " << total << endl;

   writeTo(perception, kinematics, kinematics);
   writeTo(perception, vision, vision);
   writeTo(perception, localisation, localisation);
   writeTo(perception, behaviour, behaviour);
   writeTo(perception, total, total);

   if (dumper) {
      if (dump_timer.elapsed_us() > dump_rate) {
         dump_timer.restart();
         try {
            dumper->dump(blackboard);
         } catch(const std::exception &e) {
            attemptingShutdown = true;
         }
      }
   }
}

void PerceptionThread::readOptions(const boost::program_options::variables_map& config) {
   const string &e = config["vision.camera_controls"].as<string>();
   vector<string> vs;
   split(vs, e, is_any_of(",;"));
   for (vector<string>::const_iterator ci = vs.begin(); ci != vs.end(); ++ci) {
      vector<string> nv;
      split(nv, *ci, is_any_of(":"));
      if (nv.size() != 3)
         llog(ERROR) << "controls should be cam:control_id:value" << endl;
      else{
    	 if(strtol(nv[0].c_str(),NULL,10) == 0){
    		 visionAdapter.V.camera->setCamera(BOTTOM_CAMERA);
    	 }else{
    		 visionAdapter.V.camera->setCamera(TOP_CAMERA);

    	 }
    	 if(strtoul(nv[1].c_str(), NULL, 10) == 0)
    		 visionAdapter.V.camera->setControl(22,1);
    	 if(strtoul(nv[1].c_str(), NULL, 10) == 17)
    		 visionAdapter.V.camera->setControl(22,0);
        if(strtoul(nv[1].c_str(), NULL, 10) == 19)
         visionAdapter.V.camera->setControl(22,0);
         visionAdapter.V.camera->setControl(strtoul(nv[1].c_str(), NULL, 10),
                                            strtol (nv[2].c_str(), NULL, 10));
      }

   }

   const string &dumpPath = config["debug.dump"].as<string>();
   dump_rate = config["vision.dumprate"].as<int>() * 1000;
   if (dumpPath == "") {
      delete dumper;
      dumper = NULL;
   } else {
      if (! dumper || dumper->getPath() != dumpPath) {
         delete dumper;
         dumper = new PerceptionDumper(dumpPath.c_str());
      }
   }

   OffNaoMask_t dumpMask = config["debug.mask"].as<int>();
   blackboard->write(&(blackboard->mask), dumpMask);
}

