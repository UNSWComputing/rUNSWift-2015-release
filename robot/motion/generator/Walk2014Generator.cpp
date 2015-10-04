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

/**
 * Walk2014Generator.cpp
 * BH 18th Jan 2014
 * The period of each foot-step is set by T. T generates the forcing function by alternatively lifting each foot
 * Control:
 * The change of support foot is driven by the ZMP switching sign, but must be > T/2. If > 3*T we switch to try revive
 * The front-back sway is controlled by ankle tilts proportional to the GyroY
 * The user specifies forward(m), left(m), turn(radians) to activate the walk.
 * If these values are all zero the robot stands with the motors turned off.
 * The CoM is moved forward when walking to position it more over the center of the foot
 */

#include "motion/generator/Walk2014Generator.hpp"
#include <cmath>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/speech.hpp"

#define SHIFT_PERIOD 2.4 // time to shift weight on to one leg
#define SHIFT_END_PERIOD 2.5 // time to shift weight back from one leg
#define KICK_STEP_HEIGHT 0.045 // how far to lift kicking foot
#define BACK_PHASE 0.75 // time to move kick foot back
#define MIN_KICK_PHASE 0.5 // time to swing kick foot
#define MAX_KICK_PHASE 0.2 // time to swing kick foot
#define THROUGH_PHASE 0.3 // time to hold kick foot
#define END_PHASE 0.6 // time to return kick foot to zero position
#define TURN_THRESHOLD DEG2RAD(25) // min angle for triggering turn step
#define TURN_PERIOD 0.75 // time to perform turn step
#define TURN_STEP_HEIGHT 0.024 // how far to lift turning foot
#define TURN_LEAN DEG2RAD(20.25) // sideways lean while turning
#define TURN_SCALE 1.15 // how much to scale up turn
#define EPSILON 0.01       //10 mm
#define TURN_EPSILON 0.05  //2.8 degrees
#define KICK_LEAN_V4 22.2
#define KICK_LEAN_V5 21.8

//mm value of safety barrier
#define FOOT_AVOID_RADIUS 75
#define TURN_CLIPPING 0.10 //radians
#define NINETY 1.51
#define FORTY_FIVE 1.51 / 2
#define TT_DEGREE FORTY_FIVE / 2

using boost::program_options::variables_map;
using namespace std;
using namespace Joints;
using namespace Sensors;

const float MM_PER_M  = 1000.0;                            // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;                    // time in seconds to crouch
const float COM_OFFSET = 0.01;                             // center of mass offset in x direction in meters
const float FORWARD_CHANGE = 0.15;                          // was 0.08. max change of 100mm/sec at each leg change to ratchet up/down
const float LEFT_CHANGE = 0.2;
const float STAND_HIP_HEIGHT = 0.248;                      // for tall power saving stand, matches INITIAL action command
const float KNEE_PITCH_RANGE = DEG2RAD(60);                // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .23; //.25;                 // seconds to walk one step, ie 1/2 walk cycle
const float WALK_HIP_HEIGHT = .23;                         // Walk hip height - seems to work from .2 to .235
const float MAX_FORWARD = .3;                              // meters
const float MAX_LEFT = .2;                                 // meters
const float MAX_TURN = .80;                                // radians
const float BASE_LEG_LIFT = 0.010;                         // meters

float KICK_LEAN = KICK_LEAN_V5;

void ellipsoidClampWalk(float &forward, float &left, float &turn);
float evaluateWalkVolume(float x, float y, float z);

void checkNaN(float n, string s){
   if(n != n){
	   cout << s << " is NAN" << endl;
   }
}

Walk2014Generator::Walk2014Generator(Blackboard *bb)
   :t(0.0f), z(0.0f), PI(3.1415927), blackboard(bb) {
   initialise();
   llog(INFO) << "Walk2014Generator constructed" << std::endl;

}

Walk2014Generator::~Walk2014Generator() {
   llog(INFO) << "Walk2014Generator destroyed" << std::endl;
}

void Walk2014Generator::initialise() {
	llog(INFO) << "Walk2014 initializing" << endl;
	dt = 0.01;                                             // 100 Hz motion thread
	t = 0.0;                                               // initialise timers (in seconds)
	timer = 0.0;                                           // timer to crouch to walking height
	globalTime = 0;                                        // use for diagnostic purposes only
	T = BASE_WALK_PERIOD;                                  // seconds - the period of one step in a two step walk cycle
	stopping = false;                                      // legacy code for stopping robot?
	stopped = true;                                        // legacy code for stopped robot?
	leftL = leftR = lastLeft = left = 0.0;                 // Side-step for left, right foot, and (last) left command in meters
	turnRL = turnRL0 = 0;                                  // Initial turn variables for feet
	forwardL  = forwardR  = 0.0;                           // forward step per for left and right foot in meters
	forwardR0 = forwardL0 = 0;                             // initial last positions for left and right feet keep constant during next walk step
	forward = lastForward = 0.0;                           // Current and previous forward value
	shoulderPitchL = shoulderPitchR = 0;                   // arm swing while walking
	shoulderRollL  = shoulderRollR  = 0;                   // Not used yet
	hiph = hiph0 = STAND_HIP_HEIGHT;                       // make robot stand initially based on Stand command
	foothL = foothR = 0;                                   // robots feet are both on the ground initially
	thigh = Limbs::ThighLength/MM_PER_M;                   // thigh length in meters
	tibia = Limbs::TibiaLength/MM_PER_M;                   // tibia length in meters
	ankle = Limbs::FootHeight/MM_PER_M;                    // height of ankle above ground
	nextFootSwitchT = 0.0;                                 // next time-point to switch support foot (in seconds)
	stiffness = 0.9;                                       // initial motor stiffness
	walk2014Option = NONE;                                 // initial walk 2014 option
	walkState = NOT_WALKING;                               // initial walkState
	supportFoothasChanged = false;                         // triggers support foot change actions
	comOffset = 0;                                         // Center of Mass offset in sagittal plane used to spread weight along feet in x-dir
	prevTurn = prevForwardL = prevForwardR = 0;            // odometry
	prevLeftL = prevLeftR = 0;                             // odometry
	exactStepsRequested = false;

   // Kick specific
   kickT = 0;
   kickPhase = MAX_KICK_PHASE;
   rock = 0;
   kneePitchL = kneePitchR = lastKneePitch = 0;
   anklePitchL = anklePitchR = 0;
   lastKickForward = 0;
   lastSide = 0;
   lastKickTime = T;
   dynamicSide = 0.0f;
   turnAngle = 0;
   lastTurn = 0;
   lastTurn = 0;
   motionOdometry.reset();
}

int onLeft = 0;
	int onRight = 0;
const int footOffset = 50;
const int toeOffset = 60;
const int toeRadius = 40;
//const int footAvoidRadius = 50;
const int footAvoidRadius = 150;
bool intersects(FootInfo& f, Point& p)
{
	//cout << "Foot will land at " << p.x() << " " << p.y() << " rectangle is at: "
		//	<< box.p.x() << " " << box.p.y() << " and " << box.rrp.x() << " " << box.rrp.y() << endl;
	//immediately check if the point lies within the rectangle
	//beware of coordinate system change
	if (p.x() >= f.robotBounds.a.x() && p.x() <= f.robotBounds.b.x()){
		if (p.y() <= f.robotBounds.a.y() && p.y() >= f.robotBounds.b.y()){
			return true;
		}
	}

	Point circleDist(abs(p.x() - f.robotBounds.a.x()), abs(p.y() - f.robotBounds.a.y()));
    int rectWidth = abs(f.robotBounds.b.x() - f.robotBounds.a.x());
    int rectHeight = abs(f.robotBounds.b.y() - f.robotBounds.a.y());
    if (circleDist.x() > (rectWidth/2 + toeRadius)) { return false; }
    if (circleDist.y() > (rectHeight/2 + toeRadius)) { return false; }

    if (circleDist.x() <= (rectWidth/2)) { return true; }
    if (circleDist.y() <= (rectHeight/2)) { return true; }

    double cornerDistance = (circleDist.x() - rectWidth/2)*(circleDist.x() - rectWidth/2) +
                         (circleDist.y() - rectHeight/2)*(circleDist.y() - rectHeight/2);

    return cornerDistance <= toeRadius*toeRadius;
}

void Walk2014Generator::avoidFeet(float &forward, float &left, float &turn, BodyModel& bodyModel) {
	//Given a set of clamped walk parameters check if any of the feet would result in crash
	vector<FootInfo> boxes	 = readFrom(vision, feetBoxes);

	bool clipped = false;
	int oldForward = forward;
	int oldLeft = left;
	if (forward != 0 || left != 0 || turn != 0){
		Point toeFall;

		if (bodyModel.isLeftPhase){
			toeFall = Point(forward + toeOffset, left + footOffset);
		} else {
			toeFall = Point(forward + toeOffset, left - footOffset);
		}
		while (true){
			bool inBox = false;
			for (vector<FootInfo>::iterator it = boxes.begin(); it != boxes.end(); ++it){
				if (intersects(*it, toeFall)){
					inBox = true;
					break;
				}
			}

			if (inBox){
			//cout << forward << " " << left << " cutting original forward and left by 0.7" << endl;
				forward *=0.7;
				left *= 0.7;
				clipped = true;
				if (bodyModel.isLeftPhase){
					toeFall = Point(forward + toeOffset, left + footOffset);
				} else {
					toeFall = Point(forward + toeOffset, left - footOffset);
				}
				if (left <= 5  && forward <= 5){
					break;
				}
			} else {
				break;
			}
			//need to chop forward and left components from the walk to be outside of the box
		}
		if (clipped){
			cout << "### Forward " << oldForward << " -> " << forward << " Left " << oldLeft << " -> " << left << endl;
			SAY("I c Feet");
		}
	}
}

void ellipsoidClampWalk(float &forward, float &left, float &turn, float speed) {
   const float MIN_SPEED = 0.0f;
   const float MAX_SPEED = 1.0f;
   speed = crop(speed, MIN_SPEED, MAX_SPEED);

   // limit max to 66-100% depending on speed
   float M_FORWARD = MAX_FORWARD * 0.66 + MAX_FORWARD * 0.34 * speed;
   float M_LEFT = MAX_LEFT * 0.66 + MAX_LEFT * 0.34 * speed;
   float M_TURN = MAX_TURN * 0.66 + MAX_TURN * 0.34 * speed;

   float clampedForward = crop(forward, -M_FORWARD, M_FORWARD);
   float clampedLeft = crop(left, -M_LEFT, M_LEFT);
   float clampedTurn = crop(turn, -M_TURN, M_TURN);

   // Values in range [-1..1]
   float forwardAmount = clampedForward / M_FORWARD;
   float leftAmount = clampedLeft / M_LEFT;
   float turnAmount = clampedTurn / M_TURN;

   float x = fabs(forwardAmount);
   float y = fabs(leftAmount);
   float z = fabs(turnAmount);

   // see if the point we are given is already inside the allowed walk params volume
   if (evaluateWalkVolume(x, y, z) > 1.0) {
      float scale = 0.5;
      float high = 1.0;
      float low = 0.0;

      // This is basically a binary search to find the point on the surface.
      for (unsigned i = 0; i < 10; i++) {
         // give priority to turn. keep it the same
         x = fabs(forwardAmount) * scale;
         y = fabs(leftAmount) * scale;

         if (evaluateWalkVolume(x, y, z) > 1.0) {
            float newScale = (scale + low) / 2.0;
            high = scale;
            scale = newScale;
         } else {
            float newScale = (scale + high) / 2.0;
            low = scale;
            scale = newScale;
         }
      }

      forwardAmount *= scale;
      leftAmount *= scale;
   }

   forward = M_FORWARD * forwardAmount;
   left = M_LEFT * leftAmount;
   turn = clampedTurn;
}

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z) {
   // his affects the relationship between forward and left.
   float e = 1.05; //1.25

   // lower value allows turn to be higher with a high forward/left, higher values dont allow a high turn
   float n = 1;

   float r = 2.0 / e;
   float t = 2.0 / n;

   return pow(pow(x, r) + pow(y, r), (t / r)) + pow(z, t);
}

JointValues Walk2014Generator::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors,
                                          BodyModel &bodyModel,
                                          float ballX,
                                          float ballY) {
   // 0. The very first time walk is called, the previous stand height could have been anything, so make sure we interpolate from that
   if (walk2014Option == NONE) {
      // Calculate the current hip height by checking how bent the knee is 
      hiph = sensors.joints.angles[LKneePitch] / KNEE_PITCH_RANGE * (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) + STAND_HIP_HEIGHT;
   }

	// 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
	if (t == 0 && walk2014Option != STEP) {
		//WALK_HIP_HEIGHT = WALK_HIP_HEIGHT;
		active = request->body;
      lastForward = forward;                           // back up old value in m/s
      lastLeft = left;                              // used to detect when the left walk parameter changes sign
		forward       = (float)active.forward/MM_PER_M;    // in meters
		left          = (float)active.left/MM_PER_M;       // in meters
		turn          = active.turn;                       // in radians
		power         = active.power;                      // controls stiffness when standing and kicking when walking*
		bend          = active.bend;                       // knee-bend parameter
		speed         = active.speed;                      // used to distinguish between jabKick and walkKick
		foot          = active.foot;                       // kicking foot
		isFast        = active.isFast;
		if (stopping) {                                    // not used at present
		}
		else {
		 stopped = false;                                  // (re)activate
		}
		// 1.0 For backwards compatibility with old interface (can be deleted when behaviours are updated)
		if(forward==0 and left==0 and turn==0 and power==0) bend=0;
		// 1.1 Scale back values to try to ensure stability.
//		for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
//			float temp = sensors.joints.temperatures[i];
//		    if(temp > 70)
//		    	speed = MIN(0.5,speed);
//		    if(temp > 75)
//		    	speed = MIN(0.5,speed);
//		}
		if (!exactStepsRequested) {
			ellipsoidClampWalk(forward, left, turn, speed);
		}
		float f = forward * MM_PER_M;
		float l = left * MM_PER_M;
		avoidFeet(f, l, turn, bodyModel);
		forward = f /MM_PER_M;
		left = l / MM_PER_M;
		// 1.2 Modify T when sidestepping and turning
		T = BASE_WALK_PERIOD + 0.1*abs(left)/MAX_LEFT;

		// 1.3 jabKick
		//if(isFast and speed==0) cout << "time = "<< globalTime << " jabKick " << foot << endl;

		// 1.5 ratchet forward by FORWARD_CHANGE
		if (!exactStepsRequested) {
         if (abs(forward-lastForward)>FORWARD_CHANGE) {                // ie greater than a FORWARD_CHANGE / sec change
            forward = lastForward + (forward-lastForward)/abs(forward-lastForward)*FORWARD_CHANGE;
         }
         if (abs(left-lastLeft) > LEFT_CHANGE) {
            left = lastLeft + (left-lastLeft)/abs(left-lastLeft)*LEFT_CHANGE;
         }
		}

      // 1.6 Walk Calibration
      // The definition of forward, left and turn is the actual distance/angle traveled in one second
      // One walk-cycle consists of two Phases, a left phase (left swing foot) and a right phase (right swing foot)

      if (!exactStepsRequested) {
         forward *= 2*T; // theoretical calibration. 2 - because there are two steps per walk cycle
         left    *= 2*T;
         turn    *= 2*T;
         // linear calibration to achieve actual performance ie turn in action command achieves turn/sec in radians on the real robot
         forward *= 1.0;
         left    *= 0.82;
         turn    *= 1.43;
      }
      turn *= -1;   // reverses sign
	}
	// 2. Update timer
	t += dt;
	globalTime += dt;
   lastKickTime += dt;

	// 3. Determine Walk2014 Option
   if(request->body.actionType != ActionCommand::Body::KICK && kickT > 0 && request->body.actionType != ActionCommand::Body::REF_PICKUP){ // we want to stop kicking, but also don't let ref pick up take over during kick **HACK ALERT** this is particularly for mario whose foot sensor dies while kicking
      // Finish transition out if in the middle of a kick by skipping to the end phase
      if (canAbbortKick()) {
         kickT = BACK_PHASE + kickPhase + THROUGH_PHASE;
      }
	}
   else if(active.actionType == ActionCommand::Body::KICK) {
      // This makes sure that the action type gets set back to walk just after a kick is finished.
      // If we don't leave enough time for this to happen, motion moves back into a kick before behaviour
      // can change its mind.
      if (lastKickTime < 2 * T) {
         request->body.actionType = ActionCommand::Body::WALK;
      } else if(abs(hiph-WALK_HIP_HEIGHT)<.0001){                  // make sure we retain the designated walking height
         if (walk2014Option == WALK || walk2014Option == STEP) { 
            // make sure walk is in neutral stance before kicking, L is symmetrical to R
//            cout << "prepKick" << fabs(forwardL) << " " << fabs(leftL) << " " << turnRL << " " << t << endl;
            if (fabs(forwardL) < EPSILON && fabs(leftL) < EPSILON && fabs(turnRL) < TURN_EPSILON && t == dt) {
               // Assuming already at t = 0 from active getting set to kick
               // Any new settings the first time walk2014Option==KICK go here

               // Calculate if turn step is required before kicking
               if (walk2014Option == WALK && fabs(active.kickDirection) >= TURN_THRESHOLD) {
                  // Halve the kick direction since HypL and HypR are connected
                  turnAngle = MIN(DEG2RAD(90), fabs(active.kickDirection)) / 2; // Safe max value for Hyp is ~45
                  if ((active.foot == ActionCommand::Body::LEFT && !bodyModel.isLeftPhase) ||
                     (active.foot != ActionCommand::Body::LEFT && bodyModel.isLeftPhase)) {
                     walk2014Option = STEP;
                  }

               // If no turn step is required, kick
               } else if (fabs(active.kickDirection) < TURN_THRESHOLD) {
                  // prep kick when the other foot is lifted
                  if (active.foot == ActionCommand::Body::LEFT && !bodyModel.isLeftPhase) {
                     prepKick(true, bodyModel);
                  } else if (active.foot != ActionCommand::Body::LEFT && bodyModel.isLeftPhase) {
                     prepKick(false, bodyModel);
                  }
               }
            }
         }
		   else if(walk2014Option != KICK) {
            // Calculate if turn is required before kicking
            if (fabs(active.kickDirection) >= TURN_THRESHOLD) {
               turnAngle = MIN(DEG2RAD(90), fabs(active.kickDirection)) / 2; // Safe max value for Hyp is ~45
               walk2014Option = STEP;
            } else {
               prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
            }
         }
	   }
	   else{                                               // hiph not= walkHipHeight
		   if(walk2014Option!=CROUCH) { // robot starts crouching to walking height
			   hiph0 = hiph;
			   timer = 0;
		   }
		   walk2014Option = CROUCH;                        // continue crouching
	   }
      
   } // end Kick test
   else if(walk2014Option==WALK and walkState!=NOT_WALKING){   // we are in the process of walking
	   if(forward==0 and left==0 and turn==0){             // request to stop walking
		   walkState = STOPPING;
	   }
   }
   else
	   if(bend==0){                                         // if we are not walking and wish to stand and power off
		   if(abs(hiph-STAND_HIP_HEIGHT)<.0001){               // and robot has reached stand height
			   walk2014Option = STAND;                       // (keep) standing
		   }
		   else{                                             // if hiph not= standHipHeight
			   if(walk2014Option!=STANDUP){
				   hiph0 = hiph;
				   timer = 0;
			   }
			   walk2014Option = STANDUP;                     // stand up first
		   }
	}
	else if(forward==0 and left==0 and turn==0 and bend==1){ // not walking, but ready to go again, ie don't stand up
		if(abs(hiph-WALK_HIP_HEIGHT)<.0001){
			walk2014Option = READY;
		}
		else {
			   if(walk2014Option!=CROUCH) { // robot starts crouching to walking height
				   hiph0 = hiph;
				   timer = 0;
			   }
			   walk2014Option = CROUCH;                        // continue crouching
		}
	}
	else{                                                  // if some walk parameters are non-zero
	   if(abs(hiph-WALK_HIP_HEIGHT)<.0001){                  // and we are at the designated walking height
		   if(walk2014Option != WALK) {
			   // Any new settings the first time walk2014Option==WALK go here (just for testing the walk)
			   walkState = STARTING;
			   nextFootSwitchT = T;
		   }
		   walk2014Option = WALK;                          // (keep) walking
	   }
	   else{                                               // hiph not= walkHipHeight
		   if(walk2014Option!=CROUCH) {                    // robot starts crouching to walking height
			   hiph0 = hiph;
			   timer = 0;
		   }
		   walk2014Option = CROUCH;                        // continue crouching
	   }
	}

	// 4. Execute Walk2014 Option
	if(walk2014Option==STAND){                             // Place CoM over ankle and turn set power to motors
	   hiph = STAND_HIP_HEIGHT;
	   forward = left = turn = 0;
	   t = nextFootSwitchT = 0;
	   stiffness = power;
	   if(stiffness<0.2) stiffness = 0.2;
	   comOffset = 0;
	}
	else if(walk2014Option == STANDUP){
	   hiph = hiph0+(STAND_HIP_HEIGHT-hiph0)*parabolicStep(timer,CROUCH_STAND_PERIOD,0);
	   forward = left = turn = 0;
	   comOffset -= 0.02*comOffset;                        // reduce offset to zero to allow stiffness to be turned down
	   stiffness = 1;
	   t = nextFootSwitchT = 0;
	   timer += dt;
	}
	else if(walk2014Option == CROUCH){
		forward = left = turn = 0;
		stiffness = 1;
		hiph = hiph0+(WALK_HIP_HEIGHT-hiph0)*parabolicStep(timer,CROUCH_STAND_PERIOD,0);
		comOffset = COM_OFFSET*parabolicStep(timer,CROUCH_STAND_PERIOD,0);   // move comOffset to 0.01 meters when walking
		t = nextFootSwitchT = 0;
		timer += dt;                                        // inc. option timer
	}
	else if(walk2014Option == WALK){
	   stiffness = 1;
	}
	else if(walk2014Option == KICK){
	   stiffness = 1;
	}
	else if(walk2014Option == STEP){
	   stiffness = 1;
	   nextFootSwitchT = TURN_PERIOD;
	}
	if(walk2014Option == READY){
		forward = left = turn = 0;
		stiffness = power;
		if(stiffness<0.4) stiffness = 0.4;                 // need enough stiffness to keep crouching posture
		t = nextFootSwitchT = 0;
	}

   // 5. Determine walk variables throughout the walk step phase
   if(walk2014Option == WALK and nextFootSwitchT>0){
	   // 5.1 Calculate the height to lift each swing foot
	   float maxFootHeight = BASE_LEG_LIFT + abs(forward)*0.01 + abs(left)*0.02;
	   float varfootHeight = maxFootHeight*parabolicReturn(t/nextFootSwitchT);  // 0.012 lift of swing foot from ground
	   // 5.2 When walking in an arc, the outside foot needs to travel further than the inside one.
	   // void
	   // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt, for left swing foot
	   if(bodyModel.isLeftPhase) {                         // if the support foot is right
	       if (bodyModel.isLeftPhase != lastIsLeftPhase) {
	           // 5.3.1L forward (the / by 4 is because the CoM moves as well and forwardL is wrt the CoM
	           forwardR = forwardR0+((forward)/4-forwardR0)*linearStep(t,nextFootSwitchT);
	           forwardL = forwardL0+parabolicStep(t,nextFootSwitchT,0)*(-(forward)/4-forwardL0); // swing-foot follow-through
	           // 5.3.3L Determine how much to lean from side to side - removed
	           // 5.3.4L left
	           if(left>0) {
	               leftR =  leftAngle()/2;                     // share left between both feet, hence /2
	               // if changing direction of left command, cancel swingAngle out of next left step
	               if(lastLeft*left<0) leftR -= swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.1));
	               leftL = -leftR;
	           }
	           else {
	               leftL = swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.0));
	               leftR = -leftL;
	           }
	           // 5.3.5L turn
	           if(turn<0) turnRL = turnRL0 + (-.67*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
	           else       turnRL = turnRL0 + (-.33*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0); //turn back to restore previous turn angle
	       }
	       // 5.3.6L determine how high to lift the swing foot off the ground
	       foothL = varfootHeight;                         // lift left swing foot
	       foothR = 0;                                     // do not lift support foot;
	   }
	   // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
	   if(not bodyModel.isLeftPhase){                      // if the support foot is left
	       if (bodyModel.isLeftPhase != lastIsLeftPhase) {
	           // 5.3.1R forward
	           forwardL = forwardL0+((forward)/4-forwardL0)*linearStep(t,nextFootSwitchT);
	           forwardR = forwardR0+parabolicStep(t,nextFootSwitchT,0)*(-(forward)/4-forwardR0); // swing-foot follow-through
	           // 5.3.2R Jab-Kick with right foot
	           // 5.3.3R lean - not used
	           // 5.3.4R left
	           if(left<0) {
	               leftL = leftAngle()/2;                      // divide by 2 to share left between both feet
	               // if changing direction of left command, cancel swingAngle out of next left step
	               if(lastLeft*left<0) leftL -= swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.1));
	               leftR = -leftL;
	           }
	           else {
	               leftR = swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.0));
	               leftL = -leftR;
	           }
	           // 5.3.5R turn
	           if(turn<0) turnRL = turnRL0 + (.33*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
	           else       turnRL = turnRL0 + (.67*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
	           // 5.3.6R Foot height
	       }
	       foothR = varfootHeight;
	       foothL = 0;
	   }
	   // 5.4 Special conditions when priming the walk
	   if(walkState == STARTING) {
		   turnRL = 0;                                     // don't turn on start of rocking - may relax this in future
		   foothL /= 3.5;                                  // reduce max lift due to short duration - may need to adjust this later
		   foothR /= 3.5;                                  // "
		   leftR = leftL = 0;                              // don't try to step left on starting and stopping
           forwardL = forwardR = 0;                        // don't move forward or backward
	   }
	   // 5.5 "natural" arm swing while walking/kicking to counterbalance foot swing
	   shoulderPitchR = -forwardR * 6;//10;                     // forwardR is in meters, 10 is an arbitrary scale factor to match previous walk
	   shoulderPitchL = -forwardL * 6;//10;
   }
   else if(walk2014Option == KICK){ // Kicking
      if (active.foot == ActionCommand::Body::LEFT) {
         makeForwardKickJoints(KICK_LEAN, KICK_STEP_HEIGHT, SHIFT_PERIOD, foothL, forwardL, leftL, kneePitchL, shoulderRollL, anklePitchL, ballY, request);
         leftR = leftL * 0.1; // Balance slightly more over support foot if need be
      } else { // with added adjustments for right side
         makeForwardKickJoints(-KICK_LEAN + 1.5, KICK_STEP_HEIGHT + 0.005, SHIFT_PERIOD, foothR, forwardR, leftR, kneePitchR, shoulderRollR, anklePitchR, ballY, request);
         leftR = -leftR; // switch signs for right side
         leftL = leftR * 0.12;
      }
   }
   else if(walk2014Option == STEP){ // Turn step
	   float footHeight = TURN_STEP_HEIGHT*parabolicReturn(t/nextFootSwitchT);
	   if(active.foot != ActionCommand::Body::LEFT) {     // if the support foot is right while turning
		   foothL = footHeight;                            // lift left swing foot
		   foothR = 0;                                     // do not lift support foot;
         rock = TURN_LEAN * parabolicReturn(t/nextFootSwitchT);
	   } else {                                           // if the support foot is left while turning
		   foothR = footHeight;                            // lift right swing foot
		   foothL = 0;                                     // do not lift support foot;
         rock = -TURN_LEAN * parabolicReturn(t/nextFootSwitchT);
      }
      // Interpolate turn step over turnT with dead period to avoid dragging against the ground
      turnRL = turnAngle * TURN_SCALE * parabolicStep(t, nextFootSwitchT, 0.15);
      // Once the turn step has pretty much finished, readjust the kick direction
      if (t >= nextFootSwitchT - dt) {
         if (active.kickDirection >= TURN_THRESHOLD) {
            active.kickDirection -= (turnAngle * 2);
         } else if (active.kickDirection <= -TURN_THRESHOLD) {
            active.kickDirection += (turnAngle * 2);
         }
      }
      lastTurn = turnRL;
   }
   else{ // When walk option is not WALK or KICK
	   foothL = foothR = 0;
   }

   // 6. Changing Support Foot. Note bodyModel.isLeftPhase means left foot is swing foot.
	//    t>0.75*T tries to avoid bounce, especially when side-stepping
	//    lastZMPL*ZMPL<0.0 indicates that support foot has changed
	//    t>3*T tires to get out of "suck" situations
   if((t>0.75*T and bodyModel.ZMPL*bodyModel.lastZMPL < 0) or t>3*T) supportFoothasChanged = true;
   if(supportFoothasChanged) {
       lastIsLeftPhase = bodyModel.isLeftPhase;
       bodyModel.setIsLeftPhase(bodyModel.ZMPL < 0); // set isLeft phase in body model for kinematics etc
   }
   if(supportFoothasChanged and walk2014Option==WALK) {
	   supportFoothasChanged = false;                      //reset
	   // 6.1 Recover previous "left" swing angle
       if(bodyModel.isLeftPhase) swingAngle = leftL;
       else                      swingAngle = leftR;
	   // 6.2 Decide on timing of next walk step phase
	   if(walkState == NOT_WALKING){                       // Start the walk
		   nextFootSwitchT = T;
		   walkState = STARTING;
	   } else
	   if(walkState == STARTING){
		   nextFootSwitchT = T;
		   walkState = WALKING;
	   } else
	   if(walkState == WALKING){
		   nextFootSwitchT = T;
		   walkState = WALKING;                            // continue walking until interrupted by a command to stop (or kick?)
	   } else
	   if(walkState == STOPPING){
		   nextFootSwitchT = T;
		   walkState = NOT_WALKING;
	   }
	   else cout << "Should never get here: walkState error" << endl;
	   // 6.3 reset step phase time
	   t = 0;                                              // reset step phase timer
	   // 6.4 backup values
	   turnRL0 = turnRL;                                   // store turn value for use in next step
       forwardR0 = forwardR;                               // sagittal right foot
       forwardL0 = forwardL;                               // sagittal left foot
       // 6.5 Other stuff on support foot change
       // none at the moment
  } // end of changing support foot

   // 7. Sagittal Balance
   filteredGyroY = 0.8*filteredGyroY + 0.2*sensors.sensors[InertialSensor_GyrY];
   balanceAdjustment = filteredGyroY/25;                   // adjust ankle tilt in proportion to filtered gryoY
   if(walk2014Option==READY) balanceAdjustment = 0;        // to stop swaying while not walking

   // 7.5 Coronal Balance
   filteredGyroX = 0.8*filteredGyroX + 0.2*sensors.sensors[InertialSensor_GyrX];
   coronalBalanceAdjustment = filteredGyroX/25;            // adjust ankle roll in proportion to filtered gryoX

   // 8. Odometry update for localisation
   *odometry = *odometry + motionOdometry.updateOdometry(sensors, updateOdometry(bodyModel.isLeftPhase));

   // Diagnostic Printouts - commented out for production code
   //	cout << sensors.sensors[Sensors::InertialSensor_GyrY] << endl;
//    cout << t <<" "<< leftL <<" "<< leftR << endl;

   // 9. Work out joint angles from walk variables above
   // 9.1 Left foot closed form inverse kinematics
   float leghL = hiph - foothL - ankle;                    // vertical height between ankle and hip in meters
   float legX0L = leghL / cos(leftL);                      // leg extension (eliminating knee) when forwardL = 0
   float legXL = sqrt(legX0L*legX0L
		   +(forwardL+comOffset)*(forwardL+comOffset));    //leg extension at forwardL
   float beta1L = acos((thigh*thigh+legXL*legXL-tibia*tibia)/(2.0f*thigh*legXL)); // acute angle at hip in thigh-tibia triangle
   float beta2L = acos((tibia*tibia+legXL*legXL-thigh*thigh)/(2.0f*tibia*legXL)); // acute angle at ankle in thigh-tibia triangle
   float tempL = legX0L/legXL; if(tempL>1.0f) tempL=1.0f;  // sin ratio to calculate leg extension pitch. If > 1 due to numerical error round down.
   float deltaL = asin(tempL);                             // leg extension angle
   float dirL = 1.0f; if ((forwardL+comOffset) > 0.0f) dirL = -1.0f; // signum of position of foot
   float HpL = beta1L + dirL*(M_PI/2.0f-deltaL);           // Hip pitch is sum of leg-extension + hip acute angle above
   float ApL = beta2L + dirL*(deltaL - M_PI/2.0f);         // Ankle pitch is a similar calculation for the ankle joint
   float KpL = HpL + ApL;                                  // to keep torso upright with both feet on the ground, the knee pitch is always the sum of the hip pitch and the ankle pitch.

   // 9.2 right foot closed form inverse kinematics (comments as above but for right leg)
   float leghR = hiph - foothR - ankle;
   float legX0R = leghR / cos(leftR);
   float legXR = sqrt(legX0R*legX0R
		   +(forwardR+comOffset)*(forwardR+comOffset));
   float dirR = 1.0f; if ((forwardR+comOffset) > 0.0f) dirR = -1.0f;
   float beta1R = acos((thigh*thigh+legXR*legXR-tibia*tibia)/(2.0f*thigh*legXR));
   float beta2R = acos((tibia*tibia+legXR*legXR-thigh*thigh)/(2.0f*tibia*legXR));
   float tempR = legX0R/legXR; if(tempR>1.0f) tempR=1.0f;
   float deltaR = asin(tempR);
   float HpR = beta1R + dirR*(M_PI/2.0f-deltaR);
   float ApR = beta2R + dirR*(deltaR - M_PI/2.0f);
   float KpR = HpR + ApR;

   // 9.3 Sert hip and ankle values
   float HrL = -leftL;
   float HrR = -leftR;
   float ArL = -HrL;
   float ArR = -HrR;
   if (walk2014Option == KICK || walk2014Option == STEP) {
      HrL += rock;
      HrR += rock;
      ArL -= rock;
      ArR -= rock;
   }

   // 9.4 Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
   // Turning
   XYZ_Coord tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
   XYZ_Coord sL;
   float Hyp = -turnRL;
   for (int i = 0; i < 3; i++) {
	  sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
	  XYZ_Coord e((tL.x-sL.x), (tL.y-sL.y), (tL.z-sL.z));
	  Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
	  HpL -= hpr.Hp;
	  HrL += hpr.Hr;
   }
   // ApL and ArL to make sure LEFT foot is parallel to ground
   XYZ_Coord up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
   XYZ_Coord ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
   ApL = ApL + asin(sL.z-up.z);
   ArL = ArL + asin(sL.z-ur.z);

   // 9.5 Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
   // Map to LEFT - we reuse the left foot IK because of symmetry right foot
   float Hr = -HrR;
   float Ar = -ArR;
   // Target foot origin in body coords
   XYZ_Coord t = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
   XYZ_Coord s;
   Hyp = -turnRL;
   for (int i = 0; i < 3; i++) {
	  s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
	  XYZ_Coord e((t.x-s.x), (t.y-s.y), (t.z-s.z));
	  Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
	  HpR -= hpr.Hp;
	  Hr += hpr.Hr;
   }
   // 9.6 Ap and Ar to make sure foot is parallel to ground
   XYZ_Coord u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
   XYZ_Coord u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
   ApR = ApR + asin(s.z-u1.z);
   Ar = Ar + asin(s.z-u2.z);
   // map back from left foot to right foot
   HrR = -Hr;
   ArR = -Ar;

   // 10. Set joint values and stiffness
   JointValues j = sensors.joints;
   for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) j.stiffnesses[i] = stiffness;

   // 10.1 Arms
   j.angles[LShoulderPitch] = DEG2RAD(90)+shoulderPitchL;
   j.angles[LShoulderRoll] = DEG2RAD(7)+shoulderRollL;
   j.angles[LElbowYaw] = DEG2RAD(0); //DEG2RAD(-90); //swing bent arms
   j.angles[LElbowRoll] = DEG2RAD(0); //DEG2RAD(-30)+shoulderPitchL;  //swing bent arms
   j.angles[LWristYaw] = DEG2RAD(0);
   j.angles[RShoulderPitch] = DEG2RAD(90)+shoulderPitchR;
   j.angles[RShoulderRoll] = DEG2RAD(-7)-shoulderRollR;
   j.angles[RElbowYaw] = DEG2RAD(0); //DEG2RAD(90);  //swing bent arms
   j.angles[RElbowRoll] = DEG2RAD(0); //DEG2RAD(30)-shoulderPitchR; //swing bent arms
   j.angles[RWristYaw] = DEG2RAD(0);

   float armStiffness = 0.1f;
   j.stiffnesses[Joints::LShoulderPitch] = armStiffness;
   j.stiffnesses[Joints::LShoulderRoll] = armStiffness;
   j.stiffnesses[Joints::LElbowYaw] = armStiffness;
   j.stiffnesses[Joints::LElbowRoll] = armStiffness;
   j.stiffnesses[Joints::LWristYaw] = armStiffness;
   j.stiffnesses[Joints::RShoulderPitch] = armStiffness;
   j.stiffnesses[Joints::RShoulderRoll] = armStiffness;
   j.stiffnesses[Joints::RElbowYaw] = armStiffness;
   j.stiffnesses[Joints::RElbowRoll] = armStiffness;
   j.stiffnesses[Joints::RWristYaw] = armStiffness;

   // 10.2 Turn
   j.angles[Joints::LHipYawPitch] = -turnRL;

   // 10.3 Sagittal Joints
   j.angles[Joints::LHipPitch]    = -HpL;
   j.angles[Joints::RHipPitch]    = -HpR;
   j.angles[Joints::LKneePitch]   =  KpL;
   j.angles[Joints::RKneePitch]   =  KpR;
   // Only activate balance control if foot is on the ground
   j.angles[Joints::LAnklePitch]  = -ApL;
   j.angles[Joints::RAnklePitch]  = -ApR;
   if(walk2014Option == WALK and nextFootSwitchT>0) {
	   if(bodyModel.isLeftPhase) j.angles[Joints::RAnklePitch]  += balanceAdjustment;
	   else                      j.angles[Joints::LAnklePitch]  += balanceAdjustment;
   }
   else if (walk2014Option == KICK) {
	   if(bodyModel.isLeftPhase) j.angles[Joints::LAnklePitch]  += balanceAdjustment;
	   else                      j.angles[Joints::RAnklePitch]  += balanceAdjustment;
   }
   else {
	   j.angles[Joints::RAnklePitch]  += balanceAdjustment;
	   j.angles[Joints::LAnklePitch]  += balanceAdjustment;
   }

   // 10.4 Coronal Joints
   j.angles[Joints::LHipRoll]     =  HrL;
   j.angles[Joints::RHipRoll]     =  HrR;
   j.angles[Joints::LAnkleRoll]   =  ArL;
   j.angles[Joints::RAnkleRoll]   =  ArR;

   // Add in joint adjustments for kicks
   if (walk2014Option == KICK) {
      addKickJoints(j);
      // Add in some coronal balancing
	   if(bodyModel.isLeftPhase) j.angles[Joints::RAnkleRoll]  += coronalBalanceAdjustment;
	   else                      j.angles[Joints::LAnkleRoll]  += coronalBalanceAdjustment;
   }

   //quick check if position went out of joint space
   for(int i = 0; i < Joints::NUMBER_OF_JOINTS; i++){
      checkNaN(j.angles[i], Joints::fliteJointNames[i]);
   }

   return j;
}

void Walk2014Generator::walkKick(float ballX, BodyModel &bodyModel) {
	// plan forward
	float delta = 0.05; // meters
	float ballDistX = ballX/MM_PER_M;
	if(ActionCommand::Body::LEFT) { // left foot walk-kick
		//cout <<"forward-0 "<< ballX <<" "<< forward << endl;
		if(bodyModel.isLeftPhase and (ballDistX-delta+forwardL0)<forward) {
			forward = (ballDistX-delta-forwardL0)/2;
			//cout <<"forward-1 "<< ballX <<" "<< forward <<" "<< forwardL0 <<" "<< delta << endl;
		}
		if(!bodyModel.isLeftPhase and (ballDistX-delta-forwardR0)<forward/2) {
			forward = ballDistX-delta-forwardR0;
			//cout <<"forward-2 "<< ballX <<" "<< forward <<" "<< forwardR0 <<" "<< delta << endl;
		}
		if(bodyModel.isLeftPhase and (ballDistX-delta+forwardL0)<delta)  forward = abs(power)*MAX_FORWARD;
	    if(!bodyModel.isLeftPhase and (ballDistX-delta-forwardR0)<delta) forward = abs(power)*MAX_FORWARD;
		//cout <<"forward-3 "<< ballX <<" "<< forward << endl;
	}
	if(ActionCommand::Body::RIGHT) { // left foot walk-kick
		//cout <<"forward-0 "<< ballX <<" "<< forward << endl;
		if(!bodyModel.isLeftPhase and (ballDistX-delta+forwardR0)<forward) {
			forward = (ballDistX-delta-forwardR0)/2;
			//cout <<"forward-1 "<< ballX <<" "<< forward <<" "<< forwardL0 <<" "<< delta << endl;
		}
		if(bodyModel.isLeftPhase and (ballDistX-delta-forwardL0)<forward/2) {
			forward = ballDistX-delta-forwardL0;
			//cout <<"forward-2 "<< ballX <<" "<< forward <<" "<< forwardR0 <<" "<< delta << endl;
		}
		if(!bodyModel.isLeftPhase and (ballDistX-delta+forwardR0)<delta)  forward = abs(power)*MAX_FORWARD;
	    if(bodyModel.isLeftPhase and (ballDistX-delta-forwardL0)<delta) forward = abs(power)*MAX_FORWARD;
		//cout <<"forward-3 "<< ballX <<" "<< forward << endl;
	}
    if(forward>MAX_FORWARD) forward = MAX_FORWARD; // in case things go wrong
    if(forward>MAX_FORWARD) forward = MAX_FORWARD; // in case things go wrong
	//cout <<"forward-4 "<< ballX <<" "<< forward << endl;
}

void Walk2014Generator::prepKick(bool isLeft, BodyModel &bodyModel) {
   t = 0;
   walk2014Option = KICK;
   turnAngle = 0;
   kickT = 0;
   if (isLeft) {
      lastKickForward = forwardL;
      lastKneePitch = kneePitchL;
      lastSide = leftL;
      bodyModel.setIsLeftPhase(true);
      forwardR = 0;
      foothR = 0;
   } else {
      lastKickForward = forwardR;
      lastKneePitch = kneePitchR;
      lastSide = -leftR;
      bodyModel.setIsLeftPhase(false);
      forwardL = 0;
      foothL = 0;
   }
}

// nicer for the motors
bool Walk2014Generator::canAbbortKick() {
    float totalShift = SHIFT_PERIOD / 4;
    float halfShift = totalShift / 2;
    float pauseTime = (BACK_PHASE - totalShift) / 2;
    return kickT - pauseTime < halfShift
            || (kickT >= totalShift + pauseTime && kickT < BACK_PHASE);
}

void Walk2014Generator::makeForwardKickJoints(float kickLean, float kickStepH, float shiftPeriod, float &footh, float &forwardDist,
   float &side, float &kneePitch, float &shoulderRoll, float &anklePitch, float &ballY, ActionCommand::All* request) {

   kickT += dt;
   kickPhase = MAX_KICK_PHASE;
   float totalPhase = BACK_PHASE + kickPhase + THROUGH_PHASE + END_PHASE;

   // Update side position of ball as late as possible.
   if (request->body.misalignedKick) {
      dynamicSide = 0; // This will try to kick the robot with the outside of the foot.
   } else if (kickT < BACK_PHASE * 0.8) {
      dynamicSide = fabs(ballY) - 40; // offset to kick point on foot
   }
   // Max safe/useful value for lifting leg (without going past joint limits/balancing)
   float sideAmp = -MAX(0, MIN(50, dynamicSide)) / 200.0;
   float kickAmp = -0.07; // how far forward the kick should reach
   float kickPower;
   if (v4) {
       kickPower = pow(power, 1.5);
   } else {
       kickPower = pow(power, 1.7);
   }
   float kickBackAmp = -kickAmp * kickPower * 0.9;
   float shoulderRollAmp = -sideAmp / 2.5; // how much arm should lift to make room for raised kicking leg 

   float kneePitchAmp = DEG2RAD(35);
   float anklePitchAmp = DEG2RAD(35);
   float anklePitchRetracted = DEG2RAD(5);
   if (!v4) {
       kickStepH += 0.01;
   }

   if (lastTurn > 0) {
      shiftPeriod = 3.5;
   }

   // Shift weight over and swing foot back
   if (kickT < BACK_PHASE) {
      float totalShift = shiftPeriod / 4;
      float halfShift = totalShift / 2;
      float pauseTime = (BACK_PHASE - totalShift) / 2;
      if (t >= pauseTime) {
          float t2 = t - pauseTime;
          // We spend the first part shifting our weight over.
          if (t2 < totalShift) {
             rock = DEG2RAD(kickLean) * parabolicStep(t2, totalShift, 0);
             // If the robot had turned, bring the feet back together after some time for foot to lift off the ground
             if (lastTurn > 0) {
                if (t2 >= halfShift) {
                   turnRL = lastTurn * (1 - parabolicStep(t2 - halfShift, halfShift, 0.0));
                } else {
                   turnRL = lastTurn;
                }
                // Add in some extra lean while shifting weight along with turn
                rock += DEG2RAD(kickLean) / 3.5 * parabolicReturn(t2/totalShift);
             }
          } else {
             turnRL = 0;
             lastTurn = 0;
          }

          if (t2 >= totalShift / 3) {
              float t3 = t2 - totalShift / 3;
              float endT = BACK_PHASE - totalShift / 3 - pauseTime;
              footh = kickStepH * parabolicStep(t3, endT, 0);
          }

          float kickT2 = kickT - pauseTime;
          // Once we're halfway through shifting our weight, start moving the foot back.
          if (kickT2 >= halfShift) {
             float shiftedKickT = kickT2 - halfShift;
             float endT = BACK_PHASE - halfShift - pauseTime;
             forwardDist = interpolateSmooth(0, kickBackAmp, shiftedKickT, endT);
             side = interpolateSmooth(0, sideAmp, shiftedKickT, endT);
             shoulderRoll = interpolateSmooth(0, shoulderRollAmp, shiftedKickT, endT);
             kneePitch = interpolateSmooth(0, kneePitchAmp * 0.9, shiftedKickT, endT);
             anklePitch = interpolateSmooth(0, anklePitchRetracted, shiftedKickT, endT);
          }
      }
   // Swing foot forward.
   } else if (kickT < (BACK_PHASE + kickPhase)) {
      forwardDist = interpolateSmooth(kickBackAmp, kickAmp, kickT - BACK_PHASE, kickPhase);
      side = sideAmp;
      shoulderRoll = shoulderRollAmp;
      kneePitch = interpolateSmooth(kneePitchAmp * 0.9, -kneePitchAmp, kickT - BACK_PHASE, kickPhase);
      float anklePitchStart = 0;
      if (kickT >= BACK_PHASE + anklePitchStart) {
          anklePitch = interpolateSmooth(anklePitchRetracted, anklePitchAmp, kickT - BACK_PHASE - anklePitchStart, kickPhase - anklePitchStart);
      }
   // Hold...
   } else if (kickT < (BACK_PHASE + kickPhase + THROUGH_PHASE)) {
      forwardDist = kickAmp;
      side = sideAmp;
      shoulderRoll = shoulderRollAmp;
      kneePitch = -kneePitchAmp;
      anklePitch = anklePitchAmp;
   // Return foot.
   } else if (kickT < (BACK_PHASE + kickPhase + THROUGH_PHASE + END_PHASE)) {
      forwardDist = interpolateSmooth(lastKickForward, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      side = interpolateSmooth(lastSide, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      shoulderRoll = interpolateSmooth(lastShoulderRollAmp, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      kneePitch = interpolateSmooth(lastKneePitch, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      anklePitch = interpolateSmooth(lastAnklePitch, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
   // Shift weight back to both feet.
   } else if (kickT < (totalPhase + SHIFT_END_PERIOD / 4)) {
      forwardDist = 0;
      kneePitch = 0;
      anklePitch = 0;
      double endT = totalPhase - kickT;
      rock = lastRock * sin((endT + SHIFT_END_PERIOD / 4) / SHIFT_END_PERIOD * 2 * M_PI);
      footh = lastFooth * sin((endT + SHIFT_END_PERIOD / 4) / SHIFT_END_PERIOD * 2 * M_PI);
   } else {
      kickT = 0;
      rock = 0;
      footh = 0;
      walk2014Option = WALK;
      walkState = NOT_WALKING;
      request->body.actionType = ActionCommand::Body::WALK;
      lastKickTime = 0;
   }

   // abborting or ending a kick from these values
   if (kickT < (BACK_PHASE + kickPhase + THROUGH_PHASE)) {
       lastKickForward = forwardDist;
       lastKneePitch = kneePitch;
       lastSide = side;
       lastFooth = footh;
       lastAnklePitch = anklePitch;
       lastRock = rock;
       lastShoulderRollAmp = shoulderRollAmp;
   }
}

void Walk2014Generator::addKickJoints(JointValues &j){
   j.angles[Joints::LKneePitch] += kneePitchL;
   j.angles[Joints::LAnklePitch] += anklePitchL + shoulderRollL;
   j.angles[Joints::LShoulderPitch] -= kneePitchL;

   j.angles[Joints::RKneePitch] += kneePitchR;
   j.angles[Joints::RAnklePitch] += anklePitchR + shoulderRollR;
   j.angles[Joints::RShoulderPitch] -= kneePitchR;
}

float Walk2014Generator::leftAngle()
{
	float left_at_t = left*parabolicStep(t,nextFootSwitchT,0.0);
	float height = hiph-ankle;
	return atan(left_at_t/height);
}

float Walk2014Generator::linearStep(float time, float period) {
	if(time<=0) return 0;
	if(time>=period) return 1;
	return time/period;
}

float Walk2014Generator::parabolicReturn(float f){         //normalised [0,1] up and down
        double x = 0;
        double y = 0;
        if(f<0.25f) {
            y = 8*f*f;
        }
        if(f>=0.25f && f<0.5f) {
            x = 0.5f-f;
            y = 8*x*x;
            y = 1.0f-y;
        }
        if(f>=0.5f && f<0.75f){
            x = f-0.5f;
            y = 8*x*x;
            y = 1.0f-y;
        }
        if(f>=0.75f && f<=1.0f){
            x = 1.0f-f;
            y = 8*x*x;
        }
        return y;
}

float Walk2014Generator::parabolicStep(float time, float period, float deadTimeFraction){ //normalised [0,1] step up
    float deadTime = period*deadTimeFraction/2;
	if(time<deadTime+dt/2) return 0;
	if(time>period-deadTime-dt/2) return 1;
	float timeFraction = (time-deadTime)/(period-2*deadTime);
	if(time<period/2) return 2.0*timeFraction*timeFraction;
	return 4*timeFraction-2*timeFraction*timeFraction-1;
}

Odometry Walk2014Generator::updateOdometry(bool isLeftSwingFoot){
	// Work out incremental forward, left, and turn values for next time step
	float height = hiph-ankle;
	float turnOdo = -(turnRL - prevTurn);
	float leftOdo = (height*tan(leftR) - prevLeftR);
	float forwardOdo = (forwardR - prevForwardR);
	if(!isLeftSwingFoot){
	  turnOdo *= -1;
	  leftOdo = (height*tan(leftL) - prevLeftL);
	  forwardOdo = (forwardL - prevForwardL);
	}
	forwardOdo *= MM_PER_M;
	leftOdo    *= MM_PER_M;
	//Calibrate odometry to match the actual speed
	forwardOdo *= 1;
	leftOdo    *= 1.23;
	turnOdo    *= -.58;
	//cout << forwardOdo <<" "<< leftOdo <<" "<< turnOdo << endl;

	// backup odometry values
	prevTurn = turnRL;
	prevLeftL = height*tan(leftL);
	prevLeftR = height*tan(leftR);
	prevForwardL = forwardL;
	prevForwardR = forwardR;
   return Odometry(forwardOdo, leftOdo, turnOdo);
}

bool Walk2014Generator::isActive() {
   return !stopped;
}

void Walk2014Generator::readOptions(const boost::program_options::variables_map &config) {
	v4 = config["motion.v4"].as<bool>();
	if (v4){
		KICK_LEAN = KICK_LEAN_V4;
		std::cout << "Selecting v4 kick lean of " << KICK_LEAN_V4 << endl;
	}
}
void Walk2014Generator:: reset() {
   initialise();
   llog(INFO) << "Walk2014 reset" << endl;
}

void Walk2014Generator:: stop() {
   // this does nothing
   stopping = true;
   llog(INFO) << "Walk2014 stop" << endl;
}

float Walk2014Generator::interpolateSmooth(float start, float end, float tCurrent, float tEnd) {
   return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

XYZ_Coord Walk2014Generator::mf2b(float Hyp, float Hp, float Hr,
                                  float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf) {
// MFOOT2BODY Transform coords from foot to body.
// This code originates from 2010 using symbolic equations in Matlab to perform the coordinate transforms - see team report (BH)
// In future this approach to IK for the Nao should be reworked in closed form, significantly reducing the size of the code the
// the computational complexity (BH)
   XYZ_Coord result;
   float pi = M_PI;
   float tibia        = this->tibia*1000;
   float thigh        = this->thigh*1000;
   float k  = sqrt(2.0);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0);
   float c3 = cos(Hyp - pi/2.0);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0/2.0*pi);
   float s4 = sin(Hr + 1.0/4.0*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0/2.0*pi);
   result.x = thigh*(s2*s3 - c2*c3*c4) + tibia*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - yf*(c6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) + c3*s4*s6) + zf*(s6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) - c3*c6*s4) + xf*(c1*(s1*(s2*s3 - c2*c3*c4) -
       c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)));
   result.y = xf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)) + s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f))) + tibia*(c5*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) + thigh*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - yf*(s6*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)))) - zf*(c6*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))));
   result.z = yf*(s6*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c6*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)))) -
       tibia*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       thigh*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       xf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       zf*(c6*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s6*(c1*(c5*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))));
   return result;
}

Walk2014Generator::Hpr Walk2014Generator::hipAngles(float Hyp, float Hp,
                                  float Hr, float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf, XYZ_Coord e) {
// Code from 2010 to perform interative Inverse Kinematics.
// Symbolic equations generated in Matlab - see 2010 team report for details and reference
   Hpr result;
   float pi = M_PI;
   float tibia        = this->tibia*1000;
   float thigh        = this->thigh*1000;
   float k  = sqrt(2.0);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0);
   float c3 = cos(Hyp - pi/2.0);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0/2.0*pi);
   float s4 = sin(Hr + 1.0/4.0*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0/2.0*pi);
   float j11 = thigh*(c4*s3 + c2*c3*s2) - tibia*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + xf*(c1*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2))) + c6*yf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4))) - s6*zf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)));
   float j12 = yf*(c6*(c1*(c3*s1*s2*s4 - c3*c4*c5*s4) +
       s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) - c2*c3*s6) -
       tibia*(c3*s1*s2*s4 - c3*c4*c5*s4) - zf*(s6*(c1*(c3*s1*s2*s4 -
       c3*c4*c5*s4) + s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) + c2*c3*c6) +
       xf*(c1*(c3*c4*s1*s4 + c3*c5*s2*s4) - s5*(c3*s1*s2*s4 -
       c3*c4*c5*s4)) + c3*c4*s4*thigh;
   float j21 = xf*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))) -
       tibia*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) -
       thigh*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f))) -
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)));
   float j22 = tibia*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) + xf*(c1*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f))) + yf*(s6*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f - (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) + zf*(c6*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) +
       c4*thigh*((c2*k)/2.0f - (k*s3*s4)/2.0f);
   float j31 = tibia*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) +
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       (c3*k*s2)/2.0f)) -
       xf*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))) +
       thigh*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) -
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)));
   float j32 = -tibia*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - xf*(c1*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f))) - yf*(s6*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) - zf*(c6*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) -
       c4*thigh*((c2*k)/2.0f + (k*s3*s4)/2.0f);
   float xbe = e.x;
   float ybe = e.y;
   float zbe = e.z;
   float lambda = 0.4f;
   float la2 = lambda*lambda;
   float la4 = la2*la2;
   float j322 = j32*j32;
   float j222 = j22*j22;
   float j122 = j12*j12;
   float j212 = j21*j21;
   float j112 = j11*j11;
   float j312 = j31*j31;
   float sigma = 1.0f/(la4 + j112*j222 + j122*j212 + j112*j322 + j122*j312 +
   j212*j322 + j222*j312 + j112*la2 + j122*la2 + j212*la2 + j222*la2 +
   j312*la2 + j322*la2 - 2.0f*j11*j12*j21*j22 - 2.0f*j11*j12*j31*j32 -
   2.0f*j21*j22*j31*j32);
   result.Hp = sigma*xbe*(j11*j222 + j11*j322 + j11*la2 - j12*j21*j22 -
   j12*j31*j32) + sigma*ybe*(j122*j21 + j21*j322 + j21*la2 - j11*j12*j22 -
   j22*j31*j32) + sigma*zbe*(j122*j31 + j222*j31 + j31*la2 - j11*j12*j32 -
   j21*j22*j32);
   result.Hr =  sigma*xbe*(j12*j212 + j12*j312 + j12*la2 - j11*j21*j22 -
   j11*j31*j32) + sigma*ybe*(j112*j22 + j22*j312 + j22*la2 - j11*j12*j21 -
   j21*j31*j32) + sigma*zbe*(j112*j32 + j212*j32 + j32*la2 - j11*j12*j31 -
   j21*j22*j31);
   return result;
}

