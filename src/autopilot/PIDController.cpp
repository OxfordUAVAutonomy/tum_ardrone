 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "PIDController.h"
#include "AutopilotStructures.h"
#include "../HelperFunctions.h"

// TODO: Clean this code up, get rid of magic numbers, uniform variable name conventions, etc.
double angleFromTo2(double angle, double min, double sup)
{
  while(angle < min) angle += 360;
  while(angle >=  sup) angle -= 360;
  return angle;
}

void i_term_increase(double& i_term, double new_err, double cap)
{
  if(new_err < 0 && i_term > 0)
    i_term = std::max(0.0, i_term + 2.5 * new_err);
  else if(new_err > 0 && i_term < 0)
    i_term = std::min(0.0, i_term + 2.5 * new_err);
  else
    i_term += new_err;

  if(i_term > cap) i_term =  cap;
  if(i_term < -cap) i_term =  -cap;
}

PIDController::PIDController(void)
{
  lastTimeStamp = 0;
}

PIDController::~PIDController(void)
{
}

// generates and sends a new control command to the drone, 
// based on the currently active command ant the drone's position.
ControlCommand PIDController::getControlCommand(
  DronePosition position, 
  DroneSpeed speed, 
  DronePosition target)/*,
  double scaleAccuracy)*/
{
  // TODO: compare target to stored target and set new target if necessary!
  target.yaw = angleFromTo2(target.yaw,-180,180);
  if (target != lastTarget)
  {
    ROS_INFO("setting new target");
    setNewTarget(target);
  }

  DronePosition difference = target - position; //getDifference(position, target);

  TooN::Vector<4> new_error = TooN::makeVector(
    difference.x,
    difference.y,
    difference.z,
    difference.yaw
  );
  new_error[3] = angleFromTo2(new_error[3],-180,180);	
  TooN::Vector<4> d_error = TooN::makeVector(
    -speed.dx,
    -speed.dy,
    -speed.dz,
    -speed.dyaw
  );

  double yaw = position.yaw;

  calcControl(new_error, d_error, yaw);/*, scaleAccuracy);*/

  last_error = new_error;
  return lastSentControl;
}

void PIDController::setNewTarget(DronePosition newTarget)
{
  lastTarget = newTarget;
  lastTarget.yaw = angleFromTo2(lastTarget.yaw,-180,180);
  targetSetAtClock = getMS()/1000.0;
  targetNew = TooN::makeVector(1.0,1.0,1.0,1.0);
  last_error = i_term = TooN::makeVector(0,0,0,0);
}

void PIDController::calcControl(
  TooN::Vector<4> new_error, 
  TooN::Vector<4> d_error, 
  double yaw)/*,
  double scaleAccuracy)*/
{

  double agr = aggressiveness;
  agr *= 0.75;
  /*if(!ptamIsGood) agr *= 0.75;
  agr *= scaleAccuracy;*/

  TooN::Vector<4> d_term = d_error; // d-term:just differentiate
  TooN::Vector<4> p_term = new_error;	// p-term is error.

  // rotate error to drone CS, invert pitch
  double yawRad = yaw * 2 * 3.141592 / 360;	
  d_term[0] = cos(yawRad)*d_error[0] - sin(yawRad)*d_error[1];
  d_term[1] = - sin(yawRad)*d_error[0] - cos(yawRad)*d_error[1];

  p_term[0] = cos(yawRad)*new_error[0] - sin(yawRad)*new_error[1];
  p_term[1] = - sin(yawRad)*new_error[0] - cos(yawRad)*new_error[1];

  // integrate & cap
  double sec = getMS()/1000.0 - lastTimeStamp; 
  lastTimeStamp = getMS()/1000.0;
  i_term_increase(i_term[2],new_error[2] * sec, 0.2f / Ki_gaz);
  i_term_increase(i_term[1],new_error[1] * sec, 0.1f / Ki_rp+(1e-10));
  i_term_increase(i_term[0],new_error[0] * sec, 0.1f / Ki_rp+(1e-10));

  // kill integral term when first crossing target
  // that is, targetNew is set, it was set at least 100ms ago, and err changed sign.
  for(int i=0;i<4;i++)
    if(targetNew[i] > 0.5 && getMS()/1000.0 - targetSetAtClock > 0.1 && last_error[i] * new_error[i] < 0)
    {
      i_term[i] = 0; 
      targetNew[i] = 0;
    }

  // YAW
  lastSentControl.yaw = Kp_yaw * p_term[3] + Kd_yaw * d_term[3]; // yaw can be translated directly
  lastSentControl.yaw = std::min(max_yaw,std::max(-max_yaw,(double)(lastSentControl.yaw*agr)));

  // RP
  // calculate signals only based on d and p:
  double cX_p = Kp_rp * p_term[0];
  double cY_p = Kp_rp * p_term[1];

  double cX_d = Kd_rp * d_term[0];
  double cY_d = Kd_rp * d_term[1];

  double cX_i = Ki_rp * i_term[0];
  double cY_i = Ki_rp * i_term[1];

  lastSentControl.roll = cX_p + cX_d + cX_i;
  lastSentControl.pitch = cY_p + cY_d + cY_i;

  // clip
  lastSentControl.roll = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.roll*agr)));
  lastSentControl.pitch = std::min(max_rp,std::max(-max_rp,(double)(lastSentControl.pitch*agr)));

  // GAZ
  double gazP = Kp_gaz * p_term[2];
  double gazD = Kd_gaz * d_term[2];
  double gazI = Ki_gaz * i_term[2];

  lastSentControl.gaz = std::min(max_gaz_rise/rise_fac,std::max(max_gaz_drop, gazP + gazD + gazI));
  if(lastSentControl.gaz > 0) lastSentControl.gaz *= rise_fac;
}

DronePosition PIDController::getLastTarget(void)
{
  return lastTarget;
}

TooN::Vector<4> PIDController::getLastError(void)
{
  return last_error;
}

ControlCommand PIDController::getLastSentControl(void)
{
  return lastSentControl;
}

