#pragma once
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
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H


#include "TooN/se3.h"
#include <queue>
#include "geometry_msgs/Twist.h"
#include "tum_ardrone/filter_state.h"

#include "AutopilotStructures.h"

class PIDController
{
private:
  DronePosition lastTarget;
  ControlCommand lastSentControl;
	
  // used for integral term
  TooN::Vector<4> targetNew;  // 0=target has been reached before
                              // 1=target is new

  // need to keep track of integral terms
  TooN::Vector<4> i_term;
  TooN::Vector<4> last_error;

  double targetSetAtClock;
  double lastTimeStamp;

  void calcControl(
    TooN::Vector<4> new_err, 
    TooN::Vector<4> d_error, 
    double yaw);/*, 
    double scaleAccuracy);*/

  void setNewTarget(DronePosition newTarget);

public:
  PIDController(void);
  ~PIDController(void);

  // generates and sends a new control command to the drone, based on the currently active command ant the drone's position.
  ControlCommand getControlCommand(
    DronePosition position, 
    DroneSpeed speed, 
    DronePosition target);/*,
    double scaleAccuracy);*/

  DronePosition getLastTarget(void);
  TooN::Vector<4> getLastError(void);
  ControlCommand getLastSentControl(void);

  // PID control parameters. settable via dynamic_reconfigure
  double max_yaw;
  double max_rp;
  double max_gaz_rise;
  double max_gaz_drop;

  double rise_fac;
  double aggressiveness;

  double Ki_yaw;
  double Kd_yaw;
  double Kp_yaw;

  double Ki_gaz;
  double Kd_gaz;
  double Kp_gaz;

  double Ki_rp;
  double Kd_rp;
  double Kp_rp;
};
#endif /* __PIDCONTROLLER_H */

