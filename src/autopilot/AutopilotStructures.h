#pragma once

#ifndef __AUTOPILOT_STRUCTURES_H
#define __AUTOPILOT_STRUCTURES_H

extern "C" {
  #include "autopilotAI/types.h"
}

// TODO: can we use aggregation initialization, a C++11 feature...?
struct ControlCommand : public control_commandt
{
  ControlCommand() /*: 
    control_commandt{0, 0, 0, 0}*/
  {
    roll = 0;
    pitch = 0;
    yaw = 0;
    gaz = 0;
  }

  ControlCommand(
    double _roll, 
    double _pitch, 
    double _yaw, 
    double _gaz) /*: 
      control_commandt{ _roll, _pitch, _yaw, _gaz}*/
  {
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    gaz = _gaz;
  }

  ControlCommand(
    control_commandt cmd)
  {
    roll = cmd.roll;
    pitch = cmd.pitch;
    yaw = cmd.yaw;
    gaz = cmd.gaz;
  }
};

struct DronePosition : public positiont
{
  inline DronePosition(
    double _x, 
    double _y, 
    double _z, 
    double _yaw) /*:
      positiont{ _x, _y, _z, _yaw }*/
  {
    x = _x;
    y = _y;
    z = _z;
    yaw = _yaw;
  }

  inline DronePosition() /*:
    positiont{ 0, 0, 0, 0}*/
  { 
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;
  }

  inline DronePosition(
    positiont position)
  {
    x = position.x;
    y = position.y;
    z = position.z;
    yaw = position.yaw;
  }

  bool operator !=(const DronePosition &that) const
  {
     // TODO: problems with double comparison?
     return x != that.x || y != that.y || z != that.z || yaw != that.yaw;
  }

  DronePosition operator -(const DronePosition &that) const
  {
    DronePosition difference(
      this->x - that.x,
      this->y - that.y,
      this->z - that.z,
      this->yaw - that.yaw);
    return difference;
  }
};

struct DroneSpeed
{
  double dx, dy, dz, dyaw;

  inline DroneSpeed(
    double _dx,
    double _dy,
    double _dz,
    double _dyaw) :
    dx(_dx),
    dy(_dy),
    dz(_dz),
    dyaw(_dyaw)
  {
  }

  inline DroneSpeed() :
    dx(0),
    dy(0),
    dz(0),
    dyaw(0)
  {
  }
};

positiont getDifference(positiont, positiont);
double getNormSquared(positiont);

#endif /* __AUTOPILOT_STRUCTURES_H */
