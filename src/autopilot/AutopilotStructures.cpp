#include "AutopilotStructures.h"

positiont getDifference(positiont origin, positiont target)
{
  positiont diff = {
    target.x - origin.x,
    target.y - origin.y,
    target.z - origin.z,
    target.yaw - origin.yaw,
  };
  return diff;
}

double getNormSquared(positiont pos)
{
  return (pos.x * pos.x) + (pos.y * pos.y) + (pos.z * pos.z);// + (pos.yaw * pos.yaw) );
}

