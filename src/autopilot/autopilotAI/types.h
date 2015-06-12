#ifndef AUTOPILOT_TYPES_H
#define AUTOPILOT_TYPES_H

typedef const char* messaget;

typedef struct {
  double x;
  double y;
  double z;
  double yaw;
} positiont;

typedef struct {
  double roll;
  double pitch;
  double yaw;
  double gaz;
} control_commandt;

#endif /* AUTOPILOT_TYPES_H */
