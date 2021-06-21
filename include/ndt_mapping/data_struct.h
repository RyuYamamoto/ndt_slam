#ifndef _DATA_STRUCT_
#define _DATA_STRUCT_

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  Pose() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
  Pose(double x, double y, double z, double roll, double pitch, double yaw)
  : x(x), y(y), z(y), roll(roll), pitch(pitch), yaw(yaw)
  {
  }
};

#endif
