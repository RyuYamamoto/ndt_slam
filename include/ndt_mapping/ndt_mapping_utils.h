#pragma 0

#include <iostream>
#include <cmath>
#include <string>

namespace ndt_mapping_utils
{
double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if(M_PI <= diff_rad) diff_rad = diff_rad - 2 * M_PI;
  else if(diff_rad < -M_PI) diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}
}
