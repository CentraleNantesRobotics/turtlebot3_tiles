#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <vector>

#define TURTLEBOT

namespace camera
{

#ifdef TURTLEBOT
constexpr auto px{501.39729};
constexpr auto py{499.81892};
constexpr auto u0{315.43596};
constexpr auto v0{232.04706};
constexpr double dist_c[]{0.147004, -0.256822, -0.003018, -0.000727, 0.000000};
#else
constexpr auto px{543.762755};
constexpr auto py{541.752868};
constexpr auto u0{331.044014};
constexpr auto v0{245.939526};
constexpr double dist_c[]{-0.051708,0.047812,-0.002718,0.000733,0.000000};
#endif

inline std::vector<double> dist()
{
  return {dist_c, dist_c+5};
}

}


#endif // CALIBRATION_H
