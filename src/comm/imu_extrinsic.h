#ifndef LIVOX_ROS_DRIVER2_IMU_EXTRINSIC_H_
#define LIVOX_ROS_DRIVER2_IMU_EXTRINSIC_H_

#include <cmath>

namespace livox_ros {

struct ImuRotation {
  double r[3][3];
};

inline ImuRotation MakeImuRotation(double roll_deg, double pitch_deg, double yaw_deg) {
  const double kDeg = M_PI / 180.0;
  const double cr = std::cos(roll_deg * kDeg), sr = std::sin(roll_deg * kDeg);
  const double cp = std::cos(pitch_deg * kDeg), sp = std::sin(pitch_deg * kDeg);
  const double cy = std::cos(yaw_deg * kDeg), sy = std::sin(yaw_deg * kDeg);
  ImuRotation m;
  m.r[0][0] = cp * cy;
  m.r[0][1] = sr * sp * cy - cr * sy;
  m.r[0][2] = cr * sp * cy + sr * sy;
  m.r[1][0] = cp * sy;
  m.r[1][1] = sr * sp * sy + cr * cy;
  m.r[1][2] = cr * sp * sy - sr * cy;
  m.r[2][0] = -sp;
  m.r[2][1] = sr * cp;
  m.r[2][2] = cr * cp;
  return m;
}

// out may alias in.
inline void ApplyImuRotation(const ImuRotation& m, const float in[3], float out[3]) {
  const double x = in[0], y = in[1], z = in[2];
  out[0] = static_cast<float>(m.r[0][0] * x + m.r[0][1] * y + m.r[0][2] * z);
  out[1] = static_cast<float>(m.r[1][0] * x + m.r[1][1] * y + m.r[1][2] * z);
  out[2] = static_cast<float>(m.r[2][0] * x + m.r[2][1] * y + m.r[2][2] * z);
}

}  // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER2_IMU_EXTRINSIC_H_
