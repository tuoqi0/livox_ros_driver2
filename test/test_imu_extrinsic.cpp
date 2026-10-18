// The IMU rotation must build the same matrix the point path does; if the two
// drift apart the cloud and the IMU end up in different frames, which is the
// bug this exists to prevent.
#include <robot_core/test/doctest.h>

#include <cmath>

#include "comm/imu_extrinsic.h"

using livox_ros::ApplyImuRotation;
using livox_ros::ImuRotation;
using livox_ros::MakeImuRotation;

namespace {
void Rotate(double roll, double pitch, double yaw, const float in[3],
            float out[3]) {
  ApplyImuRotation(MakeImuRotation(roll, pitch, yaw), in, out);
}
} // namespace

TEST_CASE("ImuExtrinsic.ZeroAnglesLeaveTheReadingAlone") {
  const float in[3] = {1.0f, -2.0f, 3.0f};
  float out[3];
  Rotate(0, 0, 0, in, out);
  CHECK(std::abs(static_cast<double>(out[0]) - static_cast<double>(1.0f)) <=
        (1e-6));
  CHECK(std::abs(static_cast<double>(out[1]) - static_cast<double>(-2.0f)) <=
        (1e-6));
  CHECK(std::abs(static_cast<double>(out[2]) - static_cast<double>(3.0f)) <=
        (1e-6));
}

TEST_CASE("ImuExtrinsic.YawTurnsXTowardsY") {
  const float in[3] = {1.0f, 0.0f, 0.0f};
  float out[3];
  Rotate(0, 0, 90, in, out);
  CHECK(std::abs(static_cast<double>(out[0]) - static_cast<double>(0.0f)) <=
        (1e-6));
  CHECK(std::abs(static_cast<double>(out[1]) - static_cast<double>(1.0f)) <=
        (1e-6));
}

TEST_CASE("ImuExtrinsic.PitchTurnsXTowardsMinusZ") {
  const float in[3] = {1.0f, 0.0f, 0.0f};
  float out[3];
  Rotate(0, 90, 0, in, out);
  CHECK(std::abs(static_cast<double>(out[2]) - static_cast<double>(-1.0f)) <=
        (1e-6));
}

TEST_CASE("ImuExtrinsic.RollTurnsYTowardsZ") {
  const float in[3] = {0.0f, 1.0f, 0.0f};
  float out[3];
  Rotate(90, 0, 0, in, out);
  CHECK(std::abs(static_cast<double>(out[2]) - static_cast<double>(1.0f)) <=
        (1e-6));
}

TEST_CASE("ImuExtrinsic.IsOrthonormal") {
  const ImuRotation m = MakeImuRotation(17.0, -33.0, 121.0);
  for (int i = 0; i < 3; ++i) {
    double norm = 0.0;
    for (int k = 0; k < 3; ++k)
      norm += m.r[i][k] * m.r[i][k];
    CHECK_MESSAGE(std::abs(static_cast<double>(norm) -
                           static_cast<double>(1.0)) <= (1e-12),
                  "row ", i);
    for (int j = i + 1; j < 3; ++j) {
      double dot = 0.0;
      for (int k = 0; k < 3; ++k)
        dot += m.r[i][k] * m.r[j][k];
      CHECK_MESSAGE(std::abs(static_cast<double>(dot) -
                             static_cast<double>(0.0)) <= (1e-12),
                    "rows ", i, ",", j);
    }
  }
  const double det =
      m.r[0][0] * (m.r[1][1] * m.r[2][2] - m.r[1][2] * m.r[2][1]) -
      m.r[0][1] * (m.r[1][0] * m.r[2][2] - m.r[1][2] * m.r[2][0]) +
      m.r[0][2] * (m.r[1][0] * m.r[2][1] - m.r[1][1] * m.r[2][0]);
  CHECK(std::abs(static_cast<double>(det) - static_cast<double>(1.0)) <=
        (1e-12));
}

TEST_CASE("ImuExtrinsic.MatchesThePointPathFormula") {
  // Mirrors LidarPubHandler::SetLidarsExtParam verbatim.
  const double roll = 12.0, pitch = -25.0, yaw = 47.0, kPi = M_PI;
  const double cos_roll = cos(roll * kPi / 180.0),
               sin_roll = sin(roll * kPi / 180.0);
  const double cos_pitch = cos(pitch * kPi / 180.0),
               sin_pitch = sin(pitch * kPi / 180.0);
  const double cos_yaw = cos(yaw * kPi / 180.0),
               sin_yaw = sin(yaw * kPi / 180.0);
  double point[3][3];
  point[0][0] = cos_pitch * cos_yaw;
  point[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  point[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
  point[1][0] = cos_pitch * sin_yaw;
  point[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  point[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
  point[2][0] = -sin_pitch;
  point[2][1] = sin_roll * cos_pitch;
  point[2][2] = cos_roll * cos_pitch;

  const ImuRotation m = MakeImuRotation(roll, pitch, yaw);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      CHECK_MESSAGE(std::abs(static_cast<double>(m.r[i][j]) -
                             static_cast<double>(point[i][j])) <= (1e-12),
                    i, ",", j);
}

TEST_CASE("ImuExtrinsic.ATiltedMountReportsGravityBackAlongMinusZ") {
  const double pitch = 30.0;
  const float in[3] = {static_cast<float>(-9.81 * sin(pitch * M_PI / 180.0)),
                       0.0f,
                       static_cast<float>(-9.81 * cos(pitch * M_PI / 180.0))};
  float out[3];
  Rotate(0, -pitch, 0, in, out);
  CHECK(std::abs(static_cast<double>(out[0]) - static_cast<double>(0.0f)) <=
        (1e-5));
  CHECK(std::abs(static_cast<double>(out[1]) - static_cast<double>(0.0f)) <=
        (1e-5));
  CHECK(std::abs(static_cast<double>(out[2]) - static_cast<double>(-9.81f)) <=
        (1e-5));
}

TEST_CASE("ImuExtrinsic.OutputMayAliasInput") {
  float v[3] = {1.0f, 0.0f, 0.0f};
  ApplyImuRotation(MakeImuRotation(0, 0, 90), v, v);
  CHECK(std::abs(static_cast<double>(v[0]) - static_cast<double>(0.0f)) <=
        (1e-6));
  CHECK(std::abs(static_cast<double>(v[1]) - static_cast<double>(1.0f)) <=
        (1e-6));
}
