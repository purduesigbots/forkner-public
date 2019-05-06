#include "main.h"

using namespace okapi;

namespace test {
void purepursuitAngle() {
  odom::start();
  odom::state c;
  c.theta = 110_deg;
  // c.theta = 90_deg;
  // c.theta = -180_deg;
  pros::delay(1000);
  odom::setState(c);
  pros::delay(100);
  drive::generateDrivePath({Point{0_in, 0_in, 0_rad},
                            Point{24_in, -24_in, 0_rad},
                            Point{40_in, -24_in, 0_rad}},
                           "Test", DRIVE_LINEAR_MAX_VEL - 0.3, 2.0, 2.0);
  odom::state expectedState;
  // expectedState.theta = -90_deg;
  expectedState.theta = 90_deg;
  // expectedState.theta = PI / 2;
  pros::delay(100);
  drive::setDriveTarget("Test", expectedState);
  drive::waitUntilSettled(0);
  odom::stop();
}
} // namespace test
