#include "main.h"

namespace test {
void capFlip() {
	odom::start();
	flywheel::start();
	pros::delay(200);
	odom::state c;
	odom::setState(c);
	flipper::down();
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {10_in, 0_in, 0_rad}},
	                         "Align");
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {30_in, 0_in, 0_rad}}, "Test",
	                         DRIVE_LINEAR_MAX_VEL - 0.3, 2.0, 4.0);
	flywheel::intake(-8000);
	drive::setDriveTarget("Align", c);
	drive::waitUntilSettled(0);
	c.y += 10_in;
	flipper::upBlocking();

	drive::setDriveTarget("Test", c);
	pros::delay(400);
	drive::waitUntilSettled(0, false, true);
	flywheel::intake(0);
}
}  // namespace test
