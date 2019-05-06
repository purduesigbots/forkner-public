#include "main.h"

namespace test {
void purepursuitTuning() {
	odom::start();
	odom::state c;
	c.theta = -90_deg;
	pros::delay(1000);
	odom::setState(c);
	// c.theta = 90_deg;
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{24_in, -27_in, 0_rad},
	     Point{44_in, -27_in, 0_rad}},
	    "First Path", DRIVE_LINEAR_MAX_VEL, 2.4, 2.0);
	// drive::generateDrivePath(
	//     {Point{0_in, 0_in, 0_rad}, Point{44_in, 0_in, 0_rad}},
	//     "First Path", DRIVE_LINEAR_MAX_VEL, 2.0, 2.0);
	// drive::generateDrivePath({Point{0_ft, 0_ft, 0_deg}, Point{20_in, 0_ft,
	// 0_deg}}, "First Path");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{24_in, 27_in, 0_rad},
	     Point{44_in, 27_in, 0_rad}},
	    "Second Path", DRIVE_LINEAR_MAX_VEL - 0.4, 2.0, 5.0);

	// drive::setDriveAMGains(1.00, 0.35, 0.15, 0, 0, 0, 80);
	drive::setDriveAMGains(1.00, 0.35, 0.15, 170, 0, 0.25, 10, 80);
	pros::delay(100);
	// c.x += 27_in;
	drive::setDriveTarget("First Path", c, true, false, true);
	// drive::setDriveTarget("First Path", c, false, true);
	drive::waitUntilSettled(0);
	pros::lcd::print(7, "%f", drive::driveController->getError());

	odom::stop();
}
}  // namespace test
