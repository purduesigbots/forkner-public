/**
 * Line Emergency Stop test
 *
 * Verifies that the robot will stop if the intake crosses the auton line
 */
#include "main.h"

using namespace okapi;

namespace test {
void lineEStop() {
	odom::start();
	odom::state expectedState;
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{40_in, 0_in, 0_rad}},
	    "Cross The Auton Line");
	drive::setDriveTarget("Cross The Auton Line", expectedState);
	drive::waitUntilSettled(0, true);
	odom::stop();
}
}  // namespace test
