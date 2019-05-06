#include "main.h"

using namespace okapi;

namespace test {
void driveTuning() {
	odom::start();
	pros::delay(150);
	odom::state start;
	odom::setState(start);

	// LMPFTuner tuner(drive::linearController,
	// 								false,
	// 								4_s,
	// 								1,
	// 								0.5,
	// 								1.0,
	// 								0.5,
	// 								0.8,
	// 								0.0001,
	// 								0.001,
	// 								0.1,
	// 								1.0,
	// 								TimeUtilFactory::withSettledUtilParams(DRIVE_LINEAR_POS_TOLERANCE,
	// DRIVE_LINEAR_DERIV_TOLERANCE), 								5, 								20,
	// 2, 1);

	// LMPFTuner::Output out = tuner.autotune();
	// pros::lcd::print(7, "f %f, p %f, i %f, d %f", out.kF, out.kP, out.kI,
	// out.kD); drive::linearController->generatePath({0, 1}, "Test");
	// drive::setLinearDriveTarget("Test", start, drive::direction::y);
	// drive::waitUntilSettled(0);

	odom::stop();
}
}  // namespace test
