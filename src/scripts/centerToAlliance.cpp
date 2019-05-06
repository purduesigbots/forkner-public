/**
 * Source for the center column first auton script..
 *
 * Goal: Shoot all center flags, flip a cap, then hit all alliance side flags.
 *
 * Back Left of the field (Red side furthest wall from flags) is the (0,0)
 * point. Facing the flags is 0 theta.
 *
 * Grep points:
 * - From Ball Under Cap
 * - Turn To Center Flags
 * - From Alliance Wall Align
 * - Turn To Alliance Flags
 */
#include "main.h"

using namespace okapi;

namespace auton {
void caConfig() {
	lcdSelector::titles.push_back("Center->Alliance");
	lcdSelector::inits.push_back(auton::centerToAllianceInit);
	lcdSelector::scripts.push_back(auton::centerToAlliance);
}

void centerToAllianceInit(auton::color side) {
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{38_in, 0_in, 0_rad}},
	    "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL - 0.2, 2.0, 2.0);
	drive::generateLinearDrivePath({0, 13 * IN_TO_METERS}, "From Ball Under Cap");
	drive::generateTurnPath({0, PI / 2}, "Turn To Platform Align");
}

void centerToAlliance(auton::color side, uint32_t allianceFireTime,
                      uint32_t centerFireTime) {
	int32_t exitCode;
	odom::start(true);
	flywheel::start();
	uint32_t scriptStartTime = pros::millis();
	pros::delay(50);  // XXX: for some reason VexOS returns garbage encoder
	                  // values for the first ~100ms

	// put the fire times in ms
	allianceFireTime *= 1000;
	centerFireTime *= 1000;
	// Grab the ball off of the platform first if we need to fire the last shots
	// in too little to pick up a second ball after the alliance high shot
	bool platformBallFirst = false;
	if (allianceFireTime) platformBallFirst = true;

	// This variable will be used throughout the program to specify where we
	// think the robot ought to be. This will be used as the starting spot for
	// the profiled movements.
	expectedState.y = 2.2_m;  // was 2.0 for other script
	if (side == red) {
		expectedState.x = 0.38_m;
		expectedState.theta = 90_deg;
	} else {
		expectedState.x = 3.277_m;
		expectedState.theta = -90_deg;
	}

	odom::setState(expectedState);

	flywheel::setVelocitySetpoint(FLYWHEEL_SPEED_OP_CONTROL);  // was auton high
	flywheel::indexOne(6000);

	/**************************************************************************
	 *
	 * Grab the Ball From Under the Cap
	 *
	 **************************************************************************/
	drive::setDriveTarget("To Ball Under Cap", expectedState);
	flywheel::intake(12000);
	drive::waitUntilSettled(2500);
	flywheel::intake(
	    12000);  // stops after this movement for some reason without this...?
	if (side == red) {
		expectedState.x += 38_in;
	} else {
		expectedState.x -= 38_in;
	}

	flipper::tip();
	flywheel::intake(12000);
	pros::delay(500);

	drive::setLinearDriveTarget("From Ball Under Cap", expectedState, true);
	flipper::down();
	drive::freeDrivePath("To Ball Under Cap");
	drive::generateTurnPath({0, 0.3}, "Turn To Center Flags");
	drive::waitUntilSettled(2000);
	if (side == red) {
		expectedState.x -= 13_in;
	} else {
		expectedState.x += 13_in;
	}

	/**************************************************************************
	 *
	 * Align to Parking Platform
	 *
	 **************************************************************************/
	if (side == red) {
		drive::setTurnTarget("Turn To Platform Align", expectedState, true);
	} else {
		drive::setTurnTarget("Turn To Platform Align", expectedState);
	}
	flipper::retract();
	drive::freeDrivePath("From Ball Under Cap");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{10_in, 0_in, 0_rad}},
	    "Align To Platform");
	drive::waitUntilSettled(1500);
	expectedState.theta = 0_rad;
	if (side == red) {
		expectedState.x += .75_in;
		expectedState.y -= .75_in;
	} else {
		expectedState.x += .75_in;
		expectedState.y -= .75_in;
	}

	drive::setDriveAMGains(0, 0, 0, 0, 0, 1.0, 10, 0);
	drive::setDriveMaxCurrent(DRIVE_ALIGN_CUR);
	drive::setDriveTarget("Align To Platform", expectedState, true);
	drive::freeTurnPath("Turn To Platform Align");
	drive::waitUntilSettled(0);
	drive::setDriveGainDefaults();
	drive::setDriveMaxCurrent(2500);
	expectedState.y = 1.93_m;  // distance to the front of the center platform

	// Angle To Center Flags - Grep point
	const double yDistToFlags = 70 * IN_TO_METERS;  // was 45
	double xDistFromWallToFlags;
	if (side == red) {
		xDistFromWallToFlags = 65 * IN_TO_METERS;  // tune this
	} else {
		xDistFromWallToFlags = 69 * IN_TO_METERS;  // tune this
	}
	double angle =
	    PI / 2 - atan2(yDistToFlags, abs(xDistFromWallToFlags -
	                                     odom::getState().x.convert(meter)));
	if (side == blue) angle *= -1;
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {14_in, 0_in, 0_rad}},
	                         "Align To Center Flags");

	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	odom::state curState;
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	curState.theta = expectedState.theta;
	// This is risky, but it currently works the best. The odometry isn't accurate
	// for long enough to go without a reset
	odom::setState(curState);

	/**************************************************************************
	 *
	 * Fire at Center Flags
	 *
	 **************************************************************************/
	expectedState.theta = QAngle(angle);
	drive::setDriveTarget("Align To Center Flags", expectedState, false, false,
	                      true);
	drive::freeDrivePath("Align To Platform");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{56_in, 0_in, 0_rad}},
	    "To Center Low Flag", DRIVE_LINEAR_MAX_VEL - 0.2, 2.4, 6.0);
	drive::waitUntilSettled(3000);
	drive::setDriveGainDefaults();
	expectedState.x += odom::getXLength(12_in, expectedState.theta);
	expectedState.y += odom::getYLength(12_in, expectedState.theta);

	// expectedState.x = odom::getState().x;
	pros::delay(400);
	if (centerFireTime) {
		int32_t curElapsedTime = pros::millis() - scriptStartTime;
		centerFireTime *= 1000;
		if (centerFireTime > curElapsedTime) {
			pros::delay(centerFireTime - curElapsedTime);
		}
	}
	flywheel::doubleShootBlocking(false);
	pros::delay(200);

	if (side == red) {
		expectedState.theta = 0.1_rad;
	} else {
		expectedState.theta = -0.1_rad;
	}
	drive::setDriveTarget("To Center Low Flag", expectedState);
	flywheel::intake(1000);
	drive::freeTurnPath("Turn To Center Flags");
	drive::generateDrivePath({Point{0_in, 0_in, 0_rad}, Point{5_in, 0_in, 0_rad}},
	                         "Back Up To Align");
	exitCode = drive::waitUntilSettled(0);

	// if (exitCode == DRIVE_EXIT_LINE) {
	// 	flywheel::stop();
	// 	odom::stop();
	// 	return;  // end the script if we hit the line
	// }
	expectedState.y = 3.5_m;  // Won't be exactly 12 feet because the encoder
	                          // wheels aren't at the front of the robot
	expectedState.theta = 0_rad;

	QLength prevEX = expectedState.x;
	expectedState.y = odom::getState().y;
	expectedState.x = odom::getState().x;
	expectedState.theta = 0_rad;

	flywheel::intake(12000);

	drive::setDriveTarget("Back Up To Align", expectedState, true);
	drive::generateLinearDrivePath({0, 10 * IN_TO_METERS}, "Align To Wall");
	drive::waitUntilSettled(0);
	expectedState.y -= 5_in;

	drive::setLinearDriveTarget("Align To Wall", expectedState);
	drive::freeDrivePath("Back Up To Align");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{55_in, 0_in, 0_rad}},
	    "From Center Low Flag");
	drive::waitUntilSettled(1000);
	expectedState.y = 3.47_m;  // Won't be exactly 12 feet because the encoder
	                           // wheels aren't at the front of the robot
	expectedState.x = prevEX;
	expectedState.theta = 0_rad;

	pros::delay(300);

	// Calibrate odometry after the wall hit
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	curState.theta = 0_rad;
	odom::setState(curState);
	pros::delay(10);  // to make sure that the odometry is right before we start
	                  // next movement

	/**************************************************************************
	 *
	 * Grab First Ball Off of Cap
	 *
	 **************************************************************************/
	if (side == red) {
		expectedState.theta = 0.05_rad;
	} else {
		expectedState.theta = -0.05_rad;
	}
	drive::setDriveGainDefaults();
	drive::setDriveTarget("From Center Low Flag", expectedState, true);
	drive::freeDrivePath("To Center Low Flag");
	drive::generateTurnPath({0, 0.65}, "Turn To First Cap Ball");
	drive::waitUntilSettled(3000);
	if (side == red) {
		expectedState.theta = 0.1_rad;
	} else {
		expectedState.theta = -0.1_rad;
	}
	expectedState.x -= odom::getXLength(55_in, expectedState.theta);
	expectedState.y -= odom::getYLength(55_in, expectedState.theta);
	expectedState.theta = 0_rad;  // since the previous drive was crooked
	// can possibly shift left a bit here if it would help

	drive::setTurnGainsOverDamped();
	if (side == red) {
		drive::setTurnTarget("Turn To First Cap Ball", expectedState, true);
	} else {
		drive::setTurnTarget("Turn To First Cap Ball", expectedState);
	}
	flipper::tip();
	drive::freeDrivePath("From Center Low Flag");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{24_in, 0_in, 0_rad}},
	    "To First Cap Ball");
	// check the intake here?
	drive::waitUntilSettled(1000);
	if (side == red) {
		expectedState.y += 0.75_in;
		expectedState.x -= 1_in;
		expectedState.theta = -0.55_rad;
	} else {
		expectedState.y += 0.75_in;
		expectedState.x -= 1_in;
		expectedState.theta = 0.55_rad;
	}

	flywheel::setIntakeLift(HIGH);
	flywheel::intake(12000);
	flywheel::indexOne();
	drive::setDriveAMGains(0, 0, 0, 0, 0.65, 0.25, 0,
	                       0);  // lower lag comp so the move is less jerky
	drive::setDriveTarget("To First Cap Ball", expectedState);
	drive::freeTurnPath("Turn To First Cap Ball");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{13_in, 0_in, 0_rad}},
	    "From First Cap Ball");
	drive::waitUntilSettled(2000);
	drive::setDriveGainDefaults();
	expectedState.x += odom::getXLength(24_in, expectedState.theta);
	expectedState.y += odom::getYLength(24_in, expectedState.theta);

	drive::setDriveTarget("From First Cap Ball", expectedState, true);
	drive::freeDrivePath("To First Cap Ball");
	drive::generateTurnPath({0, PI / 4}, "Turn To Alliance Wall Align");
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {15_in, 0_in, 0_rad}},
	                         "To Cap Flip", DRIVE_LINEAR_MAX_VEL, 10.0, 30.0);
	pros::delay(500);
	flywheel::setIntakeLift(LOW);  // try to flip the ball towards the low flag
	drive::waitUntilSettled(0);
	flipper::down();
	expectedState.x -= odom::getXLength(13_in, expectedState.theta);
	expectedState.y -= odom::getYLength(13_in, expectedState.theta);

	/**************************************************************************
	 *
	 * Flip the Cap
	 *
	 **************************************************************************/
	if (side == red) {
		expectedState.theta = -0.75_rad;
	} else {
		expectedState.theta = 0.75_rad;
	}
	drive::setDriveTarget("To Cap Flip", expectedState);
	flywheel::intake(0);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{20_in, 0_in, 0_rad}}, "Flip Cap",
	    DRIVE_LINEAR_MAX_VEL, 10.0, 30.0);
	drive::waitUntilSettled(0);
	expectedState.x += odom::getXLength(15_in, expectedState.theta);
	expectedState.y += odom::getYLength(15_in, expectedState.theta);

	flipper::upBlocking();

	drive::setDriveTarget("Flip Cap", expectedState);
	drive::freeDrivePath("To Cap Flip");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{36_in, 0_in, 0_rad}}, "From Cap Flip");
	drive::waitUntilSettled(0, false, true);
	expectedState.x += odom::getXLength(21_in, expectedState.theta);
	expectedState.y += odom::getYLength(21_in, expectedState.theta);

	pros::delay(20);  // give pure pursuit some time to get done so the bot
	                  // doesn't freak out

	drive::setDriveTarget("From Cap Flip", expectedState, true);
	drive::freeDrivePath("Flip Cap");
	drive::waitUntilSettled(0);
	expectedState.x -= odom::getXLength(36_in, expectedState.theta);
	expectedState.y -= odom::getYLength(36_in, expectedState.theta);
	flywheel::intake(12000);
	flipper::retract();

	/**************************************************************************
	 *
	 * Align to the Alliance Side Wall
	 *
	 **************************************************************************/
	if (side == red) {
		drive::setTurnTarget("Turn To Alliance Wall Align", expectedState, true);
	} else {
		drive::setTurnTarget("Turn To Alliance Wall Align", expectedState);
	}
	flywheel::intake(12000);
	drive::freeDrivePath("From First Cap Ball");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{48_in, 0_in, 0_rad}},
	    "To Alliance Wall Align", DRIVE_LINEAR_MAX_VEL, 1.5, 3.0);
	drive::waitUntilSettled(1000);
	drive::setTurnGainsDefault();
	if (side == red) {
		expectedState.theta = -90_deg;
	} else {
		expectedState.theta = 90_deg;
	}

	flywheel::intake(12000);
	if (side == red) {
		expectedState.y -= 1_in;  // we undershoot the flag otherwise
	} else {
		expectedState.y -= 1_in;  // we undershoot the flag otherwise
	}
	drive::setDriveTarget("To Alliance Wall Align", expectedState);
	drive::freeTurnPath("Turn To Alliance Wall Align");
	drive::generateLinearDrivePath({0, 9 * IN_TO_METERS},
	                               "From Alliance Wall Align");
	drive::waitUntilSettled(2000);
	if (side == red) {
		expectedState.x = 0.2_m;  // check this
		expectedState.y += 2_in;
	} else {
		expectedState.x = 3.4_m;  // check this
		expectedState.y += 2_in;
	}

	flywheel::intake(-9000);
	pros::delay(150);  // if we accidentally grab three balls then this will spit
	                   // out the third
	flywheel::intake(12000);
	flywheel::indexOne();

	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	// Calibrate odometry after the wall hit
	curState.x = expectedState.x;
	curState.y = odom::getState().y;
	curState.theta = expectedState.theta;
	odom::setState(curState);
	pros::delay(10);  // to make sure that the odometry is right before we start
	                  // next movement

	drive::setLinearDriveTarget("From Alliance Wall Align", expectedState, true);
	drive::freeDrivePath("To Alliance Wall Align");
	flywheel::indexOne(12000);
	drive::waitUntilSettled(2000);
	if (side == red) {
		expectedState.x += 9_in;
	} else {
		expectedState.x -= 9_in;
	}

	bool ballForAllianceHigh = true;
	if (!flywheel::indexerCheck()) ballForAllianceHigh = false;

	if (ballForAllianceHigh && !platformBallFirst) {
		if (side == red) {
			drive::generateTurnPath({0, 1.75}, "Turn To Alliance Flags");
		} else {
			drive::generateTurnPath({0, 1.75}, "Turn To Alliance Flags");
		}
	} else {
		drive::generateTurnPath({0, 2.15}, "Turn To Platform Ball");
	}

	/**************************************************************************
	 *
	 * Fire at the High Alliance Flag if we can
	 *
	 **************************************************************************/
	drive::setTurnGainsDefault();
	if (ballForAllianceHigh && !platformBallFirst) {
		drive::setTurnGainsOverDamped();
		if (side == red) {
			drive::setTurnTarget("Turn To Alliance Flags", expectedState);
		} else {
			drive::setTurnTarget("Turn To Alliance Flags", expectedState, true);
		}
		drive::freeDrivePath("From Alliance Wall Align");
		if (side == red) {
			drive::generateTurnPath({0, 2.0}, "Turn To Platform Ball");
		} else {
			// The platform is slightly further from the wall when on blue
			drive::generateTurnPath({0, 2.0}, "Turn To Platform Ball");
		}
		drive::waitUntilSettled(2000);
		drive::setTurnGainsDefault();
		expectedState.theta = 0_rad;

		flywheel::intake(-9000);
		pros::delay(100);
		flywheel::intake(12000);
		pros::delay(150);  // some extra time to help the bot settle
		flywheel::shootBlocking();
		pros::delay(300);  // make sure the ball is through before we drive forward
	}

	/**************************************************************************
	 *
	 * Regardless of whether or not this will be our first or second shot, get the
	 *ball off the platform
	 *
	 **************************************************************************/
	if (ballForAllianceHigh && !platformBallFirst) {
		if (side == red) {
			drive::setTurnTarget("Turn To Platform Ball", expectedState);
		} else {
			drive::setTurnTarget("Turn To Platform Ball", expectedState, true);
		}
		flipper::tip();
		drive::freeDrivePath("From Alliance Wall Align");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{30_in, 0_in, 0_rad}},
		    "To Platform Ball");
		drive::waitUntilSettled(1500);
		if (side == red) {
			expectedState.theta = 2.25_rad;
		} else {
			expectedState.theta = -2.25_rad;
		}
	} else {
		if (side == red) {
			drive::setTurnTarget("Turn To Platform Ball", expectedState, true);
		} else {
			drive::setTurnTarget("Turn To Platform Ball", expectedState);
		}
		flipper::tip();
		drive::freeDrivePath("From Alliance Wall Align");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{34_in, 0_in, 0_rad}},
		    "To Platform Ball");
		drive::waitUntilSettled(1500);
		odom::state c = odom::getState();
		if (side == red) {
			expectedState.theta = 2.35_rad;
			c.theta += 360_deg;
		} else {
			expectedState.theta = -2.35_rad;
			c.theta -= 360_deg;
		}
		odom::setState(c);
	}

	drive::setDriveTarget("To Platform Ball", expectedState);
	flywheel::setIntakeLift(HIGH);
	pros::delay(300);
	flywheel::intake(12000, true);
	drive::freeTurnPath("Turn To Platform Ball");
	if (ballForAllianceHigh && !platformBallFirst) {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{32_in, 0_in, 0_rad}},
		    "From Platform Ball", DRIVE_LINEAR_MAX_VEL, 2.4, 2.5);
	} else {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{33_in, 0_in, 0_rad}},
		    "From Platform Ball", DRIVE_LINEAR_MAX_VEL, 2.4, 2.5);
	}
	flywheel::intake(12000, true);
	drive::waitUntilSettled(0);
	if (ballForAllianceHigh && !platformBallFirst) {
		expectedState.x += odom::getXLength(32_in, expectedState.theta);
		expectedState.y += odom::getYLength(32_in, expectedState.theta);
	} else {
		expectedState.x += odom::getXLength(34_in, expectedState.theta);
		expectedState.y += odom::getYLength(34_in, expectedState.theta);
	}

	flywheel::intake(12000, true);
	pros::delay(300);

	if (side == red) {
		expectedState.theta = 2.25_rad;
	} else {
		expectedState.theta = -2.25_rad;
	}
	drive::setDriveTarget("From Platform Ball", expectedState,
	                      true /*, false, true*/);  // travel same path in reverse
	drive::freeDrivePath("To Platform Ball");
	drive::generateTurnPath({0, 2.15}, "Turn To Alliance Flags");
	pros::delay(400);
	flywheel::setIntakeLift(LOW);
	pros::delay(300);
	flywheel::indexOne();
	drive::waitUntilSettled(3000);
	flipper::retract();
	if (ballForAllianceHigh && !platformBallFirst) {
		expectedState.x -= odom::getXLength(32_in, expectedState.theta);
		expectedState.y -= odom::getYLength(32_in, expectedState.theta);
	} else {
		expectedState.x -= odom::getXLength(33_in, expectedState.theta);
		expectedState.y -= odom::getYLength(33_in, expectedState.theta);
	}

	if (side == red) {
		drive::setTurnTarget("Turn To Alliance Flags", expectedState, true);
	} else {
		drive::setTurnTarget("Turn To Alliance Flags", expectedState);
	}
	drive::freeDrivePath("To Platform Ball");
	if (platformBallFirst) {
		// hit the middle flag next if time is of the essence
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{19_in, 0_in, 0_rad}},
		    "To Alliance Middle");
	} else {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{19_in, 0_in, 0_rad}},
		    "To Alliance Middle");
	}
	drive::waitUntilSettled(2500);
	expectedState.theta = 0_rad;

	/**************************************************************************
	 *
	 * Fire at the High Alliance Flag if we haven't already
	 *
	 **************************************************************************/
	if (allianceFireTime >= 41000) {
		if (allianceFireTime) {
			uint32_t timeSinceStart = pros::millis() - scriptStartTime;
			if (allianceFireTime > timeSinceStart)
				pros::delay(allianceFireTime - timeSinceStart);
			else
				pros::delay(200);
		} else {
			pros::delay(200);  // some extra time to help the bot settle
		}

		flywheel::doubleShootBlocking();

		flywheel::stop();
		odom::stop();
		return;
	} else if (!ballForAllianceHigh || platformBallFirst) {
		if (allianceFireTime) {
			uint32_t timeSinceStart = pros::millis() - scriptStartTime;
			if (allianceFireTime > timeSinceStart)
				pros::delay(allianceFireTime - timeSinceStart);
			else
				pros::delay(300);
		} else {
			pros::delay(300);  // some extra time to help the bot settle
		}

		flywheel::shootBlocking();
		pros::delay(500);  // make sure the ball is through before we drive forward
	}

	/**************************************************************************
	 *
	 * Hit the Alliance Low Flag
	 *
	 **************************************************************************/
	flywheel::intake(12000);
	// turn towards the flags slightly for shot consistency
	if (side == red) {
		expectedState.theta = 0.1_rad;
	} else {
		expectedState.theta = -0.1_rad;
	}
	drive::setDriveTarget("To Alliance Middle", expectedState);
	flywheel::setVelocitySetpoint(FLYWHEEL_SPEED_AUTON_MIDDLE);
	drive::waitUntilSettled(2000);
	expectedState.x += odom::getXLength(20_in, expectedState.theta);
	expectedState.y += odom::getYLength(20_in, expectedState.theta);
	// try to drive slightly away from the poles
	if (side == red) {
		expectedState.theta = -0.25_rad;
	} else {
		expectedState.theta = 0.25_rad;
	}

	uint32_t now = pros::millis();
	drive::freeDrivePath("To Alliance Middle");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{22_in, 0_in, 0_rad}},
	    "To Low Alliance Flag");
	uint32_t timeDiff = (pros::millis() - now) < 200 ? pros::millis() - now : 200;
	pros::delay(500 -
	            timeDiff);  // this will help the bot be stable before shooting

	flywheel::shootBlocking();  // middle flag

	flywheel::intake(12000);
	drive::setDriveTarget("To Low Alliance Flag", expectedState);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{24_in, 0_in, 0_rad}},
	    "From Low Alliance Flag");
	drive::waitUntilSettled(0);
	expectedState.y += 25_in;

	drive::setDriveTarget("From Low Alliance Flag", expectedState, true);
	drive::freeDrivePath("To Low Alliance Flag");
	drive::waitUntilSettled(0);

	flywheel::stop();
	odom::stop();
}
}  // namespace auton
