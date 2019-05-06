#include "main.h"

#define INTAKE_FLIP_SPEED -5000

using namespace okapi;

namespace auton {
void psConfig() {
	lcdSelector::titles.push_back("Prog Skills");
	lcdSelector::inits.push_back(auton::progSkillsInit);
	lcdSelector::scripts.push_back(auton::progSkills);
	lcdSelector::titles.push_back("Prog Skills Reckless");
	lcdSelector::inits.push_back(auton::progSkillsInit);
	lcdSelector::scripts.push_back(auton::progSkillsReckless);
}

void progSkillsInit(color side) {
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{42_in, 0_in, 0_rad}},
	    "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{51_in, 0_in, 0_rad}},
	    "From Ball Under Cap", DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
	drive::generateTurnPath({0, 1.57}, "Turn To Flags");
}

void progSkillsReckless(color side, uint32_t allianceFireTime,
                        uint32_t centerFireTime) {
	progSkillsBase(side, allianceFireTime, centerFireTime, true);
}

void progSkills(color side, uint32_t allianceFireTime,
                uint32_t centerFireTime) {
	progSkillsBase(side, allianceFireTime, centerFireTime, false);
}

void progSkillsBase(color side, uint32_t allianceFireTime,
                    uint32_t centerFireTime, bool reckless) {
	uint32_t now, timeDiff;
	int32_t exitCode;
	uint32_t scriptStartTime = pros::millis();
	odom::start(true);
	flywheel::start();
	flywheel::intake(12000);
	uint32_t startTime = pros::millis();
	pros::delay(50);  // XXX: for some reason VexOS returns garbage encoder
	                  // values for the first ~100ms

	// This variable will be used throughout the program to specify where we
	// think the robot ought to be. This will be used as the starting spot for
	// the profiled movements.
	odom::state expectedState;
	expectedState.y = 1.0_m;
	expectedState.x = 0.38_m;
	expectedState.theta = 90_deg;

	odom::setState(expectedState);

	flywheel::setVelocitySetpoint(FLYWHEEL_SPEED_OP_CONTROL);

	/**************************************************************************
	 *
	 * Get Ball from Under Cap 1
	 *
	 **************************************************************************/
	flywheel::indexOne(12000);
	flipper::tip();
	drive::setDriveTarget("To Ball Under Cap", expectedState);
	drive::waitUntilSettled(0);
	expectedState.x += 42_in;

	drive::setDriveTarget("From Ball Under Cap", expectedState, true);
	flywheel::intake(12000);
	drive::freeDrivePath("To Platform Ball");
	drive::generateDrivePath(
	    {Point{0_ft, 0_ft, 0_deg}, Point{56_in, 0_ft, 0_rad}},
	    "To High Alliance");
	drive::waitUntilSettled(0);
	expectedState.x -= 51_in;

	/**************************************************************************
	 *
	 * Fire at the Alliance flags
	 *
	 **************************************************************************/
	drive::setTurnTarget("Turn To Flags", expectedState, true);
	drive::freeDrivePath("From Platform Ball");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{28_in, 0_in, 0_rad}},
	    "Back From Alliance Low", DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
	drive::waitUntilSettled(1000);
	expectedState.theta = 0_deg;

	expectedState.theta = 0.05_rad;
	drive::setDriveAMGains(0, 0, 0, 0, 0, 0, 0, 100);
	drive::setDriveTarget("To High Alliance", expectedState, false, false, true);
	flywheel::intake(-8000);
	pros::delay(80);
	flywheel::intake(12000);
	flipper::retract();
	drive::freeTurnPath("Turn To Flags");
	drive::generateDrivePath(
	    {Point{0_ft, 0_ft, 0_deg}, Point{60_in, 0_ft, 0_rad}},
	    "To Low Alliance Flag");
	drive::waitUntilSettled(3200);  // TODO: tweak this, was 5000
	expectedState.x += odom::getXLength(56_in, expectedState.theta);
	expectedState.y += odom::getYLength(56_in, expectedState.theta);

	pros::delay(200);
	flywheel::doubleShootBlocking(true);

	/**************************************************************************
	 *
	 * Hit the low flag and align to the flag side wall
	 *
	 **************************************************************************/
	expectedState.theta = -0.05_rad;  // was .18
	flywheel::intake(12000);
	drive::setDriveTarget("To Low Alliance Flag", expectedState);
	drive::freeDrivePath("To High Alliance");
	drive::generateTurnPath({0, 1.57}, "Turn To First Cap", DRIVE_TURN_MAX_VEL,
	                        20.0, 40.0);
	drive::waitUntilSettled(0);
	expectedState.y = 3.47_m;  // Won't be exactly 12 feet because the encoder
	                           // wheels aren't at the front of the robot
	expectedState.x = 0.38_m;  // same as starting pos
	expectedState.theta = 0_rad;

	flywheel::intake(12000);

	// Calibrate odometry after the wall hit
	odom::state curState;
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	double ct = odom::getState().theta.convert(radian);
	if (ct > -0.05 && ct < 0.0) {
		// only reset odom in an acceptable range
		curState.theta = 0_rad;
	} else {
		curState.theta = odom::getState().theta;
	}
	odom::setState(curState);
	pros::delay(10);  // to make sure that the odometry is right before we start
	                  // next movement

	// if we pick up a ball or two when we drive at the wall, then queue them up
	// to shoot
	flywheel::indexOne();

	/**************************************************************************
	 *
	 * Back up from alliance flags
	 *
	 **************************************************************************/
	drive::setDriveTarget("Back From Alliance Low", expectedState, true);
	drive::freeDrivePath("Align To Wall");
	drive::freeDrivePath("To Low Alliance Flag");
	drive::generateDrivePath({Point{0_in, 0_in, 0_rad}, Point{9_in, 0_in, 0_rad}},
	                         "To First Cap Ball");
	drive::waitUntilSettled(0);
	expectedState.y -= 28_in;

	// clearing the intake again just in case
	flywheel::indexOne();

	/**************************************************************************
	 *
	 * Grab the first ball off of the blue cap
	 *
	 **************************************************************************/
	drive::setTurnTarget("Turn To First Cap", expectedState);
	drive::freeDrivePath("Back From Alliance Low");
	if (!flywheel::isIntakeStalled()) flywheel::setIntakeLift(HIGH);
	drive::waitUntilSettled(500);
	// expectedState.theta = 2_rad;
	expectedState.theta = 90_deg;
	expectedState.y += 0.75_in;
	expectedState.x -= 1_in;

	drive::setDriveAMGains(
	    0, 0, 0, 0, 0, 0.2,
	    0);  // with the intake up we're more likely to rock the bot
	drive::setDriveTarget("To First Cap Ball", expectedState);
	flywheel::intake(12000);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{26_in, 0_in, 0_rad}},
	    "From First Cap Ball");
	drive::waitUntilSettled(0);
	drive::setDriveGainDefaults();
	flywheel::intake(12000);  // just in case
	expectedState.x += odom::getXLength(9_in, expectedState.theta);
	expectedState.y += odom::getYLength(9_in, expectedState.theta);
	// use Pure Pursuit to fix our angle
	expectedState.theta = 90_deg;

	flipper::downBlocking();

	expectedState.y -= 5_in;
	drive::setDriveTarget("From First Cap Ball", expectedState, true);
	now = pros::millis();
	drive::freeDrivePath("To First Cap Ball");
	// negative y = positive x
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{32_in, -19_in, 0_rad},
	     Point{55_in, -19_in, 0_rad}},
	    "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL - 0.25, 1.6,
	    6.0);  // was 1.5, 2.0
	timeDiff = (pros::millis() - now) < 200 ? pros::millis() - now : 200;
	pros::delay(600 - timeDiff);
	flywheel::setIntakeLift(LOW);
	drive::waitUntilSettled(0);
	expectedState.x = 0.2_m;  // tune this

	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	curState.x = expectedState.x;
	curState.y = odom::getState().y;
	curState.theta = expectedState.theta;
	odom::setState(curState);
	pros::delay(10);

	/**************************************************************************
	 *
	 * S-Curve to the ball under the red cap
	 *
	 **************************************************************************/
	drive::setDriveAMGains(1.00, 0.35, 0.15, 140, 0, 0.25, 10, 100);
	drive::setDriveTarget(
	    "To Ball Under Cap",
	    expectedState);  // previously had constrained the  end heading
	flipper::tip();
	drive::freeDrivePath("From First Cap Ball");
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {12_in, 0_in, 0_rad}},
	                         "From Ball Under Cap");
	drive::waitUntilSettled(0);
	expectedState.x += 55_in;
	expectedState.y -= 19_in;

	flywheel::intake(12000);

	drive::setDriveGainDefaults();
	drive::setDriveTarget("From Ball Under Cap", expectedState, true);
	drive::freeDrivePath("To Ball Under Cap");
	drive::generateTurnPath({0, PI / 2}, "Turn To Platform Align",
	                        DRIVE_TURN_MAX_VEL, 20.0, 80.0);
	drive::waitUntilSettled(0);
	expectedState.x -= 12_in;

	drive::setTurnTarget("Turn To Platform Align", expectedState, true);
	flipper::retract();
	flywheel::intake(-6000);  // clear any extra balls out of the intake
	pros::delay(80);
	flywheel::intake(12000);
	drive::freeDrivePath("From Ball Under Cap");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{15_in, 0_in, 0_rad}},
	    "Align To Platform");
	drive::waitUntilSettled(1000);  // was 1500
	expectedState.theta = 0_rad;

	drive::setDriveAMGains(0, 0, 0, 0, 0, 1.0, 10);
	// drive::setDriveMaxCurrent(240)
	drive::setDriveMaxCurrent(DRIVE_ALIGN_CUR);
	drive::setDriveTarget("Align To Platform", expectedState, true);
	drive::freeTurnPath("Turn To Platform Align");
	double angle = 0;
	const double yDistToFlags = 75 * IN_TO_METERS;    // was 85
	double xDistFromWallToFlags = 66 * IN_TO_METERS;  // tune this, was 65
	angle = PI / 2 - atan2(yDistToFlags, abs(xDistFromWallToFlags -
	                                         odom::getState().x.convert(meter)));
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {12_in, 0_in, 0_rad}},
	                         "Align To Center Flags");
	drive::waitUntilSettled(0);
	drive::setDriveGainDefaults();
	drive::setDriveMaxCurrent(2500);
	expectedState.y = 1.93_m;  // distance to the front of the center platform

	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	curState.theta = expectedState.theta;
	odom::setState(curState);
	pros::delay(10);

	/**************************************************************************
	 *
	 * Fire at the middle flags
	 *
	 **************************************************************************/
	expectedState.theta = QAngle(angle);
	drive::setDriveTarget("Align To Center Flags", expectedState, false, false,
	                      true);
	flywheel::intake(-8000);
	pros::delay(80);
	flywheel::intake(12000);
	drive::freeDrivePath("Align To Platform");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{56_in, 0_in, 0_rad}},
	    "To Center Low Flag");
	drive::waitUntilSettled(3000);
	drive::setDriveGainDefaults();
	expectedState.x += odom::getXLength(12_in, expectedState.theta);
	expectedState.y += odom::getYLength(12_in, expectedState.theta);

	pros::delay(200);
	flywheel::doubleShootBlocking(false);

	expectedState.theta = 0.05_rad;
	drive::setDriveTarget("To Center Low Flag", expectedState);
	flywheel::intake(1000);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{32_in, 0_in, 0_rad}},
	    "From Center Low Flag", DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
	drive::waitUntilSettled(0);
	expectedState.y = 3.2_m;  // Won't be exactly 12 feet because the encoder
	                          // wheels aren't at the front of the robot
	expectedState.theta = 0_rad;

	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	ct = odom::getState().theta.convert(radian);
	if (ct > -0.05 && ct < 0.0) {
		// only reset odom in an acceptable range
		curState.theta = 0_rad;
	} else {
		curState.theta = odom::getState().theta;
	}
	odom::setState(curState);
	pros::delay(10);  // to make sure that the odometry is right before we start
	                  // next movement

	/**************************************************************************
	 *
	 * Flip cap closest to starting alliance corner
	 *
	 **************************************************************************/
	drive::setDriveTarget("From Center Low Flag", expectedState, true);
	flywheel::intake(12000);
	drive::freeDrivePath("To Center Low Flag");
	drive::generateTurnPath({0, 1.57}, "Turn To Flip Cap", DRIVE_TURN_MAX_VEL,
	                        20.0, 40.0);
	drive::waitUntilSettled(0);
	expectedState.y -= 32_in;

	drive::setTurnTarget("Turn To Flip Cap", expectedState, true);
	flipper::down();
	flywheel::intake(0);
	drive::freeDrivePath("From Center Low Flag");
	// drive::generateDrivePath(
	//     {Point{0_in, 0_in, 0_rad}, Point{8_in, 0_in, 0_rad}}, "Flip Near Cap",
	//     DRIVE_LINEAR_MAX_VEL, 10.0, 30.0);
	// drive::generateDrivePath(
	//     {Point{0_in, 0_in, 0_rad}, Point{12_in, 0_in, 0_rad}}, "To Near Cap",
	//     DRIVE_LINEAR_MAX_VEL, 10.0, 30.0);
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{22_in, 0_in, 0_rad}}, "Flip Cap",
	    DRIVE_LINEAR_MAX_VEL, 10.0, 30.0);
	drive::waitUntilSettled(500);
	expectedState.theta = -90_deg;

	// drive::setDriveTarget("To Near Cap", expectedState);
	// drive::generateDrivePath(
	//     {Point{0_in, 0_in, 0_rad}, Point{82_in, 0_in, 0_rad}}, "Flip Far Cap",
	//     DRIVE_LINEAR_MAX_VEL + 0.2, 5.0, 10.0);
	// drive::waitUntilSettled(500); // had no delay before
	// expectedState.x -= 12_in;
	//
	// flipper::up();
	// drive::setDriveTarget("Flip Near Cap", expectedState);
	// drive::freeDrivePath("To Near Cap");
	// drive::freeTurnPath("Turn To Flip Cap");
	// drive::waitUntilSettled(0);
	// expectedState.x -= 8_in;
	drive::setDriveTarget("Flip Cap", expectedState);
	drive::freeTurnPath("Turn To Flip Cap");
	while (expectedState.x.convert(inch) - odom::getState().x.convert(inch) < 6)
		pros::delay(5);
	flipper::up();
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{84_in, 0_in, 0_rad}}, "Flip Far Cap",
	    DRIVE_LINEAR_MAX_VEL, 5.0, 10.0);
	drive::waitUntilSettled(5000);
	expectedState.x -= 22_in;

	/**************************************************************************
	 *
	 * Flip cap closest to far alliance corner
	 *
	 **************************************************************************/
	drive::setDriveTarget("Flip Far Cap", expectedState, true);
	// drive::freeDrivePath("Flip Near Cap");
	drive::freeDrivePath("Flip Cap");
	drive::generateDrivePath({{0_in, 0_in, 0_rad}, {46_in, 0_in, 0_rad}},
	                         "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL, 2.4, 4.0);
	pros::delay(500);
	flywheel::intake(12000);
	drive::waitUntilSettled(5000);
	expectedState.x += 84_in;

	/**************************************************************************
	 *
	 * Grab ball from under far cap and then flip the cap
	 *
	 **************************************************************************/
	expectedState.theta = -2.2_rad;
	// expectedState.y -= 3_in;  // NEW
	drive::setDriveTarget("To Ball Under Cap", expectedState);
	flipper::tip();
	flywheel::intake(8000);
	drive::freeDrivePath("Flip Far Cap");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{33_in, 0_in, 0_rad}},
	    "From Ball Under Cap", DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
	drive::waitUntilSettled(0);
	expectedState.x += odom::getXLength(45_in, expectedState.theta);
	expectedState.y += odom::getYLength(45_in, expectedState.theta);
	expectedState.theta = -90_deg;

	drive::setDriveTarget("From Ball Under Cap", expectedState, true);
	drive::freeDrivePath("To Ball Under Cap");
	drive::generateDrivePath(
	    {Point{0_in, 0_in, 0_rad}, Point{12_in, 0_in, 0_rad}},
	    "To Platform Ball");
	pros::delay(500);
	if (!flywheel::isIntakeStalled()) flywheel::setIntakeLift(HIGH);
	drive::waitUntilSettled(0);
	expectedState.x += 33_in;

	/**************************************************************************
	 *
	 * Grab the ball off of the far side platform
	 *
	 **************************************************************************/
	expectedState.theta = -145_deg;
	drive::setDriveTarget("To Platform Ball", expectedState);
	flywheel::intake(-12000);
	pros::delay(100);
	flywheel::intake(12000);
	drive::freeDrivePath("From Ball Under Cap");
	if (reckless) {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{22_in, 0_in, 0_rad}},
		    "From Platform Ball" /*, DRIVE_LINEAR_MAX_VEL, 3.0, 10.0*/);
	} else {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{16_in, 0_in, 0_rad}},
		    "From Platform Ball" /*, DRIVE_LINEAR_MAX_VEL, 3.0, 10.0*/);
	}
	drive::waitUntilSettled(0);
	expectedState.x += odom::getXLength(12_in, expectedState.theta);
	expectedState.y += odom::getYLength(12_in, expectedState.theta);

	// reset the y value
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	curState.theta = odom::getState().theta;
	odom::setState(curState);
	pros::delay(10);

	flipper::downBlocking();

	if (reckless) {
		expectedState.theta = -180_deg;
		drive::setDriveTarget("From Platform Ball", expectedState, true, false,
		                      true);
		drive::freeDrivePath("To Platform Ball");
		drive::generateTurnPath({0, 3.3}, "Turn To Flags", DRIVE_TURN_MAX_VEL, 20.0,
		                        40.0);
		flywheel::setIntakeLift(LOW);
		drive::waitUntilSettled(3000);
		expectedState.x -= odom::getXLength(22_in, expectedState.theta);
		expectedState.y -= odom::getYLength(22_in, expectedState.theta);

		drive::setTurnTarget("Turn To Flags", expectedState);
		drive::freeDrivePath("From Platform Ball");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{60_in, 0_in, 0_rad}},
		    "To Far Low Flag");
		drive::waitUntilSettled(2000);
		expectedState.theta = 0_deg;

		flywheel::doubleShootBlocking();

	} else {
		drive::setDriveTarget("From Platform Ball", expectedState, true);
		drive::freeDrivePath("To Platform Ball");
		drive::generateTurnPath({0, 2.2}, "Turn To Wall Align", DRIVE_TURN_MAX_VEL,
		                        20.0, 40.0);
		flywheel::setIntakeLift(LOW);
		drive::waitUntilSettled(0);
		expectedState.x -= odom::getXLength(16_in, expectedState.theta);
		expectedState.y -= odom::getYLength(16_in, expectedState.theta);

		/**************************************************************************
		 *
		 * Align to far side platform
		 *
		 **************************************************************************/
		drive::setTurnTarget("Turn To Wall Align", expectedState, true);
		flywheel::intake(-12000);
		pros::delay(100);
		flywheel::intake(12000);
		flipper::retract();
		drive::freeDrivePath("From Platform Ball");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{30_in, 0_in, 0_rad}}, "Align To Wall",
		    DRIVE_LINEAR_MAX_VEL, 3.0, 10.0);
		drive::waitUntilSettled(500);  // was 1500
		expectedState.theta = 90_deg;
		odom::state c = odom::getState();
		c.theta += 360_deg;
		odom::setState(c);

		drive::setDriveTarget("Align To Wall", expectedState);
		flywheel::setIntakeLift(HIGH);
		drive::freeTurnPath("Turn To Wall Align");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{31_in, 0_in, 0_rad}}, "To Far Flags");
		drive::waitUntilSettled(0);
		expectedState.x = 3.1_m;

		drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
		curState.x = expectedState.x;
		curState.y = odom::getState().y;
		curState.theta = expectedState.theta;
		odom::setState(curState);
		pros::delay(10);

		/**************************************************************************
		 *
		 * Fire at the far Alliance Flags
		 *
		 **************************************************************************/
		// expectedState.y -= 1_in;
		drive::setDriveTarget("To Far Flags", expectedState, true);
		drive::freeDrivePath("Align To Wall");
		drive::generateTurnPath({0, 1.45}, "Turn To Far Flags");
		flywheel::setIntakeLift(LOW);
		flywheel::intake(-12000);
		pros::delay(100);
		flywheel::intake(12000);
		drive::waitUntilSettled(0);
		expectedState.x -= 31_in;
		expectedState.y += 2_in;  // was 1

		drive::setTurnGainsOverDamped();
		drive::setTurnTarget("Turn To Far Flags", expectedState, true);
		drive::freeDrivePath("To Far Flags");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{60_in, 0_in, 0_rad}},
		    "To Far Low Flag");
		drive::waitUntilSettled(2000);
		drive::setTurnGainsDefault();

		flywheel::doubleShootBlocking(false);
	}

	/**************************************************************************
	 *
	 * Hit low far flag
	 *
	 **************************************************************************/
	expectedState.theta = 0_rad;
	drive::setDriveTarget("To Far Low Flag", expectedState);
	flywheel::intake(12000);
	drive::freeDrivePath("To Far Middle Flag");
	if (reckless) {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{34_in, -25_in, 0_rad},
		     Point{100_in, -25_in, 0_rad}},
		    "From Far Low Flag");
	} else {
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{34_in, -20_in, 0_rad},
		     Point{80_in, -20_in, 0_rad}},
		    "From Far Low Flag");
	}
	drive::waitUntilSettled(0);
	expectedState.y = 3.2_m;  // Won't be exactly 12 feet because the encoder
	                          // wheels aren't at the front of the robot
	expectedState.x = 3.1_m - 34_in;
	expectedState.theta = 0_rad;
	drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	curState.x = odom::getState().x;
	curState.y = expectedState.y;
	ct = odom::getState().theta.convert(radian);
	if (ct > -0.05 && ct < 0.0) {
		// only reset odom in an acceptable range
		curState.theta = 0_rad;
	} else {
		curState.theta = odom::getState().theta;
	}
	odom::setState(curState);
	pros::delay(10);  // to make sure that the odometry is right before we start
	                  // next movement

	/**************************************************************************
	 *
	 * Park
	 *
	 **************************************************************************/
	if (reckless) {
		drive::setDriveAMGains(0.2, 0.2, 200, 0, 0.5, 0, 0);
		drive::setDriveTarget("From Far Low Flag", expectedState, true);
		flywheel::intake(-12000);
		drive::freeDrivePath("To Far Low Flag");
		drive::generateTurnPath({0, 1.57}, "Turn To Caps", DRIVE_TURN_MAX_VEL, 20.0,
		                        40.0);
		drive::waitUntilSettled(0);
		drive::setDriveGainDefaults();
		expectedState.x += 25_in;
		expectedState.y -= 100_in;

		drive::setTurnTarget("Turn To Caps", expectedState, true);
		drive::freeDrivePath("From Far Low Flag");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{38_in, 0_in, 0_rad},
		     Point{46_in, 20_in, 0_rad}},
		    "Flip Caps");
		drive::waitUntilSettled(500);
		expectedState.theta = -90_deg;

		drive::setDriveTarget("Flip Caps", expectedState);
		flywheel::intake(-12000);
		flipper::tip();
		drive::freeTurnPath("Turn To Caps");
		drive::generateDrivePath(
		    {Point{0_in, 0_in, 0_rad}, Point{24_in, 0_in, 0_rad},
		     Point{46_in, -40_in, 0_rad}},
		    "Back To Platform");
		drive::waitUntilSettled(0);
		expectedState.x -= 46_in;
		expectedState.y -= 20_in;

		drive::setDriveTarget("Back To Platform", expectedState, true);
		drive::freeDrivePath("Flip Caps");
		drive::waitUntilSettled(0);
		expectedState.x += 46_in;
		expectedState.y += 40_in;
	} else {
		drive::setDriveAMGains(0.2, 0.2, 200, 0, 0.5, 0, 0);
		drive::setDriveTarget("From Far Low Flag", expectedState, true);
		flywheel::intake(-12000);
		drive::freeDrivePath("To Far Low Flag");
		drive::generateTurnPath({0, 1.57}, "Turn To Platform", DRIVE_TURN_MAX_VEL,
		                        20.0, 40.0);
		drive::waitUntilSettled(0);
		drive::setDriveGainDefaults();
		expectedState.x += 20_in;
		expectedState.y -= 80_in;

		drive::setTurnGainsOverDamped();
		flipper::tip();
		pros::delay(200);
		if (!flywheel::isIntakeStalled()) flywheel::setIntakeLift(HIGH);
		flywheel::intake(-12000);
		drive::setTurnTarget("Turn To Platform", expectedState, true);
		drive::freeDrivePath("From Far Low Flag");
		drive::waitUntilSettled(1000);
		drive::setTurnGainsDefault();
		expectedState.theta = -90_deg;
	}

	drive::park();

	odom::stop();
	flywheel::stop();
	pros::lcd::print(7, "Script Time: %f",
	                 (pros::millis() - scriptStartTime) / 1000.0);
}
}  // namespace auton
