/**
 * Source for the alliace side first auton script..
 *
 * Goal: Shoot all alliance side flags, flip a cap, then hit all middle flags.
 *
 * Back Left of the field (Red side furthest wall from flags) is the (0,0)
 * point. Facing the flags is 0 theta.
 *
 * Grep points:
 * - Turn To Alliance Flags
 * - Turn To First Cap
 * - To Ball Under Cap
 * - Align To Platform
 * - Angle To Center Flags
 * - Align To Center Flags
 */
#include "main.h"

using namespace okapi;

namespace auton {
void acConfig() {
  lcdSelector::titles.push_back("Alliance->Center");
  lcdSelector::inits.push_back(auton::allianceToCenterInit);
  lcdSelector::scripts.push_back(auton::allianceToCenter);
}

void allianceToCenterInit(color side) {
  if (side == red) {
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{19_in, 0_in, 0_rad}},
        "To Platform Ball");
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{26_in, 0_in, 0_rad}},
        "From Platform Ball");
    drive::generateTurnPath({0, 2.2}, "Turn To Alliance Flags");
  } else {
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{19_in, 0_in, 0_rad}},
        "To Platform Ball");
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{26_in, 0_in, 0_rad}},
        "From Platform Ball");
    drive::generateTurnPath({0, 2.2}, "Turn To Alliance Flags");
  }
}

void allianceToCenter(color side, uint32_t allianceFireTime,
                      uint32_t centerFireTime) {
  uint32_t now, timeDiff;
  int32_t exitCode;
  uint32_t scriptStartTime = pros::millis();
  odom::start(true);
  flywheel::start();
  uint32_t startTime = pros::millis();
  flywheel::setVelocitySetpoint(FLYWHEEL_SPEED_OP_CONTROL);
  pros::delay(150); // XXX: for some reason VexOS returns garbage encoder
                    // values for the first ~100ms

  // This variable will be used throughout the program to specify where we
  // think the robot ought to be. This will be used as the starting spot for
  // the profiled movements.
  expectedState.y = 2.0_m;
  if (side == red) {
    expectedState.x = 0.38_m;
    expectedState.theta = 90_deg;
  } else {
    expectedState.x = 3.277_m;
    expectedState.theta = -90_deg;
  }

  odom::setState(expectedState);

  printf("\n\n%d\n", pros::millis() - scriptStartTime);

  /**************************************************************************
   *
   * Get Ball off the Platform
   *
   **************************************************************************/
  if (side == red) {
    expectedState.theta = 2.2_rad;
  } else {
    expectedState.theta = -2.2_rad;
  }
  drive::setDriveTarget("To Platform Ball", expectedState);
  flywheel::intake(12000);
  flywheel::setIntakeLift(HIGH);
  drive::waitUntilSettled(0);
  if (side == red) {
    expectedState.x += odom::getXLength(17.5_in, expectedState.theta);
    expectedState.y += odom::getYLength(17.5_in, expectedState.theta);
  } else {
    expectedState.x += odom::getXLength(17.5_in, expectedState.theta);
    expectedState.y += odom::getYLength(17.5_in, expectedState.theta);
  }

  if (side == red) {
    expectedState.theta = 2.25_rad;
  } else {
    expectedState.theta = -2.25_rad;
  }
  drive::setDriveTarget("From Platform Ball", expectedState, true);
  flywheel::intake(12000);
  flipper::down();
  pros::delay(400);
  flywheel::setIntakeLift(false);
  pros::delay(300); // determine how long it takes the intake to go down
  drive::freeDrivePath("To Platform Ball");
  drive::generateDrivePath(
      {Point{0_ft, 0_ft, 0_deg}, Point{20_in, 0_ft, 0_deg}},
      "To Middle Alliance Flag");
  drive::waitUntilSettled(0);
  expectedState.x -= odom::getXLength(26_in, expectedState.theta);
  expectedState.y -= odom::getYLength(26_in, expectedState.theta);

  /**************************************************************************
   *
   * Fire at the Alliance flags
   *
   **************************************************************************/
  if (side == red) {
    drive::setTurnTarget("Turn To Alliance Flags", expectedState, true);
  } else {
    drive::setTurnTarget("Turn To Alliance Flags", expectedState);
  }
  flipper::retract();
  drive::waitUntilSettled(3000);
  // turn towards the flags slightly for shot consistency
  if (side == red) {
    expectedState.theta = 0.05_rad;
  } else {
    expectedState.theta = -0.05_rad;
  }

  drive::freeDrivePath("From Platform Ball");
  drive::generateDrivePath(
      {Point{0_ft, 0_ft, 0_deg}, Point{37_in, 0_ft, 0_rad}},
      "To Low Alliance Flag");
  flywheel::intake(-12000); // clear any extra balls out of the intake
  pros::delay(100);
  flywheel::intake(12000);

  pros::delay(300); // this will help the bot be stable before shooting

  // Wait until the specified second shot time
  if (allianceFireTime) {
    int32_t curElapsedTime = pros::millis() - scriptStartTime;
    allianceFireTime *= 1000;
    if (allianceFireTime > curElapsedTime) {
      pros::delay(allianceFireTime - curElapsedTime);
    }
  }

  flywheel::shootBlocking(); // high flag
  pros::delay(
      500); // make sure the bot doesn't drive before the ball is in the air

  flywheel::intake(12000);
  drive::setDriveTarget("To Middle Alliance Flag", expectedState);
  flywheel::setVelocitySetpoint(FLYWHEEL_SPEED_AUTON_MIDDLE);
  drive::waitUntilSettled(2000);
  expectedState.x += odom::getXLength(20_in, expectedState.theta);
  expectedState.y += odom::getYLength(20_in, expectedState.theta);
  // try to drive slightly away from the poles
  if (side == red) {
    expectedState.theta = -0.15_rad; // was .15
  } else {
    expectedState.theta = 0.15_rad;
  }

  now = pros::millis();
  drive::freeTurnPath("Turn To Flags");
  drive::freeDrivePath("To Middle Alliance Flag");
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{22_in, 0_in, 0_rad}}, // was 20
      "Back From Alliance Low");
  timeDiff = (pros::millis() - now) < 200 ? pros::millis() - now : 200;
  pros::delay(500 -
              timeDiff); // this will help the bot be stable before shooting

  flywheel::shootBlocking(); // middle flag

  /**************************************************************************
   *
   * Hit the low flag and align to the flag side wall
   *
   **************************************************************************/
  flywheel::intake(12000);
  drive::setDriveTarget("To Low Alliance Flag", expectedState);
  drive::generateTurnPath({0, 2}, "Turn To First Cap");
  drive::generateDrivePath({Point{0_in, 0_in, 0_rad}, Point{5_in, 0_in, 0_rad}},
                           "Back Up To Align");
  drive::waitUntilSettled(0);
  QLength prevEX;
  if (side == red) {
    prevEX = expectedState.x + 2_in;
  } else {
    prevEX = expectedState.x - 2_in;
  }
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
  drive::waitUntilSettled(3000);
  expectedState.y = 3.47_m; // Won't be exactly 12 feet because the encoder
                            // wheels aren't at the front of the robot
  expectedState.x = prevEX;
  expectedState.theta = 0_rad;

  pros::delay(300);

  // Calibrate odometry after the wall hit
  odom::state curState;
  curState.x = odom::getState().x;
  curState.y = expectedState.y;
  curState.theta = 0_rad;
  odom::setState(curState);
  pros::delay(10); // to make sure that the odometry is right before we start
                   // next movement

  // if we pick up a ball or two when we drive at the wall, then queue them up
  // to shoot
  flywheel::indexOne(12000);

  /**************************************************************************
   *
   * Back up from alliance flags
   *
   **************************************************************************/
  drive::setDriveTarget("Back From Alliance Low", expectedState, true);
  drive::freeDrivePath("Align To Wall");
  drive::freeDrivePath("To Low Alliance Flag");
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{13_in, 0_in, 0_rad}},
      "To First Cap Ball");
  drive::waitUntilSettled(2000);
  expectedState.y -= 22_in; // was 20

  // clearing the intake again just in case
  flywheel::indexOne();

  /**************************************************************************
   *
   * Grab the first ball off of the blue cap
   *
   **************************************************************************/
  if (side == red) {
    drive::setTurnTarget("Turn To First Cap", expectedState);
  } else {
    drive::setTurnTarget("Turn To First Cap", expectedState, true);
  }
  drive::freeDrivePath("Back From Alliance Low");
  pros::delay(500);
  flywheel::setIntakeLift(HIGH);
  drive::waitUntilSettled(1000);
  if (side == red) {
    expectedState.theta = 2_rad;
  } else {
    expectedState.theta = -2_rad;
  }
  // The encoder position will change somewhat because they're not at the point
  // of rotation
  if (side == red) {
    expectedState.y += 0.75_in;
    expectedState.x -= 1_in;
  } else {
    expectedState.y += 0.75_in;
    expectedState.x += 1_in;
  }

  drive::setDriveAMGains(0, 0, 0, 0, 0, 0.2, 0);
  drive::setDriveTarget("To First Cap Ball", expectedState);
  flywheel::intake(12000);
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{14_in, 0_in, 0_rad}},
      "From First Cap Ball");
  drive::waitUntilSettled(2000);
  drive::setDriveGainDefaults();
  flywheel::intake(12000); // just in case
  expectedState.x += odom::getXLength(13_in, expectedState.theta);
  expectedState.y += odom::getYLength(13_in, expectedState.theta);
  // use Pure Pursuit to fix our angle
  if (side == red) {
    expectedState.theta = 90_deg;
  } else {
    expectedState.theta = -90_deg;
  }

  expectedState.y -= 2_in;
  drive::setDriveTarget("From First Cap Ball", expectedState, true);
  now = pros::millis();
  drive::freeDrivePath("To First Cap Ball");
  drive::generateDrivePath({{0_in, 0_in, 0_rad}, {15_in, 0_in, 0_rad}},
                           "Align To Cap");
  timeDiff = (pros::millis() - now) < 200 ? pros::millis() - now : 200;
  pros::delay(800 - timeDiff);
  flywheel::setIntakeLift(LOW);
  flipper::down();
  drive::waitUntilSettled(0);
  if (side == red) {
    expectedState.x -= 11_in;
  } else {
    expectedState.x += 11_in;
  }

  /**************************************************************************
   *
   * Flip the Cap
   *
   **************************************************************************/
  flywheel::intake(-8000);
  expectedState.y -= 4_in; // we're not hitting the cap dead center otherwise
  drive::setDriveTarget("Align To Cap", expectedState);
  drive::freeDrivePath("From First Cap Ball");
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{20_in, 0_in, 0_rad}}, "Flip Cap",
      DRIVE_LINEAR_MAX_VEL, 2.0, 4.0);
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{55_in, 0_in, 0_rad}}, "From Cap Flip");
  drive::waitUntilSettled(0);
  if (side == red) {
    expectedState.x += 15_in;
  } else {
    expectedState.x -= 15_in;
  }

  flipper::upBlocking();

  flywheel::intake(-12000);
  drive::setDriveTarget("Flip Cap", expectedState);
  pros::lcd::print(6, "here");
  drive::freeDrivePath("Align To Cap");
  pros::delay(300);
  int32_t exitCondition = drive::waitUntilSettled(0, false, true);
  if (exitCondition == DRIVE_EXIT_LINE)
    pros::delay(20);
  if (side == red) {
    expectedState.x += 20_in;
  } else {
    expectedState.x -= 20_in;
  }
  expectedState.y += 2_in;

  drive::setDriveAMGains(0, 0, 0, 0, 0, 0.1, 0);
  drive::setDriveTarget("From Cap Flip", expectedState, true);
  flywheel::intake(-12000); // clear out any extra balls
  drive::freeDrivePath("Flip Cap");
  // negative y = positive x
  if (side == red) {
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{32_in, -20_in, 0_rad},
         Point{49_in, -20_in, 0_rad}},
        "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL - 0.25, 1.6, 1.6);
  } else {
    drive::generateDrivePath(
        {Point{0_in, 0_in, 0_rad}, Point{32_in, 20_in, 0_rad},
         Point{49_in, 20_in, 0_rad}},
        "To Ball Under Cap", DRIVE_LINEAR_MAX_VEL - 0.25, 1.6, 1.6);
  }
  drive::waitUntilSettled(0);
  drive::setDriveGainDefaults();
  if (side == red) {
    expectedState.x = 0.2_m; // tune this
  } else {
    expectedState.x = 3.35_m;
  }
  flywheel::intake(12000);

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
  drive::setDriveAMGains(1.00, 0.35, 0.15, 150, 0, 0.25, 10,
                         100); // kAngleComp was 170
  drive::setDriveTarget("To Ball Under Cap", expectedState);
  flipper::retract();
  drive::freeDrivePath("From Cap Flip");
  if (side == red) {
    drive::generateDrivePath({{0_in, 0_in, 0_rad}, {6_in, 0_in, 0_rad}},
                             "From Ball Under Cap");
  } else {
    drive::generateDrivePath({{0_in, 0_in, 0_rad}, {6_in, 0_in, 0_rad}},
                             "From Ball Under Cap");
  }
  pros::delay(1000);
  flywheel::intake(12000);
  pros::delay(2000); // ?
  drive::waitUntilSettled(4000);
  if (side == red) {
    expectedState.x += 49_in;
    expectedState.y -= 20_in;
  } else {
    expectedState.x -= 49_in;
    expectedState.y -= 20_in;
  }

  flipper::tip();
  flywheel::intake(12000);
  pros::delay(300); // give it some time to intake the ball
  flywheel::intake(12000);
  pros::delay(400);

  drive::setDriveGainDefaults();
  drive::setDriveTarget("From Ball Under Cap", expectedState, true);
  flipper::down();
  drive::freeDrivePath("To Ball Under Cap");
  drive::generateTurnPath({0, PI / 2}, "Turn To Platform Align");
  drive::waitUntilSettled(0);
  if (side == red) {
    expectedState.x -= 6_in;
  } else {
    expectedState.x += 6_in;
  }

  if (side == red) {
    drive::setTurnTarget("Turn To Platform Align", expectedState, true);
  } else {
    drive::setTurnTarget("Turn To Platform Align", expectedState);
  }
  flipper::retract();
  flywheel::intake(-6000); // clear any extra balls out of the intake
  pros::delay(80);
  flywheel::intake(12000);
  drive::freeDrivePath("From Ball Under Cap");
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{20_in, 0_in, 0_rad}},
      "Align To Platform");
  drive::waitUntilSettled(1500);
  expectedState.theta = 0_rad;
  if (side == red) {
    expectedState.x += .75_in;
    expectedState.y -= .75_in;
  } else {
    expectedState.x -= .75_in;
    expectedState.y += .75_in;
  }

  drive::setDriveAMGains(0, 0, 0, 0, 0, 1.0, 10);
  drive::setDriveMaxCurrent(DRIVE_ALIGN_CUR);
  drive::setDriveTarget("Align To Platform", expectedState, true);
  drive::freeTurnPath("Turn To Platform Align");
  double angle = 0;
  // Angle To Center Flags - Grep point
  const double yDistToFlags = 65 * IN_TO_METERS; // was 45
  double xDistFromWallToFlags;
  if (side == red) {
    xDistFromWallToFlags = 65 * IN_TO_METERS; // tune this
  } else {
    xDistFromWallToFlags = 68 * IN_TO_METERS; // tune this
  }
  angle = PI / 2 - atan2(yDistToFlags, abs(xDistFromWallToFlags -
                                           odom::getState().x.convert(meter)));
  if (side == blue)
    angle *= -1;
  drive::generateDrivePath({{0_in, 0_in, 0_rad}, {11_in, 0_in, 0_rad}},
                           "Align To Center Flags");

  drive::waitUntilSettled(0);
  drive::setDriveGainDefaults();
  drive::setDriveMaxCurrent(2500);
  expectedState.y = 1.93_m; // distance to the front of the center platform

  drive::setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  curState.x = odom::getState().x;
  curState.y = expectedState.y;
  // if (abs(curState.theta.convert(radian) -
  // expectedState.theta.convert(radian)) < 0.1)
  // XXX: for whatever reason, the odometry must be reset here or the bot thinks
  // it needs to be 90 degrees to the right
  pros::lcd::print(6, "%f %f", curState.theta.convert(radian),
                   expectedState.theta.convert(radian));
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
  drive::freeDrivePath("Align To Platform");
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{56_in, 0_in, 0_rad}},
      "To Center Low Flag", DRIVE_LINEAR_MAX_VEL - 0.2, 2.4, 6.0);
  drive::waitUntilSettled(3000);
  drive::setDriveGainDefaults();
  expectedState.x += odom::getXLength(11_in, expectedState.theta);
  expectedState.y += odom::getYLength(11_in, expectedState.theta);

  pros::delay(200);
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
  drive::generateDrivePath(
      {Point{0_in, 0_in, 0_rad}, Point{32_in, 0_in, 0_rad}},
      "From Center Low Flag");
  exitCode = drive::waitUntilSettled(0);
  //
  // if (exitCode == DRIVE_EXIT_LINE) {
  // 	flywheel::stop();
  // 	odom::stop();
  // 	return;  // end the script if we hit the line
  // }
  expectedState.y = 3.2_m; // Won't be exactly 12 feet because the encoder
                           // wheels aren't at the front of the robot
  expectedState.theta = 0_rad;

  drive::setDriveTarget("From Center Low Flag", expectedState, true);
  flywheel::intake(-3000);
  drive::waitUntilSettled(0);

  odom::stop();
  flywheel::stop();
}
} // namespace auton
