/**
 * Drivetrain stuff
 */
#include "main.h"

using namespace okapi;

/**
 * Linear Drive LMPF Constants
 */
#define DRIVE_LINEAR_KF 0.8f
#define DRIVE_LINEAR_KP 0.8f
#define DRIVE_LINEAR_KI 0.001f
#define DRIVE_LINEAR_KD 0.01f

#define DRIVE_LINEAR_POS_TOLERANCE 0.03f
#define DRIVE_LINEAR_DERIV_TOLERANCE 0.005f

/**
 * Turn LMPF Constants
 */
#define DRIVE_TURN_KF_DEFAULT 0.8f
#define DRIVE_TURN_KP_DEFAULT 0.65f
#define DRIVE_TURN_KI_DEFAULT 0.0014f
#define DRIVE_TURN_KD_DEFAULT 0.015f

// Over damped constants
#define DRIVE_TURN_KF_OD 0.8f
#define DRIVE_TURN_KP_OD 0.62f
#define DRIVE_TURN_KI_OD 0.0012f
#define DRIVE_TURN_KD_OD 0.018f

// under damped constants
#define DRIVE_TURN_KF_UD 0.8f
#define DRIVE_TURN_KP_UD 0.65f
#define DRIVE_TURN_KI_UD 0.0018f
#define DRIVE_TURN_KD_UD 0.017f

#define DRIVE_TURN_POS_TOLERANCE 0.01f
#define DRIVE_TURN_DERIV_TOLERANCE 0.0005f

/**
 * Auton Line Detection Constant
 */
#define LINE_CUTOFF_VALUE 1600
#define LINE_SAMPLES 10

/**
 * Cheesy Drive Constants
 */
#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.02f
#define CD_TURN_NONLINEARITY                                                   \
  0.65 // This factor determines how fast the wheel
       // traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 1.0
// Unused constants from 254's implementation
// static double kLowWheelNonLinearity = 0.5;
// static double kHighNegInertiaScalar = 4.0;
// static double kLowNegInertiaThreshold = 0.65;
// static double kLowNegInertiaTurnScalar = 3.5;
// static double kLowNegInertiaCloseScalar = 4.0;
// static double kLowNegInertiaFarScalar = 5.0;
// static double kHighSensitivity= 0.95;
// static double kLowSensitiity = 1.3;
// static double kQuickStopDeadband = 0.2;
// static double kQuickStopWeight = 0.1;
// static double kQuickStopScalar = 5.0;

std::shared_ptr<ChassisControllerIntegrated> controller;
std::shared_ptr<drive::TurnOutput> turnOutput;
std::shared_ptr<drive::TurnInput> turnInput;
std::shared_ptr<drive::LinearOutput> linearOutput;
std::shared_ptr<drive::LinearInput> linearInput;
std::unique_ptr<pros::ADIAnalogIn> lineTracker;
std::unique_ptr<MedianFilter<LINE_SAMPLES>> lineFilter;

std::unique_ptr<Motor> lm1, lm2, lm3, lm4;
std::unique_ptr<Motor> rm1, rm2, rm3, rm4;
std::unique_ptr<MotorGroup> bandReleaser;

namespace drive {
std::shared_ptr<PurePursuitFollower> driveController;
std::shared_ptr<LinearMotionProfileFollower> turnController;
std::shared_ptr<LinearMotionProfileFollower> linearController;

QAngle _expectedTheta = 0_rad;

void init() {
  controller = ChassisControllerFactory::createPtr(
      {-DRIVE_LEFT_1, DRIVE_LEFT_2, -DRIVE_LEFT_3, DRIVE_LEFT_4},
      {DRIVE_RIGHT_1, -DRIVE_RIGHT_2, DRIVE_RIGHT_3, -DRIVE_RIGHT_4},
      AbstractMotor::gearset::green, {4_in, 12_in});

  driveController = std::make_shared<PurePursuitFollower>(
      DRIVE_LINEAR_MAX_VEL, DRIVE_LINEAR_MAX_ACCEL, DRIVE_LINEAR_MAX_JERK,
      controller);
  turnOutput = std::make_shared<drive::TurnOutput>();
  turnInput = std::make_shared<drive::TurnInput>();
  turnController = std::make_shared<LinearMotionProfileFollower>(
      DRIVE_TURN_MAX_VEL, DRIVE_TURN_MAX_ACCEL, DRIVE_TURN_MAX_JERK,
      DRIVE_TURN_KF_DEFAULT, DRIVE_TURN_KP_DEFAULT, DRIVE_TURN_KI_DEFAULT,
      DRIVE_TURN_KD_DEFAULT, turnInput, turnOutput,
      TimeUtilFactory::withSettledUtilParams(DRIVE_TURN_POS_TOLERANCE,
                                             DRIVE_TURN_DERIV_TOLERANCE));

  linearOutput = std::make_shared<drive::LinearOutput>();
  linearInput = std::make_shared<drive::LinearInput>();
  linearController = std::make_shared<LinearMotionProfileFollower>(
      DRIVE_LINEAR_MAX_VEL, DRIVE_LINEAR_MAX_ACCEL, DRIVE_LINEAR_MAX_JERK,
      DRIVE_LINEAR_KF, DRIVE_LINEAR_KP, DRIVE_LINEAR_KI, DRIVE_LINEAR_KD,
      linearInput, linearOutput,
      TimeUtilFactory::withSettledUtilParams(DRIVE_LINEAR_POS_TOLERANCE,
                                             DRIVE_LINEAR_DERIV_TOLERANCE));

  lineTracker = std::make_unique<pros::ADIAnalogIn>(LINE_TRACKER);
  lineFilter = std::make_unique<MedianFilter<LINE_SAMPLES>>();

  lm1 = std::make_unique<Motor>(DRIVE_LEFT_1, true,
                                AbstractMotor::gearset::green);
  lm2 = std::make_unique<Motor>(DRIVE_LEFT_2);
  lm3 = std::make_unique<Motor>(DRIVE_LEFT_3, true,
                                AbstractMotor::gearset::green);
  lm4 = std::make_unique<Motor>(DRIVE_LEFT_4);
  rm1 = std::make_unique<Motor>(DRIVE_RIGHT_1);
  rm2 = std::make_unique<Motor>(DRIVE_RIGHT_2, true,
                                AbstractMotor::gearset::green);
  rm3 = std::make_unique<Motor>(DRIVE_RIGHT_3);
  rm4 = std::make_unique<Motor>(DRIVE_RIGHT_4, true,
                                AbstractMotor::gearset::green);

  Motor bandReleaseLeft(BAND_RELEASE_LEFT, false, AbstractMotor::gearset::green,
                        AbstractMotor::encoderUnits::degrees);
  Motor bandReleaseRight(BAND_RELEASE_RIGHT, true,
                         AbstractMotor::gearset::green,
                         AbstractMotor::encoderUnits::degrees);
  std::initializer_list bandReleaseMotors = {bandReleaseLeft, bandReleaseRight};
  bandReleaser = std::make_unique<MotorGroup>(bandReleaseMotors);

  driveController->flipDisable(true);
  turnController->flipDisable(true);
  linearController->flipDisable(true);
}

void stopTasks() {
  driveController->flipDisable(true);
  turnController->flipDisable(true);
  linearController->flipDisable(true);
}

void generateDrivePath(std::initializer_list<okapi::Point> iwaypoints,
                       const std::string &ipathId, double imaxVel,
                       double imaxAccel, double imaxJerk) {
  driveController->generatePath(iwaypoints, ipathId, imaxVel, imaxAccel,
                                imaxJerk);
}

void generateTurnPath(std::initializer_list<double> iwaypoints,
                      const std::string ipathId, double imaxVel,
                      double imaxAccel, double imaxJerk) {
  turnController->generatePath(iwaypoints, ipathId, imaxVel, imaxAccel,
                               imaxJerk);
}

void generateLinearDrivePath(std::initializer_list<double> iwaypoints,
                             const std::string ipathId, double imaxVel,
                             double imaxAccel, double imaxJerk) {
  linearController->generatePath(iwaypoints, ipathId, imaxVel, imaxAccel,
                                 imaxJerk);
}

void setDriveTarget(std::string itarget, odom::state iexpected, bool ireversed,
                    bool icurvatureMode, bool iconstrainEnd) {
  if (logger)
    logger->log("M, Starting \"%s\" Drive Movement\n", itarget.c_str());
  turnController->flipDisable(true);
  linearController->flipDisable(true);
  driveController->flipDisable(false);
  driveController->setTarget(itarget, iexpected, ireversed, icurvatureMode,
                             iconstrainEnd);
}

void setTurnTarget(std::string itarget, odom::state iexpected, bool ireversed) {
  if (logger)
    logger->log("M, Starting \"%s\" Turn Movement\n", itarget.c_str());
  driveController->flipDisable(true);
  linearController->flipDisable(true);
  turnController->flipDisable(false);
  turnController->setTarget(itarget, iexpected.theta.convert(radian),
                            ireversed);
}

void setLinearDriveTarget(std::string itarget, odom::state iexpected,
                          bool ireversed) {
  if (logger)
    logger->log("M, Starting \"%s\" Linear Drive Movement\n", itarget.c_str());
  _expectedTheta = iexpected.theta;
  driveController->flipDisable(true);
  turnController->flipDisable(true);
  linearController->flipDisable(false);
  double e =
      sin(PI / 2 - _expectedTheta.convert(radian)) *
          iexpected.y.convert(meter) +
      cos(PI / 2 - _expectedTheta.convert(radian)) * iexpected.x.convert(meter);
  linearController->setTarget(itarget, e, ireversed);
}

int32_t waitUntilSettled(int itimeout, bool icheckForLine, bool iflipCap) {
  if (!driveController->isDisabled())
    if (icheckForLine)
      return driveController->waitUntilSettled(itimeout, []() {
        return (lineFilter->filter(lineTracker->get_value()) <
                LINE_CUTOFF_VALUE);
      });
    else if (iflipCap)
      return driveController->waitUntilSettled(itimeout, []() {
        lineFilter->filter(lineTracker->get_value());
        return capCam->capFlipped();
      });
    else
      driveController->waitUntilSettled(itimeout, []() {
        lineFilter->filter(lineTracker->get_value());
        return false;
      });
  else if (!turnController->isDisabled())
    turnController->waitUntilSettled(itimeout);
  else if (!linearController->isDisabled())
    linearController->waitUntilSettled(itimeout);
  return 0;
}

void setDriveAMGains(/* REDACTED */) {
  driveController->setAngleModeGains(/* REDACTED */);
}

void setDriveCMGains(/* REDACTED */) {
  driveController->setCurvatureModeGains(/* REDACTED */);
}

void setDriveRamseteGains(double ib, double izeta, double ilinVelkP,
                          double iangVelkP, double isettleError) {
  driveController->setRamseteGains(ib, izeta, ilinVelkP, iangVelkP,
                                   isettleError);
}

void setDriveGainDefaults() {
  driveController->setAngleModeGains(/* REDACTED */);
}

void setTurnGains(double ikF, double ikP, double ikI, double ikD,
                  double iintegralCap) {
  turnController->setGains(ikF, ikP, ikI, ikD, iintegralCap);
}

void setTurnGainsDefault() {
  turnController->setGains(DRIVE_TURN_KF_DEFAULT, DRIVE_TURN_KP_DEFAULT,
                           DRIVE_TURN_KI_DEFAULT, DRIVE_TURN_KD_DEFAULT, 1);
}

void setTurnGainsOverDamped() {
  turnController->setGains(DRIVE_TURN_KF_OD, DRIVE_TURN_KP_OD, DRIVE_TURN_KI_OD,
                           DRIVE_TURN_KD_OD, 1);
}

void setTurnGainsUnderDamped() {
  turnController->setGains(DRIVE_TURN_KF_UD, DRIVE_TURN_KP_UD, DRIVE_TURN_KI_UD,
                           DRIVE_TURN_KD_UD, 1);
}

void freeDrivePath(std::string itarget) {
  pros::delay(10); // give time for a cycle so we don't segfault or anything
  driveController->removePath(itarget);
}

void freeTurnPath(std::string itarget) {
  pros::delay(10);
  turnController->removePath(itarget);
}

void freeLinearDrivePath(std::string itarget) {
  pros::delay(10);
  linearController->removePath(itarget);
}

void setBrakeMode(okapi::AbstractMotor::brakeMode imode) {
  controller->setBrakeMode(imode);
}

void setDriveMaxVoltage(int ilimit) { controller->setMaxVoltage(ilimit); }

void setDriveMaxCurrent(int ilimit) {
  lm1->setCurrentLimit(ilimit);
  lm2->setCurrentLimit(ilimit);
  lm3->setCurrentLimit(ilimit);
  lm4->setCurrentLimit(ilimit);
  rm1->setCurrentLimit(ilimit);
  rm2->setCurrentLimit(ilimit);
  rm3->setCurrentLimit(ilimit);
  rm4->setCurrentLimit(ilimit);
}

/**
 * The "curvature" drive control written by FRC team 254.
 *
 * See the Java source at
 * https://github.dev/Team254/FRC-2015/blob/9dcc11886a49d29f16e597e317c995ca248efaed/src/com/team254/frc2015/CheesyDriveHelper.java#L24
 *
 * Essentially this does three things:
 *
 * Cheesy Drive applies some non-linearity to the joystick input so that
 * there is more control at the low speeds (larger changes in joystick inputs
 * result in smaller changes in real speed here) but when the joystick is
 * pushed to a high speed, you jump up to full speed faster.
 *
 * Additionally, when you a driving forward but also turning a bit, the turn
 * input affects the "curvature" of the movement rather than adding/subtracting
 * linearly from the wheel speeds. The turn output is a sum of the throttle and
 * turn inputs, meaning that the robot will turn faster when it's moving
 * forward at a higher speed. Again, the goal here is more control at low
 * speeds.
 *
 * Third, that turn input is affected by a negative inertia accumulator. Most
 * robots have a fair bit of turning inertia, which can make it easy to
 * accidentally overshoot a turn. The negative inertia accumulator acts almost
 * like a reverse integral controller - the longer the robot has been turning
 * (fast) for, the slower the robot will turn.
 */

// We apply a sinusoidal curve (twice) to the joystick input to give finer
// control at small inputs.
static double _turnRemapping(double iturn) {
	double denominator = sin(PI / 2 * CD_TURN_NONLINEARITY);
	double firstRemapIteration =
	    sin(PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
	return sin(PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
double quickStopAccumlator = 0.0;
double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
	if (negInertiaAccumlator > 1) {
		negInertiaAccumlator -= 1;
	} else if (negInertiaAccumlator < -1) {
		negInertiaAccumlator += 1;
	} else {
		negInertiaAccumlator = 0;
	}

	if (quickStopAccumlator > 1) {
		quickStopAccumlator -= 1;
	} else if (quickStopAccumlator < -1) {
		quickStopAccumlator += 1;
	} else {
		quickStopAccumlator = 0.0;
	}
}

double prevTurn = 0.0;
double prevThrottle = 0.0;
void cheesyDrive(double ithrottle, double iturn) {
	bool turnInPlace = false;
	double linearCmd = ithrottle;
	if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
		// The controller joysticks can output values near zero when they are
		// not actually pressed. In the case of small inputs like this, we
		// override the throttle value to 0.
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (ithrottle - prevThrottle > DRIVE_SLEW) {
		linearCmd = prevThrottle + DRIVE_SLEW;
	} else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
		// We double the drive slew rate for the reverse direction to get
		// faster stopping.
		linearCmd = prevThrottle - (DRIVE_SLEW * 2);
	}

	double remappedTurn = _turnRemapping(iturn);

	double left, right;
	if (turnInPlace) {
		// The remappedTurn value is squared when turning in place. This
		// provides even more fine control over small speed values.
		left = remappedTurn * std::abs(remappedTurn);
		right = -remappedTurn * std::abs(remappedTurn);

		// The FRC Cheesy Drive Implementation calculated the
		// quickStopAccumulator here:
		// if (Math.abs(linearPower) < 0.2) {
		// 	double alpha = 0.1;
		// 	quickStopAccumulator = (1 - alpha) * quickStopAccumulator
		// 			+ alpha * Util.limit(wheel, 1.0) * 5;
		// }
		// But I apparently left that out of my implementation? Seemed to work
		// without it though.
	} else {
		double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
		negInertiaAccumlator += negInertiaPower;

		double angularCmd =
		    abs(linearCmd) *  // the more linear vel, the faster we turn
		        (remappedTurn + negInertiaAccumlator) *
		        CD_SENSITIVITY -  // we can scale down the turning amount by a
		                          // constant
		    quickStopAccumlator;

		right = left = linearCmd;
		left += angularCmd;
		right -= angularCmd;

		_updateAccumulators();
	}

	controller->tank(left, right);

	prevTurn = iturn;
	prevThrottle = ithrottle;
}

void park() {
  EmaFilter currentFilter(0.1);
  AverageFilter<30> velocityFilter;
  uint32_t now = pros::millis();
  uint8_t platformCount = 0;
  double prevVel = 0;
  uint32_t platformHitTimer = 0;

  releaseBands();
  double current, velocity;
  while (platformCount < 2 && (pros::millis() - now) < 4500) {
    cheesyDrive(1.0, 0);

    current = getCurrentDraw();
    current = currentFilter.filter(current);
    velocity = getVelocity();
    velocity = velocityFilter.filter(velocity);
    printf("%f, %f, %d\n", current, velocity, 200 * platformCount);

    if ((pros::millis() - platformHitTimer) < 400) {
      flywheel::intake(12000);
      flywheel::setIntakeLift(LOW);
    } else {
      flywheel::intake(0);
      flywheel::setIntakeLift(HIGH);
    }

    // Check for a current spike of > 200mA and a velocity drop of more than 0.5
    // rpm, and prevent counting another platform hit for a full second to avoid
    // false positives.
    if (/*current > 150 && */ velocity < (prevVel - 0.25) &&
        (pros::millis() - platformHitTimer) > 1500) {
      platformCount++;
      platformHitTimer = pros::millis();
    }

    prevVel = velocity;
    pros::delay(5);
  }
  cheesyDrive(0.95, 0);
  auto velSettle = SettledUtilFactory::create(25, 20, 80_ms);
  while (!velSettle.isSettled(velocity) &&
         (pros::millis() - platformHitTimer) < 600) {
    velocity = getVelocity();
    velocity = velocityFilter.filter(velocity);
    pros::delay(5);
  }
  controller->setBrakeMode(AbstractMotor::brakeMode::hold); // maybe hold?
  controller->forward(0); // use velocity mode to enact the brake
}

int32_t getLeftVoltage() { return lm4->getVoltage(); }

int32_t getRightVoltage() { return rm4->getVoltage(); }

int32_t getCurrentDraw() {
  return (lm4->getCurrentDraw() + rm4->getCurrentDraw()) / 2;
}

double getVelocity() {
  return (std::abs(lm4->getActualVelocity()) +
          std::abs(rm4->getActualVelocity())) /
         2;
}

static void bandReleaseRoutine(void *ign) {
  bandReleaser->moveAbsolute(200, 200);
  flipper::tip();
  pros::delay(500);
  bandReleaser->moveAbsolute(0, 200);
  flywheel::setIntakeLift(HIGH);
}

void releaseBands() { pros::Task bandReleaseTask(bandReleaseRoutine); }

void TurnOutput::controllerSet(double value) { controller->rotate(value); }

double TurnInput::controllerGet() {
  return odom::getState().theta.convert(radian);
}

void LinearOutput::controllerSet(double value) {
  controller->tank(value, value);
}

double LinearInput::controllerGet() {
  odom::state s = odom::getState();
  return sin(PI / 2 - _expectedTheta.convert(radian)) * s.y.convert(meter) +
         cos(PI / 2 - _expectedTheta.convert(radian)) * s.x.convert(meter);
}
} // namespace drive
