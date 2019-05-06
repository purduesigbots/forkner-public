/**
 * Flywheel Functionality
 */
#include "main.h"
#include <algorithm>

using namespace okapi;

// The distance in degrees to give to a profiled movement to move a ball into
// indexing or from indexing to firing
#define FLYWHEEL_INDEX_DISTANCE 20
// Margin of error where we can say that the flywheel is at speed
#define FLYWHEEL_VELOCITY_TOLERANCE 0.5f
#define FLYWHEEL_TIMEOUT 500
#define FLYWHEEL_AUTON_TIMEOUT 3000
#define FLYWHEEL_OPCONTROL_TIMEOUT 1000
#define FLYWHEEL_STUTTER_TIMEOUT 250

#define FLYWHEEL_INDEXER_TIMEOUT                                               \
  (pros::competition::is_autonomous()                                          \
       ? FLYWHEEL_AUTON_TIMEOUT                                                \
       : (lcdSelector::getScriptName() == "Prog Skills")                       \
             ? FLYWHEEL_AUTON_TIMEOUT                                          \
             : FLYWHEEL_OPCONTROL_TIMEOUT)

#define FLYWHEEL_CURRENT_SPIKE 0.2f
/**
 * Flywheel velocity control gain
 */
#define FLYWHEEL_TBH_GAIN 80.0f

bool initialized = false; // this is going to get toggled when the flywheel
                          // initializes all of the below objects

std::unique_ptr<pros::Task> velocityTask;
std::unique_ptr<pros::Task> indexerTask;
std::unique_ptr<pros::Task> intakeTask;
int velocityIndex;

volatile int numShotsWaiting = 0;
volatile bool doubleShootFlag = false;
volatile bool doubleShootNoWaitFlag = false;
// volatile bool indexFlag = false;
volatile bool indexFlag = true;
volatile bool atVelocityFlag = false;
volatile int intakeVoltage = 12000;
volatile int postIndexIntakePower = 12000;
volatile bool intakeHold = false;
volatile bool noStallCheck = false;
volatile bool curSpike = false;
volatile bool velDrop = false;
std::unique_ptr<pros::Mutex> shotMutex;
std::unique_ptr<pros::Mutex> indexMutex;
std::unique_ptr<pros::Mutex> doubleShootMutex;
std::unique_ptr<pros::Mutex> doubleShootNoWaitMutex;
std::unique_ptr<pros::Mutex> postIndexIntakePowerMutex;
std::unique_ptr<pros::Mutex> intakeMutex;
std::unique_ptr<pros::Mutex> intakeHoldMutex;

std::unique_ptr<okapi::Motor> flyTop;
std::unique_ptr<okapi::Motor> flyRight;
std::unique_ptr<okapi::Motor> flyBottom;
std::unique_ptr<okapi::Motor> indexerMotor;
std::unique_ptr<okapi::Motor> intakeMotor;

pros::ADIDigitalIn indexerPhotogate(INDEXER_PHOTOGATE);
std::unique_ptr<pros::ADIDigitalOut> intakeLiftSolenoid, angleChanger;

#define numFlywheelVelocities 3

int tbh = 0;
int tbhInitial[numFlywheelVelocities];

const int flywheelVelocities[numFlywheelVelocities] = {
    480, // high flags in auton
    550, // middle flags in auto
    600, // Op Control
};

const char *const flywheelVelocityStrings[numFlywheelVelocities] = {
    "High Auton     ", "Middle Auton   ", "Op Control     "};

bool prevPhotogateStatus = true;
static bool photogateRisingEdge() {
  bool curPhotogateStatus = indexerPhotogate.get_value();
  if (curPhotogateStatus && !prevPhotogateStatus) {
    prevPhotogateStatus = curPhotogateStatus;
    return true;
  } else {
    prevPhotogateStatus = curPhotogateStatus;
    return false;
  }
}

static bool photogateFallingEdge() {
  bool curPhotogateStatus = indexerPhotogate.get_value();
  if (!curPhotogateStatus && prevPhotogateStatus) {
    prevPhotogateStatus = curPhotogateStatus;
    return true;
  } else {
    prevPhotogateStatus = curPhotogateStatus;
    return false;
  }
}

static bool shotWaiting(int istart) {
  if (!istart)
    istart = pros::millis();
  return !photogateFallingEdge() && !(curSpike && velDrop) &&
         (pros::millis() - istart < FLYWHEEL_INDEXER_TIMEOUT);
}

const double gear_ratio = 5.0;
const double moment = 0.0005; // should be roughly correct from calculations
const double K_t =
    2 * MOTOR_BLUE_KT; // We'll have twice the stall torque with two motors
const double R = MOTOR_BLUE_R;
const double K_v = MOTOR_BLUE_KV;
const double RPM_TO_RADS = 0.1047;
const double VAR_RADS = 2.75847841614875; // taken from 10s sample
const double VAR_RPM = 251.638001161161;  // taken from 10s sample
const double tau = 163;
const double dt = 0.01;

Eigen::Matrix<double, 1, 1> F_A;
Eigen::Matrix<double, 1, 1> F_B;
Eigen::Matrix<double, 1, 1> F_C;

Eigen::Matrix<double, 1, 1> F_Q;
Eigen::Matrix<double, 1, 1> F_R;
Eigen::Matrix<double, 1, 1> F_P;
KalmanFilter<1, 1, 1> *velFilter;

/**
 * Allows the flywheel to be run in a couple of different modes.
 *
 * The first is simply to maintain a velocity setpoint. The driver can select
 * a given setpoint for the desired height and distance, and the flywheel will
 * maintain that speed.
 *
 * The second mode is to transition from the high flag setpoint to the low flag
 * setpoint for the given distance. The task function will monitor the flywheel
 * motor current to detect a shot ball and will drop the setpoint immediately
 * thereafter.
 */
void _velocityMonitor(void *ign) {
  int prevCmd = 0;
  double prevError = 0;
  okapi::SettledUtil settleUtil = okapi::SettledUtilFactory::create(
      FLYWHEEL_VELOCITY_TOLERANCE, FLYWHEEL_VELOCITY_TOLERANCE * 100, 100_ms);
  okapi::EmaFilter curFilter(0.4);
  double current = 0, prevCurrent = 0;
  int curSpikeCounter = 0;

  // Start the flywheel up at highest speed since it's probably easier to coast
  // down from there
  const int maxIndex = (sizeof(flywheelVelocities) / sizeof(int)) - 1;
  velocityIndex = maxIndex;
  // Set the initial guesses at output values
  for (int i = 0; i < numFlywheelVelocities; i++) {
    tbhInitial[i] = flywheelVelocities[i] * 2;
  }
  tbh = tbhInitial[velocityIndex];
  flyTop->moveVoltage(12000);
  flyRight->moveVoltage(12000);
  flyBottom->moveVoltage(12000);
  prevCmd = tbh;

  Eigen::Matrix<double, 1, 1> x0, x, u;
  velFilter->init(pros::millis() / 1000, x0);
  int prevDesVel = RPM_TO_RADS * flywheelVelocities[velocityIndex];
  double prevVel = 0;
  int velDropCounter = 0;
  uint32_t now = pros::millis();
  while (true) {
    int cmd;
    bool tbhCalcFlag = true; // we'll skip this calculation if new setpoint
    double desiredVel = RPM_TO_RADS * flywheelVelocities[velocityIndex];
    u(0) = desiredVel * 12.0 / 60.0;
    Eigen::Matrix<double, 1, 1> y;
    y(0) = RPM_TO_RADS * flyTop->getActualVelocity();
    velFilter->update(y, u);
    x = velFilter->state();

    double velocity = x(0);
    double error = desiredVel - velocity;

    if (velDropCounter && velDropCounter < 12) {
      velDrop = true;
      velDropCounter++;
    } else if ((velocity < desiredVel * (60 / 63))) {
      velDrop = true;
      velDropCounter = 1;
    } else {
      velDrop = false;
      velDropCounter = 0;
    }
    prevVel = velocity;

    current = curFilter.filter(flyTop->getCurrentDraw());
    if (curSpikeCounter && curSpikeCounter < 12) {
      curSpike = true;
      curSpikeCounter++;
    } else if (current > 0.14) {
      curSpike = true;
      curSpikeCounter = 1;
    } else {
      curSpike = false;
      curSpikeCounter = 0;
    }
    prevCurrent = current;

    // Update the flywheel state
    if (settleUtil.isSettled(error)) {
      atVelocityFlag = true;
      tbhCalcFlag = false;
    } else {
      atVelocityFlag = false;
    }

    if (flywheelVelocities[velocityIndex] != prevDesVel)
      tbhCalcFlag = false;
    prevDesVel = flywheelVelocities[velocityIndex];

    // Take Back Half
    if (tbhCalcFlag) {
      if (std::signbit(error) != std::signbit(prevError)) {
        // Take back half
        tbh = (prevCmd + tbh) / 2;
        tbh = std::clamp(tbh, 0, 12000);

        // Update the velocity setpoints so the initial guess at our speed is
        // better
        tbhInitial[velocityIndex] = tbh;

        cmd = tbh;
      } else {
        cmd = prevCmd + FLYWHEEL_TBH_GAIN * error;
      }
    }
    prevError = error;
    cmd = std::clamp(cmd, 0, 12000);
    prevCmd = cmd;

    // Set the flywheel speed
    flyTop->moveVoltage((int)cmd);
    flyRight->moveVoltage((int)cmd);
    flyBottom->moveVoltage((int)cmd);

    // Debug
    pros::lcd::print(FLYWHEEL_VELOCITY_SETPOINT_LINE, "Flywheel set to: %10s",
                     flywheelVelocityStrings[velocityIndex]);
    pros::lcd::print(FLYWHEEL_MOTOR_TELEMETRY_LINE,
                     "FlyVel: %4.1lfrpm Torque: %2.3lfNm Cmd: %3d", velocity,
                     flyTop->getTorque(), cmd);

    pros::Task::delay_until(&now, dt * 1000);
  }
}

/**
 * This task serves a couple of purposes - to properly pull a single ball from
 * the intake into the indexer without shooting it, and to shoot a ball when the
 * flywheel is ready.
 */
void _indexerMonitor(void *ign) {
  while (true) {
    // Prioritize shooting requests
    if (numShotsWaiting) {
      int timeout = pros::millis();
      if (pros::competition::is_autonomous()) {
        // fire immediately if in opcontrol, otherwise wait to get a more
        // accurate shot
        while (!atVelocityFlag && pros::millis() - timeout < FLYWHEEL_TIMEOUT) {
          pros::delay(5);
        }
      }

      // Flywheel is ready, fire!
      intakeMutex->take(20);
      intakeMotor->moveVoltage(2000);
      indexerMotor->moveVoltage(12000);

      timeout = pros::millis();
      int start = pros::millis();
      prevPhotogateStatus = false;
      bool shotHappened = false;
      while (shotWaiting(start)) {
        if (pros::millis() - timeout > FLYWHEEL_STUTTER_TIMEOUT) {
          // stutter the intake to shake loose any stuck balls
          intakeMotor->moveVoltage(-8000);
          timeout = pros::millis() + FLYWHEEL_STUTTER_TIMEOUT * 4;
          while (pros::millis() - timeout < 150) {
            if (!shotWaiting(0)) {
              // shot went through, exit whole loop
              shotHappened = true;
              break;
            }
            pros::delay(5);
          }
          timeout = pros::millis();
          intakeMotor->moveVoltage(12000);
        }
        if (shotHappened)
          break;
        pros::delay(5);
      }

      indexerMotor->moveVelocity(0);
      indexerMotor->moveRelative(FLYWHEEL_INDEX_DISTANCE, 300);
      intakeMotor->moveVoltage(postIndexIntakePower);
      angleChanger->set_value(ANGLE_CHANGER_IN);
      intakeMutex->give();
      pros::delay(200); // this is probably fine to be hardcoded because we're
                        // in a task
      indexerMotor->moveVelocity(0);
      shotMutex->take(20);
      numShotsWaiting--;
      shotMutex->give();
    } else if (doubleShootFlag) {
      // Fire at high flag
      angleChanger->set_value(
          ANGLE_CHANGER_IN); // make sure that the mech is in
      int timeout = pros::millis();
      if (pros::competition::is_autonomous()) {
        // fire immediately if in opcontrol, otherwise wait to get a more
        // accurate shot
        while (!atVelocityFlag && pros::millis() - timeout < FLYWHEEL_TIMEOUT) {
          pros::delay(5);
        }
      }

      // Flywheel is ready, fire!
      intakeMutex->take(20);
      intakeMotor->moveVoltage(4000);
      intakeMotor->setCurrentLimit(2500);
      indexerMotor->moveVoltage(12000);
      timeout = pros::millis();
      int start = pros::millis();
      prevPhotogateStatus = false;
      bool shotHappened = false;
      while (shotWaiting(start)) {
        if (pros::millis() - timeout > FLYWHEEL_STUTTER_TIMEOUT) {
          // stutter the intake to shake loose any stuck balls
          intakeMotor->moveVoltage(-12000);
          timeout = pros::millis();
          while (pros::millis() - timeout < 150) {
            intakeMotor->moveVoltage(-12000);
            if (!shotWaiting(0)) {
              // shot went through, exit whole loop
              shotHappened = true;
              break;
            }
            pros::delay(5);
          }
          timeout = pros::millis();
          intakeMotor->moveVoltage(12000);
        }
        if (shotHappened)
          break;
        pros::delay(5);
      }
      indexerMotor->moveVelocity(0);

      // Fire at middle flag
      angleChanger->set_value(ANGLE_CHANGER_OUT);

      // fire immediately if in opcontrol, otherwise wait to get a more accurate
      // shot
      while (!doubleShootNoWaitFlag && !atVelocityFlag &&
             pros::millis() - timeout < FLYWHEEL_TIMEOUT) {
        pros::delay(5);
      }

      // Flywheel is ready, fire!
      indexerMotor->moveVoltage(12000);
      timeout = pros::millis();
      start = pros::millis();
      prevPhotogateStatus = false;
      shotHappened = false;
      while (!photogateFallingEdge() &&
             (pros::millis() - start < FLYWHEEL_INDEXER_TIMEOUT)) {
        if (pros::millis() - timeout > FLYWHEEL_STUTTER_TIMEOUT) {
          // stutter the intake to shake loose any stuck balls
          intakeMotor->moveVoltage(-8000);
          while (pros::millis() - timeout < 150) {
            if (!shotWaiting(0)) {
              // shot went through, exit whole loop
              shotHappened = true;
              break;
            }
            pros::delay(5);
          }
          timeout = pros::millis();
          intakeMotor->moveVoltage(12000);
        }
        if (shotHappened)
          break;
        pros::delay(5);
      }

      indexerMotor->moveVelocity(0);
      intakeMotor->moveVoltage(postIndexIntakePower);
      intakeMutex->give();

      pros::delay(100);
      angleChanger->set_value(ANGLE_CHANGER_IN);

      doubleShootMutex->take(20);
      doubleShootFlag = false;
      doubleShootMutex->give();

      doubleShootNoWaitMutex->take(20);
      doubleShootNoWaitFlag = false;
      doubleShootNoWaitMutex->give();
    } else if (indexFlag && !flywheel::indexerCheck()) {
      indexerMotor->moveVelocity(150);
      int timeout = pros::millis();
      bool timedout = false;
      while (true) {
        if (pros::millis() - timeout > FLYWHEEL_INDEXER_TIMEOUT) {
          timedout = true;
          break;
        } else if (photogateRisingEdge()) {
          timedout = false;
          break;
        }
        pros::delay(5);
      }
      indexerMotor->moveVelocity(0);
      if (!timedout) {
        indexerMotor->moveRelative(FLYWHEEL_INDEX_DISTANCE, 300);
        pros::delay(200); // this is probably fine to be hardcoded because
                          // we're in a task
      }
    }
    pros::delay(5);
  }
}

void _intakeMonitor(void *ign) {
  intakeVoltage = 12000;
  const int stallVel = 20;
  const int holdVolt = 3000;
  const int holdCur = 1500;
  int index = 0;
  int prevVel = stallVel;
  uint32_t now = pros::millis();
  while (pros::millis() - now < 3000) {
    // let the motor speed up before checking stall
    intakeMotor->moveVoltage(intakeVoltage);
    pros::delay(5);
  }
  while (true) {
    if (!indexFlag && !numShotsWaiting && intakeMutex->take(0)) {
      if (intakeMotor->getActualVelocity() < stallVel && prevVel < stallVel &&
          intakeVoltage > holdVolt && !noStallCheck) {
        intakeMotor->setCurrentLimit(holdCur);
        intakeMotor->moveVoltage(-3000);
        pros::delay(150);
        intakeMotor->moveVoltage(holdVolt);
        intakeHoldMutex->take(5);
        intakeHold = true;
        intakeHoldMutex->give();
      } else {
        intakeMotor->setCurrentLimit(2500);
        intakeMotor->moveVoltage(intakeVoltage);
        intakeHoldMutex->take(5);
        intakeHold = false;
        intakeHoldMutex->give();
      }

      intakeMutex->give();

      if (!(index % 500)) {
        prevVel = intakeMotor->getActualVelocity();
        index = 0;
      } else {
        index++;
      }
    } else if (intakeMutex->take(0)) {
      intakeMotor->moveVoltage(intakeVoltage);
      intakeMutex->give();

      intakeHoldMutex->take(5);
      intakeHold = false;
      intakeHoldMutex->give();
    } else {
      intakeHoldMutex->take(5);
      intakeHold = false;
      intakeHoldMutex->give();
    }

    pros::delay(5);
  }
}

namespace flywheel {

void start() {
  if (!initialized) {
    flyTop = std::make_unique<okapi::Motor>(FLYWHEEL_TOP);
    flyTop->setGearing(okapi::AbstractMotor::gearset::blue);
    flyTop->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    flyBottom = std::make_unique<okapi::Motor>(-FLYWHEEL_BOTTOM);
    flyBottom->setGearing(okapi::AbstractMotor::gearset::blue);
    flyBottom->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    flyRight = std::make_unique<okapi::Motor>(FLYWHEEL_RIGHT);
    flyRight->setGearing(okapi::AbstractMotor::gearset::blue);
    flyRight->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    indexerMotor = std::make_unique<okapi::Motor>(
        INDEXER, false, okapi::AbstractMotor::gearset::blue,
        okapi::AbstractMotor::encoderUnits::degrees);
    indexerMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    intakeMotor = std::make_unique<okapi::Motor>(
        INTAKE, true, okapi::AbstractMotor::gearset::green);
    intakeMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

    intakeLiftSolenoid = std::make_unique<pros::ADIDigitalOut>(INTAKE_LIFT);
    angleChanger = std::make_unique<pros::ADIDigitalOut>(ANGLE_CHANGER);

    shotMutex = std::make_unique<pros::Mutex>();
    indexMutex = std::make_unique<pros::Mutex>();
    doubleShootMutex = std::make_unique<pros::Mutex>();
    doubleShootNoWaitMutex = std::make_unique<pros::Mutex>();
    postIndexIntakePowerMutex = std::make_unique<pros::Mutex>();
    intakeMutex = std::make_unique<pros::Mutex>();
    intakeHoldMutex = std::make_unique<pros::Mutex>();

    F_A << -tau * dt;
    F_B << (1.0 / 12.0);
    F_C << 1.0;

    F_Q << 7.0;
    F_R << VAR_RADS;
    F_P << VAR_RADS;
    velFilter = new KalmanFilter<1, 1, 1>(0.01, F_A, F_B, F_C, F_Q, F_R, F_P);

    velocityTask = std::make_unique<pros::Task>(_velocityMonitor);
    indexerTask = std::make_unique<pros::Task>(_indexerMonitor);
    intakeTask = std::make_unique<pros::Task>(_intakeMonitor);

    setIntakeLift(LOW);
    angleChanger->set_value(ANGLE_CHANGER_IN);
    initialized = true;
  } else {
    velocityTask->resume();
    indexerTask->resume();
    intakeTask->resume();
  }
}

void stop() {
  if (initialized) {
    velocityTask->suspend();
    indexerTask->suspend();
    intakeTask->suspend();
    flyTop->moveVoltage(0);
    flyRight->moveVoltage(0);
    flyBottom->moveVoltage(0);
    indexerMotor->moveVoltage(0);
    intakeMotor->moveVoltage(0);
  }
}

void shoot() {
  if (numShotsWaiting > 1)
    return;
  shotMutex->take(5);
  numShotsWaiting++;
  shotMutex->give();
}

void shootBlocking() {
  if (logger)
    logger->log("F, Firing Once\n");
  shotMutex->take(20);
  int prev = numShotsWaiting;
  numShotsWaiting++;
  shotMutex->give();
  while (numShotsWaiting > prev) {
    pros::delay(5);
  }
}

void doubleShoot(bool noDelay) {
  doubleShootMutex->take(20);
  doubleShootFlag = true;
  doubleShootMutex->give();

  doubleShootNoWaitMutex->take(20);
  doubleShootNoWaitFlag = noDelay;
  doubleShootNoWaitMutex->give();
}

void doubleShootBlocking(bool noDelay) {
  if (logger)
    logger->log("F, Firing Twice\n");
  doubleShootMutex->take(20);
  doubleShootFlag = true;
  doubleShootMutex->give();

  doubleShootNoWaitMutex->take(20);
  doubleShootNoWaitFlag = noDelay;
  doubleShootNoWaitMutex->give();
  while (doubleShootFlag) {
    pros::delay(5);
  }
}

void indexOne(int intakePower) {
  if (logger)
    logger->log("F, Indexing a Ball\n");
  indexMutex->take(20);
  indexFlag = 1;
  indexMutex->give();
}

bool indexerCheck() { return indexerPhotogate.get_value(); }

bool intakeCheck() { return intakeHold; }

void intake(int power, bool inoStallCheck) {
  if (intakeMutex->take(5)) {
    intakeVoltage = power;
    intakeMutex->give();
    noStallCheck = inoStallCheck;
  }
  if (postIndexIntakePowerMutex->take(5)) {
    postIndexIntakePower = power;
    postIndexIntakePowerMutex->give();
  }
}

void setIntakeLift(bool state) { intakeLiftSolenoid->set_value(state); }

void setIntakeCurrentLimit(int limit) { intakeMotor->setCurrentLimit(limit); }

void setAngleChanger(bool val) { angleChanger->set_value(val); }

const char *setVelocitySetpoint(int velIndex) {
  if (logger)
    logger->log("F, Setting the %d Velocity Setpoint\n", velIndex);
  velocityIndex = std::clamp(velIndex, 0, numFlywheelVelocities);
  tbh = tbhInitial[velocityIndex];
  return flywheelVelocityStrings[velocityIndex];
}

const char *getVelocitySetpointString() {
  return flywheelVelocityStrings[velocityIndex];
}

bool isIntakeStalled() { return intakeHold; }

} // namespace flywheel
